"""Glass-to-visible latency for the camera, measured the way a policy consumes it.

``aruco_latency_test.cpp`` times a ROS consumer. This times the bindings, and it
times them through the shape a deployment actually reads a camera in: a thread
pulling :meth:`Reader.read`, a per-frame reduction, one newest frame kept and the
rest dropped, and a shutter stamp that is earlier than any stamp the host can
take. :class:`ThreadedSource` is that shape with nothing specific to a robot in
it, and :func:`measure` works on anything wearing the same four methods, so a
deployment can hand in its own camera object and time the real one instead.

Three instants per frame, all on ``CLOCK_MONOTONIC``:

    shown     a marker is put on the screen -- the glass instant
    arrival   the encoded packet reached the host; the driver's own stamp
    visible   ``latest()`` first hands that frame out

    glass -> visible     what the consumer waits for a frame
    glass -> arrival     the in-camera pipeline the host cannot otherwise see
    arrival -> visible   decode + reduce + thread handoff + poll

A source reports a shutter of ``arrival - sensor_delay``, because the camera's own
pipeline is not observable from here and ages downstream have to be measured
against something. Here the true shutter is known -- it is ``shown`` -- so the gap
between the two is a direct check on whatever ``sensor_delay`` was configured.

Two biases inflate every absolute, and the C++ test carries both, which is what
keeps the two comparable. ``shown`` is when ``imshow`` returned rather than when
photons left the panel, so the monitor's own latency (5-15 ms on a typical LCD)
is in the figure; and a frame catches a marker that has already been up for part
of the flash period, which adds half of it again -- about 16 ms at the 33 ms
default. Neither touches a *difference* between two runs, which is what this is
good for; an absolute wants both subtracted.

Needs the ``bench`` dependency group and a monitor the camera is pointed at:

    uv run --group bench python -m insta360_driver.latency
"""

import argparse
import shutil
import subprocess
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

import numpy as np

__all__ = ["Flash", "FrameSource", "Sample", "ThreadedSource", "measure", "reduce_frame", "report"]

MARKER_DICT_NAME = "DICT_4X4_250"
"""250 ids is a long cycle: at 30 fps a stale detection cannot alias onto the id
just shown. 4x4 is the coarsest grid on offer, which is what survives a reduction
down to a couple of hundred pixels."""

CANVAS_WIDTH, CANVAS_HEIGHT = 1280, 720
WINDOW = "flash"


def reduce_frame(image: np.ndarray, size: int, radius: float = 0.5) -> np.ndarray:
    """One frame as a policy trained on a fisheye reads it: block-meaned to
    ``size`` square and blanked outside the disc.

    Args:
        image: ``HxW`` or ``HxWx3`` uint8, square, as the reader delivers it.
        size: the side of the result. ``width // size`` must be an exact block,
            so the reduction is a mean over whole pixels and never a resample.
        radius: the kept disc, in widths of the result. 0.5 is the inscribed
            circle, which is the fisheye's own image circle.

    Returns:
        ``size x size`` uint8, same channel count, zero outside the disc.

    The block mean is integer and rounds half up, so it is reproducible across
    machines in a way a float resize is not. This is the reduction the corpus is
    built with; it is written out here rather than imported so that profiling the
    driver needs nothing but numpy.
    """
    height, width = image.shape[:2]
    scale = width // size if size else 0
    if scale < 1 or size * scale != width or height != width:
        raise ValueError(
            f"a {width}x{height} frame does not reduce to {size} square by whole "
            f"blocks; pick a size that divides {width} exactly"
        )
    block = image.reshape(height // scale, scale, width, *image.shape[2:]).sum(1, dtype=np.uint32)
    block = block.reshape(size, size, scale, *image.shape[2:]).sum(2, dtype=np.uint32)
    small: np.ndarray = ((block + scale * scale // 2) // (scale * scale)).astype(np.uint8)
    offset = np.arange(size) + 0.5 - size / 2.0
    small[offset[None, :] ** 2 + offset[:, None] ** 2 > (radius * size) ** 2] = 0
    return small


class FrameSource(Protocol):
    """What :func:`measure` needs of a camera: the newest frame and the instant its
    shutter opened. A deployment's own camera object satisfies this as it stands."""

    def start(self) -> None: ...

    def latest(self) -> tuple[np.ndarray, float] | None: ...

    def trouble(self) -> str: ...

    def close(self) -> None: ...


class ThreadedSource:
    """The camera as one newest frame, the way a deployment reads it.

    A thread blocks on :meth:`Reader.read`, reduces the frame, and keeps only the
    newest: a consumer that falls behind wants the current frame, not a backlog.
    That is the arrangement the latency of a real stack is made of, so it is the
    one profiled here rather than a bare ``read()`` loop.

    Args:
        reduce: applied to every frame on the camera thread, as a deployment's
            own reduction would be. ``None`` keeps the frame as delivered.
        sensor_delay: seconds between the shutter opening and ``arrival_mono``,
            which no stamp on this side can see. :meth:`latest` subtracts it, so
            what it reports is a shutter rather than an arrival. Whether the
            figure is right is exactly what :func:`measure` checks.
        mode: 720, 1280 or 1920.
        sensor: ``rear``, ``front`` or ``all``.
        pixel_format: ``bgr8``, ``rgb8`` or ``mono8``.
        read_timeout_ms: passed to :meth:`Reader.read`; a timeout is retried, so
            it only bounds how often the thread checks whether it should stop.
    """

    def __init__(
        self,
        reduce: Callable[[np.ndarray], np.ndarray] | None = None,
        sensor_delay: float = 0.0,
        mode: int = 720,
        sensor: str = "rear",
        pixel_format: str = "rgb8",
        read_timeout_ms: int = 200,
    ) -> None:
        from insta360_driver import Reader

        self._reader = Reader()
        self._reduce = reduce
        self._sensor_delay = sensor_delay
        self._options = dict(mode=mode, sensor=sensor, pixel_format=pixel_format)
        self._read_timeout_ms = read_timeout_ms
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: np.ndarray | None = None
        self._arrival = 0.0
        self._failure: Exception | None = None

    def start(self) -> None:
        """Open the camera and start the thread. Blocks a few seconds while the
        SDK negotiates the stream."""
        self._reader.start(**self._options)
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def latest(self) -> tuple[np.ndarray, float] | None:
        """The newest frame and the instant its shutter opened, or ``None`` before
        the first one."""
        with self._lock:
            if self._frame is None:
                return None
            return self._frame, self._arrival - self._sensor_delay

    def trouble(self) -> str:
        """Why the camera is not delivering frames, or ``""`` while it is."""
        if self._failure is not None:
            return f"camera stopped: {self._failure!r}"
        with self._lock:
            if self._frame is None:
                return "no frames yet"
            age = time.monotonic() - self._arrival
        return f"no frame for {age:.1f} s" if age > 1.0 else ""

    def close(self) -> None:
        """Stop the thread, then the session. The SDK takes one client per machine
        and a client that dies without closing leaves the camera refusing
        connections until it is physically power cycled."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._reader.stop()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    frame = self._reader.read(timeout_ms=self._read_timeout_ms)
                except TimeoutError:
                    continue
                prepared = self._reduce(frame.image) if self._reduce else frame.image
                with self._lock:
                    self._frame = prepared
                    self._arrival = frame.arrival_mono
        except Exception as error:
            self._failure = error


@dataclass
class Flash:
    """How the markers are shown and how many are timed."""

    samples: int = 40
    warmup: int = 10
    marker_px: int = 600
    border_px: int = 60
    markers: int = 250
    period: float = 0.033
    reject: float = 0.8
    timeout: float = 180.0


@dataclass
class Sample:
    """One marker, from the screen to the frame a consumer can act on."""

    glass_to_visible: float
    glass_to_arrival: float
    arrival_to_visible: float
    shutter_error: float


def marker_tiles(count: int, marker_px: int) -> list[np.ndarray]:
    """One ``marker_px`` square per id, to be blitted into the display canvas."""
    import cv2  # only the flashing half needs OpenCV; the module imports without it

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT_NAME))
    return [cv2.aruco.generateImageMarker(dictionary, i, marker_px) for i in range(count)]


def display_canvas(marker_px: int, border_px: int) -> tuple[np.ndarray, np.ndarray]:
    """A mid-gray canvas with a white marker box centred on it, and a view of the
    box interior. Only the interior changes between markers, so the canvas is
    built once and written through."""
    canvas = np.full((CANVAS_HEIGHT, CANVAS_WIDTH, 3), 128, np.uint8)
    span = marker_px + 2 * border_px
    # A span past the canvas slices negative rather than raising, and the marker
    # that comes out is cropped but still detectable, so this has to be caught.
    if span > CANVAS_HEIGHT:
        raise ValueError(
            f"a {marker_px} px marker in a {border_px} px border needs {span} px, "
            f"more than the {CANVAS_HEIGHT} px canvas"
        )
    top, left = (CANVAS_HEIGHT - span) // 2, (CANVAS_WIDTH - span) // 2
    canvas[top : top + span, left : left + span] = 255
    box = canvas[
        top + border_px : top + border_px + marker_px,
        left + border_px : left + border_px + marker_px,
    ]
    return canvas, box


def raise_window() -> None:
    """Window managers bury a fullscreen window that never took focus. Ask once;
    a failure is worth a warning and not an abort."""
    if shutil.which("wmctrl") is None:
        print("wmctrl not installed -- keep the marker window on top yourself")
        return
    if subprocess.call(["wmctrl", "-r", WINDOW, "-b", "add,above"]) != 0:
        print("wmctrl could not raise the window -- keep it on top yourself")


def measure(source: FrameSource, flash: Flash, sensor_delay: float) -> list[Sample]:
    """Flash markers and time each one to the frame ``source.latest()`` hands out.

    Args:
        source: any camera wearing ``start``/``latest``/``trouble``/``close``.
            A deployment's own camera object goes straight in, which is the point:
            what gets timed is then the real path and not a rehearsal of it.
        flash: the marker cadence and how many samples to keep.
        sensor_delay: what ``source.latest()`` already subtracted, so ``arrival``
            can be put back together. A source is not asked for it: the number
            lives wherever that source was configured.

    Returns:
        One :class:`Sample` per marker timed, warm-up included; hand the list and
        ``flash.warmup`` to :func:`report`.
    """
    import cv2

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT_NAME)),
        cv2.aruco.DetectorParameters(),
    )
    tiles = marker_tiles(flash.markers, flash.marker_px)
    canvas, box = display_canvas(flash.marker_px, flash.border_px)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    box[:] = tiles[0][:, :, None]
    cv2.imshow(WINDOW, canvas)
    cv2.waitKey(50)
    raise_window()

    shown_at: dict[int, float] = {}
    samples: list[Sample] = []
    marker_id, last_detected = 0, -1
    last_shutter, last_shown = None, 0.0
    frames = 0
    started = time.monotonic()

    while len(samples) < flash.samples + flash.warmup:
        if time.monotonic() - started > flash.timeout:
            print(f"timed out with {len(samples)} samples")
            break
        if time.monotonic() - last_shown >= flash.period:
            marker_id = (marker_id + 1) % len(tiles)
            box[:] = tiles[marker_id][:, :, None]
            cv2.imshow(WINDOW, canvas)
            cv2.waitKey(1)
            last_shown = time.monotonic()
            shown_at[marker_id] = last_shown

        seen = source.latest()
        if seen is None or seen[1] == last_shutter:
            time.sleep(0.001)
            continue
        frame, shutter = seen
        visible = time.monotonic()
        last_shutter = shutter
        frames += 1

        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        ids = detector.detectMarkers(gray)[1]
        if ids is None or len(ids) == 0:
            continue
        # OpenCV 4 hands back an (N, 1) column and OpenCV 5 a flat (N,); flatten
        # rather than pick an index that is right in only one of them.
        detected = int(np.asarray(ids).reshape(-1)[0])
        if detected == last_detected:
            continue
        last_detected = detected
        shown = shown_at.get(detected)
        if shown is None:
            continue
        glass_to_visible = visible - shown
        if not 0.0 < glass_to_visible < flash.reject:
            continue
        arrival = shutter + sensor_delay
        samples.append(
            Sample(
                glass_to_visible=glass_to_visible,
                glass_to_arrival=arrival - shown,
                arrival_to_visible=visible - arrival,
                shutter_error=shutter - shown,
            )
        )
        print(
            f"sample {len(samples):3d}: glass->visible={1e3 * glass_to_visible:6.1f} ms  "
            f"shutter error={1e3 * (shutter - shown):+6.1f} ms"
        )

    cv2.destroyAllWindows()
    print(f"{len(samples)} markers timed over {frames} frames")
    return samples


def report(samples: list[Sample], warmup: int, period: float, sensor_delay: float) -> None:
    """Print the three stages and what they say about ``sensor_delay``."""
    warm = samples[warmup:]
    if len(warm) < 4:
        print(f"too few samples to report ({len(warm)} after the {warmup} warm-up)")
        print("a low count is usually reach: fill more of the fisheye with the monitor")
        return

    def row(name: str, values: list[float]) -> None:
        ms = np.asarray(values) * 1e3
        print(
            f"  {name:<20}{np.median(ms):8.1f}{ms.mean():8.1f}{ms.min():8.1f}"
            f"{ms.max():8.1f}{np.percentile(ms, 95):8.1f}"
        )

    print(f"\nn={len(warm)} (after a {warmup}-sample warm-up), milliseconds")
    print(f"  {'':<20}{'median':>8}{'mean':>8}{'min':>8}{'max':>8}{'p95':>8}")
    row("glass -> visible", [s.glass_to_visible for s in warm])
    row("glass -> arrival", [s.glass_to_arrival for s in warm])
    row("arrival -> visible", [s.arrival_to_visible for s in warm])

    delay = float(np.median([s.glass_to_arrival for s in warm])) * 1e3
    error = float(np.median([s.shutter_error for s in warm])) * 1e3
    print(
        f"\nsensor_delay is {1e3 * sensor_delay:.0f} ms and glass->arrival measures "
        f"{delay:.1f} ms, so the\nshutter this source reports lands {error:+.1f} ms from "
        f"the instant the marker went up."
    )
    print(
        f"\nEvery figure above is inflated by two biases, and both want subtracting "
        f"before an\nabsolute is read off them: half the {1e3 * period:.0f} ms flash period "
        f"({1e3 * period / 2:.1f} ms), because a frame\ncatches a marker that has already "
        f"been up for part of it, and the monitor's own\nlatency (5-15 ms on a typical LCD). "
        f"Differences between runs carry neither."
    )


def run(source: FrameSource, flash: Flash, sensor_delay: float) -> None:
    """Open ``source``, wait for a first frame, measure, report, and always close.

    The close is the part that matters: a client that dies without closing leaves
    the camera refusing connections until it is physically power cycled.
    """
    print("opening the camera (the SDK takes a few seconds to negotiate the stream)")
    try:
        source.start()
        deadline = time.monotonic() + 15.0
        while source.latest() is None:
            if time.monotonic() > deadline:
                raise SystemExit(f"no frames: {source.trouble()}")
            time.sleep(0.05)
        report(measure(source, flash, sensor_delay), flash.warmup, flash.period, sensor_delay)
    finally:
        source.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", type=int, default=720, choices=[720, 1280, 1920])
    parser.add_argument("--sensor", default="rear", choices=["rear", "front", "all"])
    parser.add_argument("--pixel-format", default="rgb8", choices=["bgr8", "rgb8", "mono8"])
    parser.add_argument(
        "--size",
        type=int,
        default=240,
        help="reduce each frame to this square before it is handed out, as a "
        "deployment does; 0 keeps the frame at full size",
    )
    parser.add_argument(
        "--sensor-delay",
        type=float,
        default=0.120,
        help="seconds the shutter precedes the driver's stamp. The run says how "
        "far off this is",
    )
    parser.add_argument("--samples", type=int, default=40, help="markers to time after warm-up")
    parser.add_argument("--warmup", type=int, default=10, help="early samples to discard")
    parser.add_argument("--marker", type=int, default=600, help="marker side on the canvas, px")
    parser.add_argument("--border", type=int, default=60, help="white quiet zone around it, px")
    parser.add_argument("--markers", type=int, default=250, help="ids to cycle through")
    parser.add_argument("--period", type=float, default=0.033, help="seconds a marker is held")
    parser.add_argument(
        "--reject", type=float, default=0.8, help="seconds above which a match is aliasing"
    )
    parser.add_argument("--timeout", type=float, default=180.0, help="seconds before giving up")
    args = parser.parse_args()

    flash = Flash(
        samples=args.samples,
        warmup=args.warmup,
        marker_px=args.marker,
        border_px=args.border,
        markers=args.markers,
        period=args.period,
        reject=args.reject,
        timeout=args.timeout,
    )
    source = ThreadedSource(
        reduce=(lambda image: reduce_frame(image, args.size)) if args.size else None,
        sensor_delay=args.sensor_delay,
        mode=args.mode,
        sensor=args.sensor,
        pixel_format=args.pixel_format,
    )
    run(source, flash, args.sensor_delay)


if __name__ == "__main__":
    main()
