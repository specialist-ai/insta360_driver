# insta360_camera

[![CI](https://github.com/specialist-ai/insta360_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/specialist-ai/insta360_driver/actions/workflows/ci.yml)

Low-latency Insta360 X3 rear-lens fisheye reader + ROS 2 publisher.

Raw (unstitched) single-lens fisheye preview over USB at ~155 ms
glass-to-glass (720×720@30).

## What it does

```
X3 → H.264 preview over USB (CameraSDK) → SPS reorder patch → libavcodec
low-delay decode → YUV420 → sensor_msgs/Image (bgr8 / mono8)
```

- **SPS patch**: the X3 advertises `max_num_reorder_frames=6` on an I/P-only
  stream; without the patch every compliant decoder adds **+200 ms** of
  buffering. Measured: 353 ms vs 155 ms glass-to-glass.
- **Timestamps**: `header.stamp` is the SDK packet arrival time (CLOCK_MONOTONIC,
  mapped to ROS time). True glass-to-stamp age is ~120 ms of in-camera
  pipeline not observable from the host — budget for it downstream.
- **Clean shutdown**: the node closes the camera session on destruction.
  A SIGKILLed client leaves a dangling session — the camera then refuses
  connections until physically power cycled. One SDK client at a time.

## Modes (probed; everything else is junk or wedges the camera)

| mode param | image | glass-to-glass (median) |
|---|---|---|
| `720` (default) | 720×720 fisheye @30 | **~155 ms** |
| `1280` | 1280×1280 fisheye @30 | ~202 ms |
| `1920` | 1920×1920 fisheye @30 | ~262 ms |

## Build

Requires ROS 2 (tested on Jazzy), ffmpeg dev libraries
(`libavcodec-dev libswscale-dev`),
`ros-<distro>-ffmpeg-image-transport-msgs` (and optionally
`ros-<distro>-ffmpeg-image-transport` for subscriber-side decoding), and a
udev rule for USB access:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-insta360.rules
sudo udevadm control --reload-rules

cd <ws>/src && git clone <this repo>  # -> src/insta360_camera
cd <ws> && colcon build --packages-select insta360_camera
```

The vendored `sdk/` contains the Linux x86_64 CameraSDK 2.1.1 headers and
`libCameraSDK.so`.

## Run

Camera connected over USB in USB/Android mode:

```bash
ros2 launch insta360_camera camera.launch.py            # 720x720 bgr8
ros2 launch insta360_camera camera.launch.py mode:=1280
ros2 launch insta360_camera camera.launch.py pixel_format:=mono8
```

Each camera in the `cameras` param publishes under its own name; the default is
one camera named after the node, so `/insta360_camera/image_raw`:
- `<name>/image_raw` — decoded once here for all live consumers (rviz2,
  rqt_image_view, image_proc, your policy). `bgr8` by default, or `mono8` via the
  `pixel_format` param (the free Y plane, skipping the YUV->BGR convert).
  QoS RELIABLE keep_last(1).
- `<name>/image_raw/ffmpeg` — `FFMPEGPacket` h264: the CAMERA'S OWN bitstream,
  pass-through (no decode->re-encode round trip, no quality loss, ~0.25 MB/s).
  Named per `ffmpeg_image_transport`, so an `image_transport` subscriber on
  `image_raw` with the `ffmpeg` transport decodes it directly; or rosbag it raw.

## Zero-copy / composition

The node is a `rclcpp_components` component (`insta360_camera::CameraNode`).
Load it into the same
`component_container` as your consumer with intra-process comms enabled and
published images transfer as pointer moves (no serialization, no copy):

```bash
ros2 run rclcpp_components component_container &
ros2 component load /ComponentManager insta360_camera insta360_camera::CameraNode -e use_intra_process_comms:=true
ros2 component load /ComponentManager <your_pkg> <YourConsumer> -e use_intra_process_comms:=true
```

(Intra-process comms must be enabled on BOTH components or the zero-copy
path silently degrades to normal DDS.)

Parameters: `cameras` (list of `"name"` or `"name:serial"`; default one camera
named after the node), `mode` (see Modes), `pixel_format` (bgr8|mono8), `sensor`
(rear|front|all), `bitrate`, `publish_h264`. Invalid mode or sensor values fail
fast at startup rather than being forwarded to the camera (arbitrary resolution
ids can wedge it). Image conversions are skipped while a topic has no subscribers.

## Multiple cameras

**One node drives every camera.** The SDK binds a fixed local port per process,
so two camera processes can't coexist (the second dies with `bind: Address
already in use`) — the cameras must share one process. List them in the `cameras`
param as `"name:serial"`; each streams on its own SDK thread and publishes under
`/<name>/image_raw` with `header.frame_id` = its name. Find the serials with
(prepend `sudo` if it prints nothing):

```bash
lsusb -v -d 2e1a: 2>/dev/null | awk '/iSerial/{print $3}'
```

```bash
ros2 run insta360_camera camera_node --ros-args \
  -p cameras:="[left_camera:<left_serial>, right_camera:<right_serial>]"
# /left_camera/image_raw  (frame_id "left_camera")
# /right_camera/image_raw (frame_id "right_camera")
```

## Reusing the reader without ROS

`insta360::Reader` (`include/insta360_camera/reader.h` + `src/reader.cpp`
+ `sps_patch.h`) has no ROS dependencies: `start(options, callback)` delivers
YUV420 frames with arrival timestamps; destructor closes the session cleanly.

`ament` is detected rather than required, so a plain `cmake -S . -B build`
outside a ROS workspace builds the reader alone. Pass
`-DINSTA360_BUILD_ROS=ON` to force the node in, or `OFF` to leave it out of a
colcon build.

## Python

```bash
uv sync --dev                    # the extension and the package
uv run pytest python/tests -q    # no camera needed
```

```python
import insta360_driver as insta360

with insta360.Reader() as reader:
    reader.start(mode=720, pixel_format='rgb8')
    frame = reader.read(timeout_ms=200)
    print(frame.image.shape, frame.arrival_mono)
```

`mode` is 720, 1280 or 1920 and `pixel_format` is `bgr8`, `rgb8` or `mono8`,
the same choices the node takes. `frame.image` is a numpy array, HxWx3 or HxW
for `mono8`.

**The binding pulls where the C++ API pushes.** A Python callback would run on
the SDK's stream thread and take the GIL inside the decode path, which is the
path this driver exists to keep short. So the C++ callback converts each frame
into a one-frame slot without touching Python, and `read()` copies out of it.
A consumer that falls behind gets the newest frame, never a backlog.

`read(timeout_ms)` waits for a frame it has not returned before and raises
`TimeoutError` if none arrives, which paces a caller to the camera rather than
to itself. `latest()` returns the newest frame without waiting, or `None`
before the first one.

`arrival_mono` is the timestamp to use. It is `CLOCK_MONOTONIC` seconds at
packet arrival, before host-side decode; `decoded_mono` is when the decode
finished. Neither includes the camera's own ~120 ms internal pipeline.

The wheel carries `libinsta360_reader.so` and `libCameraSDK.so` beside the
extension module and finds them through `$ORIGIN`. **Do not publish it.** The
CameraSDK licence permits shipping the SDK as part of an application, not on a
stand-alone basis, and every public Insta360 integration leaves it out and
tells users to apply for their own.

## Training-data compatibility

The published fisheye is geometrically an **exact 4× downscale** of the
rear-lens stream in 5.7K30 `.insv` recordings (verified by registration:
scale 0.2503, rotation ≤0.04°). Train on recordings downscaled with
`cv2.resize` to the deployment mode size; expect a small texture gap
(preview is ~15% contrastier from in-camera sharpening + 2 Mbps H.264).

## Consumer notes

- All topics are published RELIABLE; compatible with both reliable and
  best-effort subscribers.
- Decoder pick: consumers should pin the software decoder to avoid a possible
  full-range color shift through hardware decoders:
  `-p "<base>.ffmpeg.decoders.h264:=h264"` on the subscribing node.

## Measured latency with ROS in the loop (720x720@30, glass->ArUco detect)

Full end-to-end (monitor pixel -> camera -> driver -> ROS -> consumer ->
marker detected), ArUco marker method:

| Consumer | glass->detect median | ROS-only (stamp->recv) |
|---|---|---|
| Zero-copy composed (same process) | 153.5 ms | 2.26 ms |
| C++ separate process (DDS)        | 152.8 ms | 2.99 ms |
| Python rclpy (separate process)   | 154.5 ms | 3.57 ms |

All within noise of the camera's ~150 ms internal pipeline. ROS transport adds
2-3.5 ms (zero-copy < DDS < Python, as expected), so `stamp->recv` is dominated
by the ~1.7 ms decode+BGR convert. Pick the consumer by ergonomics, not latency.

Test tools live in `test/` and are **dev-only, off by default** — a normal
build ships just the driver (`insta360_reader` + `camera_node`), with no
OpenCV dependency. Opt in to the benchmarks explicitly:

```bash
colcon build --packages-select insta360_camera \
  --cmake-args -DINSTA360_BUILD_BENCHMARKS=ON
```

The tools: `aruco_latency_test` (full C++ glass->detect loop), `latency_probe`
(stamp->recv only), and `test/run_latency_comparison.sh` (all three variants).

Operational note: the camera needs ~60-90 s of idle after a clean session
close before it will accept a new session — budget settle time when
restarting the node.

## Profiling latency with no ROS at all

`insta360_driver.latency` runs the same ArUco method over the bindings instead of
a ROS topic, and it measures the camera the way a policy consumes it: a thread
pulls frames, reduces each one, keeps the newest and drops the rest, and stamps a
shutter earlier than any stamp the host can take. It is a module, not a script in
`test/`, so a deployment can import it.

Add the `bench` group (OpenCV for the markers, numpy for the reduction) and point
the camera at a monitor:

```bash
uv run --group bench python -m insta360_driver.latency
uv run --group bench python -m insta360_driver.latency --mode 1280 --size 256
uv run --group bench python -m insta360_driver.latency --size 0   # no reduction
```

The run splits the wait into the part the host cannot see and the part it adds:

| | what it covers |
|---|---|
| `glass -> visible` | the whole wait, screen to `latest()` |
| `glass -> arrival` | the in-camera pipeline, up to the driver's stamp |
| `arrival -> visible` | decode + reduce + thread handoff + poll |

`report` also bins `glass -> visible` at 10 ms, because a median cannot separate a
tight cluster from a fast path with a long tail. `histogram(values, bin_ms)` takes
any stage and any bin width and returns `(left edge in ms, count)` pairs. Edges
start at 0, so two runs are comparable.

The last line matters most. A consumer cannot see the camera's own pipeline, so it
reports a shutter of `arrival - sensor_delay` and ages later stages against it.
Here the true shutter is known, so the gap checks whatever `--sensor-delay` claims.

Two biases inflate the absolute numbers: the monitor's own latency, and **half the
flash period**, because a frame catches a marker already up (~16 ms at the 33 ms
default). Both are in the C++ numbers too, and neither changes a difference
between runs.

### Measured (720×720@30, 2026-09-11)

`python -m insta360_driver.latency --samples 1000`, so `ThreadedSource` with the
240 px reduction and `--sensor-delay 0.120`. n=1000 after warm-up, 1010 markers
over 1655 frames, milliseconds:

| | median | mean | min | max | p95 |
|---|---|---|---|---|---|
| `glass -> visible` | **153.7** | 154.9 | 127.3 | 186.4 | 172.7 |
| `glass -> arrival` | 147.9 | 147.8 | 115.7 | 175.4 | 164.8 |
| `arrival -> visible` | **5.9** | 7.1 | 4.9 | 21.1 | 13.4 |

```
glass -> visible, 10 ms bins (n=1000)
     120-130     14  ##
     130-140     81  ###########
     140-150    282  ######################################
     150-160    294  ########################################
     160-170    159  ######################
     170-180    165  ######################
     180-190      5  #
```

Three things to take from it.

**The host adds ~6 ms, the camera the other 148.** Decode, the block mean, the
thread handoff and the poll are `arrival -> visible`: single digits at the median,
p95 13.4 ms, worst 21.1 ms. Even the worst is an eighth of what the camera spends,
so latency work belongs in the camera or nowhere.

**It corroborates the ~120 ms in-camera pipeline.** Take the 147.9 ms of
`glass -> arrival`, subtract the half-period bias (16.5 ms) and a typical LCD's
5-15 ms, and ~120-127 ms is left, against the 120 ms `--sensor-delay` claims. The
reported shutter landed +27.9 ms from the marker, which is those two biases and
little else. An n=40 run four days earlier gave medians of 157.2 / 147.8 / 6.7 ms,
so the figure is stable. `glass -> visible` at 154 ms also matches the 155 ms the
ROS path measures over an entirely different transport.

**Most of the spread comes from the method, but budget the p95 anyway.** The bulk
is a block from 140 to 180 ms, with hard edges at 127 and 186 and a step down at
160 rather than a decaying tail. A single true latency produces such a ~33 ms
plateau here: the 33 ms flash and the 30 fps sensor share a period, so a marker
waits a uniform fraction of a frame interval before any frame catches it. The
camera's own jitter is therefore tighter than the histogram looks. The split
between the two levels does survive: 32.9% of frames land in the 160-180 ms
shoulder, 10-30 ms behind the median. Size a control budget with the 173 ms p95.
Compare one build against another with the 154 ms median.

### Timing your own camera object

`measure` takes any object with `start`, `latest`, `trouble` and `close`, where
`latest()` returns `(frame, shutter)` or `None`. Pass the object your deployment
reads, and the run times the real path — the real thread, reduction and delay
constant — not a reconstruction that can drift:

```python
from insta360_driver.latency import Flash, run

run(my_deployment_camera, Flash(), sensor_delay=0.120)
```

`ThreadedSource` is the built-in implementation of that shape, and
`reduce_frame(image, size)` is the block mean plus disc blank a fisheye policy is
trained on. It is integer and round-half-up, so it is reproducible across machines
in a way a float resize is not. Both are plain numpy, so neither this repo nor a
deployment depends on the other.
