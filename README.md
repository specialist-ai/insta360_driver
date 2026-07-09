# insta360_camera

[![CI](https://github.com/qiayuanl/insta360_camera/actions/workflows/ci.yml/badge.svg)](https://github.com/qiayuanl/insta360_camera/actions/workflows/ci.yml)

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

Topics sit under the node's name (its identity), so the default node
`insta360_camera` publishes them at `/insta360_camera/image_raw`:
- `image_raw` — decoded once here for all live consumers (rviz2, rqt_image_view,
  image_proc, your policy). `bgr8` by default, or `mono8` via the `pixel_format`
  param (the free Y plane, skipping the YUV->BGR convert). QoS RELIABLE keep_last(1).
- `image_raw/ffmpeg` — `FFMPEGPacket` h264: the CAMERA'S OWN bitstream,
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

Parameters: `mode` (see Modes), `pixel_format` (bgr8|mono8), `sensor`
(rear|front|all), `bitrate`, `publish_h264`, `frame_id`. Invalid mode or sensor values fail fast at startup
rather than being forwarded to the camera (arbitrary resolution ids can wedge
it). Image conversions are skipped while a topic has no subscribers.

## Reusing the reader without ROS

`insta360::Reader` (`include/insta360_camera/reader.h` + `src/reader.cpp`
+ `sps_patch.h`) has no ROS dependencies: `start(options, callback)` delivers
YUV420 frames with arrival timestamps; destructor closes the session cleanly.

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

Operational note: the camera needs ~60-90 s of idle after a clean session
close before it will accept a new session — budget settle time when
restarting the node.
