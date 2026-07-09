// Insta360 X3 low-latency reader: single-lens fisheye preview over USB.
//
// ROS-free. Opens the camera via CameraSDK, starts live streaming, patches
// the SPS (the camera advertises a 6-frame reorder depth on an I/P-only
// stream = +200ms in any compliant decoder), decodes H.264 with libavcodec
// in low-delay mode, and delivers YUV 4:2:0 frames to a callback.
//
// Latency (measured glass-to-glass, ArUco method, fw v1.1.6):
//   720x720@30  ~155 ms      1280x1280@30  ~202 ms    1920x1920@30  ~262 ms
//
// IMPORTANT operational constraints:
//  - Always destroy this object (clean StopLiveStreaming+Close). A killed
//    process leaves a dangling session; the camera then refuses connections
//    until physically power cycled.
//  - One SDK client at a time.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace insta360 {

// Pointers in Frame/Packet are valid ONLY for the duration of the callback —
// copy what you need. Callbacks run on the SDK's stream thread; exceptions
// thrown from them are caught by the reader (they must never reach the SDK).
struct Frame {
  const uint8_t* y;  // luma plane
  const uint8_t* u;  // chroma planes (4:2:0, full range)
  const uint8_t* v;
  int width, height;
  int yStride, uvStride;
  double arrivalMono;  // CLOCK_MONOTONIC seconds: encoded packet arrival
                       // from the SDK, before decode
  double decodedMono;  // CLOCK_MONOTONIC seconds: decode finished
};

struct ReaderOptions {
  // Probed-good X3 modes: 9 -> 720x720, 1 -> 1280x1280, 0 -> 1920x1920.
  // Anything else risks junk fallback modes; unsupported requests can wedge
  // the camera.
  int resolution = 9;
  int sensor = 2;  // 1=front 2=rear 3=both
  int bitrate = 2 * 1024 * 1024;
  int startRetries = 3;
  std::string serial;  // pick this device; empty = first discovered
  // Polled during start()'s retry waits so a launch shutdown can abort the
  // (otherwise ~30s) retry sequence and still close the camera cleanly.
  std::function<bool()> keepGoing;
};

using FrameCallback = std::function<void(const Frame&)>;

// The camera's original H.264 access unit (SPS already patched), as received.
struct Packet {
  const uint8_t* data;
  size_t size;
  bool keyframe;
  double arrivalMono;
};
using PacketCallback = std::function<void(const Packet&)>;

class Reader {
 public:
  Reader();
  ~Reader();  // clean camera shutdown — never skip this

  // Discover + open the camera and start streaming. Blocking (a few
  // seconds). Returns false on failure (no camera / busy / bad mode).
  // frameCb receives decoded frames; packetCb (optional) receives the
  // camera's original encoded packets before decoding.
  bool start(const ReaderOptions& opts, FrameCallback frameCb, PacketCallback packetCb = nullptr);

  // Stop streaming and close the camera session cleanly.
  void stop();

  bool running() const { return running_; }
  bool isH265() const;  // codec negotiated with the camera (valid after start)
  std::string serial() const;
  std::string firmware() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> running_{false};
};

}  // namespace insta360
