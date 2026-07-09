#include "insta360_camera/reader.h"

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>
#include <stream/stream_delegate.h>

extern "C" {
#include <libavcodec/avcodec.h>
}

#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>
#include <vector>

#include "insta360_camera/sps_patch.h"

namespace insta360 {

static double monoNow() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec / 1e9;
}

namespace {

class Delegate : public ins_camera::StreamDelegate {
 public:
  Delegate(FrameCallback frameCb, PacketCallback packetCb, bool h265) : frameCb_(std::move(frameCb)), packetCb_(std::move(packetCb)) {
    const AVCodec* codec = avcodec_find_decoder(h265 ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264);
    ctx_ = avcodec_alloc_context3(codec);
    ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    ctx_->flags2 |= AV_CODEC_FLAG2_FAST;
    ctx_->thread_count = 1;  // frame-threading adds 1 frame latency/thread
    avcodec_open2(ctx_, codec, nullptr);
    pkt_ = av_packet_alloc();
    frame_ = av_frame_alloc();
  }

  ~Delegate() {
    av_frame_free(&frame_);
    av_packet_free(&pkt_);
    avcodec_free_context(&ctx_);
  }

  void drain() { draining_ = true; }

  void OnAudioData(const uint8_t*, size_t, int64_t) override {}
  void OnGyroData(const std::vector<ins_camera::GyroData>&) override {}
  void OnExposureData(const ins_camera::ExposureData&) override {}

  void OnVideoData(const uint8_t* data, size_t size, int64_t, uint8_t, int streamIndex) override {
    // Exception barrier: this runs on the SDK's stream thread. Anything
    // escaping here (publish failure during shutdown, bad_alloc) would
    // std::terminate the process, skipping StopLiveStreaming+Close and
    // wedging the camera until a physical power cycle.
    try {
      onVideo(data, size, streamIndex);
    } catch (const std::exception& e) {
      fprintf(stderr, "[insta360] callback exception (draining): %s\n", e.what());
      draining_ = true;
    } catch (...) {
      fprintf(stderr, "[insta360] callback exception (draining)\n");
      draining_ = true;
    }
  }

  void onVideo(const uint8_t* data, size_t size, int streamIndex) {
    if (draining_ || streamIndex != 0) return;
    const double arrival = monoNow();

    // SPS advertises max_num_reorder_frames=6 on an I/P-only stream; patch
    // it to zero or the decoder buffers 6 frames (+200ms @30fps).
    std::vector<uint8_t> patched = sps_patch::patchAnnexb(data, size);
    const uint8_t* au = patched.empty() ? data : patched.data();
    const size_t auSize = patched.empty() ? size : patched.size();
    if (packetCb_) {
      Packet p;
      p.data = au;
      p.size = auSize;
      // keyframe AUs lead with SPS/PPS/IDR; handle 3- and 4-byte start codes
      int nal = -1;
      if (auSize >= 5 && au[0] == 0 && au[1] == 0 && au[2] == 0 && au[3] == 1)
        nal = au[4] & 0x1f;
      else if (auSize >= 4 && au[0] == 0 && au[1] == 0 && au[2] == 1)
        nal = au[3] & 0x1f;
      p.keyframe = (nal == 7 || nal == 8 || nal == 5);
      p.arrivalMono = arrival;
      packetCb_(p);
    }
    pkt_->data = const_cast<uint8_t*>(au);
    pkt_->size = static_cast<int>(auSize);
    if (avcodec_send_packet(ctx_, pkt_) < 0) return;
    while (avcodec_receive_frame(ctx_, frame_) == 0) {
      if (draining_) return;
      Frame f;
      f.y = frame_->data[0];
      f.u = frame_->data[1];
      f.v = frame_->data[2];
      f.width = frame_->width;
      f.height = frame_->height;
      f.yStride = frame_->linesize[0];
      f.uvStride = frame_->linesize[1];
      f.arrivalMono = arrival;
      f.decodedMono = monoNow();
      frameCb_(f);
    }
  }

 private:
  FrameCallback frameCb_;
  PacketCallback packetCb_;
  AVCodecContext* ctx_ = nullptr;
  AVPacket* pkt_ = nullptr;
  AVFrame* frame_ = nullptr;
  std::atomic<bool> draining_{false};
};

}  // namespace

struct Reader::Impl {
  std::shared_ptr<ins_camera::Camera> cam;
  std::shared_ptr<ins_camera::StreamDelegate> delegate;
  Delegate* delegateRaw = nullptr;
  std::string serial;
  std::string firmware;
  bool h265 = false;
};

Reader::Reader() : impl_(new Impl) {}

Reader::~Reader() {
  stop();
}

bool Reader::isH265() const {
  return impl_->h265;
}
std::string Reader::serial() const {
  return impl_->serial;
}
std::string Reader::firmware() const {
  return impl_->firmware;
}

bool Reader::start(const ReaderOptions& opts, FrameCallback frameCb, PacketCallback packetCb) {
  if (running_) return false;
  // A dying stdout/stderr consumer must not SIGPIPE-kill us mid-session
  // (default disposition terminates without closing the camera session).
  signal(SIGPIPE, SIG_IGN);
  ins_camera::SetLogLevel(ins_camera::LogLevel::ERR);

  ins_camera::DeviceDiscovery discovery;
  auto list = discovery.GetAvailableDevices();
  if (list.empty()) {
    fprintf(stderr,
            "[insta360] no camera found (USB mode? permissions? "
            "dangling session from a killed client?)\n");
    return false;
  }
  impl_->serial = list[0].serial_number;
  impl_->firmware = list[0].fw_version;

  impl_->cam = std::make_shared<ins_camera::Camera>(list[0].info);
  impl_->cam->SetTimeout(5000);
  const bool opened = impl_->cam->Open();
  discovery.FreeDeviceDescriptors(list);
  if (!opened) {
    fprintf(stderr, "[insta360] failed to open camera\n");
    impl_->cam.reset();
    return false;
  }

  const bool h265 = impl_->cam->GetVideoEncodeType() == ins_camera::VideoEncodeType::H265;
  impl_->h265 = h265;

  if (!impl_->cam->SetActiveSensor(static_cast<ins_camera::SensorDevice>(opts.sensor))) {
    fprintf(stderr, "[insta360] WARNING: SetActiveSensor(%d) failed\n", opts.sensor);
  }

  auto delegate = std::make_shared<Delegate>(std::move(frameCb), std::move(packetCb), h265);
  impl_->delegateRaw = delegate.get();
  impl_->delegate = delegate;
  impl_->cam->SetStreamDelegate(impl_->delegate);

  ins_camera::LiveStreamParam param;
  param.video_resolution = static_cast<ins_camera::VideoResolution>(opts.resolution);
  param.video_bitrate = opts.bitrate;
  param.enable_audio = false;
  param.enable_gyro = false;
  param.using_lrv = false;

  // In-process retries: exiting and reconnecting after a failed start wedges
  // the camera's control service.
  auto keepGoing = [&]() { return !opts.keepGoing || opts.keepGoing(); };
  bool started = false;
  for (int attempt = 1; attempt <= opts.startRetries && keepGoing(); attempt++) {
    if (impl_->cam->StartLiveStreaming(param)) {
      started = true;
      break;
    }
    fprintf(stderr, "[insta360] StartLiveStreaming failed (attempt %d/%d)\n", attempt, opts.startRetries);
    if (attempt < opts.startRetries) {
      // abortable wait: a launch SIGINT must not sit through ~30s of retries
      // (launch escalates to SIGKILL -> dirty death -> wedged camera)
      for (int i = 0; i < 32 && keepGoing(); i++) std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  if (!started) {
    impl_->cam->Close();
    impl_->cam.reset();
    return false;
  }
  running_ = true;
  return true;
}

void Reader::stop() {
  if (!impl_->cam) return;
  // Stop feeding the callback BEFORE StopLiveStreaming: a blocked callback
  // inside the SDK would stall teardown and leave a dangling camera session.
  if (impl_->delegateRaw) impl_->delegateRaw->drain();
  if (running_) {
    impl_->cam->StopLiveStreaming();
    running_ = false;
  }
  impl_->cam->Close();
  impl_->cam.reset();
  impl_->delegate.reset();
  impl_->delegateRaw = nullptr;
}

}  // namespace insta360
