// ROS 2 component publishing the Insta360 X3 rear-lens fisheye preview.
//
// Topics (under the node name, so the node IS the camera identity):
//   ~/image_raw        sensor_msgs/Image  bgr8 or mono8 (the `pixel_format`
//                                         param; mono8 is the free Y plane)
//   ~/image_raw/ffmpeg FFMPEGPacket       the camera's own H.264 bitstream,
//                                         pass-through for cheap rosbag logging
//
// header.stamp is the SDK packet-arrival time mapped to ROS time; the in-camera
// pipeline adds ~120 ms before that (not observable from the host).
//
// Runs standalone (`ros2 run insta360_camera camera_node`) or composed into a
// container with your consumer for intra-process zero-copy.

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include "insta360_camera/reader.h"

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using sensor_msgs::msg::Image;

namespace insta360_camera {

namespace {
bool subscribed(const rclcpp::PublisherBase::SharedPtr& pub) {
  return pub && pub->get_subscription_count() > 0;
}
}  // namespace

class CameraNode : public rclcpp::Node {
 public:
  explicit CameraNode(const rclcpp::NodeOptions& options) : Node("insta360_camera", options) {
    const std::string mode = declare_parameter<std::string>("mode", "720");
    frameId_ = declare_parameter<std::string>("frame_id", get_name());
    pixelFormat_ = declare_parameter<std::string>("pixel_format", "bgr8");
    const bool emitPackets = declare_parameter<bool>("publish_h264", true);
    const std::string sensor = declare_parameter<std::string>("sensor", "rear");
    const int bitrate = declare_parameter<int>("bitrate", 2 * 1024 * 1024);

    insta360::ReaderOptions opts;
    // Only probed-good modes: arbitrary resolution ids can put the camera in
    // junk fallback modes or wedge it.
    if (mode == "720") {
      opts.resolution = 9;
      widthHint_ = 720;
    } else if (mode == "1280") {
      opts.resolution = 1;
      widthHint_ = 1280;
    } else if (mode == "1920") {
      opts.resolution = 0;
      widthHint_ = 1920;
    } else {
      RCLCPP_FATAL(get_logger(), "invalid mode '%s' (use 720 | 1280 | 1920)", mode.c_str());
      throw std::invalid_argument("invalid mode");
    }
    if (sensor != "front" && sensor != "all" && sensor != "rear") {
      RCLCPP_FATAL(get_logger(), "invalid sensor '%s' (use rear | front | all)", sensor.c_str());
      throw std::invalid_argument("invalid sensor");
    }
    if (pixelFormat_ != "bgr8" && pixelFormat_ != "mono8") {
      RCLCPP_FATAL(get_logger(), "invalid pixel_format '%s' (use bgr8 | mono8)", pixelFormat_.c_str());
      throw std::invalid_argument("invalid pixel_format");
    }
    opts.sensor = (sensor == "front") ? 1 : (sensor == "all") ? 3 : 2;
    opts.bitrate = bitrate;
    opts.keepGoing = []() { return rclcpp::ok(); };

    // RELIABLE for the raw image: at 1.5 MB/msg the DDS UDP transport fragments
    // each message ~24x and best-effort drops the whole message on any lost
    // fragment, so subscribers would see a few Hz instead of 30. The H.264
    // topic is reliable too — default image_transport subscribers request
    // reliable, and a dropped P-frame corrupts decode until the next keyframe.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pubImage_ = create_publisher<Image>("~/image_raw", qos);
    if (emitPackets) pubPacket_ = create_publisher<FFMPEGPacket>("~/image_raw/ffmpeg", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    const bool ok = reader_.start(
        opts, [this](const insta360::Frame& f) { publishFrame(f); },
        emitPackets ? [this](const insta360::Packet& p) { publishPacket(p); } : insta360::PacketCallback());
    if (!ok) {
      RCLCPP_FATAL(get_logger(),
                   "camera start failed (USB mode? another client? dangling "
                   "session -> power cycle the camera)");
      throw std::runtime_error("insta360 start failed");
    }
    // Container teardown does not reliably run component destructors, so close
    // the session at rclcpp shutdown time (a dangling session wedges the
    // camera).
    shutdownCb_ = get_node_base_interface()->get_context()->add_on_shutdown_callback([this]() { reader_.stop(); });

    if (reader_.isH265()) encoding_ = "hevc";
    RCLCPP_INFO(get_logger(), "streaming: serial=%s fw=%s mode=%s codec=%s", reader_.serial().c_str(), reader_.firmware().c_str(),
                mode.c_str(), encoding_.c_str());
  }

  ~CameraNode() override {
    get_node_base_interface()->get_context()->remove_on_shutdown_callback(shutdownCb_);
    reader_.stop();
    if (sws_) sws_freeContext(sws_);
  }

 private:
  rclcpp::Time monoToRos(double mono) {
    // Map the packet's CLOCK_MONOTONIC arrival onto ROS time.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const double monoNow = ts.tv_sec + ts.tv_nsec / 1e9;
    return now() - rclcpp::Duration::from_seconds(monoNow - mono);
  }

  std::unique_ptr<Image> makeImage(const insta360::Frame& f, const rclcpp::Time& stamp, const char* encoding, size_t step) {
    auto msg = std::make_unique<Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = frameId_;
    msg->height = f.height;
    msg->width = f.width;
    msg->encoding = encoding;
    msg->step = step;
    msg->data.resize(step * f.height);
    return msg;
  }

  void publishMono(const insta360::Frame& f, const rclcpp::Time& stamp) {
    auto msg = makeImage(f, stamp, "mono8", f.width);
    for (int r = 0; r < f.height; r++)
      memcpy(msg->data.data() + static_cast<size_t>(r) * f.width, f.y + static_cast<size_t>(r) * f.yStride, f.width);
    pubImage_->publish(std::move(msg));
  }

  void publishBgr(const insta360::Frame& f, const rclcpp::Time& stamp) {
    auto msg = makeImage(f, stamp, "bgr8", static_cast<size_t>(f.width) * 3);
    // SDK delivers full-range YUV420P; set ITU-601 coefficients with full range
    // on src and dst so the BGR colors decode correctly.
    const bool fresh = !sws_;
    sws_ = sws_getCachedContext(sws_, f.width, f.height, AV_PIX_FMT_YUV420P, f.width, f.height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr,
                                nullptr, nullptr);
    if (fresh && sws_)
      sws_setColorspaceDetails(sws_, sws_getCoefficients(SWS_CS_ITU601), 1, sws_getCoefficients(SWS_CS_ITU601), 1, 0, 1 << 16, 1 << 16);
    const uint8_t* src[4] = {f.y, f.u, f.v, nullptr};
    const int srcStride[4] = {f.yStride, f.uvStride, f.uvStride, 0};
    uint8_t* dst[4] = {msg->data.data(), nullptr, nullptr, nullptr};
    const int dstStride[4] = {static_cast<int>(msg->step), 0, 0, 0};
    sws_scale(sws_, src, srcStride, 0, f.height, dst, dstStride);
    pubImage_->publish(std::move(msg));
  }

  void publishFrame(const insta360::Frame& f) {
    const auto stamp = monoToRos(f.arrivalMono);
    widthHint_ = f.width;
    if (++frames_ % 300 == 1)
      RCLCPP_INFO(get_logger(), "frame %ld: %dx%d, decode %.1fms", frames_, f.width, f.height, (f.decodedMono - f.arrivalMono) * 1e3);
    if (!subscribed(pubImage_)) return;
    if (pixelFormat_ == "mono8")
      publishMono(f, stamp);
    else
      publishBgr(f, stamp);
  }

  void publishPacket(const insta360::Packet& p) {
    if (!subscribed(pubPacket_)) {
      pts_++;
      return;
    }
    auto msg = std::make_unique<FFMPEGPacket>();
    msg->header.stamp = monoToRos(p.arrivalMono);
    msg->header.frame_id = frameId_;
    msg->width = widthHint_;
    msg->height = widthHint_;  // fisheye modes are square
    msg->encoding = encoding_;
    msg->pts = pts_++;
    msg->flags = p.keyframe ? 1 : 0;
    msg->data.assign(p.data, p.data + p.size);
    pubPacket_->publish(std::move(msg));
  }

  // config
  std::string frameId_, encoding_ = "h264", pixelFormat_ = "bgr8";
  int widthHint_ = 720;
  // pipeline
  insta360::Reader reader_;
  rclcpp::Publisher<Image>::SharedPtr pubImage_;
  rclcpp::Publisher<FFMPEGPacket>::SharedPtr pubPacket_;
  SwsContext* sws_ = nullptr;
  // lifetime / counters
  rclcpp::OnShutdownCallbackHandle shutdownCb_;
  int64_t frames_ = 0;
  // pts: 0-based counter like ffmpeg_image_transport's encoder; its decoder
  // rejects a wall-clock / AV_NOPTS_VALUE seed.
  int64_t pts_ = 0;
};

}  // namespace insta360_camera

RCLCPP_COMPONENTS_REGISTER_NODE(insta360_camera::CameraNode)
