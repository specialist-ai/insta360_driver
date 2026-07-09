// ROS 2 component publishing Insta360 X3 rear-lens fisheye preview(s).
//
// One node can drive several cameras: the SDK binds a fixed port per process,
// so multiple cameras MUST share one process. Each entry in the `cameras` param
// ("name" or "name:serial") publishes under its own name:
//   <name>/image_raw        sensor_msgs/Image  bgr8 or mono8 (the pixel_format
//                                              param; mono8 is the free Y plane)
//   <name>/image_raw/ffmpeg FFMPEGPacket       the camera's own H.264 bitstream,
//                                              pass-through for cheap rosbag logs
// header.frame_id is <name>. The default is a single camera named after the
// node, so `ros2 run insta360_camera camera_node` -> /insta360_camera/image_raw.
//
// header.stamp is the SDK packet-arrival time mapped to ROS time; the in-camera
// pipeline adds ~120 ms before that (not observable from the host).

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
    pixelFormat_ = declare_parameter<std::string>("pixel_format", "bgr8");
    const bool emitPackets = declare_parameter<bool>("publish_h264", true);
    const std::string sensor = declare_parameter<std::string>("sensor", "rear");
    const int bitrate = declare_parameter<int>("bitrate", 2 * 1024 * 1024);
    const auto specs = declare_parameter<std::vector<std::string>>("cameras", {std::string(get_name())});

    // Settings shared by every camera, validated once. Only probed-good modes:
    // arbitrary resolution ids can put the camera in junk modes or wedge it.
    insta360::ReaderOptions shared;
    int width = 0;
    if (mode == "720") {
      shared.resolution = 9;
      width = 720;
    } else if (mode == "1280") {
      shared.resolution = 1;
      width = 1280;
    } else if (mode == "1920") {
      shared.resolution = 0;
      width = 1920;
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
    shared.sensor = (sensor == "front") ? 1 : (sensor == "all") ? 3 : 2;
    shared.bitrate = bitrate;
    shared.keepGoing = []() { return rclcpp::ok(); };

    // RELIABLE for the raw image: at 1.5 MB/msg the DDS UDP transport fragments
    // each message ~24x and best-effort drops the whole message on any lost
    // fragment, so subscribers would see a few Hz instead of 30. The H.264 topic
    // is reliable too — default image_transport subscribers request reliable, and
    // a dropped P-frame corrupts decode until the next keyframe.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    for (const std::string& spec : specs) {
      auto cam = std::make_unique<Camera>();
      const auto colon = spec.find(':');
      cam->name = spec.substr(0, colon);
      cam->frameId = cam->name;
      cam->widthHint = width;
      cam->pubImage = create_publisher<Image>(cam->name + "/image_raw", qos);
      if (emitPackets)
        cam->pubPacket = create_publisher<FFMPEGPacket>(cam->name + "/image_raw/ffmpeg", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

      insta360::ReaderOptions opts = shared;
      opts.serial = (colon == std::string::npos) ? "" : spec.substr(colon + 1);
      Camera* c = cam.get();
      const bool ok = c->reader.start(
          opts, [this, c](const insta360::Frame& f) { publishFrame(*c, f); },
          emitPackets ? [this, c](const insta360::Packet& p) { publishPacket(*c, p); } : insta360::PacketCallback());
      if (!ok) {
        RCLCPP_FATAL(get_logger(), "camera '%s' start failed (bad serial? USB mode? another client? -> power cycle)", c->name.c_str());
        throw std::runtime_error("insta360 start failed");
      }
      if (c->reader.isH265()) c->encoding = "hevc";
      RCLCPP_INFO(get_logger(), "camera '%s' streaming: serial=%s fw=%s mode=%s codec=%s", c->name.c_str(), c->reader.serial().c_str(),
                  c->reader.firmware().c_str(), mode.c_str(), c->encoding.c_str());
      cameras_.push_back(std::move(cam));
    }

    // Container teardown does not reliably run component destructors, so close
    // the sessions at rclcpp shutdown time (a dangling session wedges the camera).
    shutdownCb_ = get_node_base_interface()->get_context()->add_on_shutdown_callback([this]() {
      for (auto& c : cameras_) c->reader.stop();
    });
  }

  ~CameraNode() override {
    get_node_base_interface()->get_context()->remove_on_shutdown_callback(shutdownCb_);
    for (auto& c : cameras_) {
      c->reader.stop();
      if (c->sws) sws_freeContext(c->sws);
    }
  }

 private:
  // Per-camera state. Each camera runs on its own SDK stream thread, so nothing
  // here is shared between cameras (no locking needed).
  struct Camera {
    std::string name, frameId, encoding = "h264";
    insta360::Reader reader;
    rclcpp::Publisher<Image>::SharedPtr pubImage;
    rclcpp::Publisher<FFMPEGPacket>::SharedPtr pubPacket;
    SwsContext* sws = nullptr;
    int widthHint = 720;
    int64_t frames = 0;
    // pts: 0-based counter like ffmpeg_image_transport's encoder; its decoder
    // rejects a wall-clock / AV_NOPTS_VALUE seed.
    int64_t pts = 0;
  };

  rclcpp::Time monoToRos(double mono) {
    // Map the packet's CLOCK_MONOTONIC arrival onto ROS time.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const double monoNow = ts.tv_sec + ts.tv_nsec / 1e9;
    return now() - rclcpp::Duration::from_seconds(monoNow - mono);
  }

  std::unique_ptr<Image> makeImage(const Camera& cam, const insta360::Frame& f, const rclcpp::Time& stamp, const char* encoding,
                                   size_t step) {
    auto msg = std::make_unique<Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = cam.frameId;
    msg->height = f.height;
    msg->width = f.width;
    msg->encoding = encoding;
    msg->step = step;
    msg->data.resize(step * f.height);
    return msg;
  }

  void publishMono(Camera& cam, const insta360::Frame& f, const rclcpp::Time& stamp) {
    auto msg = makeImage(cam, f, stamp, "mono8", f.width);
    for (int r = 0; r < f.height; r++)
      memcpy(msg->data.data() + static_cast<size_t>(r) * f.width, f.y + static_cast<size_t>(r) * f.yStride, f.width);
    cam.pubImage->publish(std::move(msg));
  }

  void publishBgr(Camera& cam, const insta360::Frame& f, const rclcpp::Time& stamp) {
    auto msg = makeImage(cam, f, stamp, "bgr8", static_cast<size_t>(f.width) * 3);
    // SDK delivers full-range YUV420P; set ITU-601 coefficients with full range
    // on src and dst so the BGR colors decode correctly.
    const bool fresh = !cam.sws;
    cam.sws = sws_getCachedContext(cam.sws, f.width, f.height, AV_PIX_FMT_YUV420P, f.width, f.height, AV_PIX_FMT_BGR24, SWS_BILINEAR,
                                   nullptr, nullptr, nullptr);
    if (fresh && cam.sws)
      sws_setColorspaceDetails(cam.sws, sws_getCoefficients(SWS_CS_ITU601), 1, sws_getCoefficients(SWS_CS_ITU601), 1, 0, 1 << 16, 1 << 16);
    const uint8_t* src[4] = {f.y, f.u, f.v, nullptr};
    const int srcStride[4] = {f.yStride, f.uvStride, f.uvStride, 0};
    uint8_t* dst[4] = {msg->data.data(), nullptr, nullptr, nullptr};
    const int dstStride[4] = {static_cast<int>(msg->step), 0, 0, 0};
    sws_scale(cam.sws, src, srcStride, 0, f.height, dst, dstStride);
    cam.pubImage->publish(std::move(msg));
  }

  void publishFrame(Camera& cam, const insta360::Frame& f) {
    const auto stamp = monoToRos(f.arrivalMono);
    cam.widthHint = f.width;
    if (++cam.frames % 300 == 1)
      RCLCPP_INFO(get_logger(), "camera '%s' frame %ld: %dx%d, decode %.1fms", cam.name.c_str(), cam.frames, f.width, f.height,
                  (f.decodedMono - f.arrivalMono) * 1e3);
    if (!subscribed(cam.pubImage)) return;
    if (pixelFormat_ == "mono8")
      publishMono(cam, f, stamp);
    else
      publishBgr(cam, f, stamp);
  }

  void publishPacket(Camera& cam, const insta360::Packet& p) {
    if (!subscribed(cam.pubPacket)) {
      cam.pts++;
      return;
    }
    auto msg = std::make_unique<FFMPEGPacket>();
    msg->header.stamp = monoToRos(p.arrivalMono);
    msg->header.frame_id = cam.frameId;
    msg->width = cam.widthHint;
    msg->height = cam.widthHint;  // fisheye modes are square
    msg->encoding = cam.encoding;
    msg->pts = cam.pts++;
    msg->flags = p.keyframe ? 1 : 0;
    msg->data.assign(p.data, p.data + p.size);
    cam.pubPacket->publish(std::move(msg));
  }

  std::string pixelFormat_ = "bgr8";
  std::vector<std::unique_ptr<Camera>> cameras_;
  rclcpp::OnShutdownCallbackHandle shutdownCb_;
};

}  // namespace insta360_camera

RCLCPP_COMPONENTS_REGISTER_NODE(insta360_camera::CameraNode)
