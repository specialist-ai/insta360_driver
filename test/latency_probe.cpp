// Latency probe: subscribes to an Image topic and reports stamp->receive
// age statistics (median/mean/min/max) every `window` frames.
//
// Runs standalone (separate process over DDS) or composed into the same
// container as the camera (intra-process). header.stamp on the camera topic
// is the SDK packet arrival time, so "age" = decode + publish + transport +
// executor queue.

#include <algorithm>
#include <numeric>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace insta360_camera {

class LatencyProbe : public rclcpp::Node {
 public:
  explicit LatencyProbe(const rclcpp::NodeOptions& options) : Node("latency_probe", options) {
    const auto topic = declare_parameter<std::string>("topic", "/insta360_camera/image_raw");
    window_ = declare_parameter<int>("window", 300);
    sub_ = create_subscription<sensor_msgs::msg::Image>(topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                                                        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                                                          const double ageMs = (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3;
                                                          ages_.push_back(ageMs);
                                                          if (firstNs_ == 0) firstNs_ = now().nanoseconds();
                                                          if (static_cast<int>(ages_.size()) >= window_) report();
                                                        });
    RCLCPP_INFO(get_logger(), "probing %s (window %d)", topic.c_str(), window_);
  }

 private:
  void report() {
    std::vector<double> v = ages_;
    ages_.clear();
    std::sort(v.begin(), v.end());
    const double sum = std::accumulate(v.begin(), v.end(), 0.0);
    const double elapsedS = (now().nanoseconds() - firstNs_) * 1e-9;
    firstNs_ = 0;
    RCLCPP_INFO(get_logger(),
                "STATS n=%zu rate=%.1fHz ageMs median=%.2f mean=%.2f "
                "min=%.2f max=%.2f p95=%.2f",
                v.size(), v.size() / elapsedS, v[v.size() / 2], sum / v.size(), v.front(), v.back(),
                v[static_cast<size_t>(v.size() * 0.95)]);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::vector<double> ages_;
  int window_;
  int64_t firstNs_ = 0;
};

}  // namespace insta360_camera

RCLCPP_COMPONENTS_REGISTER_NODE(insta360_camera::LatencyProbe)
