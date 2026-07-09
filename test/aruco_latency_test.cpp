// Full glass-to-detection latency test in C++: displays ArUco markers
// fullscreen, receives images from the camera topic, detects the markers,
// and reports the display->detect latency. DICT_4X4_250 gives a long marker
// cycle, so a stale detection can't alias onto the freshly shown id.
//
// Component + executable:
//   separate process:  ros2 run insta360_camera aruco_latency_test
//   zero-copy:         load insta360_camera::ArucoLatencyTest into the same
//                      container as the camera with intra-process comms
//
// Requires a display (fullscreen marker window) and the camera pointing at it.
// Exits (rclcpp::shutdown) when done.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <map>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace insta360_camera {

namespace {
double steadyS() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class ArucoLatencyTest : public rclcpp::Node {
 public:
  explicit ArucoLatencyTest(const rclcpp::NodeOptions& options) : Node("aruco_latency_test", options) {
    samplesTarget_ = declare_parameter<int>("samples", 40);
    warmup_ = declare_parameter<int>("warmup", 10);
    const auto topic = declare_parameter<std::string>("topic", "/insta360_camera/image_raw");
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(), [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          const double ageMs = (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3;
          // Accept either driver pixel_format: mono8 is already grayscale
          // (clone off the message buffer), bgr8 needs a conversion.
          cv::Mat gray;
          if (msg->encoding == "mono8")
            gray = cv::Mat(msg->height, msg->width, CV_8UC1, (void*)msg->data.data(), msg->step).clone();
          else
            cv::cvtColor(cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data(), msg->step), gray, cv::COLOR_BGR2GRAY);
          std::lock_guard<std::mutex> lk(m_);
          gray_ = gray;  // fresh allocation, safe to hold under lock
          stampAgeMs_ = ageMs;
          count_++;
        });
    gui_ = std::thread([this] { run(); });
  }

  ~ArucoLatencyTest() override {
    stop_ = true;
    if (gui_.joinable()) gui_.join();
  }

 private:
  void run() {
    constexpr int kNumMarkers = 250;
    const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    const auto params = cv::aruco::DetectorParameters::create();

    // Pre-render one display frame per marker: a 300px marker in a 50px white
    // border, centered on a 1280x720 mid-gray canvas.
    constexpr int kMarker = 300, kBorder = 50, kW = 1280, kH = 720;
    std::vector<cv::Mat> frames(kNumMarkers);
    for (int i = 0; i < kNumMarkers; i++) {
      cv::Mat marker;
      cv::aruco::drawMarker(dict, i, kMarker, marker);
      cv::Mat canvas(kH, kW, CV_8UC3, cv::Scalar(128, 128, 128));
      const int total = kMarker + 2 * kBorder;
      cv::Mat roi = canvas(cv::Rect((kW - total) / 2, (kH - total) / 2, total, total));
      roi.setTo(cv::Scalar(255, 255, 255));
      cv::Mat markerBgr;
      cv::cvtColor(marker, markerBgr, cv::COLOR_GRAY2BGR);
      markerBgr.copyTo(roi(cv::Rect(kBorder, kBorder, kMarker, kMarker)));
      frames[i] = canvas;
    }

    cv::namedWindow("flash", cv::WINDOW_NORMAL);
    cv::setWindowProperty("flash", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("flash", frames[0]);
    cv::waitKey(50);
    // WMs bury windows from background processes; raise explicitly.
    if (std::system("wmctrl -r flash -b add,above") != 0) RCLCPP_WARN(get_logger(), "wmctrl raise failed — keep window on top");

    std::map<int, double> displayTime;
    std::vector<double> latencies, ages;
    int markerId = 0, lastSeen = -1;
    uint64_t lastCount = 0;
    double lastShown = 0.0;
    const double t0 = steadyS();

    while (!stop_ && rclcpp::ok() && static_cast<int>(latencies.size()) < samplesTarget_ + warmup_) {
      if (steadyS() - t0 > 180.0) {
        RCLCPP_ERROR(get_logger(), "timeout with %zu samples", latencies.size());
        break;
      }
      if (steadyS() - lastShown >= 0.033) {
        markerId = (markerId + 1) % kNumMarkers;
        cv::imshow("flash", frames[markerId]);
        cv::waitKey(1);
        const double shown = steadyS();
        displayTime[markerId] = shown;
        lastShown = shown;
      }
      cv::Mat gray;
      double age = 0.0;
      {
        std::lock_guard<std::mutex> lk(m_);
        if (count_ == lastCount)
          gray = cv::Mat();
        else {
          gray = gray_;
          age = stampAgeMs_;
          lastCount = count_;
        }
      }
      if (gray.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dict, corners, ids, params);
      const double tDet = steadyS();
      if (!ids.empty() && ids[0] != lastSeen) {
        auto it = displayTime.find(ids[0]);
        if (it != displayTime.end()) {
          const double dtS = tDet - it->second;
          if (dtS > 0 && dtS < 0.8) {
            latencies.push_back(dtS * 1e3);
            ages.push_back(age);
            RCLCPP_INFO(get_logger(), "sample %2zu: glass->detect=%.1fms stamp->recv=%.2fms", latencies.size(), latencies.back(), age);
          }
        }
        lastSeen = ids[0];
      }
    }
    cv::destroyAllWindows();

    if (static_cast<int>(latencies.size()) > warmup_ + 3) {
      std::vector<double> warmLatencies(latencies.begin() + warmup_, latencies.end());
      std::vector<double> warmAges(ages.begin() + warmup_, ages.end());
      std::sort(warmLatencies.begin(), warmLatencies.end());
      std::sort(warmAges.begin(), warmAges.end());
      const double mean = std::accumulate(warmLatencies.begin(), warmLatencies.end(), 0.0) / warmLatencies.size();
      RCLCPP_INFO(get_logger(),
                  "CPP RESULTS n=%zu glass->detect median=%.1fms mean=%.1fms "
                  "min=%.1fms | stamp->recv median=%.2fms",
                  warmLatencies.size(), warmLatencies[warmLatencies.size() / 2], mean, warmLatencies.front(),
                  warmAges[warmAges.size() / 2]);
    }
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::thread gui_;
  std::atomic<bool> stop_{false};
  std::mutex m_;
  cv::Mat gray_;
  double stampAgeMs_ = 0.0;
  uint64_t count_ = 0;
  int samplesTarget_ = 40, warmup_ = 10;
};

}  // namespace insta360_camera

RCLCPP_COMPONENTS_REGISTER_NODE(insta360_camera::ArucoLatencyTest)
