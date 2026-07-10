// Glass-to-detection latency + stream-integrity test in C++. Displays ArUco
// markers fullscreen, receives images from one or more camera topics, detects
// the markers, and reports the display->camera->detect latency. DICT_4X4_250
// gives a long marker cycle, so a stale detection can't alias onto the freshly
// shown id.
//
// Two uses, same tool:
//   quick bench (default): one topic, collect `samples` latencies, print a
//     one-line CPP RESULTS summary, exit. Composable for zero-copy runs.
//       separate process: ros2 run insta360_camera aruco_latency_test
//       zero-copy:        load insta360_camera::ArucoLatencyTest into the
//                         camera's container with intra-process comms
//   stress (duration>0): watch N topics concurrently for `duration` seconds,
//     reporting per-window latency distribution (median/p95/p99/max) AND stream
//     integrity (dropped / stalled / out-of-order frames):
//       ros2 run insta360_camera aruco_latency_test --ros-args
//         -p 'topics:=[/left_camera/image_raw, /right_camera/image_raw]'
//         -p duration:=3600.0
//
// Latency is anchored to frame ARRIVAL (captured the instant the callback
// receives the frame), not to detection time, so a busy detector, the display
// loop, or host load cannot inflate the tail. The detect-based median is also
// reported for continuity with the consumer comparison.
//
// Requires a display (fullscreen marker window) and the camera pointing at it.
// Exits (rclcpp::shutdown) when done.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
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
constexpr int kNumMarkers = 250;
constexpr double kPeriod = 1.0 / 30.0;      // nominal frame period; drops/jitter judged against it
constexpr double kOutlierMs = 250.0;        // a latency sample above this is flagged as a tail outlier
constexpr double kStallSec = 0.10;          // display-update gap beyond this = the loop was starved (host contention)
constexpr double kStallCooldownSec = 0.30;  // drop latency samples this long after a stall (stale marker still in flight)
constexpr int kMaxLagMarkers = 10;          // a detected marker this far behind the live one = misread or compositor lag, not latency

double steadyS() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double percentile(const std::vector<double>& sorted, double p) {
  if (sorted.empty()) return 0.0;
  return sorted[std::min(sorted.size() - 1, static_cast<size_t>(sorted.size() * p))];
}

// A 300px marker in a 50px white border, centered on a 1280x720 mid-gray canvas.
constexpr int kMarkerPx = 300, kBorderPx = 50, kCanvasW = 1280, kCanvasH = 720;

cv::Rect boxRoi() {
  const int box = kMarkerPx + 2 * kBorderPx;
  return {(kCanvasW - box) / 2, (kCanvasH - box) / 2, box, box};
}
cv::Rect markerRoi() {
  const cv::Rect box = boxRoi();
  return {box.x + kBorderPx, box.y + kBorderPx, kMarkerPx, kMarkerPx};
}

// The static background: mid-gray with the white box the marker is painted into.
cv::Mat makeMarkerCanvas() {
  cv::Mat canvas(kCanvasH, kCanvasW, CV_8UC3, cv::Scalar(128, 128, 128));
  canvas(boxRoi()).setTo(cv::Scalar(255, 255, 255));
  return canvas;
}

// Paint marker `id` into the box, reusing the canvas. Rendering on demand keeps
// one ~2.7 MB canvas instead of pre-rendering all 250 (~660 MB — cold enough to
// get swapped out and stall the display loop for hundreds of ms).
void paintMarker(cv::Mat& canvas, const cv::Ptr<cv::aruco::Dictionary>& dict, int id) {
  cv::Mat marker, markerBgr;
  cv::aruco::drawMarker(dict, id, kMarkerPx, marker);
  cv::cvtColor(marker, markerBgr, cv::COLOR_GRAY2BGR);
  markerBgr.copyTo(canvas(markerRoi()));
}
}  // namespace

class ArucoLatencyTest : public rclcpp::Node {
 public:
  explicit ArucoLatencyTest(const rclcpp::NodeOptions& options) : Node("aruco_latency_test", options) {
    samplesTarget_ = declare_parameter<int>("samples", 40);
    warmup_ = declare_parameter<int>("warmup", 10);
    duration_ = declare_parameter<double>("duration", 0.0);  // 0 = bench (samples); >0 = stress (seconds)
    reportSec_ = declare_parameter<double>("report_sec", 60.0);
    const auto topics = declare_parameter<std::vector<std::string>>("topics", {std::string("/insta360_camera/image_raw")});
    for (const std::string& topic : topics) {
      auto cam = std::make_unique<Cam>();
      cam->name = topic.substr(1, topic.find("/image") - 1);
      Cam* c = cam.get();
      cam->sub = create_subscription<sensor_msgs::msg::Image>(topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                                                              [this, c](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(*c, msg); });
      cams_.push_back(std::move(cam));
    }
    gui_ = std::thread([this] { run(); });
    raise_ = std::thread([this] { raiseWindowLoop(); });
  }

  ~ArucoLatencyTest() override {
    stop_ = true;
    if (gui_.joinable()) gui_.join();
    if (raise_.joinable()) raise_.join();
  }

 private:
  struct Cam {
    std::string name;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

    // Written by the subscription callback; read under `m`.
    std::mutex m;
    cv::Mat gray;
    double arrival = 0.0;     // steadyS at callback entry = frame arrival
    double stampAgeMs = 0.0;  // last stamp->recv ms
    double prevStamp = 0.0;   // last header.stamp (s)
    uint64_t count = 0;       // frames received
    uint64_t winFrames = 0, winDrops = 0, winGaps = 0, winStalls = 0, winOOO = 0, winAgeN = 0;
    double winMaxGap = 0.0, winAgeSum = 0.0;
    uint64_t totFrames = 0, totDrops = 0, totGaps = 0, totStalls = 0, totOOO = 0;
    double maxGapEver = 0.0;

    // Owned by the detector thread; no lock.
    uint64_t lastCount = 0;
    int lastSeen = -1;
    std::vector<double> lat, latDetect, ages;  // arrival-based, detect-based, stamp->recv
    std::vector<double> winLat;
    uint64_t winOutliers = 0, totOutliers = 0;
  };

  void onImage(Cam& c, sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const double arr = steadyS();  // frame just arrived — capture before any work
    const double stampSec = rclcpp::Time(msg->header.stamp).seconds();
    const double ageMs = (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3;
    cv::Mat gray;
    if (msg->encoding == "mono8")
      gray = cv::Mat(msg->height, msg->width, CV_8UC1, (void*)msg->data.data(), msg->step).clone();
    else
      cv::cvtColor(cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data(), msg->step), gray, cv::COLOR_BGR2GRAY);

    std::lock_guard<std::mutex> lk(c.m);
    if (c.prevStamp > 0) {
      const double gap = stampSec - c.prevStamp;
      if (gap <= 0) {
        c.winOOO++;
      } else {
        if (gap > c.winMaxGap) c.winMaxGap = gap;
        if (gap > 1.5 * kPeriod) c.winGaps++;  // late but not lost (jitter)
        if (gap > 1.9 * kPeriod) c.winDrops += static_cast<uint64_t>(std::llround(gap / kPeriod)) - 1;
        if (gap > 0.5) c.winStalls++;
      }
    }
    c.prevStamp = stampSec;
    c.gray = gray;  // fresh allocation, safe to hold under lock
    c.arrival = arr;
    c.stampAgeMs = ageMs;
    c.count++;
    c.winFrames++;
    c.winAgeSum += ageMs;
    c.winAgeN++;
  }

  void run() {
    const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    const auto params = cv::aruco::DetectorParameters::create();
    cv::Mat canvas = makeMarkerCanvas();
    paintMarker(canvas, dict, 0);

    cv::namedWindow("flash", cv::WINDOW_NORMAL);
    cv::setWindowProperty("flash", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("flash", canvas);
    cv::waitKey(50);
    if (!forceOnTop()) RCLCPP_WARN(get_logger(), "wmctrl unavailable — window may not stay fullscreen/on-top");

    const bool stress = duration_ > 0.0;
    if (stress) {
      RCLCPP_INFO(get_logger(), "stress: %zu topic(s), %.0fs, per-%.0fs windows", cams_.size(), duration_, reportSec_);
      printf("# min | cam: glass->detect(arrival) med/p95/p99/max(out>250) | fps drops stalls ooo maxgap_ms age_ms\n");
      fflush(stdout);
    }

    std::map<int, double> displayTime;
    int markerId = 0;
    const double t0 = steadyS();
    double lastShown = 0.0, winStart = t0;
    while (!stop_ && rclcpp::ok() && !done(t0)) {
      const double t = steadyS();
      if (t - lastShown >= kPeriod) {
        markerId = (markerId + 1) % kNumMarkers;
        paintMarker(canvas, dict, markerId);
        cv::imshow("flash", canvas);
        cv::waitKey(1);
        const double shown = steadyS();
        if (lastShown > 0) {
          const double gap = shown - lastShown;  // how long the previous marker sat on screen
          if (gap > maxDisplayGap_) maxDisplayGap_ = gap;
          if (gap > kStallSec) {  // the display froze: its marker went stale, so poison samples briefly
            stallUntil_ = shown + kStallCooldownSec;
            displayStalls_++;
          }
        }
        lastShown = shown;
        displayTime[markerId] = shown;
      }
      bool fresh = false;
      for (auto& c : cams_) fresh |= detect(*c, dict, params, displayTime, stress, markerId);
      if (stress && t - winStart >= reportSec_) {
        reportWindow(t, t0, t - winStart);
        winStart = t;
      }
      if (!fresh) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    cv::destroyAllWindows();
    summarize(t0);
    rclcpp::shutdown();
  }

  // Detect the newest frame of one camera and record its latency. Returns true
  // if a fresh frame was processed.
  bool detect(Cam& c, const cv::Ptr<cv::aruco::Dictionary>& dict, const cv::Ptr<cv::aruco::DetectorParameters>& params,
              const std::map<int, double>& displayTime, bool stress, int liveMarkerId) {
    cv::Mat gray;
    double arr = 0.0, age = 0.0;
    {
      std::lock_guard<std::mutex> lk(c.m);
      if (c.count == c.lastCount) return false;
      gray = c.gray;
      arr = c.arrival;
      age = c.stampAgeMs;
      c.lastCount = c.count;
    }
    // Detect on a half-res copy: ~4x faster, so detection can't stretch the
    // display loop (we use only marker ids, not corner positions — lossless).
    cv::Mat small;
    cv::resize(gray, small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(small, dict, corners, ids, params);
    const double tDet = steadyS();
    if (!ids.empty() && ids[0] != c.lastSeen) {
      const auto it = displayTime.find(ids[0]);
      if (it != displayTime.end()) {
        const double dt = (arr - it->second) * 1e3;      // arrival-based (clean)
        const double dtDet = (tDet - it->second) * 1e3;  // detect-based (for comparison)
        // How many markers the monitor lags the code: pipeline delay plus any X
        // compositor presentation backlog (which inflates the tail under load).
        const int lag = (liveMarkerId - ids[0] + kNumMarkers) % kNumMarkers;
        if (lag > maxPresentLag_) maxPresentLag_ = lag;
        if (lag > kMaxLagMarkers) {
          laggedSamples_++;  // marker implausibly far behind the live display: misread or compositor lag
        } else if (tDet < stallUntil_) {
          stalledSamples_++;  // the display was frozen by host contention — not a real latency
        } else if (dt > 0 && dt < 800) {
          c.lat.push_back(dt);
          c.ages.push_back(age);
          c.winLat.push_back(dt);
          if (dt > kOutlierMs) {
            c.winOutliers++;
            c.totOutliers++;
          }
          if (dtDet > 0 && dtDet < 800) c.latDetect.push_back(dtDet);
          if (!stress)
            RCLCPP_INFO(get_logger(), "%s sample %2zu: glass->detect(arrival)=%.1fms stamp->recv=%.2fms", c.name.c_str(), c.lat.size(), dt,
                        age);
        }
      }
      c.lastSeen = ids[0];
    }
    return true;
  }

  // Bench: stop once the first camera has enough samples (or times out).
  // Stress: stop at the requested duration.
  bool done(double t0) const {
    if (duration_ > 0.0) return steadyS() - t0 >= duration_;
    if (steadyS() - t0 > 180.0) return true;  // bench safety timeout
    return static_cast<int>(cams_.front()->lat.size()) >= samplesTarget_ + warmup_;
  }

  void reportWindow(double t, double t0, double elapsed) {
    printf("%4.0f |", (t - t0) / 60.0);
    for (auto& c : cams_) {
      double fps, maxGapMs, meanAgeMs;
      uint64_t drops, gaps, stalls, ooo;
      {
        std::lock_guard<std::mutex> lk(c->m);
        fps = elapsed > 0 ? c->winFrames / elapsed : 0.0;
        maxGapMs = c->winMaxGap * 1e3;
        meanAgeMs = c->winAgeN ? c->winAgeSum / c->winAgeN : 0.0;
        drops = c->winDrops;
        gaps = c->winGaps;
        stalls = c->winStalls;
        ooo = c->winOOO;
        c->totFrames += c->winFrames;
        c->totDrops += c->winDrops;
        c->totGaps += c->winGaps;
        c->totStalls += c->winStalls;
        c->totOOO += c->winOOO;
        c->maxGapEver = std::max(c->maxGapEver, c->winMaxGap);
        c->winFrames = c->winDrops = c->winGaps = c->winStalls = c->winOOO = c->winAgeN = 0;
        c->winMaxGap = c->winAgeSum = 0.0;
      }
      std::vector<double> lat;
      lat.swap(c->winLat);
      const uint64_t outliers = c->winOutliers;
      c->winOutliers = 0;
      std::sort(lat.begin(), lat.end());
      printf(" %s: %.1f/%.1f/%.1f/%.1f(o%lu) fps=%.2f DROP=%lu jit=%lu stall=%lu ooo=%lu gap=%.0f age=%.1f |", c->name.c_str(),
             percentile(lat, 0.5), percentile(lat, 0.95), percentile(lat, 0.99), lat.empty() ? 0.0 : lat.back(), outliers, fps, drops, gaps,
             stalls, ooo, maxGapMs, meanAgeMs);
    }
    printf(" display_stalls=%lu\n", displayStalls_);
    fflush(stdout);
  }

  void summarize(double t0) {
    for (auto& c : cams_) {
      // Bench-compatible line: detect-based median/mean/min over warm samples,
      // plus the arrival-based distribution and stamp->recv.
      if (static_cast<int>(c->latDetect.size()) <= warmup_ + 3) {
        RCLCPP_ERROR(get_logger(), "%s: only %zu samples", c->name.c_str(), c->latDetect.size());
        continue;
      }
      std::vector<double> det(c->latDetect.begin() + warmup_, c->latDetect.end());
      std::vector<double> arr(c->lat.begin() + warmup_, c->lat.end());
      std::vector<double> ag(c->ages.begin() + warmup_, c->ages.end());
      std::sort(det.begin(), det.end());
      std::sort(arr.begin(), arr.end());
      std::sort(ag.begin(), ag.end());
      const double mean = std::accumulate(det.begin(), det.end(), 0.0) / det.size();
      RCLCPP_INFO(get_logger(),
                  "CPP RESULTS %s n=%zu glass->detect median=%.1fms mean=%.1fms min=%.1fms | "
                  "arrival median=%.1f p95=%.1f p99=%.1f max=%.1f | stamp->recv median=%.2fms",
                  c->name.c_str(), det.size(), percentile(det, 0.5), mean, det.front(), percentile(arr, 0.5), percentile(arr, 0.95),
                  percentile(arr, 0.99), arr.back(), percentile(ag, 0.5));
    }
    if (duration_ <= 0.0) return;

    printf("\n#### FINAL after %.1f min ####\n", (steadyS() - t0) / 60.0);
    for (auto& c : cams_) {
      std::lock_guard<std::mutex> lk(c->m);  // callbacks may still fire until shutdown
      std::vector<double> arr = c->lat;
      std::sort(arr.begin(), arr.end());
      printf(
          "# %s: frames=%lu detections=%zu | DROPS=%lu (jitter=%lu) STALLS=%lu OUT_OF_ORDER=%lu LAT_OUTLIERS(>250ms)=%lu | "
          "max_gap=%.0fms | glass->detect(arrival) median=%.1f p95=%.1f p99=%.1f max=%.1f ms\n",
          c->name.c_str(), c->totFrames + c->winFrames, c->lat.size(), c->totDrops, c->totGaps, c->totStalls, c->totOOO, c->totOutliers,
          std::max(c->maxGapEver, c->winMaxGap) * 1e3, percentile(arr, 0.5), percentile(arr, 0.95), percentile(arr, 0.99),
          arr.empty() ? 0.0 : arr.back());
    }
    printf("# display: loop_max_gap=%.0fms stalls=%lu | excluded: freeze=%lu implausible_lag=%lu (max_lag=%d markers)\n",
           maxDisplayGap_ * 1e3, displayStalls_, stalledSamples_, laggedSamples_, maxPresentLag_);
    fflush(stdout);
  }

  // Force the flash window fullscreen and on top. OpenCV's fullscreen hint is
  // unreliable when other windows compete — it can leave a small window the
  // cameras barely resolve — so enforce both states with wmctrl.
  bool forceOnTop() { return std::system("wmctrl -r flash -b add,above,fullscreen 2>/dev/null") == 0; }

  // Re-assert fullscreen/on-top from its own thread, so a busy WM (wmctrl can
  // stall a few hundred ms) never blocks the display loop and stales a marker.
  void raiseWindowLoop() {
    while (!stop_) {
      for (int i = 0; i < 30 && !stop_; i++) std::this_thread::sleep_for(std::chrono::seconds(1));
      if (!stop_) forceOnTop();
    }
  }

  std::vector<std::unique_ptr<Cam>> cams_;
  std::thread gui_, raise_;
  std::atomic<bool> stop_{false};
  int samplesTarget_ = 40, warmup_ = 10;
  double duration_ = 0.0, reportSec_ = 60.0;
  // Display-loop health, owned by the detector thread. A frozen marker (host
  // contention) makes displayTime stale, so samples in its wake are excluded.
  double stallUntil_ = 0.0, maxDisplayGap_ = 0.0;
  uint64_t displayStalls_ = 0, stalledSamples_ = 0;
  int maxPresentLag_ = 0;
  uint64_t laggedSamples_ = 0;
};

}  // namespace insta360_camera

RCLCPP_COMPONENTS_REGISTER_NODE(insta360_camera::ArucoLatencyTest)
