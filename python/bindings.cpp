// Python bindings for the Insta360 reader.
//
// The C++ API is push: a callback on the SDK's stream thread, with pointers
// valid only for the duration of the call. This binding is pull, because a
// Python callback on that thread would have to take the GIL inside the decode
// path, and the reader exists to keep that path short.
//
// So the callback converts each frame into a slot without ever touching
// Python, and read() copies out of the slot. One frame of memory, no queue: a
// consumer that falls behind wants the newest frame, not a backlog.

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>

extern "C" {
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "insta360_camera/reader.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace {

/// One converted frame, handed from the SDK's stream thread to a reader.
struct Image {
  std::vector<std::uint8_t> pixels;
  int width{0};
  int height{0};
  int channels{0};
  double arrival_mono{0.0};
  double decoded_mono{0.0};
};

/// The newest frame, and nothing older.
///
/// The conversion runs outside the lock, so the SDK thread holds it only for
/// the swap. `index` counts frames since start() and is what tells a reader
/// whether what it sees is new.
class Slot {
 public:
  ~Slot() {
    if (sws_) {
      sws_freeContext(sws_);
    }
  }

  void configure(AVPixelFormat format, int channels) {
    std::lock_guard<std::mutex> lock(mutex_);
    format_ = format;
    channels_ = channels;
    index_ = 0;
    ready_ = Image{};
  }

  /// Called on the SDK's stream thread, without the GIL and without Python.
  void put(const insta360::Frame& frame) {
    const std::size_t stride = static_cast<std::size_t>(frame.width) * channels_;
    scratch_.pixels.resize(stride * static_cast<std::size_t>(frame.height));
    scratch_.width = frame.width;
    scratch_.height = frame.height;
    scratch_.channels = channels_;
    scratch_.arrival_mono = frame.arrivalMono;
    scratch_.decoded_mono = frame.decodedMono;

    if (channels_ == 1) {
      // The Y plane is the image already. Copy row by row: the decoder's
      // stride is wider than the picture.
      for (int row = 0; row < frame.height; ++row) {
        std::memcpy(scratch_.pixels.data() + stride * static_cast<std::size_t>(row),
                    frame.y + static_cast<std::size_t>(frame.yStride) * static_cast<std::size_t>(row), stride);
      }
    } else {
      sws_ = sws_getCachedContext(sws_, frame.width, frame.height, AV_PIX_FMT_YUV420P, frame.width, frame.height, format_, SWS_BILINEAR,
                                  nullptr, nullptr, nullptr);
      if (!sws_) {
        return;
      }
      const std::uint8_t* source[3] = {frame.y, frame.u, frame.v};
      const int source_stride[3] = {frame.yStride, frame.uvStride, frame.uvStride};
      std::uint8_t* destination[1] = {scratch_.pixels.data()};
      const int destination_stride[1] = {static_cast<int>(stride)};
      sws_scale(sws_, source, source_stride, 0, frame.height, destination, destination_stride);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::swap(ready_, scratch_);
      ++index_;
    }
    arrived_.notify_all();
  }

  /// Wait for a frame newer than `seen`. False on timeout.
  bool wait_for_newer(std::uint64_t seen, int timeout_ms, Image& out, std::uint64_t& index) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool got = arrived_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return index_ > seen; });
    if (!got) {
      return false;
    }
    out = ready_;
    index = index_;
    return true;
  }

  /// The current frame, or nothing if none has arrived yet.
  bool peek(Image& out, std::uint64_t& index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index_ == 0) {
      return false;
    }
    out = ready_;
    index = index_;
    return true;
  }

 private:
  std::mutex mutex_;
  std::condition_variable arrived_;
  Image ready_;
  Image scratch_;  // only the SDK thread touches this
  std::uint64_t index_{0};
  AVPixelFormat format_{AV_PIX_FMT_BGR24};
  int channels_{3};
  SwsContext* sws_{nullptr};
};

/// What read() hands back: the image and when it was taken.
struct PyFrame {
  nb::object image;
  double arrival_mono;
  double decoded_mono;
  std::uint64_t index;
};

nb::object to_numpy(const Image& image) {
  const std::size_t count = image.pixels.size();
  auto* data = new std::uint8_t[count];
  std::memcpy(data, image.pixels.data(), count);
  nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<std::uint8_t*>(p); });

  if (image.channels == 1) {
    const std::size_t shape[2] = {static_cast<std::size_t>(image.height), static_cast<std::size_t>(image.width)};
    return nb::cast(nb::ndarray<nb::numpy, std::uint8_t>(data, 2, shape, owner));
  }
  const std::size_t shape[3] = {static_cast<std::size_t>(image.height), static_cast<std::size_t>(image.width),
                                static_cast<std::size_t>(image.channels)};
  return nb::cast(nb::ndarray<nb::numpy, std::uint8_t>(data, 3, shape, owner));
}

/// Only the modes probed good on an X3. An arbitrary resolution id can put the
/// camera in a junk mode or wedge it until a power cycle.
int resolution_id(int mode) {
  switch (mode) {
    case 720:
      return 9;
    case 1280:
      return 1;
    case 1920:
      return 0;
    default:
      throw nb::value_error("mode must be 720, 1280 or 1920");
  }
}

int sensor_id(const std::string& sensor) {
  if (sensor == "rear") {
    return 2;
  }
  if (sensor == "front") {
    return 1;
  }
  if (sensor == "all") {
    return 3;
  }
  throw nb::value_error("sensor must be 'rear', 'front' or 'all'");
}

void pixel_format_id(const std::string& name, AVPixelFormat& format, int& channels) {
  if (name == "bgr8") {
    format = AV_PIX_FMT_BGR24;
    channels = 3;
    return;
  }
  if (name == "rgb8") {
    format = AV_PIX_FMT_RGB24;
    channels = 3;
    return;
  }
  // The free one: the Y plane is already the image.
  if (name == "mono8") {
    format = AV_PIX_FMT_GRAY8;
    channels = 1;
    return;
  }
  throw nb::value_error("pixel_format must be 'bgr8', 'rgb8' or 'mono8'");
}

/// The reader, its slot, and the frame index this consumer has already seen.
class PyReader {
 public:
  ~PyReader() {
    // The SDK session must close cleanly or the camera refuses the next
    // connection until it is physically power cycled. stop() joins the stream
    // thread, so it must not hold the GIL.
    if (Py_IsInitialized() == 0) {
      return;
    }
    nb::gil_scoped_release release;
    reader_.stop();
  }

  void start(int mode, const std::string& sensor, const std::string& pixel_format, int bitrate, const std::string& serial,
             int start_retries) {
    AVPixelFormat format;
    int channels;
    pixel_format_id(pixel_format, format, channels);

    insta360::ReaderOptions options;
    options.resolution = resolution_id(mode);
    options.sensor = sensor_id(sensor);
    options.bitrate = bitrate;
    options.startRetries = start_retries;
    options.serial = serial;

    slot_.configure(format, channels);
    seen_ = 0;

    bool ok;
    {
      nb::gil_scoped_release release;
      ok = reader_.start(options, [this](const insta360::Frame& frame) { slot_.put(frame); });
    }
    if (!ok) {
      throw std::runtime_error(
          "the camera did not start. Check that it is in USB mode, that no other "
          "program holds the session, and that the serial is right. A killed client "
          "leaves a session open until the camera is power cycled.");
    }
  }

  void stop() {
    nb::gil_scoped_release release;
    reader_.stop();
  }

  PyFrame read(int timeout_ms) {
    Image image;
    std::uint64_t index = 0;
    bool got;
    {
      nb::gil_scoped_release release;
      got = slot_.wait_for_newer(seen_, timeout_ms, image, index);
    }
    if (!got) {
      // nanobind's exception_type enum has no timeout entry, and TimeoutError
      // is what a caller pacing itself to the camera catches.
      PyErr_SetString(PyExc_TimeoutError,
                      "no new frame within the timeout. The camera streams at 30 fps, so a "
                      "timeout under ~35 ms will fire on a healthy stream.");
      throw nb::python_error();
    }
    seen_ = index;
    return PyFrame{to_numpy(image), image.arrival_mono, image.decoded_mono, index};
  }

  nb::object latest() {
    Image image;
    std::uint64_t index = 0;
    bool got;
    {
      nb::gil_scoped_release release;
      got = slot_.peek(image, index);
    }
    if (!got) {
      return nb::none();
    }
    seen_ = index;
    return nb::cast(PyFrame{to_numpy(image), image.arrival_mono, image.decoded_mono, index});
  }

  bool running() const { return reader_.running(); }
  bool is_h265() const { return reader_.isH265(); }
  std::string serial() const { return reader_.serial(); }
  std::string firmware() const { return reader_.firmware(); }

 private:
  insta360::Reader reader_;
  Slot slot_;
  std::uint64_t seen_{0};
};

}  // namespace

NB_MODULE(_core, m) {
  m.doc() = "Low-latency Insta360 fisheye reader";

  nb::class_<PyFrame>(m, "Frame", "One decoded frame and the clock it arrived on.")
      .def_ro("image", &PyFrame::image, "HxWx3 uint8, or HxW for mono8.")
      .def_ro("arrival_mono", &PyFrame::arrival_mono,
              "CLOCK_MONOTONIC seconds when the encoded packet reached the host. This is "
              "the timestamp to use: it excludes host-side decode.")
      .def_ro("decoded_mono", &PyFrame::decoded_mono, "CLOCK_MONOTONIC seconds when the decode finished.")
      .def_ro("index", &PyFrame::index, "Frames since start(), counting from 1.")
      .def("__repr__", [](const PyFrame& self) {
        return "Frame(index=" + std::to_string(self.index) + ", arrival_mono=" + std::to_string(self.arrival_mono) + ")";
      });

  nb::class_<PyReader>(m, "Reader",
                       "One camera. The session must be closed cleanly, so use it as a context "
                       "manager or call stop(). A killed client leaves the camera refusing "
                       "connections until it is physically power cycled.\n\n"
                       "One SDK client per process, and the SDK binds a fixed local port, so two "
                       "reader processes cannot coexist.")
      .def(nb::init<>())
      .def("start", &PyReader::start, "mode"_a = 720, "sensor"_a = "rear", "pixel_format"_a = "bgr8", "bitrate"_a = 2 * 1024 * 1024,
           "serial"_a = "", "start_retries"_a = 3,
           "Open the camera and start streaming. Blocks for a few seconds.\n\n"
           "mode is 720, 1280 or 1920, the square fisheye modes probed good on an X3; "
           "glass-to-glass is ~155, ~202 and ~262 ms. sensor is 'rear', 'front' or "
           "'all'. pixel_format is 'bgr8', 'rgb8' or 'mono8', where mono8 is the free "
           "Y plane. serial picks one camera; empty takes the first discovered.")
      .def("stop", &PyReader::stop, "Stop streaming and close the session.")
      .def("read", &PyReader::read, "timeout_ms"_a = 200,
           "The next frame this reader has not seen. Raises TimeoutError if none "
           "arrives, which paces a caller to the camera rather than to itself.")
      .def("latest", &PyReader::latest,
           "The newest frame without waiting, or None before the first one. Use it "
           "when a stale frame beats blocking.")
      .def_prop_ro("running", &PyReader::running)
      .def_prop_ro("is_h265", &PyReader::is_h265, "The codec the camera negotiated.")
      .def_prop_ro("serial", &PyReader::serial)
      .def_prop_ro("firmware", &PyReader::firmware)
      // Takes and returns the Python object: a lambda returning PyReader & would
      // get nanobind's default return value policy, which copies, and a Reader
      // owns a thread.
      .def("__enter__", [](nb::object self) { return self; })
      .def(
          "__exit__", [](PyReader& self, nb::handle, nb::handle, nb::handle) { self.stop(); }, "exc_type"_a.none(), "exc_value"_a.none(),
          "traceback"_a.none());
}
