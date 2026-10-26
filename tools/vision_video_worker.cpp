// Streaming video workload for the offline visual/control resource benchmark.
// stdout is a JSON-lines protocol; diagnostics belong on stderr. No hardware I/O.
#include "demo_aim/armor_auto.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <time.h>

namespace {
using Json = nlohmann::json;
struct Options {
  std::string mode{"traditional"}, video;
  std::string armor_config{"config/armor_auto.yaml"};
  int width{640}, height{480}, threads{2};
  double fps{30}, seconds{10};
  bool worker{false};
};

int integer(const std::string &value, const std::string &name, int low, int high) {
  int number{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), number);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
      number < low || number > high)
    throw std::invalid_argument(name + " must be an integer in " +
                                std::to_string(low) + ".." + std::to_string(high));
  return number;
}

double number(const std::string &value, const std::string &name, double low,
              double high) {
  std::size_t used{};
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result) || result < low || result > high)
    throw std::invalid_argument(name + " is outside its finite allowed range");
  return result;
}

void help() {
  std::cout << "Offline streaming-video CPU workload; no GUI or flight connection.\n"
               "  --video FILE [--mode traditional]\n"
               "  [--width 640 --height 480 --fps 30 --threads 2]\n"
               "  [--armor-config config/armor_auto.yaml]\n"
               "  [--seconds 10] or [--benchmark-worker]\n"
               "Worker: ready JSON -> stdin {start_monotonic_s,warmup_s,measure_s}\n"
               "-> result JSON -> wait for a stdin release line / EOF.\n"
               "Streamed decode and resize are included in process resources.\n"
               "Traditional mode disables pose: source-video intrinsics are unknown.\n";
}

Options parse(int argc, char **argv) {
  Options o;
  std::set<std::string> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (!seen.insert(key).second)
      throw std::invalid_argument("Duplicate option: " + key);
    if (key == "--benchmark-worker") {
      o.worker = true;
      continue;
    }
    if (i + 1 >= argc)
      throw std::invalid_argument("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--mode") o.mode = value;
    else if (key == "--video") o.video = value;
    else if (key == "--armor-config") o.armor_config = value;
    else if (key == "--width") o.width = integer(value, key, 16, 4096);
    else if (key == "--height") o.height = integer(value, key, 16, 4096);
    else if (key == "--threads") o.threads = integer(value, key, 1, 128);
    else if (key == "--fps") o.fps = number(value, key, 1, 240);
    else if (key == "--seconds") o.seconds = number(value, key, .1, 3600);
    else throw std::invalid_argument("Unknown option: " + key);
  }
  if (o.mode != "traditional")
    throw std::invalid_argument("--mode must be traditional");
  if (!std::filesystem::is_regular_file(o.video))
    throw std::invalid_argument("--video must be an existing local video file");
  if (o.worker && seen.count("--seconds"))
    throw std::invalid_argument("--seconds cannot override a worker schedule");
  return o;
}

// The parent uses Python time.monotonic(), backed by the same Linux clock.
double monotonic() {
  timespec t{};
  if (::clock_gettime(CLOCK_MONOTONIC, &t) != 0)
    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC) failed");
  return static_cast<double>(t.tv_sec) + t.tv_nsec * 1e-9;
}

void waitUntil(double deadline) {
  for (double remaining = deadline - monotonic(); remaining > 0;
       remaining = deadline - monotonic())
    std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
}

struct Schedule {
  double start{}, warmup{}, measure{};
};

Schedule readSchedule(const Options &o) {
  if (!o.worker) return {monotonic(), 0, o.seconds};
  std::string line;
  if (!std::getline(std::cin, line))
    throw std::runtime_error("Schedule input closed before start");
  const auto input = Json::parse(line);
  if (!input.is_object() || input.size() != 3 ||
      !input.contains("start_monotonic_s") || !input.contains("warmup_s") ||
      !input.contains("measure_s"))
    throw std::invalid_argument("Expected only start_monotonic_s, warmup_s, measure_s");
  for (const char *key : {"start_monotonic_s", "warmup_s", "measure_s"})
    if (!input.at(key).is_number())
      throw std::invalid_argument(std::string(key) + " must be numeric");
  Schedule s{input.at("start_monotonic_s").get<double>(),
             input.at("warmup_s").get<double>(), input.at("measure_s").get<double>()};
  const double now = monotonic();
  if (!std::isfinite(s.start) || !std::isfinite(s.warmup) ||
      !std::isfinite(s.measure) || s.start < now - 5 || s.start > now + 60 ||
      s.warmup < 0 || s.warmup > 600 || s.measure < .1 || s.measure > 3600)
    throw std::invalid_argument("Invalid bounded monotonic worker schedule");
  return s;
}

// Decode sequentially; the only retained image is the current frame. Reopen at
// most once per request so a corrupt/empty clip cannot produce a retry spin.
class Video {
public:
  explicit Video(const Options &o) : path_(o.video), size_(o.width, o.height) {
    open();
    const double value = capture_.get(cv::CAP_PROP_FPS);
    fps_ = std::isfinite(value) && value > 0 ? Json(value) : Json(nullptr);
    backend_ = capture_.getBackendName();
  }
  cv::Mat next() {
    cv::Mat raw;
    if (!capture_.read(raw) || raw.empty()) {
      capture_.release();
      open();
      if (!capture_.read(raw) || raw.empty())
        throw std::runtime_error("Video has no decodable frame after one reopen");
      ++loops_;
    }
    if (raw.type() != CV_8UC3)
      throw std::runtime_error("Video decoder did not produce BGR8 frames");
    if (source_size_.empty()) source_size_ = raw.size();
    if (raw.size() != source_size_)
      throw std::runtime_error("Video frame dimensions changed mid-stream");
    cv::Mat resized;
    cv::resize(raw, resized, size_, 0, 0, cv::INTER_LINEAR);
    return resized;
  }
  Json metadata() const {
    return {{"source_width", source_size_.width}, {"source_height", source_size_.height},
            {"source_fps", fps_}, {"decoder_backend", backend_}};
  }
  std::uint64_t loops() const { return loops_; }
private:
  void open() {
    if (!capture_.open(path_))
      throw std::runtime_error("Failed to open video: " + path_);
  }
  std::string path_, backend_;
  cv::Size size_, source_size_;
  cv::VideoCapture capture_;
  Json fps_;
  std::uint64_t loops_{};
};

class Algorithm {
public:
  explicit Algorithm(const Options &o) {
    auto config = aim::ArmorAutoConfig::load(o.armor_config);
    // The recording has no matching camera calibration. Never run its PnP.
    config.detector.has_camera = false;
    config.calibrated = false;
    detector_ = config.detector;
    tracker_ = std::make_unique<aim::ArmorTracker>(config);
  }
  std::size_t process(const cv::Mat &image, double now) {
    const auto result = aim::detectLightBars(image, detector_);
    bars_last_ = result.bars.size();
    tracked_last_ = tracker_->update(result.armors, image.size(), now).confirmed;
    return result.armors.size();
  }
  void resetTracking() { if (tracker_) tracker_->reset(); }
  Json metadata(const Options &o) const {
    return {{"armor_config", o.armor_config}, {"tracking_enabled", true},
            {"pose_enabled", false}, {"pose_disabled_reason", "source_video_intrinsics_unknown"},
            {"light_bars_last", bars_last_}, {"track_confirmed_last", tracked_last_}};
  }
private:
  std::unique_ptr<aim::ArmorTracker> tracker_;
  aim::LightBarConfig detector_;
  std::size_t bars_last_{};
  bool tracked_last_{};
};

struct Statistics {
  std::uint64_t frames{}, detections{}, skipped_slots{};
  double decode_ms{}, algorithm_ms{}, algorithm_max_ms{};
};

Json run(const Options &o, Video &video, Algorithm &algorithm, const Schedule &s) {
  const double measure_begin = s.start + s.warmup;
  const double end = measure_begin + s.measure;
  const double period = 1. / o.fps;
  double next = s.start, last_start = -1;
  Statistics total, measured;
  const auto loops_before = video.loops();
  waitUntil(s.start);
  const double actual_start = monotonic();
  // At each deadline process one frame. If slow, skip elapsed scheduler slots;
  // never race through a queue to catch up. The video itself remains sequential.
  for (;;) {
    waitUntil(next);
    const double begin = monotonic();
    if (begin >= end) break;
    const bool is_measured = begin >= measure_begin;
    const auto frame = video.next();
    const double decoded = monotonic();
    if (decoded <= last_start)
      throw std::runtime_error("Non-increasing monotonic replay clock");
    last_start = decoded;
    const auto detections = algorithm.process(frame, decoded);
    const double finished = monotonic();
    const double decode_ms = (decoded - begin) * 1000;
    const double algorithm_ms = (finished - decoded) * 1000;
    auto add = [&](Statistics &stats) {
      ++stats.frames;
      stats.detections += detections;
      stats.decode_ms += decode_ms;
      stats.algorithm_ms += algorithm_ms;
      stats.algorithm_max_ms = std::max(stats.algorithm_max_ms, algorithm_ms);
    };
    add(total);
    if (is_measured) add(measured);
    next += period;
    if (next < finished) {
      const auto missed = static_cast<std::uint64_t>(std::floor((finished - next) / period)) + 1;
      total.skipped_slots += missed;
      if (is_measured) measured.skipped_slots += missed;
      next += static_cast<double>(missed) * period;
    }
  }
  const double actual_end = monotonic();
  if (!measured.frames)
    throw std::runtime_error("No visual frame started during the measurement window");
  Json result = {{"event", "result"}, {"schema_version", 1}, {"role", "vision"},
                 {"success", true}, {"mode", o.mode}, {"frames", total.frames},
                 {"measured_frames", measured.frames}, {"warmup_frames", total.frames - measured.frames},
                 {"actual_fps", measured.frames / s.measure},
                 {"algorithm_mean_ms", measured.algorithm_ms / measured.frames},
                 {"algorithm_max_ms", measured.algorithm_max_ms},
                 {"decode_resize_mean_ms", measured.decode_ms / measured.frames},
                 {"detections_total", measured.detections},
                 {"input_width", o.width}, {"input_height", o.height},
                 {"target_fps", o.fps}, {"opencv_threads", cv::getNumThreads()},
                 {"threads_requested", o.threads}, {"eof_loops", video.loops() - loops_before},
                 {"skipped_replay_slots", measured.skipped_slots},
                 {"total_skipped_replay_slots", total.skipped_slots},
                 {"source_frames_dropped", 0}, {"replay_policy", "sequential_frames_no_catchup"},
                 {"scheduled_start_monotonic_s", s.start}, {"actual_start_monotonic_s", actual_start},
                 {"scheduled_measure_start_monotonic_s", measure_begin},
                 {"scheduled_end_monotonic_s", end}, {"actual_end_monotonic_s", actual_end},
                 {"warmup_s", s.warmup}, {"measure_s", s.measure},
                 {"window_assignment", "frame_start_time; last_frame_may_finish_after_window"}};
  result.update(video.metadata());
  result.update(algorithm.metadata(o));
  return result;
}
} // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      help();
      return 0;
    }
    const auto o = parse(argc, argv);
    cv::setNumThreads(o.threads);
    cv::ocl::setUseOpenCL(false);
    Video video(o);
    Algorithm algorithm(o);
    // Readiness requires one complete decode + detection, not merely file open.
    const double preparation_begin = monotonic();
    {
      const auto first = video.next();
      algorithm.process(first, monotonic());
    }
    algorithm.resetTracking();
    Json ready = {{"event", "ready"}, {"role", "vision"}, {"mode", o.mode},
                  {"video", o.video}, {"input_width", o.width}, {"input_height", o.height},
                  {"target_fps", o.fps}, {"opencv_threads", cv::getNumThreads()},
                  {"backend", "opencv_cpu"}, {"opencv_version", CV_VERSION},
                  {"scope", "streaming_video_decode_resize_and_2d_detection_no_gui_no_flight_io"},
                  {"thread_scope", "OpenCV_algorithm_threads; decoder_backend_may_use_other_threads"},
                  {"ready_probe_ms", (monotonic() - preparation_begin) * 1000}};
    ready.update(video.metadata());
    ready.update(algorithm.metadata(o));
    if (o.worker) std::cout << ready.dump() << '\n' << std::flush;
    const auto schedule = readSchedule(o);
    auto result = run(o, video, algorithm, schedule);
    result["scope"] = ready["scope"];
    result["thread_scope"] = ready["thread_scope"];
    result["backend"] = ready["backend"];
    result["opencv_version"] = CV_VERSION;
    result["video"] = o.video;
    rusage usage{};
    if (::getrusage(RUSAGE_SELF, &usage) != 0)
      throw std::runtime_error("getrusage failed");
    result["peak_rss_kib"] = usage.ru_maxrss;
    result["peak_rss_scope"] = "whole_process_lifetime_including_startup_probe_warmup_measure";
    result["video_buffer_policy"] = "current_decoded_and_resized_frame_only; no_whole_video_cache";
    std::cout << result.dump() << '\n' << std::flush;
    // Preserve resident algorithm state/buffers until the parent's final /proc sample.
    if (o.worker) {
      std::string release;
      std::getline(std::cin, release);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "vision_video_worker: " << error.what() << '\n';
    return 1;
  }
}
