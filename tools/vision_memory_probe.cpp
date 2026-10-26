// 离线、单进程传统算法基准；不采集相机、不显示窗口、不连接飞控。
#include "demo_aim/armor_auto.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/resource.h>

namespace {
using Clock = std::chrono::steady_clock;
struct Options {
  std::string mode{"traditional"}, image, armor_config;
  std::string light_config{"config/light_bars.yaml"};
  int frames{300}, warmup{30}, threads{2};
  bool light_config_explicit{false};
};

int integer(const std::string &value, const std::string &option, int minimum,
            int maximum) {
  int result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(),
                                      result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
      result < minimum || result > maximum)
    throw std::invalid_argument(option + " must be an integer in " +
                                std::to_string(minimum) + ".." +
                                std::to_string(maximum));
  return result;
}

void help() {
  std::cout
      << "Offline CPU vision memory/timing probe (Linux)\n"
      << "  --image FILE [--mode traditional]\n"
      << "  [--frames 300] [--warmup 30] [--threads 2]\n"
      << "  [--light-config config/light_bars.yaml]\n"
      << "  [--armor-config config/armor_auto.yaml]\n"
      << "Each invocation runs traditional detection on one repeatedly reused BGR image.\n"
      << "--armor-config enables tracking and supplies its detector/calibration;\n"
      << "an explicit --light-config overrides that detector configuration.\n"
      << "ru_maxrss is the whole-process lifetime peak in KiB; camera/GUI/ROS/\n"
      << "depth/flight control and accelerator memory are outside this probe.\n";
}

Options parse(int argc, char **argv) {
  Options options;
  std::set<std::string> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (!seen.insert(key).second)
      throw std::invalid_argument("Duplicate option: " + key);
    if (i + 1 >= argc)
      throw std::invalid_argument("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--mode")
      options.mode = value;
    else if (key == "--image")
      options.image = value;
    else if (key == "--light-config") {
      options.light_config = value;
      options.light_config_explicit = true;
    } else if (key == "--armor-config")
      options.armor_config = value;
    else if (key == "--frames")
      options.frames = integer(value, key, 1, 1000000);
    else if (key == "--warmup")
      options.warmup = integer(value, key, 0, 1000000);
    else if (key == "--threads")
      options.threads = integer(value, key, 1, 128);
    else
      throw std::invalid_argument("Unknown option: " + key);
  }
  if (options.mode != "traditional")
    throw std::invalid_argument("--mode must be traditional");
  if (options.image.empty() ||
      !std::filesystem::is_regular_file(options.image))
    throw std::invalid_argument("--image must name an existing image file");
  return options;
}

double milliseconds(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}
} // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      help();
      return 0;
    }
    const auto options = parse(argc, argv);
    cv::setNumThreads(options.threads);
    cv::ocl::setUseOpenCL(false);
    // Decode once, retain one common source image, and do not cache all outputs.
    const cv::Mat image = cv::imread(options.image, cv::IMREAD_COLOR);
    if (image.empty())
      throw std::runtime_error("Failed to decode image: " + options.image);

    std::unique_ptr<aim::ArmorTracker> tracker;
    aim::LightBarConfig light_config;
    if (!options.armor_config.empty()) {
      auto armor = aim::ArmorAutoConfig::load(options.armor_config);
      if (options.light_config_explicit)
        armor.detector = aim::LightBarConfig::load(options.light_config);
      light_config = armor.detector;
      tracker = std::make_unique<aim::ArmorTracker>(armor);
    } else
      light_config = aim::LightBarConfig::load(options.light_config);

    std::uint64_t iteration = 0, total_detections = 0;
    std::size_t last_detections = 0, last_light_bars = 0;
    bool last_track_confirmed = false;
    auto run_frame = [&] {
      const auto detected = aim::detectLightBars(image, light_config);
      last_detections = detected.armors.size();
      last_light_bars = detected.bars.size();
      if (tracker)
        // A fixed 30 Hz replay clock affects tracker state, not measured FPS.
        last_track_confirmed =
            tracker->update(detected.armors, image.size(),
                            static_cast<double>(iteration) / 30.0)
                .confirmed;
      ++iteration;
    };
    const auto warmup_begin = Clock::now();
    for (int i = 0; i < options.warmup; ++i)
      run_frame();
    const double warmup_ms = milliseconds(warmup_begin, Clock::now());
    // The external sampler uses this stable marker to isolate steady-state PSS.
    std::cerr << "VISION_MEMORY_MEASURE_BEGIN\n" << std::flush;

    double total_ms = 0, min_ms = std::numeric_limits<double>::max(), max_ms = 0;
    int frames = 0;
    const auto loop_begin = Clock::now();
    for (; frames < options.frames; ++frames) {
      const auto begin = Clock::now();
      run_frame();
      const double elapsed = milliseconds(begin, Clock::now());
      total_ms += elapsed;
      min_ms = std::min(min_ms, elapsed);
      max_ms = std::max(max_ms, elapsed);
      total_detections += last_detections;
    }
    const double loop_ms = milliseconds(loop_begin, Clock::now());
    nlohmann::json result = {
        {"schema_version", 1},
        {"mode", options.mode},
        {"scope", "offline_repeated_single_image_algorithm"},
        {"backend", "opencv_cpu"},
        {"opencv_version", CV_VERSION},
        {"threads_requested", options.threads},
        {"opencv_threads", cv::getNumThreads()},
        {"image", options.image},
        {"input_width", image.cols},
        {"input_height", image.rows},
        {"frames", frames},
        {"warmup_frames", options.warmup},
        {"warmup_ms", warmup_ms},
        {"algorithm_total_ms", total_ms},
        {"algorithm_mean_ms", total_ms / frames},
        {"algorithm_min_ms", min_ms},
        {"algorithm_max_ms", max_ms},
        {"loop_wall_ms", loop_ms},
        {"fps", loop_ms > 0 ? frames * 1000.0 / loop_ms : 0.0},
        {"detections_total", total_detections},
        {"detections_last", last_detections},
        {"memory_metric", "getrusage(RUSAGE_SELF).ru_maxrss"},
        {"memory_scope", "whole_process_lifetime_peak_including_startup_image_decode_warmup_and_measurement"},
        {"excluded", {"camera_capture", "gui", "ros", "depth", "flight_control",
                       "gpu_npu_device_memory"}}};
    result["light_config"] =
        options.armor_config.empty() || options.light_config_explicit
            ? options.light_config
            : std::string("from_armor_config");
    result["armor_config"] = options.armor_config;
    result["tracking_enabled"] = static_cast<bool>(tracker);
    result["pose_enabled"] = light_config.has_camera;
    result["light_bars_last"] = last_light_bars;
    if (tracker) {
      result["tracking_replay_hz"] = 30;
      result["track_confirmed_last"] = last_track_confirmed;
    }
    struct rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0)
      throw std::runtime_error("getrusage failed");
    result["peak_rss_kib"] = usage.ru_maxrss;
    std::cout << result.dump() << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "vision_memory_probe: " << error.what() << '\n';
    return 1;
  }
}
