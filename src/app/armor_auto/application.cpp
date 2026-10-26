#include "demo_aim/detail/app/armor_auto/application.hpp"
#include "demo_aim/detail/input/uvc_stream.hpp"
#include "demo_aim/detail/app/armor_calibrate.hpp"
#include "demo_aim/detail/app/common/frame_rate.hpp"
#include "demo_aim/detail/app/armor_auto/options.hpp"
#include "demo_aim/detail/app/armor_auto/output.hpp"
#include "demo_aim/detail/app/armor_auto/preview.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>

namespace aim::armor_auto_app {
namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
} // namespace

int run(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help") {
      printHelp();
      return 0;
    }
  }
  const auto options = parseOptions(argc, argv);
  const auto config = aim::ArmorAutoConfig::load(options.config);
  if ((options.show || options.calibrate) && !std::getenv("DISPLAY") &&
      !std::getenv("WAYLAND_DISPLAY"))
    throw std::runtime_error("No desktop; use --no-show");
  cv::setNumThreads(2);
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  if (options.calibrate)
    return aim::calibrateArmorCamera(
        config, options.columns, options.rows, options.square_mm,
        options.calibration_output.empty() ? config.calibration_path
                                           : options.calibration_output);
  if (!options.save_dir.empty() && std::filesystem::exists(options.save_dir))
    throw std::invalid_argument("--save-dir must be new");
  std::ofstream file;
  if (!options.output.empty()) {
    file.open(options.output, std::ios::app);
    if (!file)
      throw std::runtime_error("Cannot open output");
  }
  std::ostream &out = options.output.empty() ? std::cout : file;
  if (!config.calibrated)
    std::cerr << "RGB calibration missing: auto detection/tracking enabled, "
                 "metric pose/distance unavailable\n";
  const std::string window = "AUTO armor - RGB calibrated PnP";
  if (options.show) {
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::resizeWindow(window, 960, 720);
  }
  aim::StreamOptions stream_options;
  stream_options.device = config.device;
  stream_options.format = config.format;
  stream_options.width = config.width;
  stream_options.height = config.height;
  stream_options.fps = config.fps;
  stream_options.controls = false;
  std::unique_ptr<aim::CameraStream> camera;
  if (options.image.empty())
    camera = std::make_unique<aim::CameraStream>(stream_options);
  aim::CameraControls controls;
  aim::ArmorTracker tracker(config);
  aim::TrackedArmor target;
  cv::Mat raw, overlay;
  aim::LightBarResult detection;
  int count = 0, locked = 0, valid_count = 0;
  std::uint64_t generation = 0;
  const auto start = Clock::now();
  auto next_report = start, next_draw = start;
  app::FrameRate frame_rate{start};
  bool failed = false, updated = false;
  nlohmann::json packet;
  FrameContext last_context;
  while (!stopped && (!options.frames || count < options.frames) &&
         (!options.seconds ||
          Clock::now() - start < std::chrono::seconds(options.seconds))) {
    const auto frame_time = Clock::now();
    bool fresh = false;
    if (camera) {
      auto state = camera->poll(raw, controls);
      failed = state == aim::CameraStream::Result::Failed;
      fresh = state == aim::CameraStream::Result::Frame;
      if (generation != camera->generation()) {
        generation = camera->generation();
        tracker.reset();
        target = {};
        updated = true;
      }
    } else if (count == 0) {
      raw = cv::imread(options.image);
      if (raw.empty())
        throw std::runtime_error("Cannot read image");
      fresh = true;
    }
    if (fresh) {
      if (raw.cols != config.width || raw.rows != config.height)
        throw std::runtime_error(
            "Negotiated image size differs from config/calibration");
      detection = aim::detectLightBars(raw, config.detector);
      target = tracker.update(
          detection.armors, raw.size(),
          std::chrono::duration<double>(frame_time.time_since_epoch()).count());
      for (auto &armor : detection.armors)
        armor.reported = false; // 角度只在所选且有效目标的主状态栏显示。
      ++count;
      if (target.confirmed)
        ++locked;
      updated = true;
    }
    // Detection may take longer than the freshness limit. Timestamp after it,
    // then use this single snapshot for reporting and drawing.
    const auto now = Clock::now();
    const double age = camera ? camera->frameAge() : 0;
    const FrameContext context{
        count,
        generation,
        frame_rate.update(now, fresh),
        !raw.empty(),
        failed,
        std::chrono::duration<double>(now.time_since_epoch()).count(),
        age};
    last_context = context;
    const auto status = evaluateFrame(config, target, context);
    if (fresh && status.valid)
      ++valid_count;
    if (now >= next_report || failed || (!camera && count)) {
      packet = makeReport(config, target, context);
      out << packet.dump() << '\n' << std::flush;
      if (!out)
        throw std::runtime_error("Output failed");
      next_report = now + std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double>(1. / options.hz));
    }
    if (options.show || !options.save_dir.empty()) {
      if ((updated || !status.current) && now >= next_draw) {
        overlay = drawPreview(raw, detection, target, config, context);
        if (options.show)
          cv::imshow(window, overlay);
        updated = false;
        next_draw = now + std::chrono::milliseconds(33);
      }
    }
    if (failed || !camera)
      break;
    if (options.show) {
      const int key = cv::waitKey(1) & 255;
      if (key == 'q' || key == 27 ||
          cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) == 0)
        break;
      if (key == 'r') {
        tracker.reset();
        target = {};
        updated = true;
      }
    } else
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (!options.save_dir.empty() && !raw.empty()) {
    // The display may still contain the previous frame due to its 33 ms cap.
    // Refresh expiry after the last GUI wait, then render this raw frame anew.
    last_context.monotonic_s =
        std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    last_context.age_s = camera ? camera->frameAge() : 0;
    last_context.failed = failed;
    saveSnapshot(options.save_dir, raw, detection, target, config, last_context);
  }
  if (camera)
    camera->stop();
  if (!packet.is_null()) {
    invalidateReport(packet, count, failed);
    out << packet.dump() << '\n' << std::flush;
  }
  if (options.show)
    cv::destroyAllWindows();
  std::cerr << "Processed=" << count << " confirmed=" << locked
            << " valid_pose=" << valid_count << '\n';
  return failed ? 1 : 0;
}

} // namespace aim::armor_auto_app
