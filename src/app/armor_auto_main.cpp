#include "../input/uvc_stream.hpp"
#include "armor_calibrate.hpp"
#include "demo_aim/armor_auto.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <thread>
namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
int number(const char *s) {
  std::size_t n = 0;
  const std::string v(s);
  int x = std::stoi(v, &n);
  if (n != v.size())
    throw std::invalid_argument("Invalid integer: " + v);
  return x;
}
struct Options {
  std::string config{"config/armor_auto.yaml"}, output, image, save_dir,
      calibration_output;
  bool show{true}, calibrate{false};
  int frames{0}, seconds{0}, hz{10}, columns{0}, rows{0};
  double square_mm{0};
};
Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string k(argv[i]);
    auto next = [&]() {
      if (++i == argc)
        throw std::invalid_argument("Missing value: " + k);
      return argv[i];
    };
    if (k == "--no-show")
      o.show = false;
    else if (k == "--calibrate")
      o.calibrate = true;
    else if (k == "--config")
      o.config = next();
    else if (k == "--output")
      o.output = next();
    else if (k == "--image")
      o.image = next();
    else if (k == "--save-dir")
      o.save_dir = next();
    else if (k == "--frames")
      o.frames = number(next());
    else if (k == "--seconds")
      o.seconds = number(next());
    else if (k == "--hz")
      o.hz = number(next());
    else if (k == "--board-cols")
      o.columns = number(next());
    else if (k == "--board-rows")
      o.rows = number(next());
    else if (k == "--square-mm") {
      const std::string s(next());
      std::size_t n = 0;
      o.square_mm = std::stod(s, &n);
      if (n != s.size())
        throw std::invalid_argument("Invalid square size");
    } else if (k == "--calibration-output")
      o.calibration_output = next();
    else
      throw std::invalid_argument("Unknown option: " + k);
  }
  if (o.frames < 0 || o.seconds < 0 || o.hz < 1 || o.hz > 60)
    throw std::invalid_argument("Invalid frame/time/rate limit");
  return o;
}
void line(cv::Mat &v, const std::string &s, int y, cv::Scalar color) {
  cv::putText(v, s, {8, y}, cv::FONT_HERSHEY_SIMPLEX, .43, color, 1,
              cv::LINE_AA);
}
} // namespace
int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--camera-worker")
    return aim::cameraWorkerMain(argc, argv);
  try {
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--help") {
        std::cout
            << "Automatic armor detection/tracking + calibrated RGB PnP (no "
               "manual ROI)\n"
               "--config config/armor_auto.yaml --no-show --frames N --seconds "
               "N --hz 10\n"
               "--output FILE.jsonl (append) --image FILE --save-dir NEW_DIR\n"
               "Color config: 1=blue, 2=red. q/Esc quit, r reset target.\n"
               "Without RGB calibration: detection only, numeric pose/distance "
               "null.\n"
               "Calibration: --calibrate --board-cols N --board-rows M "
               "--square-mm S\n"
               "             --calibration-output NEW.yaml (default config "
               "calibration path)\n"
               "XYZ uses RGB optical frame X right/Y down/Z forward, meters.\n"
               "normal_yaw/pitch: board surface; bearing_yaw/pitch: target "
               "direction, degrees.\n";
        return 0;
      }
    const auto o = parse(argc, argv);
    const auto c = aim::ArmorAutoConfig::load(o.config);
    if ((o.show || o.calibrate) && !std::getenv("DISPLAY") &&
        !std::getenv("WAYLAND_DISPLAY"))
      throw std::runtime_error("No desktop; use --no-show");
    cv::setNumThreads(2);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    if (o.calibrate)
      return aim::calibrateArmorCamera(c, o.columns, o.rows, o.square_mm,
                                       o.calibration_output.empty()
                                           ? c.calibration_path
                                           : o.calibration_output);
    if (!o.save_dir.empty() && std::filesystem::exists(o.save_dir))
      throw std::invalid_argument("--save-dir must be new");
    std::ofstream file;
    if (!o.output.empty()) {
      file.open(o.output, std::ios::app);
      if (!file)
        throw std::runtime_error("Cannot open output");
    }
    std::ostream &out = o.output.empty() ? std::cout : file;
    if (!c.calibrated)
      std::cerr << "RGB calibration missing: auto detection/tracking enabled, "
                   "metric pose/distance unavailable\n";
    const std::string window = "AUTO armor - RGB calibrated PnP";
    if (o.show) {
      cv::namedWindow(window, cv::WINDOW_NORMAL);
      cv::resizeWindow(window, 960, 720);
    }
    aim::StreamOptions s;
    s.device = c.device;
    s.format = c.format;
    s.width = c.width;
    s.height = c.height;
    s.fps = c.fps;
    s.controls = false;
    std::unique_ptr<aim::CameraStream> camera;
    if (o.image.empty())
      camera = std::make_unique<aim::CameraStream>(s);
    aim::CameraControls controls;
    aim::ArmorTracker tracker(c);
    aim::TrackedArmor target;
    cv::Mat raw, overlay;
    aim::LightBarResult detection;
    int count = 0, locked = 0, valid_count = 0;
    std::uint64_t generation = 0;
    const auto start = Clock::now();
    auto next_report = start, next_draw = start;
    bool failed = false, updated = false;
    double fps = 0;
    std::string reason = "waiting_for_frame";
    nlohmann::json packet;
    while (!stopped && (!o.frames || count < o.frames) &&
           (!o.seconds ||
            Clock::now() - start < std::chrono::seconds(o.seconds))) {
      const auto now = Clock::now();
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
        raw = cv::imread(o.image);
        if (raw.empty())
          throw std::runtime_error("Cannot read image");
        fresh = true;
      }
      const double age = camera ? camera->frameAge() : 0;
      if (fresh) {
        if (raw.cols != c.width || raw.rows != c.height)
          throw std::runtime_error(
              "Negotiated image size differs from config/calibration");
        detection = aim::detectLightBars(raw, c.detector);
        target = tracker.update(
            detection.armors, raw.size(),
            std::chrono::duration<double>(now.time_since_epoch()).count());
        for (auto &armor : detection.armors)
          armor.reported = false; // 角度只在所选且有效目标的主状态栏显示。
        ++count;
        if (target.confirmed)
          ++locked;
        updated = true;
        fps =
            count /
            std::max(.001, std::chrono::duration<double>(now - start).count());
      }
      const bool current = !raw.empty() && age <= .25 && !failed;
      bool valid = false;
      if (!current)
        reason = failed ? "camera_failed" : "stale_or_missing_frame";
      else if (!target.target)
        reason = "target_lost_or_absent";
      else if (!target.confirmed)
        reason = "confirming_target";
      else if (!c.calibrated)
        reason = "rgb_calibration_missing";
      else if (!c.dimensions_confirmed)
        reason = "armor_dimensions_unconfirmed";
      else if (!target.target->pose_valid)
        reason = "pose_rejected";
      else if (target.target->pose_ambiguous)
        reason = "pose_ambiguous";
      else {
        reason = "ok";
        valid = true;
      }
      if (fresh && valid)
        ++valid_count;
      if (now >= next_report || failed || (!camera && count)) {
        packet = {{"source", "automatic_armor_rgb_pnp"},
                  {"coordinate_frame", "rgb_optical"},
                  {"target_color", c.target_color},
                  {"frame", count},
                  {"generation", generation},
                  {"fps", fps},
                  {"detection_valid", current && bool(target.target)},
                  {"tracking_confirmed", current && target.confirmed},
                  {"track_id", target.id},
                  {"valid", valid},
                  {"reason", reason},
                  {"position_camera_m", nullptr},
                  {"range_m", nullptr},
                  {"z_m", nullptr},
                  {"normal_yaw_deg", nullptr},
                  {"normal_pitch_deg", nullptr},
                  {"bearing_yaw_deg", nullptr},
                  {"bearing_pitch_deg", nullptr},
                  {"reprojection_error_px", nullptr},
                  {"armor_corners_px", nullptr},
                  {"sample_time_monotonic_s",
                   age < 100 ? nlohmann::json(std::chrono::duration<double>(
                                                  now.time_since_epoch())
                                                  .count() -
                                              age)
                             : nlohmann::json(nullptr)},
                  {"valid_until_monotonic_s",
                   valid ? nlohmann::json(std::chrono::duration<double>(
                                              now.time_since_epoch())
                                              .count() -
                                          age + .25)
                         : nlohmann::json(nullptr)},
                  {"frame_age_ms", age < 100 ? nlohmann::json(age * 1000)
                                             : nlohmann::json(nullptr)}};
        if (current && target.target) {
          auto &a = *target.target;
          packet["armor_corners_px"] = nlohmann::json::array();
          for (auto &p : a.corners)
            packet["armor_corners_px"].push_back({p.x, p.y});
        }
        if (valid) {
          const auto &a = *target.target;
          auto p = a.position_camera_m;
          packet["position_camera_m"] = {p[0], p[1], p[2]};
          packet["range_m"] = cv::norm(p);
          packet["z_m"] = p[2];
          packet["normal_yaw_deg"] = a.yaw_deg;
          packet["normal_pitch_deg"] = a.pitch_deg;
          packet["bearing_yaw_deg"] = std::atan2(p[0], p[2]) * 180 / CV_PI;
          packet["bearing_pitch_deg"] =
              std::atan2(-p[1], std::hypot(p[0], p[2])) * 180 / CV_PI;
          packet["reprojection_error_px"] = a.reprojection_error_px;
        }
        out << packet.dump() << '\n' << std::flush;
        if (!out)
          throw std::runtime_error("Output failed");
        next_report = now + std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(1. / o.hz));
      }
      if (o.show || !o.save_dir.empty()) {
        if ((updated || !current) && now >= next_draw) {
          overlay = current ? aim::drawLightBars(raw, detection)
                            : cv::Mat(c.height, c.width, CV_8UC3,
                                      cv::Scalar(20, 20, 20));
          if (current && target.target)
            cv::rectangle(overlay, target.target->box, {0, 255, 255}, 2);
          cv::rectangle(overlay, {0, 0}, {overlay.cols, 100}, {20, 20, 20},
                        cv::FILLED);
          line(overlay,
               "AUTO " + std::string(c.target_color == 1 ? "BLUE" : "RED") +
                   " | track " + std::to_string(target.id) + " | " + reason,
               19, {0, 220, 255});
          line(overlay, "PnP in RGB optical frame | q/Esc exit | r reacquire",
               38, {230, 230, 230});
          if (valid) {
            const auto &a = *target.target;
            std::ostringstream v;
            v << std::fixed << std::setprecision(3) << "Range "
              << cv::norm(a.position_camera_m) << " m | Z "
              << a.position_camera_m[2] << " m";
            line(overlay, v.str(), 58, {0, 255, 0});
            v.str("");
            v.clear();
            v << std::fixed << std::setprecision(1) << "Surface yaw "
              << a.yaw_deg << " pitch " << a.pitch_deg << " deg";
            line(overlay, v.str(), 78, {0, 255, 0});
          } else
            line(overlay, "Distance / yaw / pitch: n/a", 62, {0, 180, 255});
          if (o.show)
            cv::imshow(window, overlay);
          updated = false;
          next_draw = now + std::chrono::milliseconds(33);
        }
      }
      if (failed || !camera)
        break;
      if (o.show) {
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
    if (camera)
      camera->stop();
    if (!o.save_dir.empty() && !raw.empty()) {
      std::filesystem::create_directories(o.save_dir);
      if (!cv::imwrite(o.save_dir + "/raw.png", raw) ||
          !cv::imwrite(o.save_dir + "/overlay.png", overlay))
        throw std::runtime_error("Snapshot failed");
    }
    if (!packet.is_null()) {
      packet["valid"] = false;
      packet["detection_valid"] = false;
      packet["tracking_confirmed"] = false;
      packet["reason"] = failed ? "camera_failed" : "stopped";
      packet["frame"] = count;
      for (const char *k :
           {"position_camera_m", "range_m", "z_m", "normal_yaw_deg",
            "normal_pitch_deg", "bearing_yaw_deg", "bearing_pitch_deg",
            "reprojection_error_px", "armor_corners_px", "frame_age_ms",
            "sample_time_monotonic_s", "valid_until_monotonic_s"})
        packet[k] = nullptr;
      out << packet.dump() << '\n' << std::flush;
    }
    if (o.show)
      cv::destroyAllWindows();
    std::cerr << "Processed=" << count << " confirmed=" << locked
              << " valid_pose=" << valid_count << '\n';
    return failed ? 1 : 0;
  } catch (const std::exception &e) {
    std::cerr << "armor_auto: " << e.what() << '\n';
    return 1;
  }
}
