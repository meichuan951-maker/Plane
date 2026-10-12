#include "../input/astra_depth_worker.hpp"
#include "../input/uvc_stream.hpp"
#include "demo_aim/depth_plane.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <thread>
namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
struct Selection {
  cv::Rect roi;
  cv::Point first, current;
  bool dragging{false};
};
cv::Rect rectangle(cv::Point a, cv::Point b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::abs(a.x - b.x),
          std::abs(a.y - b.y)};
}
void mouse(int event, int x, int y, int, void *data) {
  auto &s = *static_cast<Selection *>(data);
  const cv::Point p(std::clamp(x, 0, 639), std::clamp(y, 0, 479));
  if (event == cv::EVENT_LBUTTONDOWN) {
    s.dragging = true;
    s.first = s.current = p;
    s.roi = {};
  } else if (event == cv::EVENT_MOUSEMOVE && s.dragging)
    s.current = p;
  else if (event == cv::EVENT_LBUTTONUP && s.dragging) {
    s.dragging = false;
    s.roi = rectangle(s.first, p);
  }
}
int integer(const char *arg) {
  std::size_t used = 0;
  const std::string s(arg);
  const int value = std::stoi(s, &used);
  if (used != s.size())
    throw std::invalid_argument("Invalid integer: " + s);
  return value;
}
struct Options {
  bool show{true};
  int frames{0}, seconds{0}, hz{10};
  cv::Rect roi;
  std::string output;
};
Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    auto next = [&]() {
      if (++i == argc)
        throw std::invalid_argument("Missing value: " + key);
      return argv[i];
    };
    if (key == "--no-show")
      o.show = false;
    else if (key == "--frames")
      o.frames = integer(next());
    else if (key == "--seconds")
      o.seconds = integer(next());
    else if (key == "--hz")
      o.hz = integer(next());
    else if (key == "--output")
      o.output = next();
    else if (key == "--roi") {
      const int x = integer(next()), y = integer(next());
      const int w = integer(next()), h = integer(next());
      if (x < 0 || y < 0 || w < 8 || h < 8 || w > 640 || h > 480 ||
          x > 640 - w || y > 480 - h)
        throw std::invalid_argument("ROI must be inside 640x480, at least 8x8");
      o.roi = {x, y, w, h};
    } else
      throw std::invalid_argument("Unknown option: " + key);
  }
  if (o.frames < 0 || o.seconds < 0 || o.hz < 1 || o.hz > 30 ||
      (!o.show && o.roi.empty()))
    throw std::invalid_argument("Use --roi x y w h with --no-show; hz 1..30");
  return o;
}
void line(cv::Mat &image, const std::string &text, int y, cv::Scalar color) {
  cv::putText(image, text, {8, y}, cv::FONT_HERSHEY_SIMPLEX, .43, color, 1,
              cv::LINE_AA);
}
struct Window {
  bool show;
  ~Window() {
    if (show)
      cv::destroyAllWindows();
  }
};
} // namespace
int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--camera-worker")
    return aim::astraDepthWorkerMain();
  try {
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--help") {
        std::cout
            << "Astra Pro LIVE manual depth ROI (approximate SDK projection)\n"
               "Drag inside armor surface in DEPTH image. No RGB alignment / "
               "tracking.\n"
               "q/Esc: quit; r: clear ROI. Reselect if target moves.\n"
               "--roi x y w h --no-show --frames N --seconds N --hz 10\n"
               "--output FILE.jsonl (append; default stdout; diagnostics on "
               "stderr)\n"
               "Raw depth optical frame: X right, Y down, Z forward; "
               "meters/degrees.\n"
               "normal_yaw/pitch: surface orientation; bearing_yaw/pitch: "
               "target direction.\n";
        return 0;
      }
    const auto o = parse(argc, argv);
    if (o.show && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY"))
      throw std::runtime_error("No desktop; use --no-show --roi x y w h");
    std::ofstream file;
    if (!o.output.empty()) {
      // 日志追加，保留用户已有测量记录。
      file.open(o.output, std::ios::app);
      if (!file)
        throw std::runtime_error("Cannot open output: " + o.output);
    }
    std::ostream &output = o.output.empty() ? std::cout : file;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    cv::setNumThreads(2);
    aim::StreamOptions settings;
    settings.device = "astra-depth";
    settings.frame_type = CV_32FC3;
    settings.controls = false;
    settings.stall_seconds = 5;
    settings.graceful_stop_ms = 400;
    aim::CameraStream camera(settings);
    aim::CameraControls controls;
    Selection selection{o.roi, {}, {}, false};
    const std::string window = "Astra Pro - LIVE manual depth ROI";
    Window cleanup{o.show};
    if (o.show) {
      cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
      cv::setMouseCallback(window, mouse, &selection);
    }
    std::cerr
        << "Drag inside armor in the DEPTH image. Approximate SDK projection; "
           "manual ROI is not tracked. Reselect if target moves.\n";
    cv::Mat xyz, view;
    std::optional<aim::DepthPlane> measurement;
    cv::Rect last_roi;
    std::string reason = "waiting_for_depth";
    int count = 0, valid = 0;
    std::uint64_t generation = 0;
    const auto start = Clock::now();
    auto next_report = start, next_draw = start;
    bool failed = false;
    nlohmann::json last_packet;
    while (!stopped && (!o.frames || count < o.frames) &&
           (!o.seconds ||
            Clock::now() - start < std::chrono::seconds(o.seconds))) {
      const auto state = camera.poll(xyz, controls);
      const auto now = Clock::now();
      if (camera.generation() != generation) {
        if (generation)
          selection.roi = {}; // 重连后重新确认目标，不沿用旧区域。
        generation = camera.generation();
      }
      if (state == aim::CameraStream::Result::Frame)
        ++count;
      if (state == aim::CameraStream::Result::Failed) {
        measurement.reset();
        reason = "depth_stream_failed";
        failed = true;
      } else if (camera.frameAge() > .25 || xyz.empty()) {
        measurement.reset();
        reason = "stale_or_missing_depth";
      } else if (selection.dragging || selection.roi.empty()) {
        measurement.reset();
        reason = "select_depth_roi";
      } else if (state == aim::CameraStream::Result::Frame ||
                 selection.roi != last_roi)
        measurement = aim::measureDepthPlane(xyz, selection.roi, reason);
      last_roi = selection.roi;
      if (state == aim::CameraStream::Result::Frame && measurement)
        ++valid;
      if (now >= next_report || failed) {
        const double age = camera.frameAge();
        nlohmann::json j{{"source", "manual_depth_roi"},
                         {"tracking", false},
                         {"calibration", "sdk_depth_projection_unverified"},
                         {"frame", count},
                         {"generation", generation},
                         {"valid", bool(measurement)},
                         {"reason", reason},
                         {"frame_age_ms", age < 100 ? nlohmann::json(age * 1000)
                                                    : nlohmann::json(nullptr)},
                         {"sample_time_monotonic_s",
                          age < 100
                              ? nlohmann::json(std::chrono::duration<double>(
                                                   now.time_since_epoch())
                                                   .count() -
                                               age)
                              : nlohmann::json(nullptr)},
                         {"roi",
                          {selection.roi.x, selection.roi.y,
                           selection.roi.width, selection.roi.height}},
                         {"xyz_depth_optical_m", nullptr},
                         {"z_m", nullptr},
                         {"range_m", nullptr},
                         {"normal_yaw_deg", nullptr},
                         {"normal_pitch_deg", nullptr},
                         {"bearing_yaw_deg", nullptr},
                         {"bearing_pitch_deg", nullptr},
                         {"plane_rmse_m", nullptr}};
        if (measurement) {
          const auto &p = *measurement;
          j["xyz_depth_optical_m"] = {p.centroid_m[0], p.centroid_m[1],
                                      p.centroid_m[2]};
          j["z_m"] = p.centroid_m[2];
          j["range_m"] = p.range_m;
          j["normal_yaw_deg"] = p.normal_yaw_deg;
          j["normal_pitch_deg"] = p.normal_pitch_deg;
          j["bearing_yaw_deg"] = p.bearing_yaw_deg;
          j["bearing_pitch_deg"] = p.bearing_pitch_deg;
          j["plane_rmse_m"] = p.rmse_m;
        }
        output << j.dump() << '\n' << std::flush;
        last_packet = j;
        if (!output)
          throw std::runtime_error("Measurement output failed");
        next_report = now + std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(1. / o.hz));
      }
      if (o.show && now >= next_draw) {
        if (!xyz.empty() && camera.frameAge() <= .25) {
          cv::Mat z, gray;
          cv::extractChannel(xyz, z, 2);
          z.convertTo(gray, CV_8UC1, 255 / 2.7, -0.3 * 255 / 2.7);
          cv::applyColorMap(gray, view, cv::COLORMAP_TURBO);
          view.setTo(cv::Scalar(0, 0, 0), z <= 0);
          cv::rectangle(view,
                        selection.dragging
                            ? rectangle(selection.first, selection.current)
                            : selection.roi,
                        {255, 255, 255}, 1);
        } else
          view = cv::Mat(480, 640, CV_8UC3, cv::Scalar(15, 15, 15));
        cv::rectangle(view, {0, 0}, {640, 105}, {20, 20, 20}, cv::FILLED);
        line(view, "APPROXIMATE SDK depth | MANUAL ROI - no tracking", 18,
             {0, 210, 255});
        line(view,
             "Drag inside armor; reselect if moved | r clear | q/Esc exit", 37,
             {230, 230, 230});
        if (measurement) {
          std::ostringstream a, b, c;
          a << std::fixed << std::setprecision(3) << "Range "
            << measurement->range_m << " m | Z " << measurement->centroid_m[2]
            << " m";
          b << std::fixed << std::setprecision(1) << "Surface: yaw "
            << measurement->normal_yaw_deg << " pitch "
            << measurement->normal_pitch_deg << " deg";
          c << std::fixed << std::setprecision(1) << "Bearing: yaw "
            << measurement->bearing_yaw_deg << " pitch "
            << measurement->bearing_pitch_deg << " deg";
          line(view, a.str(), 58, {0, 255, 0});
          line(view, b.str(), 78, {0, 255, 0});
          line(view, c.str(), 98, {0, 255, 0});
        } else
          line(view, "Measurement n/a: " + reason, 62, {0, 180, 255});
        cv::imshow(window, view);
        next_draw = now + std::chrono::milliseconds(33);
      }
      if (failed)
        break;
      if (o.show) {
        const int key = cv::waitKey(1) & 255;
        // GTK 不支持 VISIBLE 查询时返回 -1，不能误判成用户关窗。
        if (key == 27 || key == 'q' ||
            cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) == 0)
          break;
        if (key == 'r')
          selection = {};
      } else
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    camera.stop();
    if (!last_packet.is_null()) {
      last_packet["valid"] = false;
      last_packet["reason"] = failed ? "depth_stream_failed" : "stopped";
      last_packet["frame"] = count;
      for (const char *key :
           {"xyz_depth_optical_m", "z_m", "range_m", "normal_yaw_deg",
            "normal_pitch_deg", "bearing_yaw_deg", "bearing_pitch_deg",
            "plane_rmse_m", "frame_age_ms", "sample_time_monotonic_s"})
        last_packet[key] = nullptr;
      output << last_packet.dump() << '\n' << std::flush;
    }
    std::cerr << "Depth frames=" << count << " valid ROI frames=" << valid
              << '\n';
    return failed ? 1 : (o.frames && (count < o.frames || !valid) ? 4 : 0);
  } catch (const std::exception &e) {
    std::cerr << "astra_live_measure: " << e.what() << '\n';
    return 1;
  }
}
