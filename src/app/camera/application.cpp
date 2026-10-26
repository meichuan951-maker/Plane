// 普通 UVC 相机独立验收入口：预览/检测在主进程，取流/控制在受监控子进程。
#include "demo_aim/detail/app/camera/application.hpp"
#include "demo_aim/light_bars.hpp"
#include "demo_aim/detail/app/camera/options.hpp"
#include "demo_aim/detail/app/camera/preview.hpp"
#include "demo_aim/detail/app/camera/snapshot.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <thread>

namespace aim::camera_app {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
double seconds(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double>(b - a).count();
}
struct WindowCleanup {
  bool show;
  ~WindowCleanup() {
    if (show)
      try {
        cv::destroyAllWindows();
      } catch (...) {
      }
  }
};
} // namespace
int run(int argc, char **argv) {
  try {
    std::cout << std::unitbuf;
    if (argc == 1) {
      help();
      return 0;
    }
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--help") {
        help();
        return 0;
      }
      if (std::string(argv[i]) == "--list") {
        listDevices();
        return 0;
      }
    }
    const auto o = parse(argc, argv);
    auto config = aim::LightBarConfig::load(o.config);
    if (!config.has_camera)
      std::cout << "No camera_matrix for this camera; armor quads are drawn, "
                   "yaw/pitch stay n/a.\n";
    if (!o.save_dir.empty() && fs::exists(o.save_dir))
      throw std::invalid_argument("--save-dir must be a new directory");
    if (o.show && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY"))
      throw std::runtime_error("No desktop display; run in desktop terminal or "
                               "add --no-show --frames 120");
    cv::setNumThreads(2);
    stopped = 0;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    const std::string window("USB camera - red/blue light bar candidates");
    WindowCleanup cleanup{o.show};
    if (o.show) {
      cv::namedWindow(window, cv::WINDOW_NORMAL);
      cv::resizeWindow(window, 960, 720);
      if (o.masks) {
        cv::namedWindow("Color masks", cv::WINDOW_NORMAL);
        cv::createTrackbar("Min V", "Color masks", nullptr, 255);
        cv::createTrackbar("Min S", "Color masks", nullptr, 255);
        cv::setTrackbarPos("Min V", "Color masks", config.min_value);
        cv::setTrackbarPos("Min S", "Color masks", config.min_saturation);
      }
    }
    std::unique_ptr<aim::CameraStream> camera;
    cv::Mat raw, overlay, masks;
    if (!o.image.empty()) {
      raw = cv::imread(o.image);
      if (raw.empty())
        throw std::runtime_error("Cannot read image: " + o.image);
    } else {
      aim::StreamOptions settings;
      settings.device = o.device;
      settings.format = o.format;
      settings.width = o.width;
      settings.height = o.height;
      settings.fps = o.fps;
      settings.controls = o.show && o.controls;
      settings.reconnect_seconds = o.reconnect_seconds;
      camera = std::make_unique<aim::CameraStream>(settings);
      std::cout << "Preview starts without exposure waits or focus scans; f "
                   "scans, a enables AF, e recovers a dark frame.\n";
    }
    CameraControls controls; // 只存子进程读回的状态，不在主进程打开设备。
    FocusScan scan;
    bool focus_slider = false, gain_slider = false;
    int focus_seen = 0, gain_seen = 0;
    std::uint64_t last_ack = 0, pending_focus = 0, pending_gain = 0,
                  generation = 0;
    auto started = Clock::now(), report = started,
         last_status = started - std::chrono::seconds(1);
    auto last_control = started - std::chrono::seconds(1);
    bool first_frame = true;
    int count = 0, report_count = 0;
    double fps = 0;
    aim::LightBarResult result;
    while (!stopped && (o.frames == 0 || count < o.frames)) {
      bool fresh = true;
      if (camera) {
        const auto state = camera->poll(raw, controls);
        if (state == aim::CameraStream::Result::Failed)
          throw std::runtime_error(camera->message());
        fresh = state == aim::CameraStream::Result::Frame;
        if (camera->generation() != generation) {
          generation = camera->generation();
          scan.cancel();
          last_ack = pending_focus = pending_gain = 0;
          fps = 0;
          report = Clock::now();
          report_count = count;
        }
        if (!fresh && camera->frameAge() > 0.25) {
          raw.release();
          overlay.release();
          if (o.show && seconds(last_status, Clock::now()) >= 0.1) {
            cv::Mat status(480, 640, CV_8UC3, cv::Scalar(20, 20, 20));
            cv::putText(status, "Waiting for fresh camera frames", {12, 35},
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 200, 255}, 1,
                        cv::LINE_AA);
            cv::putText(status, camera->message().substr(0, 76), {12, 65},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, {220, 220, 220}, 1,
                        cv::LINE_AA);
            cv::putText(status, "q / Esc: exit; reconnecting automatically",
                        {12, 95}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        {220, 220, 220}, 1, cv::LINE_AA);
            cv::imshow(window, status);
            if (o.masks)
              cv::imshow("Color masks", cv::Mat::zeros(480, 1280, CV_8UC1));
            last_status = Clock::now();
          }
        }
      }
      if (o.show && o.masks) {
        config.min_value = cv::getTrackbarPos("Min V", "Color masks");
        config.min_saturation = cv::getTrackbarPos("Min S", "Color masks");
      }
      if (camera && o.show && camera->controlsReady()) {
        if (controls.has_focus && !focus_slider) {
          cv::createTrackbar("Focus", window, nullptr, controls.focus_max);
          cv::setTrackbarPos("Focus", window, controls.focus_value);
          focus_seen = controls.focus_value;
          focus_slider = true;
        }
        if (controls.has_gain && !gain_slider) {
          cv::createTrackbar("Gain", window, nullptr, controls.gain_max);
          cv::setTrackbarPos("Gain", window, controls.gain_value);
          gain_seen = controls.gain_value;
          gain_slider = true;
        }
        // 拖动滑块时节流，避免每一像素变化都触发 USB 控制传输。
        if (seconds(last_control, Clock::now()) >= 0.1) {
          const int focus =
              focus_slider ? cv::getTrackbarPos("Focus", window) : focus_seen;
          const int gain =
              gain_slider ? cv::getTrackbarPos("Gain", window) : gain_seen;
          if (focus_slider && focus != focus_seen) {
            scan.cancel();
            const auto id = camera->command(aim::CameraCommand::Focus, focus);
            if (id) {
              pending_focus = id;
              focus_seen = focus;
              last_control = Clock::now();
            }
          }
          if (gain_slider && gain != gain_seen) {
            const auto id = camera->command(aim::CameraCommand::Gain, gain);
            if (id) {
              pending_gain = id;
              gain_seen = gain;
              last_control = Clock::now();
            }
          }
        }
        scan.tick(*camera, controls, fresh ? raw : cv::Mat());
        if (camera->acknowledged() != last_ack) {
          last_ack = camera->acknowledged();
          if (!camera->commandSucceeded())
            std::cerr
                << "Camera control rejected; inspect worker warning above\n";
          if (focus_slider && last_ack >= pending_focus) {
            cv::setTrackbarPos("Focus", window, controls.focus_value);
            focus_seen = controls.focus_value;
            pending_focus = 0;
          }
          if (gain_slider && last_ack >= pending_gain) {
            cv::setTrackbarPos("Gain", window, controls.gain_value);
            gain_seen = controls.gain_value;
            pending_gain = 0;
          }
        }
      } else if (scan.active())
        scan.cancel();
      if (fresh) {
        if (first_frame) {
          std::cout << "First processed frame after "
                    << seconds(started, Clock::now()) << " s\n";
          first_frame = false;
        }
        const auto detect_start = Clock::now();
        result = aim::detectLightBars(raw, config);
        const double detect_ms = 1000 * seconds(detect_start, Clock::now());
        ++count;
        const auto now = Clock::now();
        const bool reporting = seconds(report, now) >= 1;
        if (reporting) {
          fps = (count - report_count) / seconds(report, now);
          report = now;
          report_count = count;
        }
        int red = 0, blue = 0;
        for (const auto &bar : result.bars)
          (bar.class_id == 0 ? red : blue)++;
        std::ostringstream label;
        label << (camera ? "LIVE " : "STILL ") << raw.cols << 'x' << raw.rows;
        if (camera)
          label << " | FPS " << std::fixed << std::setprecision(1) << fps;
        label << " | detect " << std::fixed << std::setprecision(1) << detect_ms
              << " ms"
              << " | RED " << red << " BLUE " << blue << " | ARMOR "
              << result.armors.size();
        for (const auto &armor : result.armors)
          if (armor.reported) {
            if (armor.pose_valid) {
              label << " | yaw " << armor.yaw_deg << " pitch "
                    << armor.pitch_deg << " err " << armor.reprojection_error_px
                    << "px";
              if (armor.pose_ambiguous)
                label << " ambiguous";
            } else
              label << " | pose n/a";
          }
        if (reporting)
          std::cout << label.str() << '\n';
        if (o.show || !o.save_dir.empty()) {
          overlay = aim::drawLightBars(raw, result);
          std::ostringstream camera_line;
          if (camera && camera->controlsReady()) {
            camera_line << "focus ";
            if (controls.focus_status == "auto")
              camera_line << "auto";
            else
              camera_line << controls.focus_value << ' '
                          << controls.focus_status;
            camera_line << " | gain " << controls.gain_value
                        << " | f scan/a AF/e exposure";
          }
          if (scan.active())
            camera_line << " | " << scan.label();
          const int banner = camera_line.str().empty() ? 28 : 46;
          cv::rectangle(overlay, {0, 0}, {overlay.cols, banner}, {20, 20, 20},
                        -1);
          cv::putText(overlay, label.str(), {8, 20}, cv::FONT_HERSHEY_SIMPLEX,
                      0.46, {255, 255, 255}, 1, cv::LINE_AA);
          if (!camera_line.str().empty())
            cv::putText(overlay, camera_line.str(), {8, 40},
                        cv::FONT_HERSHEY_SIMPLEX, 0.46, {255, 255, 255}, 1,
                        cv::LINE_AA);
          if (o.show)
            cv::imshow(window, overlay);
        }
        if (o.show && o.masks) {
          cv::hconcat(result.red_mask, result.blue_mask, masks);
          cv::imshow("Color masks", masks);
        }
      }
      int key = -1;
      if (o.show) {
        key = cv::waitKey(5) & 255;
        if (cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) == 0)
          break;
      } else if (!fresh)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if (key == 's' && !raw.empty() && !overlay.empty() &&
          (!camera || camera->frameAge() <= 0.25)) {
        const auto stamp =
            std::chrono::duration_cast<std::chrono::microseconds>(
                Clock::now().time_since_epoch())
                .count();
        save(fs::path("runs/camera_snapshots") / std::to_string(stamp), raw,
             overlay, result, config,
             camera && camera->controlsReady() ? &controls : nullptr);
      }
      if (camera && key == 'f')
        scan.start(*camera, controls);
      if (camera && (key == 'a' || key == 'e')) {
        scan.cancel();
        camera->command(key == 'a' ? aim::CameraCommand::AutoFocus
                                   : aim::CameraCommand::ExposureRecovery,
                        1);
      }
      if (key == 'q' || key == 27 || (!camera && !o.show))
        break;
    }
    const double elapsed = seconds(started, Clock::now());
    if (!o.save_dir.empty() && count > 0 && !raw.empty() &&
        (!camera || camera->frameAge() <= 0.25))
      save(o.save_dir, raw, overlay, result, config,
           camera && camera->controlsReady() ? &controls : nullptr);
    if (camera)
      camera->stop();
    std::cout << "Completed " << count << " frames";
    if (camera && elapsed > 0)
      std::cout << ", mean loop FPS " << count / elapsed;
    std::cout << "; armor yaw/pitch require this camera's calibration, no "
                 "flight control.\n";
    return stopped ? 130 : 0;
  } catch (const std::exception &error) {
    if (stopped)
      return 130;
    std::cerr << "camera_light_test: " << error.what() << '\n';
    return 1;
  }
}
} // namespace aim::camera_app
