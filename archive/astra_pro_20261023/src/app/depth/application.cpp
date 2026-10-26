#include "demo_aim/detail/app/depth/application.hpp"
#include "demo_aim/detail/input/astra_depth_worker.hpp"
#include "demo_aim/detail/input/uvc_stream.hpp"
#include "demo_aim/detail/app/common/frame_rate.hpp"
#include "demo_aim/detail/app/depth/options.hpp"
#include "demo_aim/detail/app/depth/output.hpp"
#include "demo_aim/detail/app/depth/preview.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <thread>
#include <utility>
namespace aim::depth_app {
namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
struct Window {
  bool show;
  ~Window() {
    if (show)
      cv::destroyAllWindows();
  }
};
} // namespace
int run(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--camera-worker")
    return aim::astraDepthWorkerMain();
  try {
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--help") {
        help(std::cout);
        return 0;
      }
    stopped = 0;
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
    Selection selection{o.roi, {}, {}, false, o.center_depth};
    const std::string window = "Astra Pro - LIVE depth XYZ / local world";
    Window cleanup{o.show};
    if (o.show) {
      cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
      cv::setMouseCallback(window, mouse, &selection);
    }
    std::cerr
        << (o.center_depth
                ? "Center probe: aim DEPTH crosshair inside armor surface. "
                : "Drag inside armor in the DEPTH image. ")
        << "Approximate SDK projection; no RGB association/armor identity. "
           "c: center probe; r: manual ROI.\n";
    cv::Mat xyz, view;
    std::optional<aim::DepthPlane> measurement;
    cv::Rect last_roi;
    std::string reason = "waiting_for_depth";
    int count = 0, valid = 0;
    std::uint64_t generation = 0;
    const auto start = Clock::now();
    auto next_report = start, next_draw = start;
    auto next_snapshot = start;
    app::FrameRate frame_rate(start);
    bool failed = false;
    nlohmann::json last_packet;
    while (!stopped && (!o.frames || count < o.frames) &&
           (!o.seconds ||
            Clock::now() - start < std::chrono::seconds(o.seconds))) {
      const auto state = camera.poll(xyz, controls);
      if (camera.generation() != generation) {
        if (generation)
          selection.roi = {}; // 重连后重新确认目标，不沿用旧区域。
        generation = camera.generation();
      }
      // 中心探针测当前表面，不依赖/猜测 RGB-D 对应关系。
      if (selection.center_depth)
        selection.roi = {304, 224, 32, 32};
      if (state == aim::CameraStream::Result::Frame) {
        ++count;
      }
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
      // Processing time counts toward freshness; output and UI use this
      // snapshot.
      const auto now = Clock::now();
      const double age = camera.frameAge();
      FrameContext context{
          count,
          generation,
          frame_rate.update(now, state == aim::CameraStream::Result::Frame),
          !xyz.empty(),
          failed,
          std::chrono::duration<double>(now.time_since_epoch()).count(),
          age,
          reason};
      if (!currentFrame(context)) {
        measurement.reset();
        reason = context.reason = frameReason(context);
      }
      if (state == aim::CameraStream::Result::Frame && measurement)
        ++valid;
      if (now >= next_report || failed) {
        const auto j =
            makePacket(context, selection.roi, selection.center_depth,
                       o.local_world_fixed, measurement);
        output << j.dump() << '\n' << std::flush;
        last_packet = j;
        if (!output)
          throw std::runtime_error("Measurement output failed");
        next_report = now + std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(1. / o.hz));
      }
      if ((o.show || !o.snapshot.empty()) && now >= next_draw) {
        view = drawPreview(xyz, selection, context, o.local_world_fixed,
                           measurement);
        if (o.show)
          cv::imshow(window, view);
        if (!o.snapshot.empty() && now >= next_snapshot) {
          if (!cv::imwrite(o.snapshot, view))
            throw std::runtime_error("Cannot write diagnostic snapshot");
          next_snapshot = now + std::chrono::seconds(1);
        }
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
        if (key == 'c') {
          selection = {};
          selection.center_depth = true;
        }
      } else
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    camera.stop();
    if (!last_packet.is_null()) {
      last_packet = stoppedPacket(std::move(last_packet), count, failed);
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
} // namespace aim::depth_app
