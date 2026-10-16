#include "preview.hpp"
#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

namespace aim::armor_auto_app {
namespace {
void line(cv::Mat &view, const std::string &text, int y, cv::Scalar color) {
  cv::putText(view, text, {8, y}, cv::FONT_HERSHEY_SIMPLEX, .43, color, 1,
              cv::LINE_AA);
}
} // namespace

cv::Mat drawPreview(const cv::Mat &raw, const LightBarResult &detection,
                    const TrackedArmor &target, const ArmorAutoConfig &config,
                    const FrameContext &context) {
  const auto status = evaluateFrame(config, target, context);
  cv::Mat overlay = status.current ? drawLightBars(raw, detection)
                                   : cv::Mat(config.height, config.width,
                                             CV_8UC3, cv::Scalar(20, 20, 20));
  if (status.current && target.target)
    cv::rectangle(overlay, target.target->box, {0, 255, 255}, 2);
  cv::rectangle(overlay, {0, 0}, {overlay.cols, 100}, {20, 20, 20}, cv::FILLED);
  line(overlay,
       "AUTO " + std::string(config.target_color == 1 ? "BLUE" : "RED") +
           " | track " + std::to_string(target.id) + " | " + status.reason,
       19, {0, 220, 255});
  std::ostringstream rate;
  rate << std::fixed << std::setprecision(1) << context.fps << " FPS | RGB "
       << config.width << "x" << config.height << " | q/Esc exit | r reacquire";
  line(overlay, rate.str(), 38, {230, 230, 230});
  if (status.valid) {
    const auto &armor = *target.target;
    std::ostringstream value;
    value << std::fixed << std::setprecision(3) << "Range "
          << cv::norm(armor.position_camera_m) << " m | Z "
          << armor.position_camera_m[2] << " m";
    line(overlay, value.str(), 58, {0, 255, 0});
    value.str("");
    value.clear();
    value << std::fixed << std::setprecision(1) << "Surface yaw "
          << armor.yaw_deg << " pitch " << armor.pitch_deg << " deg";
    line(overlay, value.str(), 78, {0, 255, 0});
  } else {
    line(overlay,
         !config.calibrated
             ? "RGB pose n/a: uncalibrated | distance: see DEPTH crosshair"
             : "RGB distance / yaw / pitch: n/a",
         62, {0, 180, 255});
  }
  return overlay;
}

void saveSnapshot(const std::string &directory, const cv::Mat &raw,
                  const LightBarResult &detection, const TrackedArmor &target,
                  const ArmorAutoConfig &config, const FrameContext &context) {
  if (raw.empty())
    throw std::invalid_argument("Cannot save an empty camera frame");
  const cv::Mat overlay = drawPreview(raw, detection, target, config, context);
  const std::filesystem::path path(directory);
  if (!std::filesystem::create_directories(path))
    throw std::invalid_argument("Snapshot directory already exists: " +
                                directory);
  if (!cv::imwrite((path / "raw.png").string(), raw) ||
      !cv::imwrite((path / "overlay.png").string(), overlay))
    throw std::runtime_error("Snapshot failed");
}

} // namespace aim::armor_auto_app
