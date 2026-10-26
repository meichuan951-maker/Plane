#include "demo_aim/light_bar_observation.hpp"

namespace aim {
std::vector<Detection> detectArmorObservations(const cv::Mat &bgr,
                                              const LightBarConfig &config) {
  // 这里只提取二维特征，不把其他相机的 PnP 内参带进 RGB-D 测距。
  auto detector_config = config;
  detector_config.has_camera = false;
  const auto result = detectLightBars(bgr, detector_config);
  std::vector<Detection> detections;
  detections.reserve(result.armors.size());
  for (const auto &armor : result.armors) {
    Detection detection;
    detection.box = {armor.box.x, armor.box.y,
                     armor.box.x + armor.box.width,
                     armor.box.y + armor.box.height};
    detection.class_id = armor.class_id;
    detection.confidence = armor.geometry_score;
    ArmorFeatures features;
    std::array<cv::Point2d, 4> corners;
    for (std::size_t index = 0; index < corners.size(); ++index)
      corners[index] = armor.corners[index];
    features.corners = corners;
    features.light_bars = std::array<LightBar, 2>{
        LightBar{corners[0], corners[3]}, LightBar{corners[1], corners[2]}};
    features.layout_id = "lightbar_center_quad_v1";
    detection.features = features;
    detections.push_back(detection);
  }
  return detections;
}
} // namespace aim
