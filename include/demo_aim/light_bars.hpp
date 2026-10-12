// 传统红蓝灯条与装甲四边形。有本相机内参时才解装甲朝向，不推断数字身份。
#pragma once
#include <array>
#include <iosfwd>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>
namespace aim {
struct LightBarConfig {
  int min_saturation{80}, min_value{160};
  int red_hue_low_max{12}, red_hue_high_min{168};
  int blue_hue_min{90}, blue_hue_max{135};
  double min_area{12}, min_length{8}, min_aspect{2}, max_aspect{30};
  double max_tilt_deg{40}, min_fill{0.45};
  // 亮度轮廓保留过曝核心，用轮廓内部有色像素投票；false 使用原 HSV 掩膜。
  bool brightness_color{true};
  double min_color_fraction{0.08}, min_color_dominance{0.75};
  double pair_min_ratio{0.8}, pair_max_ratio{5.0};
  double pair_max_length_ratio{1.5}, pair_max_y_offset{0.5};
  double pair_max_angle_deg{15}, pair_max_rectangular_error_deg{25};
  double pair_min_score{0.25}; // 几何评分，不是数字分类置信度。
  double pose_max_error_px{3.0}, pose_max_error_ratio{0.1};
  // 小装甲灯条模型，与 sp_vision_25 的 SMALL_ARMOR 一致，不是外壳 135×125。
  double armor_width_m{0.135}, lightbar_length_m{0.056};
  bool has_camera{false};
  int camera_width{0}, camera_height{0};
  double fx{}, fy{}, cx{}, cy{}, k1{}, k2{}, p1{}, p2{}, k3{};
  static LightBarConfig load(const std::string &path);
  void write(std::ostream &stream) const;
  void validate() const;
};
struct ColorLightBar {
  int class_id{}; // 0=red, 1=blue；仅表示颜色，不是装甲类别判断
  cv::RotatedRect rect;
  cv::Point2f top, bottom;
};
struct ArmorPose {
  double yaw_deg{}, pitch_deg{};
  double reprojection_error_px{};
  bool ambiguous{false};
  cv::Vec3d position_camera_m{}; // RGB 光学系，X 右/Y 下/Z 前，米。
};
struct ArmorCandidate {
  int class_id{};
  cv::Rect2f box;
  cv::Point2f center;
  // 左上、右上、右下、左下，对应两根灯条的上下端点。
  std::array<cv::Point2f, 4> corners{};
  bool reported{false}; // 最靠近画面中心的一块，只有它显示角度
  bool pose_valid{false};
  double yaw_deg{}, pitch_deg{};
  double geometry_score{};
  double reprojection_error_px{};
  bool pose_ambiguous{false};
  cv::Vec3d position_camera_m{};
};
struct LightBarResult {
  std::vector<ColorLightBar> bars;
  std::vector<ArmorCandidate> armors;
  cv::Mat red_mask, blue_mask;
};
// 同色、近似平行、相近高度的灯条配对；每根灯条至多参与一组。
std::vector<ArmorCandidate>
pairLightBars(const std::vector<ColorLightBar> &bars,
              const LightBarConfig &config = LightBarConfig{});
// 相机系装甲朝向。正对且板面平行于像平面时为 0。
// yaw 为正：法线偏向画面右侧（装甲左侧更近）。pitch
// 为正：法线偏向上方（下沿更近）。 没有 has_camera
// 时返回空，不使用其它相机的焦距。
std::optional<ArmorPose>
solveArmorPose(const std::array<cv::Point2f, 4> &corners,
               const LightBarConfig &config, cv::Size image_size);
LightBarResult detectLightBars(const cv::Mat &bgr,
                               const LightBarConfig &config);
cv::Mat drawLightBars(const cv::Mat &bgr, const LightBarResult &result);
} // namespace aim
