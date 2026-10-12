#include "demo_aim/light_bars.hpp"
#include <algorithm>
#include <cmath>
#include <ostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace aim {
LightBarConfig LightBarConfig::load(const std::string &path) {
  LightBarConfig c;
  const auto node = YAML::LoadFile(path);
  if (!node.IsMap())
    throw std::invalid_argument("Light bar config must be a map");
  // 拼写错误也应报错，避免用户以为调整已经生效。
  for (const auto &entry : node) {
    const auto key = entry.first.as<std::string>();
    if (key == "min_saturation")
      c.min_saturation = entry.second.as<int>();
    else if (key == "min_value")
      c.min_value = entry.second.as<int>();
    else if (key == "red_hue_low_max")
      c.red_hue_low_max = entry.second.as<int>();
    else if (key == "red_hue_high_min")
      c.red_hue_high_min = entry.second.as<int>();
    else if (key == "blue_hue_min")
      c.blue_hue_min = entry.second.as<int>();
    else if (key == "blue_hue_max")
      c.blue_hue_max = entry.second.as<int>();
    else if (key == "min_area")
      c.min_area = entry.second.as<double>();
    else if (key == "min_length")
      c.min_length = entry.second.as<double>();
    else if (key == "min_aspect")
      c.min_aspect = entry.second.as<double>();
    else if (key == "max_aspect")
      c.max_aspect = entry.second.as<double>();
    else if (key == "max_tilt_deg")
      c.max_tilt_deg = entry.second.as<double>();
    else if (key == "min_fill")
      c.min_fill = entry.second.as<double>();
    else if (key == "brightness_color")
      c.brightness_color = entry.second.as<bool>();
    else if (key == "min_color_fraction")
      c.min_color_fraction = entry.second.as<double>();
    else if (key == "min_color_dominance")
      c.min_color_dominance = entry.second.as<double>();
    else if (key == "pair_min_ratio")
      c.pair_min_ratio = entry.second.as<double>();
    else if (key == "pair_max_ratio")
      c.pair_max_ratio = entry.second.as<double>();
    else if (key == "pair_max_length_ratio")
      c.pair_max_length_ratio = entry.second.as<double>();
    else if (key == "pair_max_y_offset")
      c.pair_max_y_offset = entry.second.as<double>();
    else if (key == "pair_max_angle_deg")
      c.pair_max_angle_deg = entry.second.as<double>();
    else if (key == "pair_max_rectangular_error_deg")
      c.pair_max_rectangular_error_deg = entry.second.as<double>();
    else if (key == "pair_min_score")
      c.pair_min_score = entry.second.as<double>();
    else if (key == "pose_max_error_px")
      c.pose_max_error_px = entry.second.as<double>();
    else if (key == "pose_max_error_ratio")
      c.pose_max_error_ratio = entry.second.as<double>();
    else if (key == "armor_width_m")
      c.armor_width_m = entry.second.as<double>();
    else if (key == "lightbar_length_m")
      c.lightbar_length_m = entry.second.as<double>();
    else if (key == "camera_width")
      c.camera_width = entry.second.as<int>();
    else if (key == "camera_height")
      c.camera_height = entry.second.as<int>();
    else if (key == "camera_matrix") {
      const auto values = entry.second.as<std::vector<double>>();
      if (values.size() != 9 ||
          !std::all_of(values.begin(), values.end(),
                       [](double v) { return std::isfinite(v); }) ||
          std::abs(values[1]) > 1e-9 || std::abs(values[3]) > 1e-9 ||
          std::abs(values[6]) > 1e-9 || std::abs(values[7]) > 1e-9 ||
          std::abs(values[8] - 1) > 1e-6)
        throw std::invalid_argument(
            "camera_matrix must be [fx, 0, cx, 0, fy, cy, 0, 0, 1]");
      c.fx = values[0];
      c.cx = values[2];
      c.fy = values[4];
      c.cy = values[5];
      c.has_camera = true;
    } else if (key == "distort_coeffs") {
      const auto values = entry.second.as<std::vector<double>>();
      if (values.size() != 5)
        throw std::invalid_argument("distort_coeffs must have 5 values");
      c.k1 = values[0];
      c.k2 = values[1];
      c.p1 = values[2];
      c.p2 = values[3];
      c.k3 = values[4];
    } else
      throw std::invalid_argument("Unknown light bar config key: " + key);
  }
  c.validate();
  return c;
}
void LightBarConfig::write(std::ostream &stream) const {
  validate();
  YAML::Emitter out;
  out.SetDoublePrecision(17);
  out << YAML::BeginMap;
  const auto put = [&](const char *key, auto value) {
    out << YAML::Key << key << YAML::Value << value;
  };
  put("min_saturation", min_saturation);
  put("min_value", min_value);
  put("red_hue_low_max", red_hue_low_max);
  put("red_hue_high_min", red_hue_high_min);
  put("blue_hue_min", blue_hue_min);
  put("blue_hue_max", blue_hue_max);
  put("min_area", min_area);
  put("min_length", min_length);
  put("min_aspect", min_aspect);
  put("max_aspect", max_aspect);
  put("max_tilt_deg", max_tilt_deg);
  put("min_fill", min_fill);
  put("brightness_color", brightness_color);
  put("min_color_fraction", min_color_fraction);
  put("min_color_dominance", min_color_dominance);
  put("pair_min_ratio", pair_min_ratio);
  put("pair_max_ratio", pair_max_ratio);
  put("pair_max_length_ratio", pair_max_length_ratio);
  put("pair_max_y_offset", pair_max_y_offset);
  put("pair_max_angle_deg", pair_max_angle_deg);
  put("pair_max_rectangular_error_deg", pair_max_rectangular_error_deg);
  put("pair_min_score", pair_min_score);
  put("pose_max_error_px", pose_max_error_px);
  put("pose_max_error_ratio", pose_max_error_ratio);
  put("armor_width_m", armor_width_m);
  put("lightbar_length_m", lightbar_length_m);
  if (camera_width > 0) {
    put("camera_width", camera_width);
    put("camera_height", camera_height);
  }
  if (has_camera) {
    out << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow
        << std::vector<double>{fx, 0, cx, 0, fy, cy, 0, 0, 1};
    out << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow
        << std::vector<double>{k1, k2, p1, p2, k3};
  }
  out << YAML::EndMap;
  if (!out.good())
    throw std::runtime_error("Could not serialize light bar config");
  stream << out.c_str() << '\n';
}
void LightBarConfig::validate() const {
  if (min_saturation < 0 || min_saturation > 255 || min_value < 0 ||
      min_value > 255 || red_hue_low_max < 0 ||
      red_hue_low_max >= blue_hue_min || blue_hue_min > blue_hue_max ||
      blue_hue_max >= red_hue_high_min || red_hue_high_min > 179 ||
      !std::isfinite(min_area) || min_area <= 0 || !std::isfinite(min_length) ||
      min_length <= 0 || !std::isfinite(min_aspect) || min_aspect <= 1 ||
      !std::isfinite(max_aspect) || max_aspect < min_aspect ||
      !std::isfinite(max_tilt_deg) || max_tilt_deg < 0 || max_tilt_deg > 90 ||
      !std::isfinite(min_fill) || min_fill <= 0 || min_fill > 1)
    throw std::invalid_argument("Invalid light bar thresholds/geometry");
  const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
  if (!positive(min_color_fraction) || min_color_fraction > 1 ||
      !positive(min_color_dominance) || min_color_dominance <= 0.5 ||
      min_color_dominance > 1 || !positive(pair_min_ratio) ||
      !positive(pair_max_ratio) || pair_max_ratio < pair_min_ratio ||
      !positive(pair_max_length_ratio) || pair_max_length_ratio <= 1 ||
      !positive(pair_max_y_offset) || !positive(pair_max_angle_deg) ||
      pair_max_angle_deg > 90 || !positive(pair_max_rectangular_error_deg) ||
      pair_max_rectangular_error_deg > 90 || !positive(pair_min_score) ||
      pair_min_score > 1 || !positive(pose_max_error_px) ||
      !positive(pose_max_error_ratio) || pose_max_error_ratio > 1)
    throw std::invalid_argument(
        "Invalid color, armor pairing or pose thresholds");
  if (!std::isfinite(armor_width_m) || armor_width_m <= 0 ||
      armor_width_m > 1 || !std::isfinite(lightbar_length_m) ||
      lightbar_length_m <= 0 || lightbar_length_m > 1)
    throw std::invalid_argument("Invalid armor light-bar geometry");
  if ((camera_width > 0) != (camera_height > 0) || camera_width < 0 ||
      camera_height < 0)
    throw std::invalid_argument(
        "camera_width and camera_height must both be set");
  if (has_camera &&
      (!positive(fx) || !positive(fy) || !std::isfinite(cx) ||
       !std::isfinite(cy) || !std::isfinite(k1) || !std::isfinite(k2) ||
       !std::isfinite(p1) || !std::isfinite(p2) || !std::isfinite(k3)))
    throw std::invalid_argument("Invalid camera intrinsics");
}
} // namespace aim
