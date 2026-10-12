#include "demo_aim/armor_auto.hpp"
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace aim {
namespace {
void keys(const YAML::Node &node, std::initializer_list<const char *> allowed) {
  if (!node.IsMap())
    throw std::invalid_argument("Config section must be a map");
  for (const auto &e : node) {
    const auto key = e.first.as<std::string>();
    bool found = false;
    for (auto a : allowed)
      if (key == a)
        found = true;
    if (!found)
      throw std::invalid_argument("Unknown auto config key: " + key);
  }
}
std::string relative(const std::filesystem::path &base,
                     const std::string &path) {
  if (path.empty())
    return {};
  const std::filesystem::path p(path);
  return p.is_absolute() ? p.string() : (base / p).lexically_normal().string();
}
} // namespace
ArmorAutoConfig ArmorAutoConfig::load(const std::string &path) {
  const auto node = YAML::LoadFile(path);
  keys(node, {"target_color", "camera", "detector_config", "rgb_calibration",
              "armor", "tracking"});
  ArmorAutoConfig c;
  c.target_color = node["target_color"].as<int>();
  if (c.target_color != 1 && c.target_color != 2)
    throw std::invalid_argument("target_color must be 1 (blue) or 2 (red)");
  const auto base = std::filesystem::path(path).parent_path();
  c.detector = LightBarConfig::load(
      relative(base, node["detector_config"].as<std::string>()));
  // 主配置决定相机来源，避免 detector 文件中残留其它相机的内参。
  c.detector.has_camera = false;
  const auto camera = node["camera"];
  keys(camera, {"device", "width", "height", "fps", "format"});
  c.device = camera["device"].as<std::string>();
  c.width = camera["width"].as<int>();
  c.height = camera["height"].as<int>();
  c.fps = camera["fps"].as<int>();
  c.format = camera["format"].as<std::string>();
  if (c.device.empty() || c.width < 16 || c.height < 16 || c.width > 4096 ||
      c.height > 4096 || c.fps < 1 || c.fps > 120 ||
      (c.format != "YUYV" && c.format != "MJPG"))
    throw std::invalid_argument("Invalid auto camera settings");
  const auto armor = node["armor"];
  keys(armor, {"lightbar_center_distance_m", "lightbar_length_m",
               "dimensions_confirmed"});
  c.detector.armor_width_m = armor["lightbar_center_distance_m"].as<double>();
  c.detector.lightbar_length_m = armor["lightbar_length_m"].as<double>();
  c.dimensions_confirmed = armor["dimensions_confirmed"].as<bool>();
  const auto track = node["tracking"];
  keys(track, {"confirmation_frames", "lost_seconds",
               "association_distance_ratio", "association_size_ratio"});
  c.confirmation_frames = track["confirmation_frames"].as<int>();
  c.lost_seconds = track["lost_seconds"].as<double>();
  c.association_distance_ratio =
      track["association_distance_ratio"].as<double>();
  c.association_size_ratio = track["association_size_ratio"].as<double>();
  if (c.confirmation_frames < 2 || c.confirmation_frames > 30 ||
      !std::isfinite(c.lost_seconds) || c.lost_seconds <= 0 ||
      c.lost_seconds > 2 || !std::isfinite(c.association_distance_ratio) ||
      c.association_distance_ratio <= 0 || c.association_distance_ratio > 3 ||
      !std::isfinite(c.association_size_ratio) ||
      c.association_size_ratio < 1 || c.association_size_ratio > 4)
    throw std::invalid_argument("Invalid auto tracking settings");
  c.calibration_path =
      relative(base, node["rgb_calibration"].as<std::string>());
  if (!c.calibration_path.empty() &&
      std::filesystem::exists(c.calibration_path)) {
    const auto p = YAML::LoadFile(c.calibration_path);
    if (p["camera_device"] && p["camera_device"].as<std::string>() != c.device)
      throw std::invalid_argument(
          "RGB calibration camera_device does not match config");
    if (p["camera_width"].as<int>() != c.width ||
        p["camera_height"].as<int>() != c.height)
      throw std::invalid_argument(
          "RGB calibration resolution does not match camera settings");
    const auto matrix = p["camera_matrix"].as<std::vector<double>>();
    const auto distortion = p["distort_coeffs"].as<std::vector<double>>();
    if (matrix.size() != 9 || distortion.size() != 5 || matrix[1] != 0 ||
        matrix[3] != 0 || matrix[6] != 0 || matrix[7] != 0 || matrix[8] != 1)
      throw std::invalid_argument("Invalid RGB pinhole calibration layout");
    for (double v : matrix)
      if (!std::isfinite(v))
        throw std::invalid_argument("Nonfinite RGB calibration");
    for (double v : distortion)
      if (!std::isfinite(v))
        throw std::invalid_argument("Nonfinite RGB distortion");
    c.detector.has_camera = true;
    c.detector.camera_width = c.width;
    c.detector.camera_height = c.height;
    c.detector.fx = matrix[0];
    c.detector.fy = matrix[4];
    c.detector.cx = matrix[2];
    c.detector.cy = matrix[5];
    c.detector.k1 = distortion[0];
    c.detector.k2 = distortion[1];
    c.detector.p1 = distortion[2];
    c.detector.p2 = distortion[3];
    c.detector.k3 = distortion[4];
    if (c.detector.fx < c.width * .1 || c.detector.fx > c.width * 10 ||
        c.detector.fy < c.height * .1 || c.detector.fy > c.height * 10 ||
        c.detector.cx < 0 || c.detector.cx > c.width || c.detector.cy < 0 ||
        c.detector.cy > c.height)
      throw std::invalid_argument("Implausible RGB intrinsics");
    c.calibrated = true;
  }
  c.detector.validate();
  return c;
}
} // namespace aim
