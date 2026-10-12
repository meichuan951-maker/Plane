#pragma once
#include "demo_aim/light_bars.hpp"
#include <cstdint>
#include <optional>
namespace aim {
struct ArmorAutoConfig {
  int target_color{2}; // 用户协议：1 蓝，2 红；检测器内部仍是 0 红/1 蓝。
  std::string device, format, calibration_path;
  int width{}, height{}, fps{}, confirmation_frames{};
  double lost_seconds{}, association_distance_ratio{}, association_size_ratio{};
  bool dimensions_confirmed{false}, calibrated{false};
  LightBarConfig detector;
  static ArmorAutoConfig load(const std::string &path);
};
struct TrackedArmor {
  std::optional<ArmorCandidate> target;
  std::uint64_t id{};
  bool confirmed{false};
};
class ArmorTracker {
public:
  explicit ArmorTracker(const ArmorAutoConfig &config);
  TrackedArmor update(const std::vector<ArmorCandidate> &candidates,
                      cv::Size image_size, double time_s);
  void reset();

private:
  ArmorAutoConfig config_;
  std::optional<ArmorCandidate> previous_;
  std::uint64_t id_{0};
  int hits_{0};
  double seen_{-1}, clock_{-1};
};
} // namespace aim
