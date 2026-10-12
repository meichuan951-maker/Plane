// 本机统一时钟下的自身位姿、有限历史与目标世界坐标转换。
#pragma once
#include "demo_aim/geometry.hpp"
#include "demo_aim/observation.hpp"
#include <deque>
namespace aim {
struct PoseSample {
  double capture_s{}, received_s{}, time_error_s{};
  std::string time_domain{"host_monotonic"};
  std::uint64_t epoch{1}; // EKF 重置/重启后增加，旧 map 对齐必须失效
  std::string odom_frame{"odom_enu"};
  cv::Vec3d position_m{};
  cv::Vec4d body_flu_to_odom_wxyz{1, 0, 0, 0};
  std::optional<cv::Vec3d> velocity_odom_mps;
  bool valid{false};
  std::string reason{"uninitialized"};
  RigidTransform odomFromBody() const;
  Json json() const;
};
struct HistoryConfig {
  std::size_t capacity{256};
  double duration_s{2.}, max_gap_s{.1}, max_time_error_s{.01};
};
// 单线程使用；不外推；无效输入会中断历史，不能跨越定位失效插值。
class PoseHistory {
public:
  explicit PoseHistory(HistoryConfig config = {});
  bool push(const PoseSample &sample);
  std::optional<PoseSample> at(double capture_s,
                               const std::string &domain) const;
  void clear();
  std::size_t size() const { return samples_.size(); }

private:
  HistoryConfig config_;
  std::uint64_t epoch_floor_{}; // clear 后仍拒绝旧会话的迟到数据
  std::deque<PoseSample> samples_;
};
struct WorldCalibration {
  std::optional<RigidTransform> body_T_camera; // FLU <- 光学 RDF，未知保持空
  std::optional<RigidTransform> map_T_odom; // 世界原点/朝向对齐，不由 USB 产生
  std::uint64_t epoch{};                    // 与哪一代定位对齐
  std::string odom_frame, camera_frame{"camera_color_optical_frame"};
};
// 复用同一 map 对齐给自身位姿；失败返回 valid=false，不自动采用单位阵。
PoseSample poseInMap(const PoseSample &, const WorldCalibration &);
struct WorldTarget {
  bool valid{false};
  std::string reason{"uninitialized"}, time_domain, frame_id{"map"};
  double capture_s{};
  std::uint64_t epoch{};
  std::optional<int> track_id;
  std::optional<cv::Vec3d> point_map_m;
  Json json() const;
};
// p_map = map_T_odom * odom_T_body(t_capture) * body_T_camera * p_camera。
// 必须已有米制目标点；单目框/视线/对地 ToF 高度都不能替代目标距离。
WorldTarget projectTargetToMap(const Solution &, const PoseHistory &,
                               const WorldCalibration &, double now,
                               double max_age_s = .2,
                               const std::string &domain = "host_monotonic");
} // namespace aim
