// 确定性的自主任务决策层；输出意图，不连接飞控、不发送解锁/电机指令。
#pragma once
#include "demo_aim/localization.hpp"
#include <functional>
namespace aim {
enum class MissionPhase {
  Idle,
  Takeoff,
  Search,
  Approach,
  Contact,
  Recover,
  Return,
  Land,
  Complete,
  Abort
};
const char *phaseName(MissionPhase phase);
struct MissionConfig {
  // 必须由场地测量/规划提供，禁止默认把未知机库设为 (0,0,0)。
  cv::Vec3d home_map_m, takeoff_map_m, recovery_map_m;
  std::vector<cv::Vec3d> return_route_map_m;
  double reach_tolerance_m{.15}, stand_off_m{1.}, max_pose_age_s{.2};
  double max_target_age_s{.2}, max_state_age_s{.5}, stage_timeout_s{30.};
  double lost_target_timeout_s{.5};
  std::string time_domain{"host_monotonic"};
  void validate() const;
};
struct MissionInput {
  double now{};
  PoseSample vehicle_map; // frame 必须是 map；使用 map_T_odom 变换过
  WorldTarget target;
  // 这些是最新执行器/飞控状态，不从检测框是否存在推断。
  double flight_state_s{};
  bool flight_healthy{false}, landed{false}, manual_override{false};
  bool start_requested{false}, contact_authorized{false},
      contact_region_verified{false};
  std::optional<double> contact_confirmed_s;
  // 对整段路径和机体包络做碰撞验证。缺少验证器时不允许移动。
  std::function<bool(const cv::Vec3d &, const cv::Vec3d &)> segment_clear;
};
struct MissionIntent {
  MissionPhase phase{MissionPhase::Idle};
  std::string action{"none"}, reason{"idle"};
  std::optional<cv::Vec3d> waypoint_map_m;
  bool executable{false}; // 始终 false：尚无真实控制器/飞控指令适配
  Json json() const;
};
class MissionMachine {
public:
  explicit MissionMachine(MissionConfig config);
  MissionIntent step(const MissionInput &input);
  void reset(); // 任务完成/中止不自动重新起飞
private:
  MissionConfig config_;
  MissionPhase phase_{MissionPhase::Idle};
  double entered_s_{}, last_now_{-1}, last_target_s_{};
  std::uint64_t epoch_{};
  std::size_t route_index_{};
  bool airborne_seen_{false};
  std::optional<int> target_id_;
  std::optional<cv::Vec3d> approach_point_;
  void enter(MissionPhase phase, double now);
};
} // namespace aim
