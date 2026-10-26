// 香橙派 <-> PX4 链路：接收遥测，唯一发送接口为 TIMESYNC 探针。
// MicoVIO 连接 PX4；本类不猜测模块私有协议，不向 PX4 回灌融合后的位姿。
#pragma once
#include "demo_aim/flight_state.hpp"
#include "demo_aim/localization.hpp"
#include <memory>
namespace aim {
struct LinkConfig {
  int system_id{1}, component_id{1}, own_system_id{42}, own_component_id{191};
  double max_pose_age_s{.2}, estimator_timeout_s{.5}, heartbeat_timeout_s{1.};
  double max_position_std_m{.5}, max_angle_std_rad{.35};
  // 本实现读取 PX4 fused ODOMETRY：四元数为 body-FRD -> parent。
  // 必须依据实际固件确认后才允许输出有效定位，默认仅诊断报文。
  bool px4_odometry_contract_confirmed{false};
  void validate() const;
};
struct LinkDiagnostics {
  std::uint64_t packets{}, bad_frames{}, foreign_packets{}, rejected_odometry{},
      epoch{1};
  std::uint64_t timesync_responses{}, timesync_accepted{},
      timesync_unmatched{}, timesync_wrong_target{}, timesync_bad_rtt{};
  int clock_sync_samples{};
  bool clock_ready{false}, estimator_ready{false}, reset_required{false};
  std::uint32_t last_message_id{};
  std::optional<int> odometry_frame_id, velocity_frame_id;
  std::optional<int> timesync_target_system, timesync_target_component;
  std::string reason{"no_odometry"};
  double clock_error_s{};
  Json json() const;
};
class MavlinkLink {
public:
  explicit MavlinkLink(LinkConfig config = {});
  ~MavlinkLink();
  MavlinkLink(const MavlinkLink &) = delete;
  MavlinkLink &operator=(const MavlinkLink &) = delete;
  // 一次最多 4096 bytes，支持半包/粘包；无效定位同样返回事件以清空历史。
  std::vector<PoseSample> feed(const std::uint8_t *bytes, std::size_t size,
                               double received_s);
  // 调用方用有界写发送；不发送任何飞行、解锁、参数或消息率命令。
  std::vector<std::uint8_t> timesyncProbe(double now);
  FlightStateSnapshot flightState(double now) const;
  LinkDiagnostics diagnostics(double now) const;
  // 飞控重启/重连后明确创建新会话；同步、估计器状态和旧位姿全部失效。
  void resetSession();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace aim
