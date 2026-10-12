// 飞控状态接口：只缓存解码后的遥测，不打开通信设备，不发送控制指令。
#pragma once
#include "demo_aim/observation.hpp"
namespace aim {
struct TelemetryStamp {
  double received_monotonic_s{}; // 本机 steady_clock 接收时刻
  std::optional<std::uint64_t> remote_boot_ms; // 仅保存，不与图像时间直接相减
};
struct HeartbeatState {
  bool armed{false};
  std::uint32_t custom_mode{}; // 原始值，解读依赖飞控固件
};
struct AttitudeState {
  cv::Vec4d body_frd_to_ned_wxyz{1, 0, 0, 0}; // 单位四元数，wxyz
};
struct LocalPositionState {
  cv::Vec3d position_ned_m{}, velocity_ned_mps{};
  // 需接收端根据估计器状态显式赋值，不能凭收到位置报文自动置真。
  bool xy_valid{false}, z_valid{false}, velocity_valid{false};
};
template <typename T> struct TimedTelemetry {
  T value;
  TelemetryStamp stamp;
};
// 三类消息独立超时；新心跳不能刷新旧姿态/位置。
struct FlightStateSnapshot {
  double sampled_monotonic_s{};
  int system_id{}, component_id{};
  std::optional<TimedTelemetry<HeartbeatState>> heartbeat;
  std::optional<TimedTelemetry<AttitudeState>> attitude;
  std::optional<TimedTelemetry<LocalPositionState>> local_position;
  Json json() const;
};
// 每份缓存绑定一个设备；换设备、飞控重启/boot 时间回绕时调用 reset。
// 仅保留最新值，不提供拍摄时刻插值；同一实例需由调用方串行访问。
class FlightStateCache {
public:
  FlightStateCache(int system_id = 1, int component_id = 1,
                   double heartbeat_timeout_s = 1.,
                   double state_timeout_s = .2);
  bool updateHeartbeat(int system_id, int component_id, HeartbeatState value,
                       TelemetryStamp stamp);
  bool updateAttitude(int system_id, int component_id, AttitudeState value,
                      TelemetryStamp stamp);
  bool updateLocalPosition(int system_id, int component_id,
                           LocalPositionState value, TelemetryStamp stamp);
  FlightStateSnapshot snapshot(double now) const;
  void reset();

private:
  int system_id_, component_id_;
  double heartbeat_timeout_s_, state_timeout_s_;
  FlightStateSnapshot latest_;
  bool matches(int system_id, int component_id) const;
};
} // namespace aim
