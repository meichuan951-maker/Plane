#include "demo_aim/flight_state.hpp"
#include <cmath>
#include <stdexcept>
namespace aim {
namespace {
// 重复、乱序消息不延长有效期；boot 时间倒退要求调用方重建会话。
template <typename T>
bool store(std::optional<TimedTelemetry<T>> &slot, T value,
           TelemetryStamp stamp) {
  if (!std::isfinite(stamp.received_monotonic_s) ||
      stamp.received_monotonic_s < 0)
    throw std::invalid_argument("Invalid telemetry receive timestamp");
  if (slot && (stamp.received_monotonic_s <= slot->stamp.received_monotonic_s ||
               (stamp.remote_boot_ms && slot->stamp.remote_boot_ms &&
                *stamp.remote_boot_ms <= *slot->stamp.remote_boot_ms)))
    return false;
  slot = TimedTelemetry<T>{value, stamp};
  return true;
}
template <typename T>
void expire(std::optional<TimedTelemetry<T>> &slot, double now, double limit) {
  if (slot && (now < slot->stamp.received_monotonic_s ||
               now - slot->stamp.received_monotonic_s > limit))
    slot.reset();
}
Json stampJson(const TelemetryStamp &s, double now) {
  return {{"received_monotonic_s", s.received_monotonic_s},
          {"age_ms", (now - s.received_monotonic_s) * 1000},
          {"remote_boot_ms",
           s.remote_boot_ms ? Json(*s.remote_boot_ms) : Json(nullptr)}};
}
} // namespace
FlightStateCache::FlightStateCache(int sys, int comp, double heartbeat_timeout,
                                   double state_timeout)
    : system_id_(sys), component_id_(comp),
      heartbeat_timeout_s_(heartbeat_timeout), state_timeout_s_(state_timeout) {
  if (sys < 1 || sys > 255 || comp < 1 || comp > 255 ||
      !std::isfinite(heartbeat_timeout) || heartbeat_timeout <= 0 ||
      !std::isfinite(state_timeout) || state_timeout <= 0)
    throw std::invalid_argument("Invalid flight-state cache configuration");
}
bool FlightStateCache::matches(int sys, int comp) const {
  return sys == system_id_ && comp == component_id_;
}
bool FlightStateCache::updateHeartbeat(int sys, int comp, HeartbeatState value,
                                       TelemetryStamp stamp) {
  return matches(sys, comp) && store(latest_.heartbeat, value, stamp);
}
bool FlightStateCache::updateAttitude(int sys, int comp, AttitudeState value,
                                      TelemetryStamp stamp) {
  if (!matches(sys, comp))
    return false;
  const double norm = cv::norm(value.body_frd_to_ned_wxyz);
  if (!std::isfinite(norm) || std::abs(norm - 1.) > 1e-3)
    throw std::invalid_argument("Attitude must be a finite unit quaternion");
  return store(latest_.attitude, value, stamp);
}
bool FlightStateCache::updateLocalPosition(int sys, int comp,
                                           LocalPositionState value,
                                           TelemetryStamp stamp) {
  if (!matches(sys, comp))
    return false;
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(value.position_ned_m[i]) ||
        !std::isfinite(value.velocity_ned_mps[i]))
      throw std::invalid_argument("Non-finite local position or velocity");
  return store(latest_.local_position, value, stamp);
}
FlightStateSnapshot FlightStateCache::snapshot(double now) const {
  if (!std::isfinite(now) || now < 0)
    throw std::invalid_argument("Invalid snapshot timestamp");
  auto s = latest_;
  s.sampled_monotonic_s = now;
  s.system_id = system_id_;
  s.component_id = component_id_;
  expire(s.heartbeat, now, heartbeat_timeout_s_);
  expire(s.attitude, now, state_timeout_s_);
  expire(s.local_position, now, state_timeout_s_);
  return s;
}
void FlightStateCache::reset() { latest_ = FlightStateSnapshot{}; }
Json FlightStateSnapshot::json() const {
  Json j = {{"system_id", system_id},
            {"component_id", component_id},
            {"time_domain", "host_monotonic"},
            {"sampled_monotonic_s", sampled_monotonic_s},
            {"heartbeat", nullptr},
            {"attitude", nullptr},
            {"local_position", nullptr}};
  if (heartbeat) {
    auto v = stampJson(heartbeat->stamp, sampled_monotonic_s);
    v["armed"] = heartbeat->value.armed;
    v["custom_mode"] = heartbeat->value.custom_mode;
    j["heartbeat"] = v;
  }
  if (attitude) {
    auto v = stampJson(attitude->stamp, sampled_monotonic_s);
    const auto &q = attitude->value.body_frd_to_ned_wxyz;
    v["body_frd_to_ned_wxyz"] = {q[0], q[1], q[2], q[3]};
    j["attitude"] = v;
  }
  if (local_position) {
    auto v = stampJson(local_position->stamp, sampled_monotonic_s);
    const auto &p = local_position->value;
    v["frame_id"] = "local_ned";
    v["position_m"] = {p.position_ned_m[0], p.position_ned_m[1],
                       p.position_ned_m[2]};
    v["velocity_mps"] = {p.velocity_ned_mps[0], p.velocity_ned_mps[1],
                         p.velocity_ned_mps[2]};
    v["xy_valid"] = p.xy_valid;
    v["z_valid"] = p.z_valid;
    v["velocity_valid"] = p.velocity_valid;
    j["local_position"] = v;
  }
  return j;
}
} // namespace aim
