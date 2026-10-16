#include "demo_aim/mission.hpp"
#include <cmath>
#include <stdexcept>
namespace aim {
const char *phaseName(MissionPhase s) {
  switch (s) {
  case MissionPhase::Idle:
    return "idle";
  case MissionPhase::Takeoff:
    return "takeoff";
  case MissionPhase::Search:
    return "search";
  case MissionPhase::Approach:
    return "approach";
  case MissionPhase::Contact:
    return "contact";
  case MissionPhase::Recover:
    return "recover";
  case MissionPhase::Return:
    return "return";
  case MissionPhase::Land:
    return "land";
  case MissionPhase::Complete:
    return "complete";
  case MissionPhase::Abort:
    return "abort";
  }
  return "unknown";
}
void MissionConfig::validate() const {
  if (!finiteVector(home_map_m) || !finiteVector(takeoff_map_m) ||
      !finiteVector(recovery_map_m) || return_route_map_m.empty() ||
      return_route_map_m.size() > 1000 || time_domain.empty())
    throw std::invalid_argument(
        "Mission requires surveyed home and bounded return route");
  for (const auto &p : return_route_map_m)
    if (!finiteVector(p))
      throw std::invalid_argument("Invalid return waypoint");
  for (double x :
       {reach_tolerance_m, stand_off_m, max_pose_age_s, max_target_age_s,
        max_state_age_s, stage_timeout_s, lost_target_timeout_s})
    if (!std::isfinite(x) || x <= 0)
      throw std::invalid_argument("Invalid mission limit");
  if (takeoff_map_m[2] <= home_map_m[2] || stand_off_m <= reach_tolerance_m)
    throw std::invalid_argument(
        "Takeoff must be above home; stand-off must exceed tolerance");
}
Json MissionIntent::json() const {
  Json j = {{"phase", phaseName(phase)}, {"action", action},
            {"reason", reason},          {"frame_id", "map"},
            {"waypoint_map_m", nullptr}, {"executable", false}};
  if (waypoint_map_m) {
    const auto &p = *waypoint_map_m;
    j["waypoint_map_m"] = {p[0], p[1], p[2]};
  }
  return j;
}
MissionMachine::MissionMachine(MissionConfig c) : config_(std::move(c)) {
  config_.validate();
}
void MissionMachine::reset() {
  phase_ = MissionPhase::Idle;
  last_now_ = -1;
  epoch_ = 0;
  route_index_ = 0;
  airborne_seen_ = false;
  target_id_.reset();
  approach_point_.reset();
}
void MissionMachine::enter(MissionPhase p, double now) {
  phase_ = p;
  entered_s_ = now;
}
MissionIntent MissionMachine::step(const MissionInput &i) {
  auto out = [&](std::string action, std::string why,
                 std::optional<cv::Vec3d> p = std::nullopt) {
    return MissionIntent{phase_, std::move(action), std::move(why), p, false};
  };
  auto abort = [&](const char *why) {
    enter(MissionPhase::Abort, i.now);
    return out("handover", why);
  };
  if (phase_ == MissionPhase::Abort || phase_ == MissionPhase::Complete)
    return out("none", "terminal_latched");
  if (!std::isfinite(i.now) || i.now < 0 || i.now <= last_now_)
    return abort("invalid_step_time");
  last_now_ = i.now;
  if (i.manual_override)
    return abort("manual_override");
  if (phase_ == MissionPhase::Idle && !i.start_requested)
    return out("none", "waiting_start");
  auto fresh = [&](double stamp, double age) {
    return std::isfinite(stamp) && stamp >= 0 && stamp <= i.now &&
           i.now - stamp <= age;
  };
  bool valid_pose = i.vehicle_map.valid && i.vehicle_map.odom_frame == "map" &&
                    i.vehicle_map.time_domain == config_.time_domain &&
                    fresh(i.vehicle_map.capture_s, config_.max_pose_age_s) &&
                    i.vehicle_map.received_s <= i.now &&
                    i.vehicle_map.time_error_s <= .01;
  try {
    i.vehicle_map.validate();
  } catch (const std::invalid_argument &) {
    valid_pose = false;
  }
  if (!valid_pose) {
    if (phase_ == MissionPhase::Idle)
      return out("none", "waiting_localization");
    return abort("localization_unavailable");
  }
  if (!fresh(i.flight_state_s, config_.max_state_age_s) || !i.flight_healthy)
    return phase_ == MissionPhase::Idle ? out("none", "waiting_flight_health")
                                        : abort("flight_state_unhealthy");
  if (phase_ != MissionPhase::Idle && i.vehicle_map.epoch != epoch_)
    return abort("localization_reset");
  // 起飞后意外落地必须锁存中止，不能继续接近、接触或自动再次起飞。
  if (phase_ != MissionPhase::Idle && !i.landed)
    airborne_seen_ = true;
  if (i.landed && airborne_seen_ && phase_ != MissionPhase::Idle &&
      phase_ != MissionPhase::Land)
    return abort("unexpected_landing");
  const auto &position = i.vehicle_map.position_m;
  auto reached = [&](const cv::Vec3d &p) {
    return cv::norm(position - p) <= config_.reach_tolerance_m;
  };
  auto move = [&](const cv::Vec3d &p, const char *action = "waypoint") {
    if (!i.segment_clear || !i.segment_clear(position, p))
      return out("hold", "segment_not_validated");
    return out(action, "planned_intent", p);
  };
  const bool target_ok = i.target.valid && i.target.frame_id == "map" &&
                         i.target.time_domain == config_.time_domain &&
                         i.target.epoch == epoch_ &&
                         fresh(i.target.capture_s, config_.max_target_age_s) &&
                         i.target.track_id && i.target.point_map_m &&
                         finiteVector(*i.target.point_map_m);
  if (phase_ == MissionPhase::Idle) {
    if (!i.landed || !reached(config_.home_map_m))
      return out("none", "start_requires_home_and_landed");
    epoch_ = i.vehicle_map.epoch;
    enter(MissionPhase::Takeoff, i.now);
  } else if (i.now - entered_s_ > config_.stage_timeout_s)
    return abort("stage_timeout");
  switch (phase_) {
  case MissionPhase::Takeoff:
    if (reached(config_.takeoff_map_m) && !i.landed) {
      enter(MissionPhase::Search, i.now);
      return out("hold", "searching");
    }
    return move(config_.takeoff_map_m, "takeoff_intent");
  case MissionPhase::Search:
    if (!target_ok)
      return out("hold", "waiting_metric_target");
    target_id_ = i.target.track_id;
    last_target_s_ = i.now;
    enter(MissionPhase::Approach, i.now);
    return out("hold", "target_acquired");
  case MissionPhase::Approach: {
    if (!target_ok || i.target.track_id != target_id_) {
      if (i.now - last_target_s_ > config_.lost_target_timeout_s)
        enter(MissionPhase::Recover, i.now);
      return out("hold", "target_lost_or_changed");
    }
    last_target_s_ = i.now;
    // 站位点随目标更新；仅是几何意图，不是加速度/速度控制律。
    const auto delta = *i.target.point_map_m - position;
    const double distance = cv::norm(delta);
    if (distance < config_.stand_off_m - config_.reach_tolerance_m)
      return out("hold", "inside_stand_off");
    approach_point_ =
        *i.target.point_map_m - delta * (config_.stand_off_m / distance);
    if (reached(*approach_point_)) {
      if (!i.contact_authorized || !i.contact_region_verified)
        return out("hold", "waiting_contact_authorization_and_region");
      enter(MissionPhase::Contact, i.now);
      return out("request_contact_controller",
                 "awaiting_external_contact_execution");
    }
    return move(*approach_point_);
  }
  case MissionPhase::Contact:
    if (!i.contact_authorized || !i.contact_region_verified) {
      enter(MissionPhase::Recover, i.now);
      return out("hold", "contact_permission_revoked");
    }
    // 接触由独立检测/裁判/控制器确认；不能用目标丢失或惯导冲击直接替代。
    if (i.contact_confirmed_s && *i.contact_confirmed_s >= entered_s_ &&
        fresh(*i.contact_confirmed_s, config_.max_state_age_s)) {
      enter(MissionPhase::Recover, i.now);
      return out("hold", "contact_confirmed");
    }
    if (!target_ok || i.target.track_id != target_id_) {
      enter(MissionPhase::Recover, i.now);
      return out("hold", "contact_target_lost");
    }
    return out("request_contact_controller", "awaiting_contact_feedback");
  case MissionPhase::Recover:
    if (reached(config_.recovery_map_m)) {
      enter(MissionPhase::Return, i.now);
      route_index_ = 0;
      return out("hold", "recovered");
    }
    return move(config_.recovery_map_m);
  case MissionPhase::Return:
    if (reached(config_.return_route_map_m[route_index_])) {
      if (++route_index_ == config_.return_route_map_m.size()) {
        enter(MissionPhase::Land, i.now);
        return out("hold", "return_route_finished");
      }
      entered_s_ = i.now;
    }
    return move(config_.return_route_map_m[route_index_]);
  case MissionPhase::Land:
    if (i.landed && reached(config_.home_map_m)) {
      enter(MissionPhase::Complete, i.now);
      return out("none", "landed_at_home");
    }
    return move(config_.home_map_m, "land_intent");
  default:
    return out("none", "idle");
  }
}
} // namespace aim
