#include "demo_aim/localization.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace aim {
RigidTransform PoseSample::odomFromBody() const {
  RigidTransform t{quaternionRotation(body_flu_to_odom_wxyz), position_m};
  t.validate();
  return t;
}
Json PoseSample::json() const {
  Json j = {{"valid", valid},
            {"reason", reason},
            {"capture_s", capture_s},
            {"received_s", received_s},
            {"time_domain", time_domain},
            {"time_error_s", time_error_s},
            {"epoch", epoch},
            {"frame_id", odom_frame},
            {"body_frame_id", "body_flu"},
            {"position_m", nullptr},
            {"orientation_wxyz", nullptr},
            {"velocity_mps", nullptr}};
  if (valid) {
    j["position_m"] = {position_m[0], position_m[1], position_m[2]};
    const auto &q = body_flu_to_odom_wxyz;
    j["orientation_wxyz"] = {q[0], q[1], q[2], q[3]};
    if (velocity_odom_mps) {
      const auto &v = *velocity_odom_mps;
      j["velocity_mps"] = {v[0], v[1], v[2]};
    }
  }
  return j;
}
PoseHistory::PoseHistory(HistoryConfig c) : config_(c) {
  if (c.capacity < 2 || c.capacity > 100000 || !std::isfinite(c.duration_s) ||
      c.duration_s <= 0 || !std::isfinite(c.max_gap_s) || c.max_gap_s <= 0 ||
      !std::isfinite(c.max_time_error_s) || c.max_time_error_s < 0)
    throw std::invalid_argument("Invalid pose history limits");
}
void PoseHistory::clear() { samples_.clear(); }
bool PoseHistory::push(const PoseSample &s) {
  if (s.epoch < epoch_floor_)
    return false;
  epoch_floor_ = s.epoch;
  if (!s.valid) {
    clear();
    return false;
  }
  try {
    s.odomFromBody();
  } catch (const std::invalid_argument &) {
    clear();
    return false;
  }
  if (!std::isfinite(s.capture_s) || s.capture_s < 0 ||
      !std::isfinite(s.received_s) || s.received_s < s.capture_s ||
      !std::isfinite(s.time_error_s) || s.time_error_s < 0 ||
      s.time_error_s > config_.max_time_error_s || s.time_domain.empty() ||
      s.odom_frame.empty() || !s.epoch ||
      (s.velocity_odom_mps && !finiteVector(*s.velocity_odom_mps))) {
    clear();
    return false;
  }
  if (!samples_.empty()) {
    const auto &last = samples_.back();
    if (s.epoch < last.epoch)
      return false; // 拒绝上一会话迟到包
    if (s.epoch != last.epoch || s.time_domain != last.time_domain ||
        s.odom_frame != last.odom_frame)
      clear();
    else if (s.capture_s <= last.capture_s || s.received_s < last.received_s)
      return false;
  }
  samples_.push_back(s);
  while (samples_.size() > config_.capacity ||
         s.capture_s - samples_.front().capture_s > config_.duration_s)
    samples_.pop_front();
  return true;
}
std::optional<PoseSample> PoseHistory::at(double t,
                                          const std::string &domain) const {
  if (!std::isfinite(t) || samples_.empty() ||
      domain != samples_.back().time_domain || t < samples_.front().capture_s ||
      t > samples_.back().capture_s)
    return {};
  const auto b = std::lower_bound(
      samples_.begin(), samples_.end(), t,
      [](const PoseSample &s, double time) { return s.capture_s < time; });
  if (b->capture_s == t)
    return *b;
  const auto a = std::prev(b);
  const double dt = b->capture_s - a->capture_s;
  if (dt > config_.max_gap_s)
    return {};
  const double r = (t - a->capture_s) / dt;
  auto s = *a;
  s.capture_s = t;
  s.received_s = b->received_s;
  s.position_m = a->position_m * (1 - r) + b->position_m * r;
  s.body_flu_to_odom_wxyz =
      slerp(a->body_flu_to_odom_wxyz, b->body_flu_to_odom_wxyz, r);
  s.time_error_s = std::max(a->time_error_s, b->time_error_s);
  s.velocity_odom_mps.reset();
  if (a->velocity_odom_mps && b->velocity_odom_mps)
    s.velocity_odom_mps =
        *a->velocity_odom_mps * (1 - r) + *b->velocity_odom_mps * r;
  s.reason = "interpolated";
  return s;
}
PoseSample poseInMap(const PoseSample &pose, const WorldCalibration &c) {
  auto out = pose;
  out.valid = false;
  out.reason = "map_alignment_missing_or_expired";
  if (!pose.valid || !c.map_T_odom || pose.epoch != c.epoch ||
      pose.odom_frame != c.odom_frame)
    return out;
  try {
    const auto t = *c.map_T_odom * pose.odomFromBody();
    out.position_m = t.translation;
    out.body_flu_to_odom_wxyz = rotationQuaternion(t.rotation);
    if (pose.velocity_odom_mps)
      out.velocity_odom_mps = c.map_T_odom->rotation * *pose.velocity_odom_mps;
    out.odom_frame = "map";
    out.valid = true;
    out.reason = "ok";
  } catch (const std::invalid_argument &) {
    out.reason = "invalid_map_alignment";
  }
  return out;
}
Json WorldTarget::json() const {
  Json j = {{"valid", valid},
            {"reason", reason},
            {"frame_id", frame_id},
            {"time_domain", time_domain},
            {"capture_s", capture_s},
            {"epoch", epoch},
            {"track_id", track_id ? Json(*track_id) : Json(nullptr)},
            {"point_map_m", nullptr},
            {"position_covariance_m2", nullptr}};
  if (valid && point_map_m) {
    const auto &p = *point_map_m;
    j["point_map_m"] = {p[0], p[1], p[2]};
  }
  return j;
}
WorldTarget projectTargetToMap(const Solution &v, const PoseHistory &h,
                               const WorldCalibration &c, double now,
                               double max_age, const std::string &domain) {
  WorldTarget out;
  out.capture_s = v.capture_s;
  out.time_domain = v.time_domain;
  out.track_id = v.track_id;
  auto fail = [&](const char *why) {
    out.reason = why;
    return out;
  };
  if (!std::isfinite(max_age) || max_age <= 0 ||
      !isCurrent(v, now, max_age, domain) || !v.point_camera ||
      !finiteVector(*v.point_camera) || (*v.point_camera)[2] <= 0)
    return fail("target_position_unavailable_or_stale");
  if (!c.body_T_camera || !c.map_T_odom)
    return fail("calibration_missing");
  if (v.frame_id != c.camera_frame)
    return fail("camera_frame_mismatch");
  const auto pose = h.at(v.capture_s, domain);
  if (!pose)
    return fail("pose_not_bracketed");
  if (pose->received_s > now)
    return fail("pose_from_future");
  out.epoch = pose->epoch;
  if (c.epoch != pose->epoch || c.odom_frame != pose->odom_frame)
    return fail("map_alignment_expired");
  try {
    out.point_map_m = (*c.map_T_odom * pose->odomFromBody() * *c.body_T_camera)
                          .apply(*v.point_camera);
  } catch (const std::invalid_argument &) {
    return fail("invalid_calibration");
  }
  out.valid = true;
  out.reason = "ok";
  return out;
}
} // namespace aim
