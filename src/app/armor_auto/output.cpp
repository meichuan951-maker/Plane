#include "demo_aim/detail/app/armor_auto/output.hpp"
#include <cmath>

namespace aim::armor_auto_app {
namespace {
constexpr double max_frame_age_s = .25;
} // namespace

FrameStatus evaluateFrame(const ArmorAutoConfig &config,
                          const TrackedArmor &target,
                          const FrameContext &context) {
  FrameStatus status;
  status.current = context.has_image && std::isfinite(context.age_s) &&
                   context.age_s >= 0 && context.age_s <= max_frame_age_s &&
                   !context.failed;
  if (!status.current)
    status.reason = context.failed ? "camera_failed" : "stale_or_missing_frame";
  else if (!target.target)
    status.reason = "target_lost_or_absent";
  else if (!target.confirmed)
    status.reason = "confirming_target";
  else if (!config.calibrated)
    status.reason = "rgb_calibration_missing";
  else if (!config.dimensions_confirmed)
    status.reason = "armor_dimensions_unconfirmed";
  else if (!target.target->pose_valid)
    status.reason = "pose_rejected";
  else if (target.target->pose_ambiguous)
    status.reason = "pose_ambiguous";
  else {
    status.reason = "ok";
    status.valid = true;
  }
  return status;
}

nlohmann::json makeReport(const ArmorAutoConfig &config,
                          const TrackedArmor &target,
                          const FrameContext &context) {
  const auto status = evaluateFrame(config, target, context);
  const double sample_time = context.monotonic_s - context.age_s;
  const bool age_known =
      std::isfinite(context.age_s) && context.age_s >= 0 && context.age_s < 100;
  nlohmann::json packet = {
      {"source", "automatic_armor_rgb_pnp"},
      {"coordinate_frame", "rgb_optical"},
      {"target_color", config.target_color},
      {"frame", context.frame},
      {"generation", context.generation},
      {"fps", context.fps},
      {"detection_valid", status.current && bool(target.target)},
      {"tracking_confirmed", status.current && target.confirmed},
      {"track_id", target.id},
      {"valid", status.valid},
      {"reason", status.reason},
      {"position_camera_m", nullptr},
      {"range_m", nullptr},
      {"z_m", nullptr},
      {"normal_yaw_deg", nullptr},
      {"normal_pitch_deg", nullptr},
      {"bearing_yaw_deg", nullptr},
      {"bearing_pitch_deg", nullptr},
      {"reprojection_error_px", nullptr},
      {"armor_corners_px", nullptr},
      {"sample_time_monotonic_s",
       age_known ? nlohmann::json(sample_time) : nlohmann::json(nullptr)},
      {"valid_until_monotonic_s",
       status.valid ? nlohmann::json(sample_time + max_frame_age_s)
                    : nlohmann::json(nullptr)},
      {"frame_age_ms", age_known ? nlohmann::json(context.age_s * 1000)
                                 : nlohmann::json(nullptr)}};
  if (status.current && target.target) {
    packet["armor_corners_px"] = nlohmann::json::array();
    for (const auto &point : target.target->corners)
      packet["armor_corners_px"].push_back({point.x, point.y});
  }
  if (status.valid) {
    const auto &armor = *target.target;
    const auto &position = armor.position_camera_m;
    packet["position_camera_m"] = {position[0], position[1], position[2]};
    packet["range_m"] = cv::norm(position);
    packet["z_m"] = position[2];
    packet["normal_yaw_deg"] = armor.yaw_deg;
    packet["normal_pitch_deg"] = armor.pitch_deg;
    packet["bearing_yaw_deg"] =
        std::atan2(position[0], position[2]) * 180 / CV_PI;
    packet["bearing_pitch_deg"] =
        std::atan2(-position[1], std::hypot(position[0], position[2])) * 180 /
        CV_PI;
    packet["reprojection_error_px"] = armor.reprojection_error_px;
  }
  return packet;
}

void invalidateReport(nlohmann::json &packet, int frame, bool failed) {
  packet["valid"] = false;
  packet["detection_valid"] = false;
  packet["tracking_confirmed"] = false;
  packet["reason"] = failed ? "camera_failed" : "stopped";
  packet["frame"] = frame;
  for (const char *key :
       {"position_camera_m", "range_m", "z_m", "normal_yaw_deg",
        "normal_pitch_deg", "bearing_yaw_deg", "bearing_pitch_deg",
        "reprojection_error_px", "armor_corners_px", "frame_age_ms",
        "sample_time_monotonic_s", "valid_until_monotonic_s"})
    packet[key] = nullptr;
}

} // namespace aim::armor_auto_app
