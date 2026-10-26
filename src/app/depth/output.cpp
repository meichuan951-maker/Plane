#include "demo_aim/detail/app/depth/output.hpp"
#include <cmath>
namespace aim::depth_app {
bool currentFrame(const FrameContext &context) {
  return context.has_image && !context.failed && std::isfinite(context.age_s) &&
         context.age_s >= 0 && context.age_s <= .25;
}
std::string frameReason(const FrameContext &context) {
  if (context.failed)
    return "depth_stream_failed";
  if (!currentFrame(context))
    return "stale_or_missing_depth";
  return context.reason;
}
nlohmann::json makePacket(const FrameContext &context, const cv::Rect &roi,
                          bool center_depth, bool local_world_fixed,
                          const std::optional<DepthPlane> &measurement) {
  const bool valid = currentFrame(context) && measurement.has_value();
  const double age = context.age_s;
  const bool age_known = std::isfinite(age) && age >= 0 && age < 100;
  nlohmann::json j{
      {"source", center_depth ? "center_depth_surface" : "manual_depth_roi"},
      {"tracking", false},
      {"calibration", "sdk_depth_projection_unverified"},
      {"frame", context.frame},
      {"fps", context.fps},
      {"generation", context.generation},
      {"valid", valid},
      {"reason", frameReason(context)},
      {"frame_age_ms",
       age_known ? nlohmann::json(age * 1000) : nlohmann::json(nullptr)},
      {"sample_time_monotonic_s",
       age_known ? nlohmann::json(context.monotonic_s - age)
                 : nlohmann::json(nullptr)},
      {"roi", {roi.x, roi.y, roi.width, roi.height}},
      {"xyz_depth_optical_m", nullptr},
      {"position_local_world_m", nullptr},
      {"world_frame", local_world_fixed
                          ? nlohmann::json("fixed_depth_camera_anchor")
                          : nlohmann::json(nullptr)},
      {"world_axes", "X forward, Y left, Z up"},
      {"world_requires_stationary_camera", true},
      {"armor_identity_verified", false},
      {"z_m", nullptr},
      {"range_m", nullptr},
      {"normal_yaw_deg", nullptr},
      {"normal_pitch_deg", nullptr},
      {"bearing_yaw_deg", nullptr},
      {"bearing_pitch_deg", nullptr},
      {"plane_rmse_m", nullptr}};
  if (valid) {
    const auto &p = *measurement;
    j["xyz_depth_optical_m"] = {p.centroid_m[0], p.centroid_m[1],
                                p.centroid_m[2]};
    // 用户定义的固定局部世界系：原点不动，仅做右手系轴变换。
    // 不是 VIO/地图定位；相机一旦移动，该坐标系假设即失效。
    if (local_world_fixed)
      j["position_local_world_m"] = {p.centroid_m[2], -p.centroid_m[0],
                                     -p.centroid_m[1]};
    j["z_m"] = p.centroid_m[2];
    j["range_m"] = p.range_m;
    j["normal_yaw_deg"] = p.normal_yaw_deg;
    j["normal_pitch_deg"] = p.normal_pitch_deg;
    j["bearing_yaw_deg"] = p.bearing_yaw_deg;
    j["bearing_pitch_deg"] = p.bearing_pitch_deg;
    j["plane_rmse_m"] = p.rmse_m;
  }
  return j;
}
nlohmann::json stoppedPacket(nlohmann::json packet, int frame, bool failed) {
  packet["valid"] = false;
  packet["reason"] = failed ? "depth_stream_failed" : "stopped";
  packet["frame"] = frame;
  for (const char *key :
       {"xyz_depth_optical_m", "position_local_world_m", "z_m", "range_m",
        "normal_yaw_deg", "normal_pitch_deg", "bearing_yaw_deg",
        "bearing_pitch_deg", "plane_rmse_m", "frame_age_ms",
        "sample_time_monotonic_s"})
    packet[key] = nullptr;
  return packet;
}
} // namespace aim::depth_app
