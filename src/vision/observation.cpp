// schema=2 输出编码；JSONL/ROS 共享，未提供的量输出 null。
#include "demo_aim/observation.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>
namespace aim {
// 未标定是允许的二维运行状态，逐帧查询不通过异常传递。
bool Intrinsics::isValid() const noexcept {
  return width > 0 && height > 0 && fx > 0 && fy > 0 &&
         std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) &&
         std::isfinite(cy);
}
// 显式配置校验仍对非法输入抛出异常。
void Intrinsics::validate() const {
  if (!isValid())
    throw std::invalid_argument("Invalid camera intrinsics");
}
// 针孔反投影：X=(u-cx)Z/fx，Y=(v-cy)Z/fy；Z 是轴向深度而非欧氏距离。
cv::Vec3d Intrinsics::deproject(double u, double v, double z) const {
  return {(u - cx) * z / fx, (v - cy) * z / fy, z};
}
// 使用采集时刻而非发布时刻判断年龄，避免把旧图像的新消息当成新观测。
bool isDetectionCurrent(const Solution &s, double now, double max_age,
                        const std::string &domain) {
  const double age = now - s.capture_s;
  return s.detection_valid && s.time_domain == domain && std::isfinite(age) &&
         age >= 0 && std::isfinite(max_age) && max_age > 0 && age <= max_age;
}
bool isCurrent(const Solution &s, double now, double max_age,
               const std::string &domain) {
  return s.valid && s.position_valid &&
         isDetectionCurrent(s, now, max_age, domain);
}
// 单调时钟用于本机时效判断，避免系统时间校正使年龄跳变。
double monotonicNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

namespace {
// 空 optional 序列化为 null，避免将无效测量伪装成数值零。
template <typename T> Json optionalJson(const std::optional<T> &value) {
  return value ? Json(*value) : Json(nullptr);
}
// 三维向量写成 [x,y,z]；不存在时输出 null。
Json pointJson(const std::optional<cv::Vec3d> &p) {
  return p ? Json::array({(*p)[0], (*p)[1], (*p)[2]}) : Json(nullptr);
}
// 像素点写成 [u,v]；不存在时输出 null。
Json pixelJson(const std::optional<cv::Point2d> &p) {
  return p ? Json::array({p->x, p->y}) : Json(nullptr);
}
} // namespace
// 构造固定 schema 的诊断消息；contact_permitted 始终为 false。
Json Solution::json() const {
  Json j = {{"schema_version", 2},
            {"sequence", sequence},
            {"capture_monotonic_s", capture_s},
            {"emitted_monotonic_s", emitted_s},
            {"age_ms", age_ms},
            {"source", source},
            {"frame_id", frame_id},
            {"valid", valid},
            {"reason", reason},
            {"aligned", aligned},
            {"contact_permitted", false},
            {"track_id", optionalJson(track_id)},
            {"class_id", optionalJson(class_id)},
            {"confidence", optionalJson(confidence)},
            {"center_uv", pixelJson(center_uv)},
            {"pixel_error_uv", pixelJson(pixel_error)},
            {"point_camera_m", pointJson(point_camera)},
            {"point_body_m", pointJson(point_body)},
            {"yaw_error_rad", optionalJson(yaw_error_rad)},
            {"pitch_error_rad", optionalJson(pitch_error_rad)},
            {"depth_valid_ratio", optionalJson(depth_valid_ratio)},
            {"depth_mad_m", optionalJson(depth_mad_m)}};
  j["bbox_xyxy"] =
      box ? Json::array({box->x1, box->y1, box->x2, box->y2}) : Json(nullptr);
  j["detection_valid"] = detection_valid;
  j["bearing_valid"] = bearing_valid;
  j["position_valid"] = position_valid;
  j["time_domain"] = time_domain;
  j["received_monotonic_s"] = optionalJson(received_s);
  j["tracking_state"] = tracking_state;
  j["position_reason"] = position_reason;
  j["features_reason"] = features_reason;
  j["image_size_wh"] = {image_width, image_height};
  j["class_name"] = class_id ? (*class_id == 0   ? Json("target_red")
                                : *class_id == 1 ? Json("target_blue")
                                                 : Json(nullptr))
                             : Json(nullptr);
  j["bearing_camera_unit"] = pointJson(bearing_camera);
  j["image_velocity_px_s"] = pixelJson(image_velocity_px_s);
  j["body_frame_id"] = point_body ? Json("body_flu") : Json(nullptr);
  j["position_method"] =
      position_valid ? Json("aligned_depth_roi") : Json(nullptr);
  j["optical_depth_m"] =
      point_camera ? Json((*point_camera)[2]) : Json(nullptr);
  j["range_m"] = point_camera ? Json(cv::norm(*point_camera)) : Json(nullptr);
  j["position_covariance_m2"] = nullptr; // 尚无标定协方差，不能用置信度冒充
  j["light_bars_uv"] = nullptr;
  j["corners_uv"] = nullptr;
  j["geometry_layout_id"] = nullptr;
  if (features) {
    if (!features->layout_id.empty())
      j["geometry_layout_id"] = features->layout_id;
    if (features->light_bars) {
      j["light_bars_uv"] = Json::array();
      for (const auto &bar : *features->light_bars)
        j["light_bars_uv"].push_back(
            {{bar.top.x, bar.top.y}, {bar.bottom.x, bar.bottom.y}});
    }
    if (features->corners) {
      j["corners_uv"] = Json::array();
      for (const auto &p : *features->corners)
        j["corners_uv"].push_back({p.x, p.y});
    }
  }
  return j;
}
} // namespace aim
