// 视觉核心实现：配置校验、目标关联、深度质量判断和三维观测；不发送飞控指令。
#include "demo_aim/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <set>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace aim {
namespace {
// 同时检查一组数，避免 NaN/Inf 绕过范围判断。
bool finite(std::initializer_list<double> values) {
  return std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); });
}
// 按值复制后排序计算中位数；调用方必须先保证样本非空。
double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const auto n = values.size();
  return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
}
} // namespace
// 集中校验配置，使 YAML 加载和直接构造 Pipeline 使用相同约束。
void Config::validate() const {
  if (class_ids.empty() || std::any_of(class_ids.begin(), class_ids.end(),
                                       [](int c) { return c < 0; }))
    throw std::invalid_argument("Invalid class_ids");
  for (double v : {confidence_min, depth_roi_fraction, depth_min_valid_ratio,
                   lock_iou_min})
    if (!std::isfinite(v) || v <= 0 || v > 1)
      throw std::invalid_argument("Ratio must be in (0,1]");
  for (double v :
       {min_box_px, depth_min_m, depth_max_m, depth_max_mad_m, max_frame_age_s,
        max_sync_error_s, lock_hold_s, aligned_tolerance_px})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Limits must be finite and positive");
  if (depth_max_m <= depth_min_m || depth_min_samples < 1 || confirm_frames < 1)
    throw std::invalid_argument("Invalid depth range or confirmation count");
  // 验证齐次矩阵末行、旋转正交性及 det(R)=+1，拒绝缩放或镜像矩阵。
  if (body_T_camera) {
    const auto &t = *body_T_camera;
    for (double v : t.val)
      if (!std::isfinite(v))
        throw std::invalid_argument("Non-finite extrinsics");
    for (int j = 0; j < 4; ++j)
      if (std::abs(t(3, j) - (j == 3 ? 1. : 0.)) > 1e-7)
        throw std::invalid_argument("Invalid transform row");
    cv::Matx33d r(t(0, 0), t(0, 1), t(0, 2), t(1, 0), t(1, 1), t(1, 2), t(2, 0),
                  t(2, 1), t(2, 2));
    if (cv::norm(cv::Mat(r.t() * r - cv::Matx33d::eye())) > 1e-6 ||
        std::abs(cv::determinant(r) - 1) > 1e-6)
      throw std::invalid_argument(
          "body_T_camera must be a calibrated rigid transform");
  }
}
// 读取 YAML；缺省项沿用 Config 默认值，未知键报错以发现参数拼写错误。
Config Config::load(const std::string &path) {
  Config c;
  const auto root = YAML::LoadFile(path);
  if (!root.IsMap())
    throw std::invalid_argument("Configuration must be a YAML mapping");
  std::set<std::string> known{"class_ids", "body_T_camera"};
// 宏只用于减少同类型字段的读取重复，同时记录允许的键名。
#define LOAD(name)                                                             \
  known.insert(#name);                                                         \
  if (root[#name])                                                             \
  c.name = root[#name].as<decltype(c.name)>()
  LOAD(confidence_min);
  LOAD(min_box_px);
  LOAD(depth_min_m);
  LOAD(depth_max_m);
  LOAD(depth_roi_fraction);
  LOAD(depth_min_valid_ratio);
  LOAD(depth_max_mad_m);
  LOAD(depth_min_samples);
  LOAD(confirm_frames);
  LOAD(max_frame_age_s);
  LOAD(max_sync_error_s);
  LOAD(lock_iou_min);
  LOAD(lock_hold_s);
  LOAD(aligned_tolerance_px);
#undef LOAD
  if (root["class_ids"])
    c.class_ids = root["class_ids"].as<std::vector<int>>();
  const auto transform = root["body_T_camera"];
  if (transform && !transform.IsNull()) {
    if (!transform.IsSequence() || transform.size() != 4)
      throw std::invalid_argument("Expected 4x4 transform");
    cv::Matx44d t;
    for (int i = 0; i < 4; ++i) {
      if (!transform[i].IsSequence() || transform[i].size() != 4)
        throw std::invalid_argument("Expected 4x4 transform");
      for (int j = 0; j < 4; ++j)
        t(i, j) = transform[i][j].as<double>();
    }
    c.body_T_camera = t;
  }
  for (const auto &item : root)
    if (!known.count(item.first.as<std::string>()))
      throw std::invalid_argument("Unknown config key: " +
                                  item.first.as<std::string>());
  c.validate();
  return c;
}
// 无重叠时交集为零；输入框在上游已过滤为正尺寸。
double iou(const Box &a, const Box &b) {
  const double area =
      std::max(0., std::min(a.x2, b.x2) - std::max(a.x1, b.x1)) *
      std::max(0., std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
  const double total =
      (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - area;
  return total > 0 ? area / total : 0.;
}
// 中心 ROI 比整框更少包含背景，但一致的背景仍可能通过检查，不能保证物体身份。
DepthEstimate estimateDepth(const cv::Mat &depth, const Box &b,
                            const Config &c) {
  DepthEstimate out;
  if (depth.type() != CV_32FC1)
    throw std::invalid_argument("Depth must be CV_32FC1 metres");
  // 按宽高比例取中心区域，并将像素索引裁到图像边界。
  const auto center = b.center();
  const double hw = (b.x2 - b.x1) * c.depth_roi_fraction / 2,
               hh = (b.y2 - b.y1) * c.depth_roi_fraction / 2;
  const int x1 = std::max(0, static_cast<int>(std::floor(center.x - hw)));
  const int x2 =
      std::min(depth.cols, static_cast<int>(std::ceil(center.x + hw)));
  const int y1 = std::max(0, static_cast<int>(std::floor(center.y - hh)));
  const int y2 =
      std::min(depth.rows, static_cast<int>(std::ceil(center.y + hh)));
  if (x1 >= x2 || y1 >= y2) {
    out.reason = "empty_depth_roi";
    return out;
  }
  std::vector<double> values;
  for (int y = y1; y < y2; ++y)
    for (int x = x1; x < x2; ++x) {
      const double v = depth.at<float>(y, x);
      if (std::isfinite(v) && v >= c.depth_min_m && v <= c.depth_max_m)
        values.push_back(v);
    }
  // 先排除缺失、非有限和超范围深度；样本数量及占比都必须合格。
  out.valid_ratio =
      static_cast<double>(values.size()) / ((x2 - x1) * (y2 - y1));
  if (values.size() < static_cast<std::size_t>(c.depth_min_samples) ||
      out.valid_ratio < c.depth_min_valid_ratio) {
    out.reason = "insufficient_depth";
    return out;
  }
  // 中位数抗少量离群值；MAD 反映样本的离散程度。
  const double z = median(values);
  std::vector<double> deviations;
  for (double v : values)
    deviations.push_back(std::abs(v - z));
  out.mad = median(deviations);
  if (*out.mad > c.depth_max_mad_m) {
    out.reason = "inconsistent_depth";
    return out;
  }
  // MAD 很小时使用绝对容差下限；若不足 80% 样本属于主层，视为前后景混合。
  std::vector<double> inliers;
  for (double v : values)
    if (std::abs(v - z) <= std::max(3 * (*out.mad), c.depth_max_mad_m))
      inliers.push_back(v);
  if (static_cast<double>(inliers.size()) / values.size() < .8) {
    out.reason = "mixed_depth_layers";
    return out;
  }
  out.depth = median(inliers);
  out.reason = "ok";
  return out;
}
// 保存配置副本并校验，防止运行过程中外部修改配置影响状态。
Pipeline::Pipeline(Config config) : config_(std::move(config)) {
  config_.validate();
}
// 清除当前关联和连续确认；保留最后采集时刻，仍能拒绝倒序/重复帧。
void Pipeline::resetLock() {
  target_.reset();
  last_seen_.reset();
  consecutive_ = 0;
  detection_consecutive_ = 0;
}
// 每次新建 Solution，因此失败帧不会携带上一帧三维坐标。
Solution Pipeline::process(const Frame &f,
                           const std::vector<Detection> &detections,
                           double now) {
  if (!finite({f.capture_s, now}))
    throw std::invalid_argument("Non-finite timestamps");
  Solution s;
  s.sequence = f.sequence;
  s.capture_s = f.capture_s;
  s.emitted_s = now;
  s.age_ms = (now - f.capture_s) * 1000;
  s.source = f.source;
  s.frame_id = f.frame_id;
  s.time_domain = f.time_domain;
  s.received_s = f.received_s;
  s.image_width = f.color_bgr.cols;
  s.image_height = f.color_bgr.rows;
  auto reject = [&](const std::string &reason, bool reset = false) {
    s.reason = s.position_reason = reason;
    consecutive_ = detection_consecutive_ = 0;
    if (reset)
      resetLock();
    if (target_) {
      s.track_id = track_id_;
      s.tracking_state = "lost";
    }
    return s;
  };
  // 来源/时钟/尺寸改变时，不继承旧锁定和确认计数。
  if (f.time_domain != time_domain_ || f.source != source_ ||
      f.frame_id != frame_id_ || f.color_bgr.size() != image_size_) {
    resetLock();
    last_capture_.reset();
    time_domain_ = f.time_domain;
    source_ = f.source;
    frame_id_ = f.frame_id;
    image_size_ = f.color_bgr.size();
  }
  if (f.time_domain != "host_monotonic" && f.time_domain != "replay_monotonic")
    return reject("unsupported_time_domain", true);
  if (!f.clock_valid)
    return reject("camera_clock_unsynchronized", true);
  if (now < f.capture_s || now - f.capture_s > config_.max_frame_age_s)
    return reject("stale_or_future_frame", true);
  if (f.received_s && (!finite({*f.received_s}) ||
                       *f.received_s < f.capture_s || *f.received_s > now))
    return reject("invalid_receive_time", true);
  if (last_capture_ && f.capture_s <= *last_capture_)
    return reject("non_monotonic_frame", true);
  last_capture_ = f.capture_s;
  if (f.color_bgr.empty() || f.color_bgr.type() != CV_8UC3)
    return reject("invalid_color_image", true);
  bool calibrated = false;
  try {
    f.intrinsics.validate();
    calibrated =
        cv::Size(f.intrinsics.width, f.intrinsics.height) == f.color_bgr.size();
  } catch (const std::invalid_argument &) {
  }
  const auto &k = f.intrinsics;
  const cv::Point2d reference =
      calibrated ? cv::Point2d(k.cx, k.cy)
                 : cv::Point2d(f.color_bgr.cols / 2., f.color_bgr.rows / 2.);
  // 过滤类别、置信度、尺寸及越界框；不靠裁框把不完整目标变成有效目标。
  std::vector<Detection> candidates;
  for (const auto &d : detections) {
    const auto &b = d.box;
    if (!finite({b.x1, b.y1, b.x2, b.y2, d.confidence}) ||
        d.confidence < config_.confidence_min || d.confidence > 1 ||
        std::find(config_.class_ids.begin(), config_.class_ids.end(),
                  d.class_id) == config_.class_ids.end() ||
        b.x1 < 0 || b.y1 < 0 || b.x2 > f.color_bgr.cols ||
        b.y2 > f.color_bgr.rows || b.x2 - b.x1 < config_.min_box_px ||
        b.y2 - b.y1 < config_.min_box_px)
      continue;
    candidates.push_back(d);
  }
  // 等待窗口到期后才允许重新选择目标；窗口内丢失则返回无效观测。
  if (last_seen_ && f.capture_s - *last_seen_ > config_.lock_hold_s)
    resetLock();
  std::optional<Detection> chosen;
  // 已有目标：寻找同类别且 IoU 最大的候选，避免立刻跳到旁边的新目标。
  if (target_) {
    double best = -1;
    for (const auto &d : candidates) {
      double overlap = iou(d.box, target_->box);
      if (d.class_id == target_->class_id && overlap >= config_.lock_iou_min &&
          overlap > best) {
        best = overlap;
        chosen = d;
      }
    }
    if (!chosen)
      return reject("locked_target_missing");
  } else {
    // 无锁定时选择最靠近主点的框；距离相同才按置信度打破平局。
    double best = std::numeric_limits<double>::infinity();
    for (const auto &d : candidates) {
      auto p = d.box.center();
      const double score = (p.x - reference.x) * (p.x - reference.x) +
                           (p.y - reference.y) * (p.y - reference.y);
      if (score < best ||
          (score == best && chosen && d.confidence > chosen->confidence)) {
        chosen = d;
        best = score;
      }
    }
    if (!chosen)
      return reject("no_target");
    ++track_id_;
  }
  // 二维确认不依赖深度，丢失后不跨间隔计算像素速度。
  if (target_ && last_seen_ && detection_consecutive_ > 0)
    s.image_velocity_px_s = (chosen->box.center() - target_->box.center()) *
                            (1. / (f.capture_s - *last_seen_));
  if (s.image_velocity_px_s &&
      !finite({s.image_velocity_px_s->x, s.image_velocity_px_s->y}))
    s.image_velocity_px_s.reset();
  target_ = chosen;
  last_seen_ = f.capture_s;
  ++detection_consecutive_;
  s.track_id = track_id_;
  s.class_id = chosen->class_id;
  s.confidence = chosen->confidence;
  s.box = chosen->box;
  s.center_uv = chosen->box.center();
  s.detection_valid = detection_consecutive_ >= config_.confirm_frames;
  s.tracking_state = s.detection_valid ? "tracking" : "acquiring";
  if (!s.detection_valid)
    s.image_velocity_px_s.reset();
  if (calibrated)
    s.pixel_error = *s.center_uv - reference;
  if (chosen->features) {
    const auto inside = [&](cv::Point2d p) {
      return finite({p.x, p.y}) && p.x >= 0 && p.y >= 0 &&
             p.x < f.color_bgr.cols && p.y < f.color_bgr.rows;
    };
    bool good = true;
    const auto &features = *chosen->features;
    if (features.light_bars)
      for (const auto &bar : *features.light_bars)
        good &= inside(bar.top) && inside(bar.bottom) &&
                cv::norm(bar.top - bar.bottom) > 0;
    if (features.corners) {
      const auto &c = *features.corners;
      for (int i = 0; i < 4; ++i) {
        const auto u = c[(i + 1) % 4] - c[i],
                   v = c[(i + 2) % 4] - c[(i + 1) % 4];
        good &= inside(c[i]) && u.x * v.y - u.y * v.x > 1e-6;
      }
    }
    good &= features.light_bars.has_value() || features.corners.has_value();
    s.features_reason = good ? "ok" : "invalid_features";
    if (good)
      s.features = features;
  }
  // 三维失败只重置三维确认，不覆盖本帧二维有效性。
  auto positionReject = [&](const std::string &reason) {
    consecutive_ = 0;
    s.reason = s.position_reason = reason;
    return s;
  };
  if (!calibrated)
    return positionReject("intrinsics_unavailable_or_invalid");
  if (!f.rectified && !f.deproject)
    return positionReject("projection_model_unavailable");
  cv::Vec3d ray;
  try {
    ray = f.deproject ? f.deproject(s.center_uv->x, s.center_uv->y, 1.)
                      : k.deproject(s.center_uv->x, s.center_uv->y, 1.);
  } catch (const std::exception &) {
    return positionReject("deprojection_failed");
  }
  const double norm = cv::norm(ray);
  if (!finite({ray[0], ray[1], ray[2], norm}) || ray[2] <= 0 || norm <= 0)
    return positionReject("invalid_3d_point");
  if (s.detection_valid) {
    s.bearing_camera = ray / norm;
    s.bearing_valid = true;
    s.yaw_error_rad = std::atan2(ray[0], ray[2]);
    s.pitch_error_rad = std::atan2(-ray[1], std::hypot(ray[0], ray[2]));
  }
  if (f.depth_m.empty())
    return positionReject("depth_unavailable");
  if (!std::isfinite(f.sync_error_s) ||
      std::abs(f.sync_error_s) > config_.max_sync_error_s)
    return positionReject("rgb_depth_unsynchronized");
  if (f.depth_m.type() != CV_32FC1 || f.depth_m.size() != f.color_bgr.size())
    return positionReject("image_intrinsics_shape_mismatch");
  const auto depth = estimateDepth(f.depth_m, chosen->box, config_);
  s.depth_valid_ratio = depth.valid_ratio;
  s.depth_mad_m = depth.mad;
  if (!depth.depth)
    return positionReject(depth.reason);
  cv::Vec3d point;
  try {
    point = f.deproject
                ? f.deproject(s.center_uv->x, s.center_uv->y, *depth.depth)
                : k.deproject(s.center_uv->x, s.center_uv->y, *depth.depth);
  } catch (const std::exception &) {
    return positionReject("deprojection_failed");
  }
  if (!finite({point[0], point[1], point[2], cv::norm(point)}) || point[2] <= 0)
    return positionReject("invalid_3d_point");
  ++consecutive_;
  if (consecutive_ < config_.confirm_frames) {
    s.reason = s.position_reason = "acquiring";
    return s;
  }
  s.point_camera = point;
  s.bearing_camera = point / cv::norm(point);
  // 从光学系三维点计算视线角；Y 向下，所以俯仰角使用 -Y。
  s.yaw_error_rad = std::atan2(point[0], point[2]);
  s.pitch_error_rad = std::atan2(-point[1], std::hypot(point[0], point[2]));
  if (config_.body_T_camera) {
    const auto p =
        *config_.body_T_camera * cv::Vec4d(point[0], point[1], point[2], 1);
    s.point_body = cv::Vec3d(p[0], p[1], p[2]);
  }
  s.valid = s.position_valid = true;
  s.reason = s.position_reason = "ok";
  s.aligned =
      std::max(std::abs(s.pixel_error->x), std::abs(s.pixel_error->y)) <=
      config_.aligned_tolerance_px;
  return s;
}
// 生成调试图：白色十字为主点，绿色为有效结果，橙色为未确认/失效结果。
cv::Mat annotate(const Frame &f, const Solution &s) {
  auto image = f.color_bgr.clone();
  cv::Point origin = s.pixel_error ? cv::Point(cvRound(f.intrinsics.cx),
                                               cvRound(f.intrinsics.cy))
                                   : cv::Point(image.cols / 2, image.rows / 2);
  cv::drawMarker(image, origin, {255, 255, 255}, cv::MARKER_CROSS, 14, 1);
  if (s.box) {
    const cv::Scalar color =
        s.detection_valid ? cv::Scalar(30, 220, 30) : cv::Scalar(0, 180, 255);
    cv::rectangle(image, {cvRound(s.box->x1), cvRound(s.box->y1)},
                  {cvRound(s.box->x2), cvRound(s.box->y2)}, color, 2);
    const cv::Point center(cvRound(s.center_uv->x), cvRound(s.center_uv->y));
    cv::circle(image, center, 3, color, -1);
    cv::line(image, origin, center, color, 1);
  }
  // 只绘制上游提供并校验通过的特征。
  if (s.features) {
    if (s.features->light_bars)
      for (const auto &bar : *s.features->light_bars)
        cv::line(image, bar.top, bar.bottom, {255, 255, 0}, 2);
    if (s.features->corners)
      for (const auto &p : *s.features->corners)
        cv::circle(image, p, 3, {255, 0, 255}, -1);
  }
  cv::putText(image, std::to_string(s.sequence) + ": " + s.reason, {8, 20},
              cv::FONT_HERSHEY_SIMPLEX, .45, {255, 255, 255}, 1);
  return image;
}
} // namespace aim
