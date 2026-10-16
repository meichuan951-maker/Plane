// PNG 回放数据源：按清单逐帧重现图像、检测框和虚拟时间。
#include "demo_aim/input.hpp"
#include <cmath>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
namespace aim {
namespace {
cv::Point2d parseFeaturePixel(const Json &value) {
  if (!value.is_array() || value.size() != 2)
    throw std::invalid_argument("Feature pixel needs [u,v]");
  return {value[0].get<double>(), value[1].get<double>()};
}

std::optional<ArmorFeatures> parseArmorFeatures(const Json &detection) {
  ArmorFeatures features;
  if (detection.contains("corners_uv") && !detection["corners_uv"].is_null()) {
    const auto &corners = detection["corners_uv"];
    if (!corners.is_array() || corners.size() != 4)
      throw std::invalid_argument("Need four corners");
    features.corners = std::array<cv::Point2d, 4>{
        parseFeaturePixel(corners[0]), parseFeaturePixel(corners[1]),
        parseFeaturePixel(corners[2]), parseFeaturePixel(corners[3])};
  }
  if (detection.contains("light_bars_uv") &&
      !detection["light_bars_uv"].is_null()) {
    const auto &bars = detection["light_bars_uv"];
    if (!bars.is_array() || bars.size() != 2)
      throw std::invalid_argument("Need two light bars");
    std::array<LightBar, 2> output;
    for (int i = 0; i < 2; ++i) {
      if (!bars[i].is_array() || bars[i].size() != 2)
        throw std::invalid_argument("Need two endpoints");
      output[i] = {parseFeaturePixel(bars[i][0]),
                   parseFeaturePixel(bars[i][1])};
    }
    features.light_bars = output;
  }
  if (!features.corners && !features.light_bars)
    return std::nullopt;
  features.layout_id = detection.value("geometry_layout_id", std::string{});
  return features;
}
} // namespace

// 构造阶段读取元数据；rectified=true 是数据提供方承诺，本代码不会自动去畸变。
Replay::Replay(const std::string &path)
    : directory_(std::filesystem::path(path).parent_path()) {
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Missing replay manifest");
  stream >> manifest_;
  const int schema = manifest_.at("schema_version");
  if (schema != 2 && schema != 3)
    throw std::invalid_argument("Replay requires schema=2 or schema=3");
  legacy_ = schema == 2;
  rectified_ = manifest_.value("rectified", false);
  if (legacy_ && !rectified_)
    throw std::invalid_argument("Legacy RGB-D replay requires rectified=true");
  if (legacy_ || manifest_.contains("depth_encoding")) {
    if (manifest_.at("depth_encoding") != "uint16_scaled")
      throw std::invalid_argument("Replay depth must be uint16_scaled");
    depth_scale_ = manifest_.at("depth_scale_m").get<double>();
    if (!std::isfinite(depth_scale_) || depth_scale_ <= 0)
      throw std::invalid_argument("Invalid depth_scale_m");
  }
  if (legacy_ || (manifest_.contains("intrinsics") &&
                  !manifest_["intrinsics"].is_null())) {
    const auto &k = manifest_.at("intrinsics");
    intrinsics_ = {k.at("width"), k.at("height"), k.at("fx"),
                   k.at("fy"),    k.at("cx"),     k.at("cy")};
    intrinsics_.validate();
  }
  if (!manifest_.at("frames").is_array())
    throw std::invalid_argument("frames must be an array");
}
// 读取彩色与 16 位单通道深度；depth_scale_m
// 将整数深度换算成米，零值保留给核心判无效。
bool Replay::next(Sample &s) {
  if (index_ >= manifest_["frames"].size())
    return false;
  const auto &item = manifest_["frames"][index_];
  // 先清空上一帧样本，避免旧检测框被追加到新帧。
  s = Sample{};
  auto &f = s.frame;
  f.color_bgr =
      cv::imread((directory_ / item.at("color").get<std::string>()).string(),
                 cv::IMREAD_COLOR);
  if (f.color_bgr.empty())
    throw std::runtime_error("Unreadable color PNG");
  if (legacy_ || (item.contains("depth") && !item["depth"].is_null())) {
    if (depth_scale_ <= 0)
      throw std::invalid_argument("Depth frame needs encoding and scale");
    const auto depth =
        cv::imread((directory_ / item.at("depth").get<std::string>()).string(),
                   cv::IMREAD_UNCHANGED);
    if (depth.empty() || depth.type() != CV_16UC1)
      throw std::runtime_error("Replay needs 16-bit depth PNG");
    depth.convertTo(f.depth_m, CV_32F, depth_scale_);
  }
  f.rectified = rectified_;
  f.time_domain = "replay_monotonic";
  f.intrinsics = intrinsics_;
  f.sequence = index_++;
  // 默认处理时刻比采集晚 10 ms；可在清单中覆盖以复现超时或同步问题。
  f.capture_s = item.at("capture_monotonic_s");
  f.sync_error_s = item.value("sync_error_s", 0.);
  f.source = "recorded_detections_replay_cpp";
  if (item.contains("received_monotonic_s") &&
      !item["received_monotonic_s"].is_null())
    f.received_s = item["received_monotonic_s"].get<double>();
  s.process_s = item.value("process_monotonic_s", f.capture_s + .01);
  for (const auto &d : item.at("detections")) {
    const auto &b = d.at("xyxy");
    if (!b.is_array() || b.size() != 4)
      throw std::invalid_argument("Detection xyxy must have 4 values");
    Detection detection{
        {b[0], b[1], b[2], b[3]}, d.at("confidence"), d.at("class_id")};
    detection.features = parseArmorFeatures(d);
    s.detections.push_back(detection);
  }
  return true;
}
} // namespace aim
