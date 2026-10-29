// 无需相机和真实模型的回归测试；说明可验证框架行为，不代表实机识别准确率。
#include "demo_aim/core.hpp"
#include "demo_aim/flight_state.hpp"
#include "demo_aim/input.hpp"
#include "support/assertions.hpp"
#include "support/runner.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace aim;
namespace {
using test_support::check;
// 对浮点结果使用绝对误差容差，避免直接比较计算结果。
void near(double a, double b) {
  check(std::abs(a - b) < 1e-6, "numeric mismatch");
}
using test_support::expectThrows;
// 创建固定内参和彩色/米制深度帧，隔离相机与模型依赖。
Frame frame(double stamp = 1., float depth = 2.) {
  Frame f;
  f.color_bgr = cv::Mat::zeros(80, 100, CV_8UC3);
  f.depth_m = cv::Mat(80, 100, CV_32F, cv::Scalar(depth));
  f.rectified = true;
  f.intrinsics = {100, 80, 100, 100, 50, 40};
  f.capture_s = stamp;
  f.sequence = static_cast<std::uint64_t>(stamp * 100);
  return f;
}
// 默认检测框围绕主点；各测试只修改其关注的参数。
Detection detection(Box box = {40, 30, 60, 50}, double score = .9,
                    int cls = 0) {
  return {box, score, cls};
}
// 将连续确认设为 1，方便单帧几何测试；确认逻辑另有独立用例。
Config instant() {
  Config c;
  c.confirm_frames = 1;
  return c;
}
// 临时目录持有配置测试夹具，析构时仅清理本测试创建的目录。
struct Temp {
  std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("aim_cpp_test_" + std::to_string(monotonicNow()));
  Temp() { std::filesystem::create_directory(path); }
  ~Temp() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};
} // namespace
// 顺序运行所有具名测试，汇总通过/失败数；返回非零让 CTest 判定失败。
int main(int argc, char **argv) {
  test_support::Runner test;
  // 验证现有 YAML 可加载，默认连续确认数及未标定外参保持预期。
  test("yaml_compatibility", [&] {
    check(argc == 2);
    auto c = Config::load(argv[1]);
    check(c.confirm_frames == 3 && !c.body_T_camera);
  });
  // 主点处目标应投影到光轴，角误差为零，且不授权接触。
  test("center_projection", [] {
    Pipeline p(instant());
    auto s = p.process(frame(), {detection()}, 1.01);
    check(s.valid && s.aligned);
    near((*s.point_camera)[2], 2);
    near(*s.yaw_error_rad, 0);
    check(!s.point_body);
    check(s.json()["contact_permitted"] == false);
  });
  // 验证偏离主点后的左右/上下符号，以及标定变换的旋转和平移。
  test("optical_sign_and_extrinsic", [] {
    auto c = instant();
    c.body_T_camera =
        cv::Matx44d(0, 0, 1, .1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);
    Pipeline p(c);
    auto s = p.process(frame(), {detection({60, 50, 80, 70})}, 1.01);
    check(s.valid && !s.aligned);
    near((*s.point_camera)[0], .4);
    near((*s.point_body)[0], 2.1);
    near((*s.point_body)[1], -.4);
    check(*s.yaw_error_rad > 0 && *s.pitch_error_rad < 0);
  });
  // 正常帧后输入零、越界或非有限深度，必须失效且不保留旧三维点。
  test("invalid_depth_no_old_point", [] {
    for (float value : {0.f, .1f, 5.f, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()}) {
      Pipeline p(instant());
      check(p.process(frame(), {detection()}, 1.01).valid);
      auto s = p.process(frame(1.1, value), {detection()}, 1.11);
      check(!s.valid && !s.point_camera && s.reason == "insufficient_depth");
    }
  });
  // ROI 中深度空洞过多时，应因有效率不足而拒绝。
  test("depth_holes", [] {
    auto f = frame();
    f.depth_m(cv::Rect(46, 36, 4, 8)).setTo(0);
    Pipeline p(instant());
    check(p.process(f, {detection()}, 1.01).reason == "insufficient_depth");
  });
  // 主层仍占多数但存在明显次层时，不能只依赖很小的 MAD 放行。
  test("mixed_depth_layers", [] {
    auto f = frame();
    f.depth_m(cv::Rect(46, 36, 3, 8)).setTo(1);
    Pipeline p(instant());
    check(p.process(f, {detection()}, 1.01).reason == "mixed_depth_layers");
  });
  // 一个离群深度不应改变主层的中位数估计。
  test("single_depth_outlier", [] {
    auto f = frame();
    f.depth_m.at<float>(40, 50) = 2.9;
    Pipeline p(instant());
    near((*p.process(f, {detection()}, 1.01).point_camera)[2], 2.);
  });
  // 深度分散到两层时，MAD 检查应拒绝该 ROI。
  test("depth_dispersion", [] {
    auto f = frame();
    f.depth_m(cv::Rect(46, 36, 4, 8)).setTo(1);
    Pipeline p(instant());
    check(p.process(f, {detection()}, 1.01).reason == "inconsistent_depth");
  });
  // 用全排序统计量核对选择算法，覆盖奇偶样本、重复值、空洞和非连续 ROI。
  test("depth_statistics_match_sorted_reference", [] {
    std::mt19937 rng(42);
    auto c = instant();
    c.depth_roi_fraction = 1;
    c.depth_min_samples = 1;
    c.depth_min_valid_ratio = .1;
    const auto sortedMedian = [](std::vector<double> values) {
      std::sort(values.begin(), values.end());
      const auto n = values.size();
      return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
    };
    for (int trial = 0; trial < 160; ++trial) {
      const int width = 3 + trial % 13, height = 3 + trial % 7;
      cv::Mat storage(height + 2, width + 4, CV_32F, cv::Scalar(0));
      auto image = storage(cv::Rect(1, 1, width, height));
      std::vector<double> values;
      for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
          const unsigned sample = rng() % 100;
          const float value = sample < 8 ? NAN
                              : sample < 13 ? 0.f
                              : sample < 18 ? 2.8f
                              : 2.f + (static_cast<int>(rng() % 17) - 8) * .002f;
          image.at<float>(y, x) = value;
          if (std::isfinite(value) && value >= c.depth_min_m && value <= c.depth_max_m)
            values.push_back(value);
        }
      const auto result = estimateDepth(image, {0, 0, double(width), double(height)}, c);
      near(result.valid_ratio, double(values.size()) / (width * height));
      const double z = sortedMedian(values);
      std::vector<double> deviations, inliers;
      for (double v : values)
        deviations.push_back(std::abs(v - z));
      const double mad = sortedMedian(deviations);
      check(result.mad.has_value());
      near(*result.mad, mad);
      for (double v : values)
        if (std::abs(v - z) <= std::max(3 * mad, c.depth_max_mad_m))
          inliers.push_back(v);
      if (mad > c.depth_max_mad_m)
        check(!result.depth && result.reason == "inconsistent_depth");
      else if (double(inliers.size()) / values.size() < .8)
        check(!result.depth && result.reason == "mixed_depth_layers");
      else {
        check(result.depth && result.reason == "ok");
        near(*result.depth, sortedMedian(inliers));
      }
    }
  });
  test("invalid_intrinsics_keep_2d_without_metric_output", [] {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const auto &k : std::vector<Intrinsics>{
             {}, {0, 80, 100, 100, 50, 40}, {100, 80, -1, 100, 50, 40},
             {100, 80, nan, 100, 50, 40}, {100, 80, 100, inf, 50, 40},
             {100, 80, 100, 100, nan, 40}, {100, 80, 100, 100, 50, inf}}) {
      check(!k.isValid());
      expectThrows([&] { k.validate(); });
      auto f = frame();
      f.intrinsics = k;
      Pipeline p(instant());
      const auto s = p.process(f, {detection()}, 1.01);
      check(s.detection_valid && !s.position_valid && !s.bearing_valid &&
            !s.point_camera && s.reason == "intrinsics_unavailable_or_invalid");
    }
    auto f = frame();
    check(f.intrinsics.isValid());
    f.intrinsics.width += 1;
    Pipeline p(instant());
    check(p.process(f, {detection()}, 1.01).reason == "intrinsics_unavailable_or_invalid");
  });
  // 验证过期、未来及非有限时间不会产生有效观测。
  test("stale_future_and_nonfinite_time", [] {
    for (double now : {.99, 1.21}) {
      Pipeline p(instant());
      check(p.process(frame(), {detection()}, now).reason ==
            "stale_or_future_frame");
    }
    Pipeline p;
    expectThrows([&] {
      p.process(frame(), {}, std::numeric_limits<double>::quiet_NaN());
    });
  });
  // 分别验证 RGB-D 时间差过大与时钟域未同步两种失效原因。
  test("sync_and_camera_clock", [] {
    auto f = frame();
    f.sync_error_s = .05;
    Pipeline p(instant());
    check(p.process(f, {detection()}, 1.01).reason ==
          "rgb_depth_unsynchronized");
    f = frame(1.1);
    f.clock_valid = false;
    check(p.process(f, {detection()}, 1.11).reason ==
          "camera_clock_unsynchronized");
  });
  // 重复采集时间必须拒绝，不能用同一帧累加连续确认次数。
  test("duplicate_frame", [] {
    Pipeline p(instant());
    p.process(frame(), {detection()}, 1.01);
    check(p.process(frame(), {detection()}, 1.02).reason ==
          "non_monotonic_frame");
  });
  // 深度失败打断确认序列，恢复后必须重新连续确认。
  test("confirmation_after_bad_depth", [] {
    Pipeline p;
    check(!p.process(frame(), {detection()}, 1.01).valid);
    check(!p.process(frame(1.1, 0), {detection()}, 1.11).valid);
    check(!p.process(frame(1.2), {detection()}, 1.21).valid);
    check(!p.process(frame(1.3), {detection()}, 1.31).valid);
    check(p.process(frame(1.4), {detection()}, 1.41).valid);
  });
  // 保留窗口内不跳到其他目标；超时后允许新目标并产生新的关联编号。
  test("lock_dropout_does_not_switch", [] {
    Pipeline p(instant());
    auto first = p.process(frame(), {detection()}, 1.01);
    auto other = detection({75, 30, 95, 50});
    check(p.process(frame(1.1), {other}, 1.11).reason ==
          "locked_target_missing");
    auto next = p.process(frame(1.5), {other}, 1.51);
    check(next.valid && next.track_id != first.track_id);
  });
  // 拒绝不允许类别、低分、越界、退化框和非有限置信度。
  test("class_confidence_and_bounds", [] {
    for (const auto &d : std::vector<Detection>{
             detection({40, 30, 60, 50}, .9, 1),
             detection({40, 30, 60, 50}, .1), detection({-1, 0, 10, 10}),
             detection({5, 5, 5, 20}),
             detection({40, 30, 60, 50},
                       std::numeric_limits<double>::quiet_NaN())}) {
      Pipeline p(instant());
      check(p.process(frame(), {d}, 1.01).reason == "no_target");
    }
  });
  // SDK 回调优先于针孔反投影，回调异常必须转为失效结果。
  test("sdk_deprojection_callback", [] {
    auto f = frame();
    f.deproject = [](double, double, double z) { return cv::Vec3d(.1, .2, z); };
    Pipeline p(instant());
    near((*p.process(f, {detection()}, 1.01).point_camera)[0], .1);
    f = frame(1.1);
    f.deproject = [](double, double, double) -> cv::Vec3d {
      throw std::runtime_error("unsupported distortion");
    };
    check(p.process(f, {detection()}, 1.11).reason == "deprojection_failed");
  });
  // 输入非 float32 深度时应拒绝，防止把毫米整数当成米。
  test("wrong_depth_type_and_shape", [] {
    auto f = frame();
    f.depth_m = cv::Mat(80, 100, CV_16U);
    Pipeline p(instant());
    check(p.process(f, {detection()}, 1.01).reason ==
          "image_intrinsics_shape_mismatch");
  });
  // 拒绝镜像外参、颠倒深度区间与无限时间阈值。
  test("config_rejects_reflection_nan_and_bad_range", [] {
    auto c = instant();
    c.body_T_camera = cv::Matx44d::eye();
    (*c.body_T_camera)(0, 0) = -1;
    expectThrows([&] { c.validate(); });
    c = instant();
    c.depth_min_m = 4;
    expectThrows([&] { c.validate(); });
    c = instant();
    c.max_frame_age_s = std::numeric_limits<double>::infinity();
    expectThrows([&] { c.validate(); });
  });
  // 未知 YAML 参数直接报错，避免拼写错误被默认值掩盖。
  test("strict_unknown_yaml", [] {
    Temp t;
    std::ofstream(t.path / "bad.yaml") << "typo_threshold: 1\n";
    expectThrows([&] { Config::load((t.path / "bad.yaml").string()); });
  });
  // 验证消费端时效判断与 JSON 序列化保留有效性。
  test("downstream_watchdog_and_json", [] {
    Pipeline p(instant());
    const auto s = p.process(frame(), {detection()}, 1.01);
    check(isCurrent(s, 1.1, .2));
    check(!isCurrent(s, 1.3, .2));
    check(Json::parse(s.json().dump())["valid"] == true);
  });
  // 60 帧中预期 44 帧有效、5 帧丢失、5 帧深度失效，其余为确认阶段。
  test("synthetic_end_to_end", [] {
    Pipeline p;
    int valid = 0, lost = 0, bad = 0;
    for (int i = 0; i < 60; ++i) {
      auto sample = synthetic(i);
      auto s = p.process(sample.frame, sample.detections, sample.process_s);
      valid += s.valid;
      lost += s.reason == "locked_target_missing";
      bad += s.reason == "insufficient_depth";
    }
    check(valid == 44 && lost == 5 && bad == 5);
  });
  // IoU 仍用于传统检测框的跨帧关联，与具体检测器无关。
  test("iou_preserves_overlap_geometry", [] {
    const Box first{0, 0, 10, 10}, overlap{5, 0, 15, 10};
    near(iou(first, first), 1.);
    near(iou(first, overlap), 1. / 3.);
    near(iou(first, overlap), iou(overlap, first));
    near(iou(first, {10, 0, 20, 10}), 0.);
    near(iou(first, {20, 20, 30, 30}), 0.);
  });
  // 以下用例验证接口语义；不依赖相机、串口或模型权重。
  test("rgb_only_detection_contract", [] {
    Pipeline p(instant());
    auto f = frame();
    f.depth_m.release();
    f.intrinsics = {};
    auto s = p.process(f, {detection()}, 1.01);
    check(s.detection_valid && !s.valid && !s.position_valid &&
          !s.bearing_valid);
    check(!s.point_camera && !s.point_body && !s.yaw_error_rad &&
          !s.pixel_error);
    check(s.json()["schema_version"] == 2 &&
          s.json()["class_name"] == "target_red");
    check(s.json()["range_m"].is_null());
    check(isDetectionCurrent(s, 1.1, .2) && !isCurrent(s, 1.1, .2));
  });
  test("calibrated_bearing_without_distance", [] {
    Pipeline p(instant());
    auto f = frame();
    f.depth_m.release();
    auto s = p.process(f, {detection({60, 30, 80, 50})}, 1.01);
    check(s.detection_valid && s.bearing_valid && !s.valid);
    near(cv::norm(*s.bearing_camera), 1.);
    check((*s.bearing_camera)[0] > 0);
    check(!s.point_camera && s.position_reason == "depth_unavailable");
  });
  test("unrectified_image_needs_projection_callback", [] {
    Pipeline p(instant());
    auto f = frame();
    f.rectified = false;
    auto s = p.process(f, {detection()}, 1.01);
    check(s.detection_valid && !s.bearing_valid && !s.position_valid);
    check(s.reason == "projection_model_unavailable");
  });
  test("depth_failure_keeps_confirmed_detection", [] {
    Pipeline p;
    Solution s;
    for (int i = 0; i < 3; ++i) {
      auto f = frame(1. + i * .03, 0);
      s = p.process(f, {detection()}, f.capture_s + .01);
    }
    check(s.detection_valid && s.bearing_valid && !s.position_valid);
    s = p.process(frame(1.1), {detection()}, 1.11);
    check(s.detection_valid && !s.position_valid);
  });
  test("unsynced_depth_preserves_color_observation", [] {
    Pipeline p(instant());
    auto f = frame();
    f.sync_error_s = .1;
    auto s = p.process(f, {detection()}, 1.01);
    check(s.detection_valid && s.bearing_valid && !s.valid);
  });
  test("lost_target_clears_features_bearing_velocity", [] {
    Pipeline p(instant());
    auto first = p.process(frame(), {detection()}, 1.01);
    auto s = p.process(frame(1.1), {}, 1.11);
    check(s.tracking_state == "lost" && s.track_id == first.track_id);
    check(!s.detection_valid && !s.box && !s.point_camera &&
          !s.bearing_camera && !s.image_velocity_px_s && !s.features);
    s = p.process(frame(1.5), {}, 1.51);
    check(s.tracking_state == "searching" && !s.track_id);
  });
  test("image_motion_is_pixel_velocity", [] {
    Pipeline p(instant());
    p.process(frame(), {detection()}, 1.01);
    auto s = p.process(frame(1.1), {detection({42, 30, 62, 50})}, 1.11);
    near(s.image_velocity_px_s->x, 20.);
    near(s.image_velocity_px_s->y, 0.);
    p.process(frame(1.2), {}, 1.21);
    s = p.process(frame(1.3), {detection({44, 30, 64, 50})}, 1.31);
    check(!s.image_velocity_px_s);
  });
  test("red_blue_lock_does_not_cross_class", [] {
    auto c = instant();
    c.class_ids = {0, 1};
    Pipeline p(c);
    auto red = p.process(frame(), {detection()}, 1.01);
    auto s = p.process(frame(1.1), {detection({40, 30, 60, 50}, .99, 1)}, 1.11);
    check(!s.detection_valid && s.tracking_state == "lost");
    s = p.process(frame(1.5), {detection({40, 30, 60, 50}, .99, 1)}, 1.51);
    check(s.detection_valid && s.track_id != red.track_id &&
          s.json()["class_name"] == "target_blue");
  });
  test("feature_roundtrip_and_invalid_geometry", [] {
    auto d = detection();
    ArmorFeatures features;
    features.corners =
        std::array<cv::Point2d, 4>{{{40, 30}, {60, 30}, {60, 50}, {40, 50}}};
    features.light_bars =
        std::array<LightBar, 2>{{{{40, 30}, {40, 50}}, {{60, 30}, {60, 50}}}};
    features.layout_id = "fixture_only";
    d.features = features;
    Pipeline p(instant());
    auto s = p.process(frame(), {d}, 1.01);
    check(s.features && s.json()["light_bars_uv"].size() == 2 &&
          s.json()["corners_uv"].size() == 4);
    check(!annotate(frame(), s).empty());
    (*d.features->corners)[1] = {-1, 30};
    s = p.process(frame(1.1), {d}, 1.11);
    check(s.detection_valid && !s.features &&
          s.features_reason == "invalid_features");
    s = p.process(frame(1.2), {detection()}, 1.21);
    check(!s.features && s.json()["corners_uv"].is_null());
  });
  test("replay_clock_cannot_pass_live_watchdog", [] {
    Pipeline p(instant());
    auto f = frame();
    f.time_domain = "replay_monotonic";
    auto s = p.process(f, {detection()}, 1.01);
    check(!isCurrent(s, 1.1, .2) && !isDetectionCurrent(s, 1.1, .2));
    check(isCurrent(s, 1.1, .2, "replay_monotonic"));
  });
  test("receive_time_and_stale_frames_rejected", [] {
    for (double receive : {.9, 1.2, std::numeric_limits<double>::quiet_NaN()}) {
      auto f = frame();
      f.received_s = receive;
      Pipeline p(instant());
      auto s = p.process(f, {detection()}, 1.01);
      check(!s.detection_valid && s.reason == "invalid_receive_time");
    }
    Pipeline p(instant());
    auto s = p.process(frame(), {detection()}, 1.3);
    check(!s.detection_valid && !s.box && !s.position_valid);
  });
  test("source_change_starts_new_confirmation", [] {
    Pipeline p;
    for (int i = 0; i < 3; ++i)
      p.process(frame(1. + i * .03), {detection()}, 1.01 + i * .03);
    auto f = frame(1.2);
    f.source = "different_camera";
    auto s = p.process(f, {detection()}, 1.21);
    check(!s.detection_valid && s.tracking_state == "acquiring");
  });
  test("flight_state_expiry_is_independent", [] {
    FlightStateCache c;
    check(c.snapshot(1).json()["heartbeat"].is_null());
    check(c.updateHeartbeat(1, 1, {}, {1., {}}));
    check(c.updateAttitude(1, 1, {}, {1., 1000}));
    check(c.updateLocalPosition(1, 1, {}, {1., 1000}));
    check(c.snapshot(1.1).attitude.has_value());
    c.updateHeartbeat(1, 1, {}, {1.3, {}});
    auto s = c.snapshot(1.31);
    check(s.heartbeat && !s.attitude && !s.local_position);
    check(s.json()["system_id"] == 1 && s.json()["attitude"].is_null());
    check(!c.snapshot(.9).heartbeat && !c.snapshot(3).heartbeat);
  });
  test("flight_state_identity_order_reboot", [] {
    FlightStateCache c;
    check(!c.updateHeartbeat(2, 1, {}, {1., {}}));
    check(!c.updateAttitude(1, 2, {}, {1., 1000}));
    check(c.updateAttitude(1, 1, {}, {1., 1000}));
    check(!c.updateAttitude(1, 1, {}, {.9, 1100}));
    check(!c.updateAttitude(1, 1, {}, {1.1, 900}));
    c.reset();
    check(c.updateAttitude(1, 1, {}, {1.2, 900}));
    check(!c.snapshot(1.2).heartbeat);
  });
  test("flight_state_bad_values_and_validity", [] {
    FlightStateCache c;
    AttitudeState att;
    att.body_frd_to_ned_wxyz = {2, 0, 0, 0};
    expectThrows([&] { c.updateAttitude(1, 1, att, {1., {}}); });
    LocalPositionState p;
    p.position_ned_m[0] = std::numeric_limits<double>::infinity();
    expectThrows([&] { c.updateLocalPosition(1, 1, p, {1., {}}); });
    p = {};
    check(c.updateLocalPosition(1, 1, p, {1., {}}));
    check(c.snapshot(1.1).json()["local_position"]["xy_valid"] == false);
    expectThrows([&] { c.snapshot(std::numeric_limits<double>::quiet_NaN()); });
  });
  return test.finish();
}
