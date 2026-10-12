// 合成数据源：用于无相机、无模型时检查整条处理链，不能评估 YOLO 准确率。
#include "demo_aim/input.hpp"
#include <cmath>
#include <limits>
namespace aim {
// 用帧号构造固定的 30 Hz 虚拟时间，以及横向移动的彩色矩形和 1.5 m 深度。
Sample synthetic(std::uint64_t n) {
  Sample s;
  auto &f = s.frame;
  f.rectified = true;
  f.time_domain = "replay_monotonic";
  f.intrinsics = {320, 240, 280, 280, 160, 120};
  f.capture_s = 1. + static_cast<double>(n) / 30.;
  f.sequence = n;
  f.source = "synthetic_injected_boxes";
  f.color_bgr = cv::Mat::zeros(240, 320, CV_8UC3);
  f.depth_m = cv::Mat(240, 320, CV_32F,
                      cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  const int center = 160 + static_cast<int>(55 * std::cos(n * .08));
  const cv::Rect rect(center - 30, 100, 60, 40);
  f.color_bgr(rect).setTo(cv::Scalar(50, 150, 240));
  f.depth_m(rect).setTo(1.5);
  s.detections.emplace_back();
  auto &d=s.detections.back();
  d.box={static_cast<double>(center-30),100,static_cast<double>(center+30),140};
  d.confidence=.95; d.class_id=0;
// 故障注入：第 20～24 帧缺失检测，第 40～44 帧所有深度为 NaN。
  if (n >= 20 && n < 25)
    s.detections.clear();
  if (n >= 40 && n < 45)
    f.depth_m.setTo(std::numeric_limits<float>::quiet_NaN());
  s.process_s = f.capture_s + .01;
  return s;
}
} // namespace aim
