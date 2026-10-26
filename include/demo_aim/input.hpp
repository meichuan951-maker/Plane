// 输入适配公共接口：合成数据与实时相机提供一致的 Frame。
#pragma once
#include "demo_aim/observation.hpp"
#include <memory>
namespace aim {
// frame 是彩色帧，可选深度；detections 由合成源注入或应用检测器填充；process_s 是处理时刻。
struct Sample {
  Frame frame;
  std::vector<Detection> detections;
  double process_s{};
};
// 生成确定性的离线测试帧，不执行真实目标检测。
Sample synthetic(std::uint64_t sequence);
// 持有相机资源的 RAII 类；禁止复制，析构时停止采集。
class RealSense {
public:
  explicit RealSense(const std::string &serial = "");
  ~RealSense();
  RealSense(const RealSense &) = delete;
  RealSense &operator=(const RealSense &) = delete;
  // 等待、同步并对齐一组 RGB-D；返回的图像拥有独立内存。
  Frame read();

private:
  // 通过不完整类型隐藏 SDK，包含本头文件的调用方不需要知道 rs2 类型。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace aim
