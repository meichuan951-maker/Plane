// 输入适配公共接口：三种数据源最终提供一致的 Frame 与 Detection。
#pragma once
#include "demo_aim/observation.hpp"
#include <filesystem>
#include <memory>
namespace aim {
// frame 是彩色帧，可选深度；detections 可为注入框或回放框；process_s 是该样本处理时刻。
struct Sample {
  Frame frame;
  std::vector<Detection> detections;
  double process_s{};
};
// 生成确定性的离线测试帧，不执行真实目标检测。
Sample synthetic(std::uint64_t sequence);
// schema=2 为 RGB-D；schema=3 允许省略内参/深度，路径相对于清单。
class Replay {
public:
  explicit Replay(const std::string &path);
  // 读取下一帧并覆盖 sample；到清单末尾返回 false，损坏文件抛异常。
  bool next(Sample &sample);

private:
  std::filesystem::path directory_;
  Json manifest_;
  Intrinsics intrinsics_;
  double depth_scale_{};
  bool legacy_{true}, rectified_{false};
  std::size_t index_{0};
};
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
