// 命令行配置：只描述运行方式，视觉算法阈值放在 config/aim.yaml。
#pragma once
#include <string>
namespace aim::app {
// 默认运行 60 帧合成数据；实时相机使用传统灯条配置和可选序列号。
struct Args {
  std::string source{"synthetic"}, config{"config/aim.yaml"},
      detector_config{"config/light_bars.yaml"}, serial;
// 日志按大小和数量轮转；preview 是可选 PNG 输出目录，必须为新目录。
  std::string output{"runs/aim_cpp.jsonl"}, preview;
// frames=0 只允许实时相机。
  int frames{60}, threads{2};
  bool show{false}, ros{false};
};
// 打印参数和编译时启用的可选功能。
void help();
// 解析并校验参数组合；错误抛异常，由应用层统一转成诊断 JSON。
Args parse(int argc, char **argv);
} // namespace aim::app
