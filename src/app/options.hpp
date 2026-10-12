// 命令行配置：只描述运行方式，视觉算法阈值放在 config/aim.yaml。
#pragma once
#include <string>
namespace aim::app {
// 默认运行 60 帧合成数据；模型、清单、序列号只对相应数据源有意义。
struct Args {
  std::string source{"synthetic"}, config{"config/aim.yaml"}, weights, manifest,
      serial;
// 日志文件可自动编号；preview 是可选 PNG 输出目录，必须为新目录。
  std::string output{"runs/aim_cpp.jsonl"}, preview;
// frames=0 只允许实时相机；classes 是总类别数，input_size 是模型方形输入边长。
  int frames{60}, classes{0}, input_size{640}, threads{2};
  bool show{false}, ros{false};
};
// 打印参数和编译时启用的可选功能。
void help();
// 解析并校验参数组合；错误抛异常，由应用层统一转成诊断 JSON。
Args parse(int argc, char **argv);
} // namespace aim::app
