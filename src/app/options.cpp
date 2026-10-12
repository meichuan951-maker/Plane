// 命令行解析实现；保持参数错误尽早暴露，避免启动设备后才发现配置无效。
#include "options.hpp"
#include <iostream>
#include <stdexcept>
namespace aim::app {
// 用编译宏显示当前二进制是否支持 RealSense 与 ROS，而非仅检查系统安装情况。
void help() {
  std::cout
      << "C++ armor observation (no flight control)\n"
         "--source synthetic|synthetic-rgb|replay|realsense --config config/aim.yaml\n"
         "--weights model.onnx --num-classes N --input-size 640\n"
         "--manifest replay.json --serial SERIAL --frames N (0=continuous "
         "camera)\n"
         "--output FILE.jsonl --preview-dir NEW_DIR --show --ros --threads 2\n"
         "Existing logs get a numbered filename suffix.\n"
         "Backend: OpenCV DNN CPU; YOLOv8 detect raw output, nms=false.\n"
#ifdef AIM_HAVE_REALSENSE
         "RealSense: compiled\n"
#else
         "RealSense: disabled\n"
#endif
#ifdef AIM_HAVE_ROS2
         "ROS 2: compiled\n";
#else
         "ROS 2: disabled (configure -DAIM_WITH_ROS2=ON)\n";
#endif
}
// 要求整个字符串都是整数，拒绝诸如 60abc 的部分解析结果。
static int integer(const std::string &s) {
  std::size_t consumed = 0;
  const int value = std::stoi(s, &consumed);
  if (consumed != s.size())
    throw std::invalid_argument("Expected integer: " + s);
  return value;
}
// 先读取键值和布尔开关，再统一校验数据源、帧数、线程和模型约束。
Args parse(int argc, char **argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--show") {
      a.show = true;
      continue;
    }
    if (key == "--ros") {
      a.ros = true;
      continue;
    }
    if (i + 1 >= argc)
      throw std::invalid_argument("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--source")
      a.source = value;
    else if (key == "--config")
      a.config = value;
    else if (key == "--weights")
      a.weights = value;
    else if (key == "--manifest")
      a.manifest = value;
    else if (key == "--serial")
      a.serial = value;
    else if (key == "--output")
      a.output = value;
    else if (key == "--preview-dir")
      a.preview = value;
    else if (key == "--frames")
      a.frames = integer(value);
    else if (key == "--num-classes")
      a.classes = integer(value);
    else if (key == "--input-size")
      a.input_size = integer(value);
    else if (key == "--threads")
      a.threads = integer(value);
    else
      throw std::invalid_argument("Unknown argument: " + key);
  }
// 组合校验：不同数据源的必需参数不同，回放的虚拟时间不能用于实时 ROS 输出。
  if (a.source != "synthetic" && a.source != "synthetic-rgb" && a.source != "replay" &&
      a.source != "realsense")
    throw std::invalid_argument("Unknown source");
  if (a.frames < 0 || a.frames > 1000000 ||
      (a.frames == 0 && a.source != "realsense"))
    throw std::invalid_argument("Invalid frame count");
  if (a.threads < 1 || a.threads > 16)
    throw std::invalid_argument("threads must be 1..16");
  if (a.source == "realsense" && a.weights.empty())
    throw std::invalid_argument("Camera input requires custom .onnx weights");
  if (!a.weights.empty() && (a.classes < 1 || (a.source == "synthetic" || a.source == "synthetic-rgb")))
    throw std::invalid_argument(
        "Weights require model --num-classes and replay/realsense input");
  if (a.source == "replay" && a.manifest.empty())
    throw std::invalid_argument("Replay requires --manifest");
  if (a.ros && a.source != "realsense")
    throw std::invalid_argument(
        "ROS output is camera only; replay has virtual time");
  return a;
}
} // namespace aim::app
