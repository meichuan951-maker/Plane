// 命令行解析实现；保持参数错误尽早暴露，避免启动设备后才发现配置无效。
#include "demo_aim/detail/app/options.hpp"
#include "demo_aim/detail/app/common/parse.hpp"
#include <iostream>
#include <stdexcept>
namespace aim::app {
// 用编译宏显示当前二进制是否支持 RealSense 与 ROS，而非仅检查系统安装情况。
void help() {
  std::cout
      << "C++ armor observation (no flight control)\n"
         "--source synthetic|synthetic-rgb|realsense --config "
         "config/aim.yaml\n"
         "--detector-config config/light_bars.yaml (live camera)\n"
         "--serial SERIAL --frames N (0=continuous "
         "camera)\n"
         "--output FILE.jsonl --preview-dir NEW_DIR --show --ros --threads 2\n"
         "Logs rotate at 10 MiB; keep current file plus 4 backups (.1-.4).\n"
         "Detector: traditional red/blue light bar pairing.\n"
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
// 先读取键值和布尔开关，再统一校验数据源、帧数和线程约束。
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
    else if (key == "--detector-config")
      a.detector_config = value;
    else if (key == "--serial")
      a.serial = value;
    else if (key == "--output")
      a.output = value;
    else if (key == "--preview-dir")
      a.preview = value;
    else if (key == "--frames")
      a.frames = parseInteger(value, "Expected integer: ");
    else if (key == "--threads")
      a.threads = parseInteger(value, "Expected integer: ");
    else
      throw std::invalid_argument("Unknown argument: " + key);
  }
  // 合成数据不能用于实时 ROS 输出。
  if (a.source != "synthetic" && a.source != "synthetic-rgb" &&
      a.source != "realsense")
    throw std::invalid_argument("Unknown source");
  if (a.frames < 0 || a.frames > 1000000 ||
      (a.frames == 0 && a.source != "realsense"))
    throw std::invalid_argument("Invalid frame count");
  if (a.threads < 1 || a.threads > 16)
    throw std::invalid_argument("threads must be 1..16");
  if (a.ros && a.source != "realsense")
    throw std::invalid_argument("ROS output is camera only");
  return a;
}
} // namespace aim::app
