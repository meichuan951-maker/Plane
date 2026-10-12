// Linux 非阻塞 8N1 UART，RAII 释放；不自动搜索/打开任何硬件。
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace aim {
class SerialPort {
public:
  explicit SerialPort(const std::string &device, int baud = 921600);
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;
  // 最多等待 1000ms；超时返回空，断连/系统错误抛异常。
  std::vector<std::uint8_t> read(int timeout_ms = 20);
  // 整帧在 deadline 内写完；超时直接报错，禁止无界重试积压旧消息。
  void write(const std::vector<std::uint8_t> &packet, int timeout_ms = 20);

private:
  int fd_{-1};
};
} // namespace aim
