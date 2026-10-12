#pragma once
#include "uvc_controls.hpp"
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <sys/types.h>
#include <vector>

namespace aim {
struct StreamShared;
enum class CameraCommand : int { Focus, Gain, AutoFocus, ExposureRecovery };
struct StreamOptions {
  std::string device, format{"MJPG"};
  int width{640}, height{480}, fps{30};
  int frame_type{CV_8UC3}; // CV_8UC3 彩色，CV_32FC3 深度光学系 XYZ（米）。
  int graceful_stop_ms{0};
  bool controls{true};
  double stall_seconds{3}, reconnect_seconds{15};
};

// 摄像头驱动只在子进程调用；主进程只尝试读取共享帧和发送非阻塞控制消息。
// 一个共享帧槽，不积压历史图像。设备阻塞时主窗口仍可退出。
class CameraStream {
public:
  enum class Result { Frame, Waiting, Recovering, Failed };
  explicit CameraStream(StreamOptions options,
                        std::string executable = "/proc/self/exe");
  ~CameraStream();
  CameraStream(const CameraStream &) = delete;
  CameraStream &operator=(const CameraStream &) = delete;
  Result poll(cv::Mat &frame, CameraControls &controls);
  std::uint64_t command(CameraCommand kind, int value = 0);
  std::uint64_t acknowledged() const { return acknowledged_; }
  bool commandSucceeded() const { return command_ok_; }
  double frameAge() const;
  const std::string &message() const { return message_; }
  bool controlsReady() const { return controls_ready_; }
  std::uint64_t generation() const { return generation_; }
  void stop();

private:
  using Clock = std::chrono::steady_clock;
  bool start();
  void recover(const std::string &reason);
  void retire();
  void reap();
  StreamOptions options_;
  std::string executable_, message_{"Starting camera"};
  StreamShared *shared_{nullptr};
  std::size_t mapping_size_{0};
  int shared_fd_{-1}, control_fd_{-1}; // 非阻塞控制管道的写端
  pid_t worker_{-1};
  std::vector<pid_t> retired_;
  std::uint64_t sequence_{0}, requested_{0}, acknowledged_{0}, generation_{0};
  bool command_ok_{false}, controls_ready_{false}, failed_{false},
      stopped_{false};
  double frame_time_{0};
  Clock::time_point activity_, next_start_, outage_;
  bool in_outage_{true};
};

int cameraWorkerMain(int argc, char **argv);
} // namespace aim
