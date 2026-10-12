#pragma once
#include <cstdint>
#include <pthread.h>
#include <time.h>

namespace aim {
// Linux 进程间协议，仅在摄像头应用内部使用。
struct StreamControls {
  int available{0}, has_focus{0}, has_gain{0}, has_auto_focus{0};
  int focus_min{0}, focus_max{0}, focus_value{0};
  int gain_min{0}, gain_max{0}, gain_value{0};
  char focus_status[16]{};
};
struct StreamShared {
  pthread_mutex_t mutex;
  std::uint64_t capacity{0}, sequence{0}, acknowledged{0};
  double frame_time{0};
  int frame_type{16}; // CV_8UC3；由发布方明确指定。
  int width{0}, height{0}, stage{0},
      command_ok{0}; // stage: 0=start, 1=live, 2=error
  StreamControls controls;
  char message[256]{};
  // 紧接结构体的是 capacity 字节、frame_type 类型的紧凑像素数据。
};
struct StreamCommand {
  std::uint64_t id;
  int kind, value;
};
inline double streamTime() {
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return value.tv_sec + value.tv_nsec * 1e-9;
}
} // namespace aim
