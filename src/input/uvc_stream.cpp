#include "demo_aim/detail/input/uvc_stream.hpp"
#include "demo_aim/detail/input/uvc_stream_protocol.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <new>
#include <signal.h>
#include <spawn.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;

namespace aim {
namespace {
struct ScopedFd {
  int value;
  explicit ScopedFd(int descriptor) : value(descriptor) {}
  ScopedFd(const ScopedFd &) = delete;
  ScopedFd &operator=(const ScopedFd &) = delete;
  ~ScopedFd() {
    if (value >= 0)
      ::close(value);
  }
  int release() {
    const int result = value;
    value = -1;
    return result;
  }
};
// 单向控制使用管道，不需要网络能力。对端退出时不能让 SIGPIPE 结束预览。
ssize_t writeCommand(int fd, const StreamCommand &command) {
  sigset_t blocked, previous, pending;
  sigemptyset(&blocked);
  sigaddset(&blocked, SIGPIPE);
  if (pthread_sigmask(SIG_BLOCK, &blocked, &previous))
    return -1;
  sigpending(&pending);
  const bool already_pending = sigismember(&pending, SIGPIPE);
  const ssize_t result = ::write(fd, &command, sizeof(command));
  const int error = errno;
  if (result < 0 && error == EPIPE && !already_pending) {
    timespec immediate{};
    sigtimedwait(&blocked, nullptr, &immediate);
  }
  pthread_sigmask(SIG_SETMASK, &previous, nullptr);
  errno = error;
  return result;
}
} // namespace
CameraStream::CameraStream(StreamOptions options, std::string executable)
    : options_(std::move(options)), executable_(std::move(executable)) {
  if (options_.width < 16 || options_.height < 16 || options_.width > 8192 ||
      options_.height > 8192 || options_.stall_seconds <= 0 ||
      options_.reconnect_seconds < 0 ||
      (options_.frame_type != CV_8UC3 && options_.frame_type != CV_32FC3) ||
      options_.graceful_stop_ms < 0 || options_.graceful_stop_ms > 500)
    throw std::invalid_argument("Invalid camera stream settings");
  // 为协商到略大于请求尺寸的设备留余量，最大空间取决于用户请求。
  const std::size_t capacity = std::max<std::size_t>(
      1920 * 1080 * 3, std::size_t(options_.width) * options_.height *
                           CV_ELEM_SIZE(options_.frame_type));
  mapping_size_ = sizeof(StreamShared) + capacity;
  shared_fd_ = memfd_create("camera-frame", MFD_CLOEXEC);
  if (shared_fd_ < 0 || ftruncate(shared_fd_, mapping_size_) != 0) {
    if (shared_fd_ >= 0)
      ::close(shared_fd_);
    throw std::runtime_error("Could not allocate camera shared frame");
  }
  void *mapping = mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE,
                       MAP_SHARED, shared_fd_, 0);
  if (mapping == MAP_FAILED) {
    ::close(shared_fd_);
    throw std::runtime_error("Could not map camera shared frame");
  }
  shared_ = new (mapping) StreamShared{};
  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  int error = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
  if (!error)
    error = pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
  if (!error)
    error = pthread_mutex_init(&shared_->mutex, &attr);
  pthread_mutexattr_destroy(&attr);
  if (error) {
    munmap(shared_, mapping_size_);
    ::close(shared_fd_);
    throw std::runtime_error("Could not initialize camera shared mutex");
  }
  shared_->capacity = capacity;
  activity_ = next_start_ = outage_ = Clock::now();
}
CameraStream::~CameraStream() {
  stop();
  // 被内核阻塞的已终止子进程可能尚未结束；不能等待它或销毁其共享锁。
  munmap(shared_, mapping_size_);
  ::close(shared_fd_);
}
void CameraStream::reap() {
  for (auto it = retired_.begin(); it != retired_.end();) {
    int status = 0;
    const pid_t result = waitpid(*it, &status, WNOHANG);
    if (result == *it || (result < 0 && errno == ECHILD))
      it = retired_.erase(it);
    else
      ++it;
  }
}
void CameraStream::retire() {
  if (control_fd_ >= 0) {
    ::close(control_fd_);
    control_fd_ = -1;
  }
  if (worker_ > 0) {
    kill(worker_, SIGKILL);
    retired_.push_back(worker_);
    worker_ = -1;
  }
  controls_ready_ = false;
  frame_time_ = 0;
  reap();
}
void CameraStream::stop() {
  if (stopped_)
    return;
  stopped_ = true;
  if (worker_ > 0 && options_.graceful_stop_ms) {
    kill(worker_, SIGTERM);
    const auto deadline =
        Clock::now() + std::chrono::milliseconds(options_.graceful_stop_ms);
    while (Clock::now() < deadline) {
      if (waitpid(worker_, nullptr, WNOHANG) == worker_) {
        worker_ = -1;
        break;
      }
      usleep(2000);
    }
  }
  retire();
  // 给正常结束的子进程一个很短的回收窗口，绝不等待阻塞驱动。
  for (int i = 0; i < 20 && !retired_.empty(); ++i) {
    usleep(1000);
    reap();
  }
}
bool CameraStream::start() {
  reap();
  // 一个尚未退出的驱动进程可能还持有 USB；不不断创建新进程争抢设备。
  if (!retired_.empty())
    return false;
  int lock = pthread_mutex_trylock(&shared_->mutex);
  if (lock == EOWNERDEAD)
    pthread_mutex_consistent(&shared_->mutex);
  else if (lock != 0)
    return false;
  shared_->sequence = shared_->acknowledged = 0;
  shared_->frame_time = 0;
  shared_->width = shared_->height = 0;
  shared_->stage = kStreamStarting;
  shared_->frame_type = options_.frame_type;
  shared_->controls = {};
  shared_->message[0] = '\0';
  pthread_mutex_unlock(&shared_->mutex);
  sequence_ = acknowledged_ = 0;
  std::vector<std::string> args{executable_,
                                "--camera-worker",
                                options_.device,
                                std::to_string(options_.width),
                                std::to_string(options_.height),
                                std::to_string(options_.fps),
                                options_.format,
                                options_.controls ? "1" : "0"};
  std::vector<char *> argv;
  for (auto &arg : args)
    argv.push_back(arg.data());
  argv.push_back(nullptr);
  int channel[2];
  if (pipe2(channel, O_NONBLOCK | O_CLOEXEC))
    throw std::runtime_error("Could not create camera control channel");
  ScopedFd channel_read{channel[0]}, channel_write{channel[1]};
  // 源描述符也可能恰好为 198/199。先复制到协议目标范围之外，
  // 避免前一个 dup2 覆盖下一个动作还需要的控制管道。
  ScopedFd spawn_shared{
      fcntl(shared_fd_, F_DUPFD_CLOEXEC, aim::kStreamCommandFd + 1)};
  if (spawn_shared.value < 0)
    throw std::runtime_error("Could not duplicate camera shared descriptor");
  ScopedFd spawn_command{
      fcntl(channel_read.value, F_DUPFD_CLOEXEC, aim::kStreamCommandFd + 1)};
  if (spawn_command.value < 0)
    throw std::runtime_error("Could not duplicate camera control descriptor");
  // posix_spawn/exec 使用全新的 OpenCV 运行时，避免对已初始化 GUI/线程池 fork。
  posix_spawn_file_actions_t actions;
  int error = posix_spawn_file_actions_init(&actions);
  const bool actions_ready = error == 0;
  if (!error)
    error = posix_spawn_file_actions_adddup2(
        &actions, spawn_shared.value, aim::kStreamSharedFd);
  if (!error)
    error = posix_spawn_file_actions_adddup2(
        &actions, spawn_command.value, aim::kStreamCommandFd);
  if (!error)
    error = posix_spawn(&worker_, executable_.c_str(), &actions, nullptr,
                        argv.data(), environ);
  if (actions_ready)
    posix_spawn_file_actions_destroy(&actions);
  if (error) {
    worker_ = -1;
    throw std::runtime_error("Could not start camera worker: " +
                             std::string(std::strerror(error)));
  }
  control_fd_ = channel_write.release();
  ++generation_;
  activity_ = Clock::now();
  message_ = "Starting camera";
  return true;
}
void CameraStream::recover(const std::string &reason) {
  message_ = reason;
  std::cerr << "Camera stream: " << reason << "; waiting to reconnect\n";
  if (!in_outage_) {
    outage_ = Clock::now();
    in_outage_ = true;
  }
  retire();
  if (options_.reconnect_seconds == 0)
    failed_ = true;
  next_start_ = Clock::now() + std::chrono::seconds(1);
}
double CameraStream::frameAge() const {
  return frame_time_ > 0 ? std::max(0.0, streamTime() - frame_time_) : 1e9;
}
std::uint64_t CameraStream::command(CameraCommand kind, int value) {
  if (control_fd_ < 0 || !controls_ready_)
    return 0;
  StreamCommand command{++requested_, static_cast<int>(kind), value};
  if (writeCommand(control_fd_, command) !=
      static_cast<ssize_t>(sizeof(command)))
    return 0;
  return command.id;
}
CameraStream::Result CameraStream::poll(cv::Mat &frame,
                                        CameraControls &controls) {
  reap();
  if (stopped_ || failed_)
    return Result::Failed;
  const auto now = Clock::now();
  if (in_outage_ &&
      now > outage_ + std::chrono::duration<double>(options_.reconnect_seconds +
                                                    options_.stall_seconds)) {
    failed_ = true;
    retire();
    message_ +=
        "; reconnect deadline exceeded; check USB cable/port and kernel log";
    return Result::Failed;
  }
  if (worker_ < 0) {
    if (now >= next_start_ && !start())
      next_start_ = now + std::chrono::seconds(1);
    return in_outage_ ? Result::Recovering : Result::Waiting;
  }
  int lock = pthread_mutex_trylock(&shared_->mutex);
  if (lock == EOWNERDEAD) {
    pthread_mutex_consistent(&shared_->mutex);
    pthread_mutex_unlock(&shared_->mutex);
    recover("Camera worker ended while publishing a frame");
    return Result::Recovering;
  }
  if (lock == 0) {
    const auto &state = shared_->controls;
    controls_ready_ = state.available;
    controls.has_focus = state.has_focus;
    controls.has_gain = state.has_gain;
    controls.has_auto_focus = state.has_auto_focus;
    controls.focus_min = state.focus_min;
    controls.focus_max = state.focus_max;
    controls.focus_value = state.focus_value;
    controls.focus_status = state.focus_status;
    controls.gain_min = state.gain_min;
    controls.gain_max = state.gain_max;
    controls.gain_value = state.gain_value;
    acknowledged_ = shared_->acknowledged;
    command_ok_ = shared_->command_ok;
    const int stage = shared_->stage;
    if (shared_->message[0])
      message_ = shared_->message;
    bool fresh = false;
    if (stage == aim::kStreamLive && shared_->sequence != sequence_ &&
        streamTime() - shared_->frame_time <= 0.25 && shared_->width > 0 &&
        shared_->height > 0 && shared_->frame_type == options_.frame_type &&
        std::uint64_t(shared_->width) * shared_->height *
                CV_ELEM_SIZE(options_.frame_type) <=
            shared_->capacity) {
      cv::Mat view(shared_->height, shared_->width, options_.frame_type,
                   shared_ + 1);
      try {
        view.copyTo(frame);
      } catch (...) {
        pthread_mutex_unlock(&shared_->mutex);
        throw;
      }
      sequence_ = shared_->sequence;
      frame_time_ = shared_->frame_time;
      fresh = true;
    }
    pthread_mutex_unlock(&shared_->mutex);
    if (stage == aim::kStreamFailed) {
      recover(message_);
      return Result::Recovering;
    }
    if (fresh) {
      if (in_outage_)
        std::cerr << "Camera stream ready: " << message_ << '\n';
      activity_ = now;
      in_outage_ = false;
      return Result::Frame;
    }
  }
  int status = 0;
  if (waitpid(worker_, &status, WNOHANG) == worker_) {
    worker_ = -1;
    recover("Camera worker exited (status " + std::to_string(status) + ")");
    return Result::Recovering;
  }
  if (now > activity_ + std::chrono::duration<double>(options_.stall_seconds)) {
    recover("No fresh camera frame for " +
            std::to_string(options_.stall_seconds) + " s");
    return Result::Recovering;
  }
  return Result::Waiting;
}
} // namespace aim
