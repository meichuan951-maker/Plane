#include "demo_aim/detail/input/uvc_stream.hpp"
#include "demo_aim/detail/input/uvc_stream_protocol.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <opencv2/videoio.hpp>
#include <signal.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace aim {
namespace {
void lockShared(StreamShared *shared) {
  const int error = pthread_mutex_lock(&shared->mutex);
  if (error == EOWNERDEAD)
    pthread_mutex_consistent(&shared->mutex);
  else if (error)
    throw std::runtime_error("Camera shared mutex failed");
}
void publishControls(StreamShared *shared, const CameraControls &controls) {
  auto &state = shared->controls;
  state.available = controls.fd >= 0;
  state.has_focus = controls.has_focus;
  state.has_gain = controls.has_gain;
  state.has_auto_focus = controls.has_auto_focus;
  state.focus_min = controls.focus_min;
  state.focus_max = controls.focus_max;
  state.focus_value = controls.focus_value;
  state.gain_min = controls.gain_min;
  state.gain_max = controls.gain_max;
  state.gain_value = controls.gain_value;
  std::snprintf(state.focus_status, sizeof(state.focus_status), "%s",
                controls.focus_status.c_str());
}
} // namespace
int cameraWorkerMain(int argc, char **argv) {
  if (argc != 8)
    return 2;
  struct stat info {};
  if (fstat(aim::kStreamSharedFd, &info) ||
      info.st_size < static_cast<off_t>(sizeof(StreamShared)))
    return 2;
  void *mapping = mmap(nullptr, info.st_size, PROT_READ | PROT_WRITE,
                       MAP_SHARED, aim::kStreamSharedFd, 0);
  if (mapping == MAP_FAILED)
    return 2;
  auto *shared = static_cast<StreamShared *>(mapping);
  if (shared->capacity > std::uint64_t(info.st_size) - sizeof(StreamShared))
    return 2;
  // 主预览进程结束时，不留下继续采集的进程。
  prctl(PR_SET_PDEATHSIG, SIGKILL);
  if (getppid() == 1)
    return 2;
  try {
    cv::setNumThreads(1);
    setenv("OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT", "1", 0);
    cv::VideoCapture camera;
    if (!camera.open(argv[2], cv::CAP_V4L2))
      throw std::runtime_error(std::string("Cannot open ") + argv[2]);
    const std::string format(argv[6]);
    if (format != "MJPG" && format != "YUYV")
      throw std::invalid_argument("Invalid camera worker format");
    auto set = [&](int property, double value, const char *name) {
      if (!camera.set(property, value))
        std::cerr << "Warning: driver rejected " << name << '\n';
    };
    set(cv::CAP_PROP_FOURCC,
        cv::VideoWriter::fourcc(format[0], format[1], format[2], format[3]),
        "format");
    const int requested_width = std::stoi(argv[3]);
    const int requested_height = std::stoi(argv[4]);
    const int requested_fps = std::stoi(argv[5]);
    set(cv::CAP_PROP_FRAME_WIDTH, requested_width, "width");
    set(cv::CAP_PROP_FRAME_HEIGHT, requested_height, "height");
    set(cv::CAP_PROP_FPS, requested_fps, "fps");
    set(cv::CAP_PROP_BUFFERSIZE, 1, "buffer size");
    const double driver_fps = camera.get(cv::CAP_PROP_FPS);
    const std::string description =
        std::string(argv[2]) + ", V4L2, requested " +
        std::to_string(requested_width) + "x" +
        std::to_string(requested_height) + " " + format + " at " +
        std::to_string(requested_fps) + " FPS; negotiated " +
        std::to_string(int(camera.get(cv::CAP_PROP_FRAME_WIDTH))) + "x" +
        std::to_string(int(camera.get(cv::CAP_PROP_FRAME_HEIGHT))) +
        ", driver FPS " + std::to_string(driver_fps) + " (not measured)";
    std::cerr << "Camera stream: " << description << '\n';
    if (!std::isfinite(driver_fps) || driver_fps <= 0)
      std::cerr << "Warning: camera did not report a valid negotiated FPS; "
                   "measure fresh frames to verify throughput\n";
    else if (driver_fps + 0.5 < requested_fps)
      std::cerr << "Warning: requested " << requested_fps
                << " FPS, but camera negotiated " << driver_fps
                << " FPS; the requested frame rate was not accepted\n";
    CameraControls controls;
    bool controls_checked = std::string(argv[7]) != "1";
    cv::Mat raw;
    while (true) {
      StreamCommand command{};
      // 仅执行显式控制请求，不在取流启动时改写设备端成像参数。
      for (int i = 0;
           i < 16 && ::read(aim::kStreamCommandFd, &command, sizeof(command)) ==
                         static_cast<ssize_t>(sizeof(command));
           ++i) {
        bool ok = false;
        switch (static_cast<CameraCommand>(command.kind)) {
        case CameraCommand::Focus:
          ok = controls.setFocus(command.value);
          break;
        case CameraCommand::Gain:
          ok = controls.setGain(command.value);
          break;
        case CameraCommand::AutoFocus:
          ok = controls.setAutoFocus(command.value != 0);
          break;
        }
        lockShared(shared);
        publishControls(shared, controls);
        shared->acknowledged = command.id;
        shared->command_ok = ok;
        pthread_mutex_unlock(&shared->mutex);
      }
      if (!camera.read(raw) || raw.empty())
        throw std::runtime_error("Camera read failed or USB disconnected");
      const std::uint64_t bytes = raw.total() * raw.elemSize();
      if (raw.type() != CV_8UC3 || bytes > shared->capacity)
        throw std::runtime_error(
            "Negotiated image exceeds shared frame capacity or is not BGR");
      lockShared(shared);
      auto *pixels = reinterpret_cast<unsigned char *>(shared + 1);
      if (raw.isContinuous())
        std::memcpy(pixels, raw.data, bytes);
      else
        for (int row = 0; row < raw.rows; ++row)
          std::memcpy(pixels + std::size_t(row) * raw.cols * 3, raw.ptr(row),
                      raw.cols * 3);
      shared->width = raw.cols;
      shared->height = raw.rows;
      shared->frame_type = CV_8UC3;
      shared->frame_time = streamTime();
      ++shared->sequence;
      shared->stage = aim::kStreamLive;
      std::snprintf(shared->message, sizeof(shared->message), "%s",
                    description.c_str());
      publishControls(shared, controls);
      pthread_mutex_unlock(&shared->mutex);
      // 第一帧先显示，再查询可选控制；查询慢也不会挡住主窗口。
      if (!controls_checked) {
        if (!controls.openDevice(argv[2]))
          std::cerr << "Warning: camera controls unavailable\n";
        controls_checked = true;
      }
    }
  } catch (const std::exception &error) {
    lockShared(shared);
    shared->stage = aim::kStreamFailed;
    std::snprintf(shared->message, sizeof(shared->message), "%s", error.what());
    pthread_mutex_unlock(&shared->mutex);
    // 不在这里等待失效驱动清理；设备句柄由子进程结束回收。
    munmap(shared, info.st_size);
    return 1;
  }
}
} // namespace aim
