#include "../src/app/camera/preview.hpp"
#include "../src/input/uvc_stream.hpp"
#include "../src/input/uvc_stream_protocol.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
// 独立的驱动替身进程：通过与真实驱动相同的 IPC 发布帧，也可故意永远卡住。
int fixture(const std::string &mode) {
  prctl(PR_SET_PDEATHSIG, SIGKILL);
  if (getppid() == 1)
    return 2;
  struct stat info {};
  if (fstat(198, &info))
    return 2;
  auto *shared = static_cast<aim::StreamShared *>(
      mmap(nullptr, info.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, 198, 0));
  if (shared == MAP_FAILED)
    return 2;
  if (mode == "blocked-open")
    while (true)
      pause();
  if (mode == "closed-control")
    close(199);
  const bool recover = mode.rfind("recover:", 0) == 0;
  const std::string marker = recover ? mode.substr(8) : "";
  const bool second = recover && std::filesystem::exists(marker);
  const auto start = Clock::now();
  int focus = 8;
  std::uint64_t ack = 0;
  for (;;) {
    aim::StreamCommand command{};
    while (read(199, &command, sizeof(command)) == ssize_t(sizeof(command))) {
      if (mode == "blocked-control")
        while (true)
          pause();
      if (command.kind == static_cast<int>(aim::CameraCommand::Focus))
        focus = command.value;
      ack = command.id;
    }
    const double age =
        std::chrono::duration<double>(Clock::now() - start).count();
    if (recover && !second && age > 0.2) {
      std::ofstream(marker).put('1');
      pthread_mutex_lock(&shared->mutex);
      shared->stage = 2;
      std::snprintf(shared->message, sizeof(shared->message),
                    "injected disconnect");
      pthread_mutex_unlock(&shared->mutex);
      return 1;
    }
    pthread_mutex_lock(&shared->mutex);
    if (mode == "blocked-publish" && age > 0.15)
      while (true)
        pause();
    if (mode == "dead-owner" && age > 0.15) {
      raise(SIGKILL);
      return 1;
    }
    shared->stage = 1;
    shared->width = shared->height = 64;
    shared->frame_type = mode == "depth" ? CV_32FC3 : CV_8UC3;
    auto *pixels = reinterpret_cast<unsigned char *>(shared + 1);
    const int contrast = std::max(1, 240 - std::abs(focus - 8) * 25);
    if (mode == "depth") {
      cv::Mat depth(64, 64, CV_32FC3, pixels);
      depth.setTo(cv::Scalar(.25, -.3, 1.1));
    } else for (int y = 0; y < 64; ++y)
      for (int x = 0; x < 64; ++x)
        for (int c = 0; c < 3; ++c)
          pixels[(y * 64 + x) * 3 + c] =
              mode == "scan" ? ((x / 2 + y / 2) % 2 ? contrast : 0)
                             : (second ? 222 : shared->sequence % 200);
    ++shared->sequence;
    shared->frame_time = aim::streamTime();
    shared->acknowledged = ack;
    shared->command_ok = 1;
    auto &state = shared->controls;
    state.available = state.has_focus = 1;
    state.focus_min = 0;
    state.focus_max = 16;
    state.focus_value = focus;
    std::snprintf(state.focus_status, sizeof(state.focus_status), "manual");
    std::snprintf(shared->message, sizeof(shared->message), "fixture %s",
                  mode.c_str());
    pthread_mutex_unlock(&shared->mutex);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
aim::StreamOptions settings(const std::string &mode, double retries = 0) {
  aim::StreamOptions options;
  options.device = mode;
  options.width = options.height = 64;
  options.stall_seconds = 0.5;
  options.reconnect_seconds = retries;
  return options;
}
template <class Check>
void until(aim::CameraStream &stream, cv::Mat &frame,
           aim::CameraControls &controls, double limit, Check check) {
  const auto end = Clock::now() + std::chrono::duration<double>(limit);
  while (Clock::now() < end) {
    const auto begin = Clock::now();
    const auto result = stream.poll(frame, controls);
    require(Clock::now() - begin < std::chrono::milliseconds(100),
            "poll blocked the UI");
    if (check(result))
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("fixture deadline exceeded");
}
} // namespace

int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--camera-worker")
    return fixture(argv[2]);
  try {
    cv::Mat frame;
    aim::CameraControls controls;
    {
      auto s = settings("depth"); s.frame_type = CV_32FC3;
      aim::CameraStream stream(s);
      until(stream, frame, controls, 2,
            [](auto r) { return r == aim::CameraStream::Result::Frame; });
      require(frame.type() == CV_32FC3 &&
              cv::norm(frame.at<cv::Vec3f>(0, 0) - cv::Vec3f(.25, -.3, 1.1)) < 1e-6,
              "depth IPC changed XYZ type or units");
    }
    {
      auto s = settings("live"); s.frame_type = CV_32FC3;
      aim::CameraStream stream(s);
      until(stream, frame, controls, 2, [](auto r) {
        require(r != aim::CameraStream::Result::Frame,
                "BGR payload accepted as depth XYZ");
        return r == aim::CameraStream::Result::Failed;
      });
    }
    {
      aim::CameraStream stream(settings("live"));
      until(stream, frame, controls, 2,
            [](auto r) { return r == aim::CameraStream::Result::Frame; });
      const int old = frame.at<cv::Vec3b>(0, 0)[0];
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      until(stream, frame, controls, 1,
            [](auto r) { return r == aim::CameraStream::Result::Frame; });
      require(frame.at<cv::Vec3b>(0, 0)[0] > old + 5,
              "queued old frames were replayed");
      const auto command = stream.command(aim::CameraCommand::Focus, 12);
      require(command != 0, "control request not sent");
      until(stream, frame, controls, 1,
            [&](auto) { return stream.acknowledged() == command; });
      require(controls.focus_value == 12 && stream.commandSucceeded(),
              "control result missing");
    }
    for (const std::string mode :
         {"blocked-open", "blocked-publish", "dead-owner", "blocked-control"}) {
      aim::CameraStream stream(settings(mode));
      if (mode == "blocked-control") {
        until(stream, frame, controls, 2,
              [](auto r) { return r == aim::CameraStream::Result::Frame; });
        require(stream.command(aim::CameraCommand::Focus, 0) != 0,
                "could not inject blocked control");
      }
      until(stream, frame, controls, 2,
            [](auto r) { return r == aim::CameraStream::Result::Failed; });
      const auto begin = Clock::now();
      stream.stop();
      require(Clock::now() - begin < std::chrono::milliseconds(100),
              "exit waited for blocked driver");
      require(stream.frameAge() > 0.25,
              "stale frame still advertised as fresh");
      std::cout << "Passed " << mode << '\n';
    }
    {
      const auto marker = std::filesystem::temp_directory_path() /
                          ("camera-recovery-test-" + std::to_string(getpid()));
      aim::CameraStream stream(settings("recover:" + marker.string(), 3));
      until(stream, frame, controls, 4, [&](auto r) {
        return r == aim::CameraStream::Result::Frame &&
               stream.generation() == 2;
      });
      require(frame.at<cv::Vec3b>(0, 0)[0] == 222,
              "reconnect reused the previous generation's frame");
      std::filesystem::remove(marker);
    }
    {
      aim::CameraStream stream(settings("closed-control"));
      until(stream, frame, controls, 2,
            [](auto r) { return r == aim::CameraStream::Result::Frame; });
      require(stream.command(aim::CameraCommand::Focus, 0) == 0,
              "closed control channel was reported as accepting requests");
      // 若管道写入产生的 SIGPIPE 未正确处理，测试进程会在上行调用中被杀死。
    }
    {
      aim::CameraStream stream(settings("scan"));
      until(stream, frame, controls, 2,
            [](auto r) { return r == aim::CameraStream::Result::Frame; });
      aim::camera_app::FocusScan scan;
      scan.start(stream, controls);
      int frames_during_scan = 0;
      until(stream, frame, controls, 6, [&](auto r) {
        const bool fresh = r == aim::CameraStream::Result::Frame;
        if (fresh)
          ++frames_during_scan;
        scan.tick(stream, controls, fresh ? frame : cv::Mat());
        return !scan.active();
      });
      require(frames_during_scan > 100 && controls.focus_value == 8,
              "focus scan stopped preview or selected the wrong position");
    }
    std::cout
        << "Camera stream responsiveness/reconnect/scan regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
