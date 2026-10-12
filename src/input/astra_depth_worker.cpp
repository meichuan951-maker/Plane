#include "astra_depth_worker.hpp"
#include "uvc_stream_protocol.hpp"
#include <OpenNI.h>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <unistd.h>
namespace aim {
namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
void require(openni::Status s, const char *message) {
  if (s != openni::STATUS_OK)
    throw std::runtime_error(std::string(message) + ": " +
                             openni::OpenNI::getExtendedError());
}
void lock(StreamShared *s) {
  const int result = pthread_mutex_lock(&s->mutex);
  if (result == EOWNERDEAD)
    pthread_mutex_consistent(&s->mutex);
  else if (result)
    throw std::runtime_error("Depth shared mutex failed");
}
struct Context {
  Context() { require(openni::OpenNI::initialize(), "OpenNI initialize"); }
  ~Context() { openni::OpenNI::shutdown(); }
};
} // namespace
int astraDepthWorkerMain() {
  struct stat info {};
  constexpr std::size_t bytes = 640 * 480 * sizeof(cv::Vec3f);
  if (fstat(198, &info) || info.st_size < off_t(sizeof(StreamShared) + bytes))
    return 2;
  auto *shared = static_cast<StreamShared *>(
      mmap(nullptr, info.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, 198, 0));
  if (shared == MAP_FAILED)
    return 2;
  prctl(PR_SET_PDEATHSIG, SIGKILL);
  if (getppid() == 1)
    return 2;
  std::signal(SIGTERM, stop);
  std::signal(SIGINT, stop);
  try {
    cv::setNumThreads(1);
    Context context;
    openni::Array<openni::DeviceInfo> infos;
    openni::OpenNI::enumerateDevices(&infos);
    const char *uri = nullptr;
    for (int i = 0; i < infos.getSize(); ++i)
      if (infos[i].getUsbVendorId() == 0x2bc5 &&
          infos[i].getUsbProductId() == 0x0403) {
        if (uri)
          throw std::runtime_error("Connect exactly one Astra Pro");
        uri = infos[i].getUri();
      }
    if (!uri)
      throw std::runtime_error("Astra Pro not found; reconnect USB");
    openni::Device device;
    require(device.open(uri), "Open Astra Pro");
    openni::VideoStream depth;
    require(depth.create(device, openni::SENSOR_DEPTH), "Create depth");
    require(device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF),
            "Select raw depth");
    auto mode = depth.getVideoMode();
    mode.setResolution(640, 480);
    mode.setFps(30);
    mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    require(depth.setVideoMode(mode), "Select 640x480 mm depth");
    require(depth.setMirroringEnabled(false), "Disable mirroring");
    require(depth.start(), "Start depth");
    mode = depth.getVideoMode();
    if (mode.getResolutionX() != 640 || mode.getResolutionY() != 480 ||
        mode.getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM ||
        depth.getMirroringEnabled() ||
        device.getImageRegistrationMode() != openni::IMAGE_REGISTRATION_OFF)
      throw std::runtime_error("Unexpected depth layout or coordinate system");
    // 缓存 SDK 反投影的单位射线；不使用无效的 RGB 内参，也不猜 RGB-D 对应。
    cv::Mat rays(480, 640, CV_32FC3), xyz(480, 640, CV_32FC3);
    for (int y = 0; y < 480; ++y)
      for (int x = 0; x < 640; ++x) {
        float wx, wy, wz;
        require(openni::CoordinateConverter::convertDepthToWorld(
                    depth, float(x), float(y), 1000.f, &wx, &wy, &wz),
                "Depth SDK deprojection");
        if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz) ||
            std::abs(wz - 1000) > 0.1)
          throw std::runtime_error("Invalid SDK depth projection");
        rays.at<cv::Vec3f>(y, x) = {wx / 1000, -wy / 1000, wz / 1000};
      }
    for (cv::Point p :
         {cv::Point(0, 0), cv::Point(639, 479), cv::Point(320, 240)}) {
      float wx, wy, wz;
      require(openni::CoordinateConverter::convertDepthToWorld(
                  depth, float(p.x), float(p.y), 2000.f, &wx, &wy, &wz),
              "Check SDK projection scale");
      if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz) ||
          cv::norm(cv::Vec3f(wx, -wy, wz) / 1000 - 2 * rays.at<cv::Vec3f>(p)) >
              0.0001)
        throw std::runtime_error("SDK projection is not linear in depth");
    }
    int frames = 0;
    while (!stopped) {
      openni::VideoStream *pointer = &depth;
      int changed = 0;
      const auto status =
          openni::OpenNI::waitForAnyStream(&pointer, 1, &changed, 100);
      if (status == openni::STATUS_TIME_OUT)
        continue;
      require(status, "Wait for depth");
      openni::VideoFrameRef frame;
      require(depth.readFrame(&frame), "Read depth");
      const double stamp = streamTime();
      if (!frame.isValid() || frame.getWidth() != 640 ||
          frame.getHeight() != 480 ||
          frame.getVideoMode().getPixelFormat() !=
              openni::PIXEL_FORMAT_DEPTH_1_MM)
        throw std::runtime_error("Invalid depth frame");
      if (++frames <= 30)
        continue;
      for (int y = 0; y < 480; ++y) {
        const auto *z = reinterpret_cast<const openni::DepthPixel *>(
            static_cast<const char *>(frame.getData()) +
            y * frame.getStrideInBytes());
        auto *dst = xyz.ptr<cv::Vec3f>(y);
        const auto *ray = rays.ptr<cv::Vec3f>(y);
        for (int x = 0; x < 640; ++x)
          dst[x] = z[x] >= 100 && z[x] <= 10000 ? ray[x] * (z[x] * 0.001f)
                                                : cv::Vec3f(0, 0, 0);
      }
      lock(shared);
      std::memcpy(reinterpret_cast<unsigned char *>(shared + 1), xyz.data,
                  bytes);
      shared->width = 640;
      shared->height = 480;
      shared->frame_type = CV_32FC3;
      shared->frame_time = stamp;
      shared->stage = 1;
      ++shared->sequence;
      std::snprintf(shared->message, sizeof(shared->message),
                    "Astra Pro 640x480 raw depth; SDK projection unverified");
      pthread_mutex_unlock(&shared->mutex);
    }
    depth.stop();
    depth.destroy();
    device.close();
  } catch (const std::exception &e) {
    lock(shared);
    shared->stage = 2;
    std::snprintf(shared->message, sizeof(shared->message), "%s", e.what());
    pthread_mutex_unlock(&shared->mutex);
    munmap(shared, info.st_size);
    return 1;
  }
  munmap(shared, info.st_size);
  return 0;
}
} // namespace aim
