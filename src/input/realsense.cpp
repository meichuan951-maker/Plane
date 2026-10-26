// D435i 实时采集适配；编译开关关闭时提供明确报错的替代实现。
#include "demo_aim/input.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#ifdef AIM_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#endif
namespace aim {
#ifdef AIM_HAVE_REALSENSE
// 相机管线、对齐器和深度比例由 Impl 持有；析构兜底停止已启动管线。
struct RealSense::Impl {
  rs2::pipeline pipeline;
  rs2::align align{RS2_STREAM_COLOR};
  float scale{};
  std::uint64_t sequence{};
  bool started{false};
  ~Impl() {
    if (started) {
      try {
        pipeline.stop();
      } catch (...) {
      }
    }
  }
};
// 启动 640×480 的彩色和深度流；支持时减少队列积压并启用全局时间。
RealSense::RealSense(const std::string &serial)
    : impl_(std::make_unique<Impl>()) {
  rs2::config config;
  if (!serial.empty())
    config.enable_device(serial);
  config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  const auto profile = impl_->pipeline.start(config);
  impl_->started = true;
  for (auto sensor : profile.get_device().query_sensors()) {
    if (sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
      sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 1);
    if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1);
  }
  impl_->scale =
      profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
}
// 等待最多 5 秒；采集与对齐之后复制图像，避免 SDK 帧释放导致悬空内存。
Frame RealSense::read() {
  const auto frames = impl_->pipeline.wait_for_frames(5000);
  const auto color_raw = frames.get_color_frame();
  const auto depth_raw = frames.get_depth_frame();
  if (!color_raw || !depth_raw)
    throw std::runtime_error("Incomplete RGB-D capture");
  const auto cd = color_raw.get_frame_timestamp_domain(),
             dd = depth_raw.get_frame_timestamp_domain();
// 只有两路处于同一种系统/全局时间域时才认为时钟可比较。
  const bool clock = cd == dd && (cd == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME ||
                                  cd == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME);
  const double ct = color_raw.get_timestamp() / 1000.,
               dt = depth_raw.get_timestamp() / 1000.;
  const double now = monotonicNow();
  const double wall = std::chrono::duration<double>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
// 把深度投影到彩色像素网格；后续用对齐后深度 profile 的内参。
  auto aligned = impl_->align.process(frames);
  const auto depth = aligned.get_depth_frame();
  const auto color = aligned.get_color_frame();
  const auto k =
      depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
  Frame f;
  f.color_bgr =
      cv::Mat(color.get_height(), color.get_width(), CV_8UC3,
              const_cast<void *>(color.get_data()), color.get_stride_in_bytes())
          .clone();
  cv::Mat raw(depth.get_height(), depth.get_width(), CV_16UC1,
              const_cast<void *>(depth.get_data()),
              depth.get_stride_in_bytes());
  raw.convertTo(f.depth_m, CV_32F, impl_->scale);
  f.intrinsics = {k.width, k.height, k.fx, k.fy, k.ppx, k.ppy};
// 将设备的系统时间估计映射到本机单调时钟；取两路中较早时间作保守年龄。
  f.capture_s = clock ? now - (wall - std::min(ct, dt)) : now;
  f.sync_error_s = std::abs(ct - dt);
  f.clock_valid = clock;
  f.received_s = now;
  f.sequence = impl_->sequence++;
  f.source = "d435i_light_bars_cpp";
// 捕获本帧内参，通过 SDK 反投影处理支持的畸变模型。
  f.deproject = [k](double u, double v, double z) {
    float point[3], pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
    // SDK deprojection does not support modified Brown-Conrady intrinsics.
    if (k.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY)
      throw std::runtime_error("Unsupported deprojection distortion");
    rs2_deproject_pixel_to_point(point, &k, pixel, static_cast<float>(z));
    return cv::Vec3d(point[0], point[1], point[2]);
  };
  return f;
}
#else
struct RealSense::Impl {};
RealSense::RealSense(const std::string &) {
  throw std::runtime_error(
      "Built without RealSense: install SDK and reconfigure CMake");
}
Frame RealSense::read() { throw std::runtime_error("RealSense unavailable"); }
#endif
// 在 Impl 完整定义处生成析构，触发其自动资源清理。
RealSense::~RealSense() = default;
} // namespace aim
