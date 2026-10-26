// 独立测距诊断，不关联彩色装甲候选；SDK 的 World 指深度相机系，不是地图系。
#include <OpenNI.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
struct Settings {
  int width{640}, height{480}, frames{120};
  std::optional<int> u, v;
};
int number(const std::string &s) {
  std::size_t used = 0;
  const int value = std::stoi(s, &used);
  if (used != s.size())
    throw std::invalid_argument("Invalid integer: " + s);
  return value;
}
Settings parse(int argc, char **argv) {
  Settings s;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (++i == argc)
      throw std::invalid_argument("Missing value: " + key);
    const int value = number(argv[i]);
    if (key == "--width")
      s.width = value;
    else if (key == "--height")
      s.height = value;
    else if (key == "--frames")
      s.frames = value;
    else if (key == "--u")
      s.u = value;
    else if (key == "--v")
      s.v = value;
    else
      throw std::invalid_argument("Unknown option: " + key);
  }
  if (s.width < 16 || s.width > 1280 || s.height < 16 || s.height > 1280 ||
      s.frames < 31 || s.frames > 300 || s.u.has_value() != s.v.has_value() ||
      (s.u &&
       (*s.u < 3 || *s.u >= s.width - 3 || *s.v < 3 || *s.v >= s.height - 3)))
    throw std::invalid_argument("Invalid size/frame count/pixel (ROI needs "
                                "3-pixel margin; frames 31..300)");
  return s;
}
void require(openni::Status status, const char *what) {
  if (status != openni::STATUS_OK)
    throw std::runtime_error(std::string(what) + ": " +
                             openni::OpenNI::getExtendedError());
}
struct Context {
  Context() { require(openni::OpenNI::initialize(), "OpenNI initialize"); }
  ~Context() { openni::OpenNI::shutdown(); }
};
float median(std::vector<float> values) {
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  const float high = *middle;
  return values.size() % 2
             ? high
             : (high + *std::max_element(values.begin(), middle)) / 2;
}
} // namespace
int main(int argc, char **argv) {
  try {
    for (int i = 1; i < argc; ++i)
      if (std::string(argv[i]) == "--help") {
        std::cout
            << "Astra Pro depth check (no armor detection / flight control)\n"
               "--width 640 --height 480 --frames 120 [--u DEPTH_PIXEL --v "
               "DEPTH_PIXEL]\n"
               "Frames: 31..300; 30-frame warmup; maximum run time 12s.\n"
               "Default sample: center 7x7 ROI. Coordinates: depth optical "
               "frame, meters,\n"
               "X right, Y down, Z forward; this is NOT a world/map frame.\n"
               "Exit 0: valid ROI XYZ after warmup; 4: no valid ROI XYZ.\n";
        return 0;
      }
    const auto settings = parse(argc, argv);
    std::cout << std::unitbuf << std::fixed << std::setprecision(4);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    Context context;
    openni::Array<openni::DeviceInfo> infos;
    openni::OpenNI::enumerateDevices(&infos);
    const char *uri = nullptr;
    for (int i = 0; i < infos.getSize(); ++i)
      if (infos[i].getUsbVendorId() == 0x2bc5 &&
          infos[i].getUsbProductId() == 0x0403) {
        if (uri)
          throw std::runtime_error(
              "Multiple Astra Pro devices; connect one for this diagnostic");
        uri = infos[i].getUri();
      }
    if (!uri)
      throw std::runtime_error("Astra Pro 2bc5:0403 not found; check USB and "
                               "SDK library/driver paths");
    openni::Device device;
    require(device.open(uri), "Open Astra Pro");
    openni::VideoStream stream;
    require(stream.create(device, openni::SENSOR_DEPTH), "Create depth stream");
    require(device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF),
            "Select raw depth coordinates");
    auto mode = stream.getVideoMode();
    mode.setResolution(settings.width, settings.height);
    mode.setFps(30);
    mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    require(stream.setVideoMode(mode), "Set depth mode");
    require(stream.setMirroringEnabled(false), "Disable depth mirroring");
    require(stream.start(), "Start depth stream");
    mode = stream.getVideoMode();
    if (mode.getResolutionX() != settings.width ||
        mode.getResolutionY() != settings.height ||
        mode.getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM ||
        stream.getMirroringEnabled() ||
        device.getImageRegistrationMode() != openni::IMAGE_REGISTRATION_OFF)
      throw std::runtime_error(
          "Unexpected negotiated depth mode/mirroring; coordinates rejected");
    const int u = settings.u.value_or(settings.width / 2);
    const int v = settings.v.value_or(settings.height / 2);
    std::cout << "depth=" << settings.width << 'x' << settings.height << "@"
              << mode.getFps() << " sample_depth_uv=[" << u << ',' << v
              << "] registration_supported="
              << device.isImageRegistrationModeSupported(
                     openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)
              << '\n';
    const auto start = std::chrono::steady_clock::now();
    int frames = 0, depth_frames = 0, xyz_frames = 0;
    std::vector<float> depths;
    while (
        !stopped && frames < settings.frames &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count() < 12) {
      openni::VideoStream *pointer = &stream;
      int changed = 0;
      require(openni::OpenNI::waitForAnyStream(&pointer, 1, &changed, 1000),
              "Wait for depth frame");
      openni::VideoFrameRef frame;
      require(stream.readFrame(&frame), "Read depth frame");
      if (!frame.isValid() || frame.getWidth() != settings.width ||
          frame.getHeight() != settings.height ||
          frame.getVideoMode().getPixelFormat() !=
              openni::PIXEL_FORMAT_DEPTH_1_MM)
        throw std::runtime_error("Invalid depth frame layout/unit");
      ++frames;
      int valid = 0;
      std::vector<float> roi;
      for (int y = 0; y < frame.getHeight(); ++y) {
        const auto row = reinterpret_cast<const openni::DepthPixel *>(
            static_cast<const char *>(frame.getData()) +
            y * frame.getStrideInBytes());
        for (int x = 0; x < frame.getWidth(); ++x)
          if (row[x] >= 100 && row[x] <= 10000) {
            ++valid;
            if (std::abs(x - u) <= 3 && std::abs(y - v) <= 3)
              roi.push_back(row[x]);
          }
      }
      const bool report =
          frames == 1 || frames % 30 == 0 || frames == settings.frames;
      if (frames > 30 && valid)
        ++depth_frames;
      std::string reason = "insufficient_depth";
      float z = 0, mad = 0, wx = 0, wy = 0, wz = 0;
      bool point_valid = false;
      if (roi.size() >=
          30) { // 至少 60% 的 49 像素有效，不能拿孤立点冒充稳定测量。
        z = median(roi);
        std::vector<float> deviations;
        for (float value : roi)
          deviations.push_back(std::abs(value - z));
        mad = median(deviations);
        if (mad > 40)
          reason = "inconsistent_depth";
        else if (std::count_if(roi.begin(), roi.end(), [&](float value) {
                   return std::abs(value - z) <= std::max(3 * mad, 40.0f);
                 }) < roi.size() * 0.8)
          reason = "mixed_depth_layers";
        else {
          point_valid = openni::CoordinateConverter::convertDepthToWorld(
                            stream, float(u), float(v), z, &wx, &wy, &wz) ==
                            openni::STATUS_OK &&
                        std::isfinite(wx) && std::isfinite(wy) &&
                        std::isfinite(wz) && wz > 0;
          reason = point_valid ? "ok" : "deprojection_failed";
        }
      }
      if (point_valid && frames > 30) {
        ++xyz_frames;
        depths.push_back(wz / 1000);
      }
      if (report) {
        std::cout << "frame=" << frames << " valid_depth=" << valid << '/'
                  << frame.getWidth() * frame.getHeight()
                  << " ROI=" << roi.size() << "/49 reason=" << reason;
        if (point_valid)
          // OpenNI World 的 Y 向上；显式转换为本工程光学系 Y
          // 向下，并将毫米转换成米。
          std::cout << " xyz_depth_optical_m=[" << wx / 1000 << ','
                    << -wy / 1000 << ',' << wz / 1000 << "] range_m="
                    << std::sqrt(wx * wx + wy * wy + wz * wz) / 1000
                    << " MAD_m=" << mad / 1000;
        else
          std::cout << " xyz_depth_optical_m=null";
        std::cout << '\n';
      }
    }
    stream.stop();
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    std::cout << "frames=" << frames
              << " post_warmup_depth_frames=" << depth_frames
              << " valid_ROI_xyz_frames=" << xyz_frames
              << " elapsed_s=" << elapsed;
    if (!depths.empty())
      std::cout << " ROI_z_median_m=" << median(depths) << " ROI_z_min_m="
                << *std::min_element(depths.begin(), depths.end())
                << " ROI_z_max_m="
                << *std::max_element(depths.begin(), depths.end());
    std::cout << "\nROI point is not an armor coordinate until RGB-to-depth "
                 "correspondence is verified.\n";
    return stopped ? 130
                   : (frames == settings.frames && xyz_frames > 0 ? 0 : 4);
  } catch (const std::exception &e) {
    std::cerr << "astra_depth_check: " << e.what() << '\n';
    return 1;
  }
}
