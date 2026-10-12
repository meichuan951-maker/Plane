#include "options.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace aim::camera_app {
namespace fs = std::filesystem;
int integer(const std::string &s) {
  std::size_t n;
  int value = std::stoi(s, &n);
  if (n != s.size())
    throw std::invalid_argument("Not an integer: " + s);
  return value;
}
void help() {
  std::cout
      << "USB camera / traditional red-blue light bar test (no YOLO or flight "
         "control)\n"
         "camera_light_test --device /dev/video4 [options]\n"
         "  --list                        List Linux video nodes; does not "
         "open cameras\n"
         "  --config config/light_bars.yaml\n"
         "  --width 640 --height 480 --fps 30 --format MJPG|YUYV\n"
         "  --frames N                    Stop after N frames; 0=until "
         "q/Esc/Ctrl+C\n"
         "  --no-show                     Headless, no windows\n"
         "  --no-controls                 Preview without exposure/focus/gain queries or writes\n"
         "  --reconnect-seconds N         Retry budget after stream failure; default 15, 0=exit\n"
         "  --mask                        Show color masks and threshold "
         "sliders\n"
         "  --save-dir NEW_DIR             Save last "
         "raw/overlay/masks/thresholds at exit\n"
         "  --image FILE                  Process a still image instead of a "
         "camera\n"
         "Keys: q/Esc exit, f scans focus, a enables camera autofocus, "
         "e attempts dark-frame exposure recovery, "
         "s saves snapshot under "
         "runs/camera_snapshots/\n"
         "Startup preserves camera settings and does not scan focus. Press f "
         "with the armor in frame; Focus and Gain sliders are camera units.\n"
         "Capture/control run in a supervised worker process; the preview stays "
         "responsive during a stalled driver. Press e only when the frame is dark.\n"
         "Focus writes are read back from the driver; this does not verify "
         "physical lens movement. Scans wait for settling at each position.\n"
         "Same-color light bars are drawn as an armor quadrilateral. yaw/pitch\n"
         "appear only for the plate nearest the image center, and only when\n"
         "camera_matrix in the config belongs to this camera.\n"
         "FPS is completed capture+processing+display throughput, NOT sensor "
         "FPS.\n";
}
void listDevices() {
  const fs::path base("/sys/class/video4linux");
  if (!fs::exists(base)) {
    std::cout << "No video devices visible. Check USB connection/device "
                 "permissions.\n";
    return;
  }
  for (const auto &entry : fs::directory_iterator(base)) {
    std::ifstream stream(entry.path() / "name");
    std::string name;
    std::getline(stream, name);
    std::cout << "/dev/" << entry.path().filename().string() << "  " << name
              << '\n';
  }
  std::cout << "Some nodes are metadata only. Use v4l2-ctl -d DEVICE --all to "
               "check Video Capture.\n";
}
Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--no-show") {
      o.show = false;
      continue;
    }
    if (key == "--mask") {
      o.masks = true;
      continue;
    }
    if (key == "--no-controls") {
      o.controls = false;
      continue;
    }
    if (i + 1 >= argc)
      throw std::invalid_argument("Missing value for " + key);
    const std::string value(argv[++i]);
    if (key == "--device")
      o.device = value;
    else if (key == "--image")
      o.image = value;
    else if (key == "--config")
      o.config = value;
    else if (key == "--format")
      o.format = value;
    else if (key == "--save-dir")
      o.save_dir = value;
    else if (key == "--width")
      o.width = integer(value);
    else if (key == "--height")
      o.height = integer(value);
    else if (key == "--fps")
      o.fps = integer(value);
    else if (key == "--frames")
      o.frames = integer(value);
    else if (key == "--reconnect-seconds")
      o.reconnect_seconds = integer(value);
    else
      throw std::invalid_argument("Unknown option: " + key);
  }
  if (o.device.empty() == o.image.empty())
    throw std::invalid_argument(
        "Specify exactly one of --device /dev/videoN or --image FILE");
  if (o.width < 16 || o.width > 8192 || o.height < 16 || o.height > 8192 ||
      o.fps < 1 || o.fps > 240 || o.frames < 0 || o.frames > 1000000 ||
      o.reconnect_seconds < 0 || o.reconnect_seconds > 300)
    throw std::invalid_argument("Invalid size/fps/frame count");
  if (o.format != "MJPG" && o.format != "YUYV")
    throw std::invalid_argument("--format must be MJPG or YUYV");
  return o;
}
}
