#include "demo_aim/detail/input/uvc_controls.hpp"
#include <algorithm>
#include <fcntl.h>
#include <iostream>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
namespace aim {
CameraControls::~CameraControls() {
  if (fd >= 0)
    ::close(fd);
}
bool CameraControls::set(unsigned id, int value) const {
  v4l2_control control{};
  control.id = id;
  control.value = value;
  return fd >= 0 && ioctl(fd, VIDIOC_S_CTRL, &control) == 0;
}
bool CameraControls::get(unsigned id, int &value) const {
  v4l2_control control{};
  control.id = id;
  if (fd < 0 || ioctl(fd, VIDIOC_G_CTRL, &control) < 0)
    return false;
  value = control.value;
  return true;
}
bool CameraControls::query(unsigned id, int &min_value, int &max_value,
                           int &value) {
  v4l2_queryctrl query{};
  query.id = id;
  if (ioctl(fd, VIDIOC_QUERYCTRL, &query) < 0 ||
      (query.flags & V4L2_CTRL_FLAG_DISABLED))
    return false;
  min_value = query.minimum;
  max_value = query.maximum;
  v4l2_control control{};
  control.id = id;
  value = query.default_value;
  if (ioctl(fd, VIDIOC_G_CTRL, &control) == 0)
    value = control.value;
  return true;
}
bool CameraControls::openDevice(const std::string &path) {
  fd = ::open(path.c_str(), O_RDWR);
  if (fd < 0)
    return false;
  int ignored = 0;
  has_exposure =
      query(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_min, exposure_max, ignored);
  has_focus = query(V4L2_CID_FOCUS_ABSOLUTE, focus_min, focus_max, focus_value);
  int auto_min = 0, auto_max = 0, auto_value = 0;
  has_auto_focus = query(V4L2_CID_FOCUS_AUTO, auto_min, auto_max, auto_value);
  if (has_auto_focus)
    focus_status = auto_value ? "auto" : "manual";
  else if (has_focus)
    focus_status = "manual";
  has_gain = query(V4L2_CID_GAIN, gain_min, gain_max, gain_value);
  return has_exposure || has_focus || has_gain;
}
void CameraControls::applyPreviewDefaults() {
  // YLR4K 切到手动快门后几帧就会全黑，预览必须停在自动曝光。
  useAutoExposure();
  if (has_gain)
    setGain(std::min(8, gain_max));
  // 数字锐度只能加重已有边缘。这颗镜头的 focus_absolute 从 0 到 289
  // 画面几乎不变，光学模糊要靠镜头环或把目标放到最近对焦距离之外。
  int sharp_min = 0, sharp_max = 0, sharp = 0;
  if (query(V4L2_CID_SHARPNESS, sharp_min, sharp_max, sharp)) {
    const int target = std::clamp(80, sharp_min, sharp_max);
    if (!set(V4L2_CID_SHARPNESS, target))
      std::cerr << "Warning: camera rejected sharpness\n";
  }
  if (has_auto_focus)
    setAutoFocus(true);
  std::cerr << "Preview defaults: auto exposure, gain " << gain_value
            << ", sharpness 80, focus " << focus_status << '\n';
}
void CameraControls::useAutoExposure() {
  if (has_exposure && !set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO))
    std::cerr << "Warning: camera rejected auto exposure\n";
}
bool CameraControls::setAutoFocus(bool enabled) {
  if (!has_auto_focus) {
    if (enabled) {
      std::cerr << "Warning: camera has no continuous autofocus control\n";
      return false;
    }
    return has_focus;
  }
  int actual = -1;
  if (!set(V4L2_CID_FOCUS_AUTO, enabled ? 1 : 0) ||
      !get(V4L2_CID_FOCUS_AUTO, actual) || actual != (enabled ? 1 : 0)) {
    focus_status = "error";
    std::cerr << "Warning: could not verify autofocus "
              << (enabled ? "enabled" : "disabled") << '\n';
    return false;
  }
  focus_status = enabled ? "auto" : "manual";
  return true;
}
bool CameraControls::pulseLongestShutter() {
  // 冷启动时自动曝光会停在全黑。写一次最长快门后再切回自动，亮度才能稳住。
  return has_exposure && set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL) &&
         set(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_max);
}
bool CameraControls::setFocus(int value) {
  value = std::clamp(value, focus_min, focus_max);
  // 手动写入前必须确认自动对焦已关闭；驱动读回不证明镜头物理动作。
  if (!has_focus || !setAutoFocus(false))
    return false;
  int actual = focus_value;
  if (!set(V4L2_CID_FOCUS_ABSOLUTE, value) ||
      !get(V4L2_CID_FOCUS_ABSOLUTE, actual)) {
    focus_status = "error";
    std::cerr << "Warning: focus write/readback failed, requested " << value
              << '\n';
    return false;
  }
  focus_value = actual;
  if (actual != value) {
    focus_status = "error";
    std::cerr << "Warning: focus requested " << value << ", driver returned "
              << actual << '\n';
    return false;
  }
  focus_status = "manual";
  return true;
}
bool CameraControls::setGain(int value) {
  value = std::clamp(value, gain_min, gain_max);
  if (!set(V4L2_CID_GAIN, value))
    return false;
  gain_value = value;
  return true;
}
}
