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
  has_focus = query(V4L2_CID_FOCUS_ABSOLUTE, focus_min, focus_max, focus_value);
  int auto_min = 0, auto_max = 0, auto_value = 0;
  has_auto_focus = query(V4L2_CID_FOCUS_AUTO, auto_min, auto_max, auto_value);
  if (has_auto_focus)
    focus_status = auto_value ? "auto" : "manual";
  else if (has_focus)
    focus_status = "manual";
  has_gain = query(V4L2_CID_GAIN, gain_min, gain_max, gain_value);
  return has_focus || has_auto_focus || has_gain;
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
