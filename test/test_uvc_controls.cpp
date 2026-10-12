#include "../src/input/uvc_controls.hpp"
#include <cerrno>
#include <cstdarg>
#include <fcntl.h>
#include <iostream>
#include <linux/videodev2.h>
#include <stdexcept>

namespace {
// 模拟驱动拒绝写入/错误读回，验证这些失败不会被当作对焦成功。
int automatic = 1, focus = 144, focus_writes = 0;
bool reject_manual = false, ignore_focus = false, fail_read = false;
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
}

extern "C" int __wrap_ioctl(int, unsigned long request, ...) {
  va_list args;
  va_start(args, request);
  auto *control = va_arg(args, v4l2_control *);
  va_end(args);
  int *value = nullptr;
  if (control->id == V4L2_CID_FOCUS_AUTO)
    value = &automatic;
  else if (control->id == V4L2_CID_FOCUS_ABSOLUTE)
    value = &focus;
  if (!value) {
    errno = EINVAL;
    return -1;
  }
  if (request == VIDIOC_G_CTRL) {
    if (fail_read) {
      errno = EIO;
      return -1;
    }
    control->value = *value;
    return 0;
  }
  if (request == VIDIOC_S_CTRL) {
    if (value == &automatic && control->value == 0 && reject_manual) {
      errno = EIO;
      return -1;
    }
    if (value == &focus) {
      ++focus_writes;
      if (automatic) {
        errno = EBUSY;
        return -1;
      }
      if (ignore_focus)
        return 0;
    }
    *value = control->value;
    return 0;
  }
  errno = EINVAL;
  return -1;
}

int main() {
  try {
    aim::CameraControls controls;
    controls.fd = ::open("/dev/null", O_RDWR);
    require(controls.fd >= 0, "could not open test descriptor");
    controls.has_focus = controls.has_auto_focus = true;
    controls.focus_max = 289;
    controls.focus_value = focus;
    require(controls.setFocus(208) && automatic == 0 && focus == 208 &&
                controls.focus_value == 208 && controls.focus_status == "manual",
            "manual request must disable AF and verify focus readback");
    require(controls.setAutoFocus(true) && automatic == 1 &&
                controls.focus_status == "auto",
            "hardware autofocus must be verified");
    reject_manual = true;
    const int writes_before = focus_writes;
    require(!controls.setFocus(0) && focus_writes == writes_before &&
                controls.focus_value == 208 && controls.focus_status == "error",
            "failed AF disable must block manual focus writes");
    reject_manual = false;
    ignore_focus = true;
    require(!controls.setFocus(0) && controls.focus_value == 208,
            "accepted but ignored write must not report requested focus");
    ignore_focus = false;
    fail_read = true;
    require(!controls.setAutoFocus(false) && controls.focus_status == "error",
            "unreadable autofocus state must not be considered verified");
    fail_read = false;
    automatic = 0;
    controls.has_auto_focus = false;
    require(controls.setFocus(0) && controls.focus_value == 0,
            "manual-only cameras must still accept focus control");
    require(!controls.setAutoFocus(true) && controls.focus_status == "manual",
            "unsupported autofocus must not be advertised as enabled");
    std::cout << "UVC focus control failure regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
