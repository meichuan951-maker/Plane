#include "demo_aim/detail/input/uvc_controls.hpp"
#include "support/assertions.hpp"
#include <cerrno>
#include <cstdarg>
#include <fcntl.h>
#include <iostream>
#include <linux/videodev2.h>
#include <stdexcept>

namespace {
// 模拟驱动拒绝写入/错误读回，验证这些失败不会被当作对焦成功。
int automatic = 1, focus = 144, gain = 6, focus_writes = 0,
    control_writes = 0;
bool reject_manual = false, ignore_focus = false, fail_read = false;
using test_support::check;
}

extern "C" int __wrap_ioctl(int, unsigned long request, ...) {
  va_list args;
  va_start(args, request);
  void *argument = va_arg(args, void *);
  va_end(args);
  if (request == VIDIOC_QUERYCTRL) {
    auto *query = static_cast<v4l2_queryctrl *>(argument);
    if (query->id == V4L2_CID_FOCUS_ABSOLUTE) {
      query->minimum = 0;
      query->maximum = 289;
      query->default_value = 144;
    } else if (query->id == V4L2_CID_FOCUS_AUTO) {
      query->minimum = 0;
      query->maximum = 1;
      query->default_value = 1;
    } else if (query->id == V4L2_CID_GAIN) {
      query->minimum = 0;
      query->maximum = 16;
      query->default_value = 8;
    } else {
      errno = EINVAL;
      return -1;
    }
    return 0;
  }
  if (request == VIDIOC_S_CTRL)
    ++control_writes;
  auto *control = static_cast<v4l2_control *>(argument);
  int *value = nullptr;
  if (control->id == V4L2_CID_FOCUS_AUTO)
    value = &automatic;
  else if (control->id == V4L2_CID_FOCUS_ABSOLUTE)
    value = &focus;
  else if (control->id == V4L2_CID_GAIN)
    value = &gain;
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
    check(controls.openDevice("/dev/null") && controls.has_focus &&
              controls.has_auto_focus && controls.has_gain,
          "optional UVC controls must discover supported capabilities");
    check(control_writes == 0 && automatic == 1 && focus == 144 && gain == 6 &&
              controls.focus_status == "auto" && controls.gain_value == 6,
          "control discovery must preserve device imaging values without writes");
    check(controls.setFocus(208) && automatic == 0 && focus == 208 &&
                controls.focus_value == 208 && controls.focus_status == "manual",
            "manual request must disable AF and verify focus readback");
    check(controls.setAutoFocus(true) && automatic == 1 &&
                controls.focus_status == "auto",
            "hardware autofocus must be verified");
    reject_manual = true;
    const int writes_before = focus_writes;
    check(!controls.setFocus(0) && focus_writes == writes_before &&
                controls.focus_value == 208 && controls.focus_status == "error",
            "failed AF disable must block manual focus writes");
    reject_manual = false;
    ignore_focus = true;
    check(!controls.setFocus(0) && controls.focus_value == 208,
            "accepted but ignored write must not report requested focus");
    ignore_focus = false;
    fail_read = true;
    check(!controls.setAutoFocus(false) && controls.focus_status == "error",
            "unreadable autofocus state must not be considered verified");
    fail_read = false;
    automatic = 0;
    controls.has_auto_focus = false;
    check(controls.setFocus(0) && controls.focus_value == 0,
            "manual-only cameras must still accept focus control");
    check(!controls.setAutoFocus(true) && controls.focus_status == "manual",
            "unsupported autofocus must not be advertised as enabled");
    std::cout << "UVC read-only discovery and focus failure regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
