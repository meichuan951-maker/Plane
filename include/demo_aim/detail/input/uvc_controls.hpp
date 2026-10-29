#pragma once
#include <string>
namespace aim {
// 可选的 Linux UVC 对焦/增益控制；MaixCAM Lite 的成像参数由设备端 MaixPy 设置。
struct CameraControls {
  int fd{-1};
  bool has_focus{false}, has_gain{false};
  bool has_auto_focus{false};
  int focus_min{0}, focus_max{255}, focus_value{0};
  int gain_min{0}, gain_max{0}, gain_value{0};
  std::string focus_status{"unknown"};
  CameraControls() = default;
  ~CameraControls();
  CameraControls(const CameraControls &) = delete;
  CameraControls &operator=(const CameraControls &) = delete;
  bool openDevice(const std::string &path);
  bool setAutoFocus(bool enabled);
  bool setFocus(int value);
  bool setGain(int value);

private:
  bool set(unsigned id, int value) const;
  bool get(unsigned id, int &value) const;
  bool query(unsigned id, int &min_value, int &max_value, int &value);
};
}
