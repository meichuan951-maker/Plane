#pragma once
#include <string>
namespace aim {
// Linux UVC 控制。实时画面只用自动曝光、对焦和增益。
struct CameraControls {
  int fd{-1};
  bool has_exposure{false}, has_focus{false}, has_gain{false};
  bool has_auto_focus{false};
  int exposure_min{1}, exposure_max{300};
  int focus_min{0}, focus_max{255}, focus_value{0};
  int gain_min{0}, gain_max{16}, gain_value{8};
  std::string focus_status{"unknown"};
  CameraControls() = default;
  ~CameraControls();
  CameraControls(const CameraControls &) = delete;
  CameraControls &operator=(const CameraControls &) = delete;
  bool openDevice(const std::string &path);
  void keepAutoExposure();
  bool pulseLongestShutter();
  void useAutoExposure();
  bool setAutoFocus(bool enabled);
  bool setFocus(int value);
  bool setGain(int value);

private:
  bool set(unsigned id, int value) const;
  bool get(unsigned id, int &value) const;
  bool query(unsigned id, int &min_value, int &max_value, int &value);
};
}
