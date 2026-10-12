#pragma once
#include "../../input/uvc_stream.hpp"
#include <chrono>
#include <vector>
namespace aim::camera_app {
double sharpness(const cv::Mat &bgr);
// 由正常预览帧驱动对焦，不另开同步读帧循环。
class FocusScan {
public:
  void start(aim::CameraStream &stream, const CameraControls &controls);
  void tick(aim::CameraStream &stream, const CameraControls &controls,
            const cv::Mat &fresh_frame);
  void cancel();
  bool active() const { return stage_ != Stage::Idle; }
  std::string label() const;

private:
  enum class Stage { Idle, Send, Await, Sample, FinishSend, FinishAwait };
  Stage stage_{Stage::Idle};
  std::vector<int> positions_;
  std::vector<double> samples_;
  std::size_t index_{0};
  std::uint64_t command_{0}, generation_{0};
  int best_{0}, coarse_{0};
  double best_score_{-1}, worst_score_{-1};
  bool fine_{false};
  std::chrono::steady_clock::time_point ready_;
};
} // namespace aim::camera_app
