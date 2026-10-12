#include "preview.hpp"
#include <algorithm>
#include <iostream>
#include <opencv2/imgproc.hpp>
namespace aim::camera_app {
double sharpness(const cv::Mat &bgr) {
  const cv::Rect roi(bgr.cols / 4, bgr.rows / 4, bgr.cols / 2, bgr.rows / 2);
  cv::Mat gray, lap;
  cv::cvtColor(bgr(roi), gray, cv::COLOR_BGR2GRAY);
  cv::Laplacian(gray, lap, CV_32F);
  cv::Scalar mean, deviation;
  cv::meanStdDev(lap, mean, deviation);
  return deviation[0] * deviation[0];
}
void FocusScan::cancel() {
  if (active())
    std::cout << "Focus scan cancelled\n";
  stage_ = Stage::Idle;
  samples_.clear();
}
void FocusScan::start(aim::CameraStream &stream,
                      const CameraControls &controls) {
  cancel();
  if (!controls.has_focus || !stream.controlsReady())
    return;
  generation_ = stream.generation();
  coarse_ = std::max(8, (controls.focus_max - controls.focus_min) / 16);
  positions_.clear();
  for (int focus = controls.focus_min;;
       focus = std::min(controls.focus_max, focus + coarse_)) {
    positions_.push_back(focus);
    if (focus == controls.focus_max)
      break;
  }
  best_ = controls.focus_value;
  best_score_ = worst_score_ = -1;
  index_ = 0;
  fine_ = false;
  stage_ = Stage::Send;
}
std::string FocusScan::label() const {
  if (!active())
    return {};
  return "focus scan " +
         std::to_string(std::min(index_ + 1, positions_.size())) + "/" +
         std::to_string(positions_.size());
}
void FocusScan::tick(aim::CameraStream &stream, const CameraControls &controls,
                     const cv::Mat &frame) {
  if (!active())
    return;
  if (stream.generation() != generation_ || !stream.controlsReady()) {
    cancel();
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (stage_ == Stage::Send || stage_ == Stage::FinishSend) {
    command_ =
        stream.command(aim::CameraCommand::Focus,
                       stage_ == Stage::Send ? positions_[index_] : best_);
    if (command_)
      stage_ = stage_ == Stage::Send ? Stage::Await : Stage::FinishAwait;
    return;
  }
  if (stage_ == Stage::Await || stage_ == Stage::FinishAwait) {
    if (stream.acknowledged() < command_)
      return;
    if (stream.acknowledged() != command_ || !stream.commandSucceeded()) {
      std::cerr << "Focus scan stopped: control rejected or superseded\n";
      cancel();
      return;
    }
    if (stage_ == Stage::FinishAwait) {
      if (best_score_ < 1 ||
          (worst_score_ > 0 && best_score_ < worst_score_ * 1.15))
        std::cerr << "Warning: weak focus response; readback does not prove "
                     "lens movement or correct focus\n";
      std::cout << "Focus selected/readback " << controls.focus_value
                << ", center sharpness " << best_score_ << '\n';
      stage_ = Stage::Idle;
      return;
    }
    ready_ = now + std::chrono::milliseconds(350);
    samples_.clear();
    stage_ = Stage::Sample;
    return;
  }
  if (stage_ != Stage::Sample || now < ready_ || frame.empty())
    return;
  samples_.push_back(sharpness(frame));
  if (samples_.size() != 3)
    return;
  std::sort(samples_.begin(), samples_.end());
  const double score = samples_[1];
  std::cout << "Focus requested/readback " << positions_[index_] << '/'
            << controls.focus_value << ", center sharpness " << score << '\n';
  if (worst_score_ < 0 || score < worst_score_)
    worst_score_ = score;
  if (score > best_score_) {
    best_score_ = score;
    best_ = positions_[index_];
  }
  if (++index_ < positions_.size()) {
    stage_ = Stage::Send;
    return;
  }
  if (!fine_) {
    const int begin = std::max(controls.focus_min, best_ - coarse_);
    const int end = std::min(controls.focus_max, best_ + coarse_);
    positions_.clear();
    for (int focus = begin;; focus = std::min(end, focus + 4)) {
      positions_.push_back(focus);
      if (focus == end)
        break;
    }
    index_ = 0;
    fine_ = true;
    stage_ = Stage::Send;
  } else
    stage_ = Stage::FinishSend;
}
} // namespace aim::camera_app
