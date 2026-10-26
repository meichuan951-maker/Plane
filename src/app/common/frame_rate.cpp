#include "demo_aim/detail/app/common/frame_rate.hpp"

namespace aim::app {

FrameRate::FrameRate(Clock::time_point start) : start_(start) {}

double FrameRate::update(Clock::time_point now, bool new_frame) {
  if (new_frame)
    ++frames_;
  const double elapsed = std::chrono::duration<double>(now - start_).count();
  if (elapsed >= 1.0) {
    rate_ = frames_ / elapsed;
    frames_ = 0;
    start_ = now;
  }
  return rate_;
}

} // namespace aim::app
