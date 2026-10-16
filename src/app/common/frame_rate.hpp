#pragma once

#include <chrono>
#include <cstdint>

namespace aim::app {

// Count completed frames over intervals of at least one second. Waiting polls
// advance the interval too, so a stalled stream eventually reports zero FPS.
class FrameRate {
public:
  using Clock = std::chrono::steady_clock;

  explicit FrameRate(Clock::time_point start) : start_(start) {}

  double update(Clock::time_point now, bool new_frame) {
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

private:
  Clock::time_point start_;
  std::uint64_t frames_{};
  double rate_{};
};

} // namespace aim::app
