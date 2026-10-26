#pragma once

#include <chrono>
#include <cstdint>

namespace aim::app {

// Count completed frames over intervals of at least one second. Waiting polls
// advance the interval too, so a stalled stream eventually reports zero FPS.
class FrameRate {
public:
  using Clock = std::chrono::steady_clock;

  explicit FrameRate(Clock::time_point start);

  double update(Clock::time_point now, bool new_frame);

private:
  Clock::time_point start_;
  std::uint64_t frames_{};
  double rate_{};
};

} // namespace aim::app
