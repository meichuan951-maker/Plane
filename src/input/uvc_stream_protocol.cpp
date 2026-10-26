#include "demo_aim/detail/input/uvc_stream_protocol.hpp"
#include <time.h>

namespace aim {

double streamTime() {
  timespec value{};
  clock_gettime(CLOCK_MONOTONIC, &value);
  return value.tv_sec + value.tv_nsec * 1e-9;
}

} // namespace aim
