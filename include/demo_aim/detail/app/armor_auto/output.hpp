#pragma once

#include "demo_aim/armor_auto.hpp"
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

namespace aim::armor_auto_app {

// Capture this after detection so processing latency is part of frame age.
struct FrameContext {
  int frame{0};
  std::uint64_t generation{0};
  double fps{0};
  bool has_image{false};
  bool failed{false};
  double monotonic_s{0};
  double age_s{0};
};

struct FrameStatus {
  bool current{false};
  bool valid{false};
  std::string reason;
};

// JSON and preview use the same gates and never trust cached pose validity
// alone.
FrameStatus evaluateFrame(const ArmorAutoConfig &config,
                          const TrackedArmor &target,
                          const FrameContext &context);
nlohmann::json makeReport(const ArmorAutoConfig &config,
                          const TrackedArmor &target,
                          const FrameContext &context);
void invalidateReport(nlohmann::json &packet, int frame, bool failed);

} // namespace aim::armor_auto_app
