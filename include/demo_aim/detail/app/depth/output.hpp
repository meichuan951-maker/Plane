#pragma once
#include "demo_aim/depth_plane.hpp"
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
namespace aim::depth_app {
// One post-processing snapshot gates both numeric output and the preview.
struct FrameContext {
  int frame{};
  std::uint64_t generation{};
  double fps{};
  bool has_image{false}, failed{false};
  double monotonic_s{}, age_s{};
  std::string reason;
};
bool currentFrame(const FrameContext &context);
std::string frameReason(const FrameContext &context);
nlohmann::json makePacket(const FrameContext &context, const cv::Rect &roi,
                          bool center_depth, bool local_world_fixed,
                          const std::optional<DepthPlane> &measurement);
nlohmann::json stoppedPacket(nlohmann::json packet, int frame, bool failed);
} // namespace aim::depth_app
