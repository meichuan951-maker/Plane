#pragma once

#include "output.hpp"

namespace aim::armor_auto_app {

cv::Mat drawPreview(const cv::Mat &raw, const LightBarResult &detection,
                    const TrackedArmor &target, const ArmorAutoConfig &config,
                    const FrameContext &context);

// Render from the captured frame rather than the throttled window cache.
// Expired frames keep their raw diagnostic image, but never a live overlay.
void saveSnapshot(const std::string &directory, const cv::Mat &raw,
                  const LightBarResult &detection, const TrackedArmor &target,
                  const ArmorAutoConfig &config, const FrameContext &context);

} // namespace aim::armor_auto_app
