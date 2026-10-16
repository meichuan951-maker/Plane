#pragma once
#include "output.hpp"
namespace aim::depth_app {
struct Selection {
  cv::Rect roi;
  cv::Point first, current;
  bool dragging{false};
  bool center_depth{false};
};
void mouse(int event, int x, int y, int flags, void *data);
cv::Mat drawPreview(const cv::Mat &xyz, const Selection &selection,
                    const FrameContext &context, bool local_world_fixed,
                    const std::optional<DepthPlane> &measurement);
} // namespace aim::depth_app
