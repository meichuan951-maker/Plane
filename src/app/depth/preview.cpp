#include "preview.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
namespace aim::depth_app {
namespace {
constexpr int header_height = 190;
void line(cv::Mat &image, const std::string &text, int y, cv::Scalar color) {
  cv::putText(image, text, {8, y}, cv::FONT_HERSHEY_SIMPLEX, .43, color, 1,
              cv::LINE_AA);
}
} // namespace
cv::Rect rectangle(cv::Point a, cv::Point b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::abs(a.x - b.x),
          std::abs(a.y - b.y)};
}
void mouse(int event, int x, int y, int, void *data) {
  auto &s = *static_cast<Selection *>(data);
  if (event == cv::EVENT_LBUTTONDOWN &&
      (x < 0 || x >= 640 || y < header_height || y >= header_height + 480))
    return;
  const cv::Point p(std::clamp(x, 0, 639),
                    std::clamp(y - header_height, 0, 479));
  if (event == cv::EVENT_LBUTTONDOWN) {
    s.center_depth = false;
    s.dragging = true;
    s.first = s.current = p;
    s.roi = {};
  } else if (event == cv::EVENT_MOUSEMOVE && s.dragging)
    s.current = p;
  else if (event == cv::EVENT_LBUTTONUP && s.dragging) {
    s.dragging = false;
    s.roi = rectangle(s.first, p);
  }
}
cv::Mat drawPreview(const cv::Mat &xyz, const Selection &selection,
                    const FrameContext &context, bool local_world_fixed,
                    const std::optional<DepthPlane> &measurement) {
  cv::Mat view;
  if (!xyz.empty() && currentFrame(context)) {
    cv::Mat z, gray;
    cv::extractChannel(xyz, z, 2);
    z.convertTo(gray, CV_8UC1, 255 / 2.7, -0.3 * 255 / 2.7);
    cv::applyColorMap(gray, view, cv::COLORMAP_TURBO);
    view.setTo(cv::Scalar(0, 0, 0), z <= 0);
    cv::rectangle(view,
                  selection.dragging
                      ? rectangle(selection.first, selection.current)
                      : selection.roi,
                  {255, 255, 255}, 1);
    if (selection.center_depth) {
      cv::line(view, {308, 240}, {332, 240}, {255, 255, 255}, 1);
      cv::line(view, {320, 228}, {320, 252}, {255, 255, 255}, 1);
    }
  } else
    view = cv::Mat(480, 640, CV_8UC3, cv::Scalar(15, 15, 15));
  cv::copyMakeBorder(view, view, header_height, 0, 0, 0, cv::BORDER_CONSTANT,
                     cv::Scalar(20, 20, 20));
  std::ostringstream rate;
  rate << std::fixed << std::setprecision(1) << "Astra Pro DEPTH | "
       << context.fps << " FPS | 640x480 | "
       << (currentFrame(context) ? "LIVE" : "WAITING / STALE");
  line(view, rate.str(), 18, {230, 230, 230});
  line(view,
       selection.center_depth
           ? "CENTER SURFACE depth | no automatic armor identity"
           : "APPROXIMATE SDK depth | MANUAL ROI - no tracking",
       38, {0, 210, 255});
  line(view, "Aim center at armor | c center | drag/r manual | q/Esc exit", 58,
       {230, 230, 230});
  line(view, "Camera: X right / Y down / Z forward, meters", 78,
       {190, 190, 190});
  line(view,
       local_world_fixed
           ? "World: fixed camera origin; X forward / Y left / Z up"
           : "World: unavailable (use --local-world-fixed)",
       98, {0, 210, 255});
  if (currentFrame(context) && measurement) {
    std::ostringstream a, b, c, camera_xyz, world_xyz;
    a << std::fixed << std::setprecision(3) << "Range " << measurement->range_m
      << " m | Z " << measurement->centroid_m[2] << " m";
    b << std::fixed << std::setprecision(1) << "Surface: yaw "
      << measurement->normal_yaw_deg << " pitch "
      << measurement->normal_pitch_deg << " deg";
    c << std::fixed << std::setprecision(1) << "Bearing: yaw "
      << measurement->bearing_yaw_deg << " pitch "
      << measurement->bearing_pitch_deg << " deg";
    const auto &p = measurement->centroid_m;
    camera_xyz << std::fixed << std::setprecision(3) << "Camera XYZ: [" << p[0]
               << ", " << p[1] << ", " << p[2] << "] m";
    world_xyz << std::fixed << std::setprecision(3) << "World XYZ: [" << p[2]
              << ", " << -p[0] << ", " << -p[1] << "] m (fixed camera only)";
    line(view, a.str(), 118, {0, 255, 0});
    line(view, camera_xyz.str(), 138, {0, 255, 0});
    line(view, local_world_fixed ? world_xyz.str() : "World XYZ: n/a", 158,
         {0, 255, 0});
    line(view, b.str() + " | " + c.str(), 178, {200, 200, 200});
  } else {
    line(view, "Range / Camera XYZ / World XYZ: n/a", 122, {0, 180, 255});
    line(view, "Measurement: " + frameReason(context), 144, {0, 180, 255});
    line(view,
         selection.center_depth
             ? "Aim the white box at a flat surface with valid depth."
             : "Select a flat interior region in the depth image below.",
         168, {230, 230, 230});
  }
  return view;
}
} // namespace aim::depth_app
