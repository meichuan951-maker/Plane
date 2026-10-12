#include "demo_aim/light_bars.hpp"
#include <algorithm>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <sstream>
namespace aim {
cv::Mat drawLightBars(const cv::Mat &bgr, const LightBarResult &result) {
  auto image = bgr.clone();
  for (const auto &armor : result.armors) {
    std::vector<cv::Point> corners;
    for (const auto &point : armor.corners)
      corners.push_back(point);
    cv::polylines(image, std::vector<std::vector<cv::Point>>{corners}, true,
                  {0, 255, 0}, 2, cv::LINE_AA);
    cv::drawMarker(image, armor.center, {0, 255, 0}, cv::MARKER_CROSS, 12, 1);
  }
  for (const auto &bar : result.bars) {
    const cv::Scalar color =
        bar.class_id == 0 ? cv::Scalar(0, 80, 255) : cv::Scalar(255, 180, 0);
    cv::Point2f corners[4];
    bar.rect.points(corners);
    for (int i = 0; i < 4; ++i)
      cv::line(image, corners[i], corners[(i + 1) % 4], color, 2);
    cv::circle(image, bar.top, 3, {0, 255, 255}, -1);
    cv::circle(image, bar.bottom, 3, {0, 255, 255}, -1);
    cv::putText(image, bar.class_id == 0 ? "RED" : "BLUE", bar.top,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
  }
  for (const auto &armor : result.armors) {
    if (!armor.reported)
      continue;
    std::ostringstream text;
    text << std::fixed << std::setprecision(1);
    if (armor.pose_valid) {
      text << "yaw " << armor.yaw_deg << " pitch " << armor.pitch_deg << " err "
           << armor.reprojection_error_px << "px";
      if (armor.pose_ambiguous)
        text << " ambiguous";
    } else
      text << "yaw n/a";
    const auto label = text.str();
    int baseline = 0;
    const auto size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    const float top =
        std::min(std::min(armor.corners[0].y, armor.corners[1].y),
                 std::min(armor.corners[2].y, armor.corners[3].y));
    int x = cvRound(armor.center.x - size.width * 0.5f);
    x = std::clamp(x, 0, std::max(0, image.cols - size.width));
    int y = cvRound(top) - 6;
    if (y < size.height + 32)
      y = std::min(image.rows - 2, cvRound(top) + size.height + 18);
    cv::putText(image, label, {x + 1, y + 1}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                {0, 0, 0}, 2, cv::LINE_AA);
    cv::putText(image, label, {x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                {0, 255, 0}, 1, cv::LINE_AA);
  }
  return image;
}
} // namespace aim
