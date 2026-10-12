#include "demo_aim/light_bars.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
namespace aim {
LightBarResult detectLightBars(const cv::Mat &bgr, const LightBarConfig &c) {
  c.validate();
  if (bgr.empty() || bgr.type() != CV_8UC3)
    throw std::invalid_argument("Light bar input must be nonempty CV_8UC3 BGR");
  cv::Mat hsv, red_high;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  LightBarResult result;
  cv::inRange(hsv, cv::Scalar(0, c.min_saturation, c.min_value),
              cv::Scalar(c.red_hue_low_max, 255, 255), result.red_mask);
  cv::inRange(hsv,
              cv::Scalar(c.red_hue_high_min, c.min_saturation, c.min_value),
              cv::Scalar(179, 255, 255), red_high);
  cv::bitwise_or(result.red_mask, red_high, result.red_mask);
  cv::inRange(hsv, cv::Scalar(c.blue_hue_min, c.min_saturation, c.min_value),
              cv::Scalar(c.blue_hue_max, 255, 255), result.blue_mask);
  // 与参考框架一样把轮廓亮度和颜色判断分开；用 V
  // 而非灰度，避免纯蓝灯亮度被压低。
  cv::Mat red_votes, blue_votes, bright;
  if (c.brightness_color) {
    red_votes = result.red_mask;
    blue_votes = result.blue_mask;
    // Mat::zeros 表达式可能复用目标缓存，必须新分配以免清掉共享的颜色投票图。
    result.red_mask = cv::Mat(bgr.size(), CV_8UC1, cv::Scalar(0));
    result.blue_mask = cv::Mat(bgr.size(), CV_8UC1, cv::Scalar(0));
    cv::extractChannel(hsv, bright, 2);
    cv::compare(bright, std::max(1, c.min_value), bright, cv::CMP_GE);
  }
  for (int pass = 0; pass < (c.brightness_color ? 1 : 2); ++pass) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(c.brightness_color
                         ? bright
                         : (pass == 0 ? result.red_mask : result.blue_mask),
                     contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (std::size_t index = 0; index < contours.size(); ++index) {
      const auto &contour = contours[index];
      const double area = cv::contourArea(contour);
      if (area < c.min_area)
        continue;
      const auto rect = cv::minAreaRect(contour);
      const double length = std::max(rect.size.width, rect.size.height);
      const double width = std::min(rect.size.width, rect.size.height);
      if (width <= 0 || length < c.min_length ||
          length / width < c.min_aspect || length / width > c.max_aspect ||
          area / (length * width) < c.min_fill)
        continue;
      cv::Point2f corners[4];
      rect.points(corners);
      auto direction = corners[1] - corners[0];
      if (cv::norm(corners[2] - corners[1]) > cv::norm(direction))
        direction = corners[2] - corners[1];
      if (direction.y < 0)
        direction = -direction;
      const double tilt =
          std::atan2(std::abs(direction.x), std::abs(direction.y)) * 180 /
          CV_PI;
      if (tilt > c.max_tilt_deg)
        continue;
      int color = pass;
      if (c.brightness_color) {
        const auto bounds = cv::boundingRect(contour);
        cv::Mat component = cv::Mat::zeros(bounds.size(), CV_8UC1), votes;
        cv::drawContours(component, contours, static_cast<int>(index),
                         cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(),
                         0, -bounds.tl());
        // 轮廓内的暗孔洞不能计为颜色或亮度支持。
        cv::bitwise_and(component, bright(bounds), component);
        const int pixels = cv::countNonZero(component);
        cv::bitwise_and(component, red_votes(bounds), votes);
        const int red = cv::countNonZero(votes);
        cv::bitwise_and(component, blue_votes(bounds), votes);
        const int blue = cv::countNonZero(votes);
        const int colored = red + blue;
        if (!pixels || colored < pixels * c.min_color_fraction ||
            std::max(red, blue) < colored * c.min_color_dominance)
          continue; // 纯白、绿色或颜色混杂的亮物体不冒充红/蓝灯条。
        color = blue > red ? 1 : 0;
        auto mask = (color == 0 ? result.red_mask : result.blue_mask)(bounds);
        cv::bitwise_or(mask, component, mask);
      }
      result.bars.push_back({color, rect, rect.center - direction * 0.5f,
                             rect.center + direction * 0.5f});
    }
  }
  std::sort(result.bars.begin(), result.bars.end(),
            [](const auto &a, const auto &b) {
              return a.rect.center.x < b.rect.center.x;
            });
  result.armors = pairLightBars(result.bars, c);
  if (!result.armors.empty()) {
    const cv::Point2f mid(bgr.cols * 0.5f, bgr.rows * 0.5f);
    std::size_t closest = 0;
    for (std::size_t i = 1; i < result.armors.size(); ++i) {
      const double di = cv::norm(result.armors[i].center - mid);
      const double dc = cv::norm(result.armors[closest].center - mid);
      if (di + 1e-3 < dc ||
          (std::abs(di - dc) <= 1e-3 &&
           result.armors[i].center.x < result.armors[closest].center.x))
        closest = i;
    }
    for (std::size_t i = 0; i < result.armors.size(); ++i) {
      auto &armor = result.armors[i];
      armor.reported = i == closest;
      if (const auto pose = solveArmorPose(armor.corners, c, bgr.size())) {
        armor.pose_valid = true;
        armor.yaw_deg = pose->yaw_deg;
        armor.pitch_deg = pose->pitch_deg;
        armor.reprojection_error_px = pose->reprojection_error_px;
        armor.pose_ambiguous = pose->ambiguous;
        armor.position_camera_m = pose->position_camera_m;
      }
    }
  }
  return result;
}
} // namespace aim
