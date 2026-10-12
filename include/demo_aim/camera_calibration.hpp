#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace aim {
struct RgbCalibration {
  cv::Mat camera, distortion;
  double rms_px{}, validation_max_rms_px{}, normal_spread_deg{};
};
RgbCalibration
calibrateRgbViews(const std::vector<std::vector<cv::Point2f>> &views,
                  cv::Size image_size, cv::Size inner_corners,
                  double square_mm);
} // namespace aim
