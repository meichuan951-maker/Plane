#pragma once
#include <opencv2/core.hpp>
#include <optional>
#include <string>
namespace aim {
struct DepthPlane {
  cv::Vec3d centroid_m, normal;
  double range_m{}, normal_yaw_deg{}, normal_pitch_deg{};
  double bearing_yaw_deg{}, bearing_pitch_deg{}, rmse_m{};
  int points{};
};
// XYZ 必须来自同一深度帧，光学系 X 右/Y 下/Z 前，单位米。
// 只测所选区域；不识别身份、不与彩色自动配准、不跟踪目标。
std::optional<DepthPlane> measureDepthPlane(const cv::Mat &xyz, cv::Rect roi,
                                            std::string &reason);
} // namespace aim
