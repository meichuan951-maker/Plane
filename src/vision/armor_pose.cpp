#include "demo_aim/light_bars.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
namespace aim {
std::optional<ArmorPose>
solveArmorPose(const std::array<cv::Point2f, 4> &corners,
               const LightBarConfig &config, cv::Size image_size) {
  config.validate();
  if (!config.has_camera)
    return std::nullopt;
  for (const auto &point : corners)
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      return std::nullopt;
  double area = 0;
  for (int i = 0; i < 4; ++i) {
    const auto &a = corners[i];
    const auto &b = corners[(i + 1) % 4];
    const auto first = b - a, second = corners[(i + 2) % 4] - b;
    const double cross = static_cast<double>(first.x) * second.y -
                         static_cast<double>(first.y) * second.x;
    if (!std::isfinite(cross) || cross <= 1e-6)
      return std::nullopt; // 错序、自交、凹四边形不进入 PnP。
    area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
  }
  if (std::abs(area) < 40)
    return std::nullopt;
  const double pixel_height = std::min(cv::norm(corners[3] - corners[0]),
                                       cv::norm(corners[2] - corners[1]));
  const double max_error = std::min(config.pose_max_error_px,
                                    config.pose_max_error_ratio * pixel_height);
  double sx = 1, sy = 1;
  if (config.camera_width > 0 && config.camera_height > 0 &&
      image_size.width > 0 && image_size.height > 0) {
    sx = static_cast<double>(image_size.width) / config.camera_width;
    sy = static_cast<double>(image_size.height) / config.camera_height;
  }
  const cv::Matx33d camera(config.fx * sx, 0, config.cx * sx, 0, config.fy * sy,
                           config.cy * sy, 0, 0, 1);
  const cv::Matx<double, 1, 5> distort(config.k1, config.k2, config.p1,
                                       config.p2, config.k3);
  const float half_w = static_cast<float>(config.armor_width_m * 0.5);
  const float half_h = static_cast<float>(config.lightbar_length_m * 0.5);
  // 装甲在 yz 平面：+y 灯条左侧，+z
  // 灯条上端。图像点顺序为左上、右上、右下、左下。
  const std::vector<cv::Point3f> object{{0, half_w, half_h},
                                        {0, -half_w, half_h},
                                        {0, -half_w, -half_h},
                                        {0, half_w, -half_h}};
  const std::vector<cv::Point2f> image_points(corners.begin(), corners.end());
  std::vector<cv::Mat> rvecs, tvecs;
  if (cv::solvePnPGeneric(object, image_points, camera, distort, rvecs, tvecs,
                          false, cv::SOLVEPNP_IPPE) < 1)
    return std::nullopt;
  int best = -1;
  double best_error = 1e9;
  ArmorPose best_pose;
  std::vector<ArmorPose> hypotheses;
  for (std::size_t i = 0; i < rvecs.size() && i < tvecs.size(); ++i) {
    cv::Mat rvec, tvec, rotation;
    rvecs[i].convertTo(rvec, CV_64F);
    tvecs[i].convertTo(tvec, CV_64F);
    if (rvec.total() != 3 || tvec.total() != 3)
      continue;
    const double depth = tvec.at<double>(2);
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec) ||
        !std::isfinite(depth) || depth <= 0.05 || depth > 30)
      continue;
    cv::Rodrigues(rvec, rotation);
    if (rotation.rows != 3 || rotation.cols != 3)
      continue;
    bool all_in_front = true;
    for (const auto &p : object) {
      const double z = rotation.at<double>(2, 0) * p.x +
                       rotation.at<double>(2, 1) * p.y +
                       rotation.at<double>(2, 2) * p.z + depth;
      if (!std::isfinite(z) || z <= 0.05)
        all_in_front = false;
    }
    if (!all_in_front)
      continue;
    const cv::Vec3d normal(-rotation.at<double>(0, 0),
                           -rotation.at<double>(1, 0),
                           -rotation.at<double>(2, 0));
    if (!(normal[2] < 0))
      continue;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object, rvec, tvec, camera, distort, projected);
    if (projected.size() != 4)
      continue;
    double error = 0;
    for (int k = 0; k < 4; ++k)
      error += cv::norm(projected[k] - image_points[k]);
    error /= 4;
    const double yaw = std::atan2(normal[0], -normal[2]) * 180 / CV_PI;
    const double pitch =
        std::atan2(-normal[1], std::hypot(normal[0], normal[2])) * 180 / CV_PI;
    if (!std::isfinite(error) || error > max_error || !std::isfinite(yaw) ||
        !std::isfinite(pitch))
      continue;
    ArmorPose candidate{yaw, pitch, error, false};
    candidate.position_camera_m = {tvec.at<double>(0), tvec.at<double>(1),
                                   tvec.at<double>(2)};
    hypotheses.push_back(candidate);
    if (error >= best_error)
      continue;
    best = static_cast<int>(i);
    best_error = error;
    best_pose = candidate;
  }
  if (best < 0)
    return std::nullopt;
  for (const auto &pose : hypotheses)
    if (pose.reprojection_error_px <=
            best_error + std::max(0.05, best_error * 0.1) &&
        std::hypot(pose.yaw_deg - best_pose.yaw_deg,
                   pose.pitch_deg - best_pose.pitch_deg) > 5)
      best_pose.ambiguous = true;
  return best_pose;
}
} // namespace aim
