#include "demo_aim/depth_plane.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
namespace aim {
std::optional<DepthPlane> measureDepthPlane(const cv::Mat &xyz, cv::Rect roi,
                                            std::string &reason) {
  reason = "invalid_roi";
  if (xyz.type() != CV_32FC3 || roi.width < 8 || roi.height < 8 || roi.x < 0 ||
      roi.y < 0 || roi.width > xyz.cols || roi.height > xyz.rows ||
      roi.x > xyz.cols - roi.width || roi.y > xyz.rows - roi.height ||
      double(roi.width) * roi.height > 640 * 480)
    return std::nullopt;
  std::vector<cv::Vec3d> points;
  for (int y = roi.y; y < roi.y + roi.height; ++y)
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const auto p = xyz.at<cv::Vec3f>(y, x);
      if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
          p[2] >= 0.1f && p[2] <= 10)
        points.emplace_back(p);
    }
  reason = "insufficient_depth";
  if (points.size() < 64 || points.size() < roi.area() * 0.8)
    return std::nullopt;
  cv::Vec3d mean{};
  for (const auto &p : points)
    mean += p;
  mean *= 1.0 / points.size();
  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (const auto &p : points) {
    const auto q = p - mean;
    covariance += q * q.t();
  }
  covariance *= 1.0 / points.size();
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors);
  reason = "insufficient_surface_extent";
  if (eigenvalues.at<double>(1) < 0.005 * 0.005)
    return std::nullopt;
  const double rmse = std::sqrt(std::max(0.0, eigenvalues.at<double>(2)));
  reason = "non_planar_or_mixed_depth";
  if (!std::isfinite(rmse) || rmse > 0.005)
    return std::nullopt;
  cv::Vec3d normal(eigenvectors.at<double>(2, 0), eigenvectors.at<double>(2, 1),
                   eigenvectors.at<double>(2, 2));
  if (normal[2] > 0)
    normal = -normal;
  reason = "surface_edge_on";
  if (-normal[2] < 0.2)
    return std::nullopt;
  reason = "ok";
  constexpr double degrees = 180 / CV_PI;
  return DepthPlane{
      mean,
      normal,
      cv::norm(mean),
      std::atan2(normal[0], -normal[2]) * degrees,
      std::atan2(-normal[1], std::hypot(normal[0], normal[2])) * degrees,
      std::atan2(mean[0], mean[2]) * degrees,
      std::atan2(-mean[1], std::hypot(mean[0], mean[2])) * degrees,
      rmse,
      static_cast<int>(points.size())};
}
} // namespace aim
