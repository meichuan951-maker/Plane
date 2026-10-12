#include "demo_aim/camera_calibration.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
namespace aim {
RgbCalibration
calibrateRgbViews(const std::vector<std::vector<cv::Point2f>> &views,
                  cv::Size size, cv::Size board, double square_mm) {
  if (views.size() < 25 || views.size() > 80 || size.width < 16 ||
      size.height < 16 || board.width < 3 || board.height < 3 ||
      board.width > 20 || board.height > 20 || !std::isfinite(square_mm) ||
      square_mm < 2 || square_mm > 200)
    throw std::invalid_argument(
        "Need >=25 valid board views and actual square size");
  float minx = size.width, miny = size.height, maxx = 0, maxy = 0;
  for (const auto &v : views) {
    if (v.size() != std::size_t(board.area()))
      throw std::invalid_argument("Board corner count mismatch");
    for (auto p : v) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
          p.x >= size.width || p.y >= size.height)
        throw std::invalid_argument("Board corners must be inside the image");
      minx = std::min(minx, p.x);
      maxx = std::max(maxx, p.x);
      miny = std::min(miny, p.y);
      maxy = std::max(maxy, p.y);
    }
  }
  if (maxx - minx < size.width * .6 || maxy - miny < size.height * .6)
    throw std::invalid_argument(
        "Move board to cover more of image (>=60% width and height)");
  std::vector<cv::Point3f> model;
  for (int y = 0; y < board.height; ++y)
    for (int x = 0; x < board.width; ++x)
      model.emplace_back(float(x * square_mm * .001),
                         float(y * square_mm * .001), 0);
  std::vector<std::vector<cv::Point2f>> train, holdout;
  for (std::size_t i = 0; i < views.size(); ++i)
    (i % 5 == 4 ? holdout : train).push_back(views[i]);
  std::vector<std::vector<cv::Point3f>> objects(train.size(), model);
  std::vector<cv::Mat> rv, tv;
  RgbCalibration p;
  p.rms_px =
      cv::calibrateCamera(objects, train, size, p.camera, p.distortion, rv, tv);
  auto &K = p.camera;
  auto &D = p.distortion;
  if (!cv::checkRange(K) || !cv::checkRange(D) || !std::isfinite(p.rms_px) ||
      p.rms_px > .8 || K.at<double>(0, 0) < size.width * .1 ||
      K.at<double>(0, 0) > size.width * 10 ||
      K.at<double>(1, 1) < size.height * .1 ||
      K.at<double>(1, 1) > size.height * 10 || K.at<double>(0, 2) < 0 ||
      K.at<double>(0, 2) > size.width || K.at<double>(1, 2) < 0 ||
      K.at<double>(1, 2) > size.height)
    throw std::runtime_error(
        "Calibration RMS/intrinsics failed; capture better views");
  std::vector<cv::Vec3d> normals;
  for (auto &r : rv) {
    cv::Mat R;
    cv::Rodrigues(r, R);
    normals.emplace_back(R.at<double>(0, 2), R.at<double>(1, 2),
                         R.at<double>(2, 2));
  }
  for (auto &a : normals)
    for (auto &b : normals)
      p.normal_spread_deg =
          std::max(p.normal_spread_deg,
                   std::acos(std::clamp(a.dot(b), -1., 1.)) * 180 / CV_PI);
  if (p.normal_spread_deg < 15)
    throw std::runtime_error(
        "Need more board yaw/pitch variation (>=15 degrees)");
  for (const auto &v : holdout) {
    cv::Mat r, t;
    std::vector<cv::Point2f> projected;
    if (!cv::solvePnP(model, v, K, D, r, t))
      throw std::runtime_error("Validation pose failed");
    cv::projectPoints(model, r, t, K, D, projected);
    double sum = 0;
    for (std::size_t i = 0; i < v.size(); ++i) {
      const auto d = v[i] - projected[i];
      sum += d.dot(d);
    }
    p.validation_max_rms_px =
        std::max(p.validation_max_rms_px, std::sqrt(sum / v.size()));
  }
  if (!std::isfinite(p.validation_max_rms_px) || p.validation_max_rms_px > 1.5)
    throw std::runtime_error(
        "Held-out calibration views failed reprojection check");
  return p;
}
} // namespace aim
