#include "demo_aim/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace aim {
bool finiteVector(const cv::Vec3d &v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
void RigidTransform::validate() const {
  for (double x : rotation.val)
    if (!std::isfinite(x))
      throw std::invalid_argument("Non-finite rotation");
  if (!finiteVector(translation) ||
      cv::norm(rotation.t() * rotation - cv::Matx33d::eye()) > 1e-6 ||
      std::abs(cv::determinant(rotation) - 1.) > 1e-6)
    throw std::invalid_argument("Transform must be a proper rigid transform");
}
cv::Vec3d RigidTransform::apply(const cv::Vec3d &p) const {
  validate();
  if (!finiteVector(p))
    throw std::invalid_argument("Non-finite point");
  return rotation * p + translation;
}
RigidTransform RigidTransform::inverse() const {
  validate();
  return {rotation.t(), -(rotation.t() * translation)};
}
RigidTransform RigidTransform::operator*(const RigidTransform &b) const {
  validate();
  b.validate();
  return {rotation * b.rotation, rotation * b.translation + translation};
}
cv::Matx33d quaternionRotation(const cv::Vec4d &q) {
  const double n = cv::norm(q);
  if (!std::isfinite(n) || std::abs(n - 1.) > 1e-3)
    throw std::invalid_argument("Expected finite unit quaternion wxyz");
  const auto u = q / n;
  const double w = u[0], x = u[1], y = u[2], z = u[3];
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
          2 * (x * z + y * w),     2 * (x * y + z * w),
          1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w),     2 * (y * z + x * w),
          1 - 2 * (x * x + y * y)};
}
cv::Vec4d rotationQuaternion(const cv::Matx33d &r) {
  RigidTransform{r, {}}.validate();
  cv::Vec4d q;
  const double trace = r(0, 0) + r(1, 1) + r(2, 2);
  if (trace > 0) {
    const double s = 2 * std::sqrt(trace + 1);
    q = {s / 4, (r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s,
         (r(1, 0) - r(0, 1)) / s};
  } else {
    int i = 0;
    if (r(1, 1) > r(i, i))
      i = 1;
    if (r(2, 2) > r(i, i))
      i = 2;
    const int j = (i + 1) % 3, k = (i + 2) % 3;
    const double s = 2 * std::sqrt(1 + r(i, i) - r(j, j) - r(k, k));
    q[0] = (r(k, j) - r(j, k)) / s;
    q[i + 1] = s / 4;
    q[j + 1] = (r(j, i) + r(i, j)) / s;
    q[k + 1] = (r(k, i) + r(i, k)) / s;
  }
  return q / cv::norm(q);
}
cv::Vec4d slerp(const cv::Vec4d &a, const cv::Vec4d &b, double t) {
  quaternionRotation(a);
  quaternionRotation(b);
  if (!std::isfinite(t) || t < 0 || t > 1)
    throw std::invalid_argument("SLERP outside [0,1]");
  auto aa = a / cv::norm(a), bb = b / cv::norm(b);
  double d = aa.dot(bb);
  if (d < 0) {
    bb = -bb;
    d = -d;
  }
  d = std::clamp(d, -1., 1.);
  if (d > .9995) {
    const auto q = aa * (1 - t) + bb * t;
    return q / cv::norm(q);
  }
  const double theta = std::acos(d);
  return (aa * std::sin((1 - t) * theta) + bb * std::sin(t * theta)) /
         std::sin(theta);
}
cv::Matx33d enuFromNed() { return {0, 1, 0, 1, 0, 0, 0, 0, -1}; }
cv::Matx33d fluFromFrd() { return {1, 0, 0, 0, -1, 0, 0, 0, -1}; }
} // namespace aim
