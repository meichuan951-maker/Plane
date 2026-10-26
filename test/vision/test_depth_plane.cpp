#include "demo_aim/depth_plane.hpp"
#include "support/assertions.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace {
using test_support::check;
} // namespace

int main() {
  try {
    cv::Mat xyz(60, 80, CV_32FC3);
    // 已知倾斜平面 n=(sin(yaw)*cos(pitch),-sin(pitch),-cos(yaw)*cos(pitch))。
    const double yaw = 20 * CV_PI / 180, pitch = -12 * CV_PI / 180;
    const cv::Vec3d n(std::sin(yaw) * std::cos(pitch), -std::sin(pitch),
                      -std::cos(yaw) * std::cos(pitch));
    for (int y = 0; y < xyz.rows; ++y)
      for (int x = 0; x < xyz.cols; ++x) {
        const double px = (x - 40) * .002, py = (y - 30) * .002;
        xyz.at<cv::Vec3f>(y, x) = {float(px), float(py),
                                   float(1 - (n[0] * px + n[1] * py) / n[2])};
      }
    std::string reason;
    auto p = aim::measureDepthPlane(xyz, {0, 0, 80, 60}, reason);
    check(p && reason == "ok", "known plane rejected");
    check(std::abs(p->normal_yaw_deg - 20) < .01 &&
              std::abs(p->normal_pitch_deg + 12) < .01,
          "surface signs/units wrong");
    check(p->rmse_m < 1e-5 && p->centroid_m[2] > .99 && p->centroid_m[2] < 1.01,
          "plane distance wrong");
    check(!aim::measureDepthPlane(xyz, {-1, 0, 20, 20}, reason),
          "outside ROI accepted");
    check(!aim::measureDepthPlane(xyz, {0, 0, 2147483647, 2147483647}, reason),
          "overflow ROI accepted");
    auto missing = xyz.clone();
    missing.rowRange(0, 20).setTo(cv::Scalar(0, 0, 0));
    check(!aim::measureDepthPlane(missing, {0, 0, 80, 60}, reason),
          "sparse depth accepted");
    auto mixed = xyz.clone();
    for (int y = 0; y < 60; ++y)
      for (int x = 0; x < 80; ++x)
        if ((x + y) % 2)
          mixed.at<cv::Vec3f>(y, x)[2] += .15f;
    check(!aim::measureDepthPlane(mixed, {0, 0, 80, 60}, reason),
          "mixed depth layers accepted");
    xyz.setTo(cv::Scalar(0, 0, std::numeric_limits<float>::quiet_NaN()));
    check(!aim::measureDepthPlane(xyz, {0, 0, 80, 60}, reason), "NaN accepted");
    std::cout << "Depth plane regression passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
