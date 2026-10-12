// 米制刚体变换。A_T_B 表示把 B 坐标变换到 A；不包含时间同步或定位融合。
#pragma once
#include <opencv2/core.hpp>
namespace aim {
struct RigidTransform {
  cv::Matx33d rotation{cv::Matx33d::eye()};
  cv::Vec3d translation{};
  void validate() const;
  cv::Vec3d apply(const cv::Vec3d &point) const;
  RigidTransform inverse() const;
  RigidTransform operator*(const RigidTransform &rhs) const;
};
bool finiteVector(const cv::Vec3d &v);
// 四元数顺序始终 wxyz，表示主动旋转；输入必须是单位四元数。
cv::Matx33d quaternionRotation(const cv::Vec4d &q);
cv::Vec4d rotationQuaternion(const cv::Matx33d &r);
cv::Vec4d slerp(const cv::Vec4d &a, const cv::Vec4d &b, double ratio);
// 仅改变坐标轴，不改变原点；真实 map 对齐还需要额外刚体变换。
cv::Matx33d enuFromNed();
cv::Matx33d fluFromFrd();
} // namespace aim
