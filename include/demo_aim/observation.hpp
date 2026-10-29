// 传输无关的目标观测契约；像素、米、弧度分别明确，缺失量用 optional。
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>
namespace aim {
using Json = nlohmann::json;
// 检测框采用原始彩色图的 xyxy 坐标；中心是测量代理点，不保证是装甲接触区域。
struct Box {
  double x1{}, y1{}, x2{}, y2{};
  cv::Point2d center() const { return {(x1 + x2) / 2, (y1 + y2) / 2}; }
};
// 原图像素坐标中的灯条端点；由独立特征提取器提供，不从检测框伪造。
struct LightBar {
  cv::Point2d top, bottom;
};
struct ArmorFeatures {
  std::optional<std::array<LightBar, 2>> light_bars; // 左灯条、右灯条
  // 物理对应点顺序：左上、右上、右下、左下。layout_id 指明几何版本。
  std::optional<std::array<cv::Point2d, 4>> corners;
  std::string layout_id;
};
// 单个检测结果：框、[0,1] 评分、颜色编号（0=红，1=蓝）。
struct Detection {
  Box box;
  double confidence{};
  int class_id{};
  std::optional<ArmorFeatures> features{};
};
// 针孔相机内参：width/height 为图像尺寸；fx/fy 为像素焦距，cx/cy 为主点。
// deproject 使用轴向深度 z；输入须为去畸变图，或改用 Frame 的 SDK 回调。
struct Intrinsics {
  int width{}, height{};
  double fx{}, fy{}, cx{}, cy{};
  bool isValid() const noexcept;
  void validate() const;
  cv::Vec3d deproject(double u, double v, double z) const;
};
// 彩色观测帧，可选对齐深度；cv::Mat 共享内存，处理期间不能修改。
struct Frame {
  // color_bgr 为 CV_8UC3；depth_m 为 CV_32FC1 米制轴向深度，必须对齐到彩色图。
  cv::Mat color_bgr, depth_m;
  // 全零内参表示未知，仍可输出二维框。
  Intrinsics intrinsics;
  // 上游已去畸变的针孔图像设为 true，否则必须提供 deproject 回调。
  bool rectified{false};
  // 采集单调时间（秒）；实时源与 now 同时钟，离线夹具使用独立虚拟时间。
  double capture_s{};
  // 实时和离线时钟不可混用；PX4 boot 时间不得直接填写进来。
  std::string time_domain{"host_monotonic"};
  std::optional<double> received_s; // 与 capture_s 同时钟域；未知则空
  std::uint64_t sequence{};
  // RGB-D 时间差（秒）；clock_valid 表示 capture_s 已可与调用方 now 比较。
  double sync_error_s{};
  bool clock_valid{true};
  std::string source{"unknown"}, frame_id{"camera_color_optical_frame"};
  // 可选回调 (u,v,z) → 相机三维点；实机由 SDK 处理畸变，未设置则用针孔模型。
  std::function<cv::Vec3d(double, double, double)> deproject;
};
// 一帧观测输出。valid 才表示三维结果可用；aligned
// 仅表示像素对准，不能授权接触。
struct Solution {
  std::uint64_t sequence{};
  // 采集/处理时刻用秒，age_ms 用毫秒；emitted_s
  // 记录处理入口时刻，不含后续写盘延迟。
  double capture_s{}, emitted_s{}, age_ms{};
  std::string source, frame_id, reason{"uninitialized"};
  // valid 是 position_valid 的旧兼容字段，不得改成“检测到框”。
  bool valid{false}, aligned{false};
  bool detection_valid{false}, bearing_valid{false}, position_valid{false};
  std::string time_domain{"host_monotonic"}, tracking_state{"searching"};
  std::string position_reason{"not_estimated"}, features_reason{"not_provided"};
  std::optional<double> received_s;
  std::optional<ArmorFeatures> features{};
  std::optional<cv::Vec3d> bearing_camera; // 光学系单位视线，未知距离
  std::optional<cv::Point2d> image_velocity_px_s; // 像素差分，不是世界速度
  int image_width{}, image_height{};
  std::optional<int> track_id, class_id;
  // 角误差为相机视线角：yaw 正向图像右，pitch 正向上；不是 PX4 姿态指令。
  std::optional<double> confidence, yaw_error_rad, pitch_error_rad,
      depth_valid_ratio, depth_mad_m;
  std::optional<Box> box;
  std::optional<cv::Point2d> center_uv, pixel_error;
  // 光学系 x 右/y 下/z 前；机体系点仅在存在外参时输出，均为米。
  std::optional<cv::Vec3d> point_camera, point_body;
  Json json() const;
};
// 消费端再次检查观测时效；停止发布时仍需调用方自己的消息超时看门狗。
bool isCurrent(const Solution &solution, double now, double max_age,
               const std::string &time_domain = "host_monotonic");
// 二维消费者独立检查时效，离线夹具需显式声明对应的虚拟时钟域。
bool isDetectionCurrent(const Solution &solution, double now, double max_age,
                        const std::string &time_domain = "host_monotonic");
// steady_clock 单调秒数，不是 Unix 时间，也不是 ROS 时间戳。
double monotonicNow();
} // namespace aim
