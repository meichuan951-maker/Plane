// 视觉算法接口；保留旧 include 路径，数据契约位于 observation.hpp。
#pragma once
#include "demo_aim/observation.hpp"
namespace aim {
// 检测、深度质量、时间与锁定阈值；默认值是 Demo 参数，不是真机标定值。
struct Config {
  // 允许锁定的类别白名单，必须与自定义模型类别表对应。
  std::vector<int> class_ids{0};
  // 最低置信度与最小框边长（像素）。
  double confidence_min{.6}, min_box_px{6};
  // 有效深度范围（米）；ROI 比例作用于框的宽和高，不是面积比例。
  double depth_min_m{.25}, depth_max_m{3}, depth_roi_fraction{.4};
  // 有效深度像素占比；MAD 为深度相对中位数的绝对偏差中位数（米）。
  double depth_min_valid_ratio{.6}, depth_max_mad_m{.04};
  // 最少有效深度样本数；至少连续多少帧通过检查才给出有效观测。
  int depth_min_samples{9}, confirm_frames{3};
  // 允许的图像年龄和 RGB-D 同步误差上限，均为秒。
  double max_frame_age_s{.2}, max_sync_error_s{.03};
  // 关联 IoU 下限、目标关联保留秒数、主点附近的像素对准容差。
  double lock_iou_min{.2}, lock_hold_s{.35}, aligned_tolerance_px{8};
  // 标定的齐次变换：p_body = body_T_camera * [p_camera,1]；平移单位米。
  // 约定机体系 x 前/y 左/z 上。未知时为空，不假设相机与机体重合。
  std::optional<cv::Matx44d> body_T_camera;
  void validate() const;
  static Config load(const std::string &path);
};
// 深度检查结果；depth 为空表示失败，reason 描述原因，质量字段可用于诊断。
struct DepthEstimate {
  std::optional<double> depth, mad;
  double valid_ratio{};
  std::string reason;
};
// 交并比：交集面积/并集面积，用于同类别检测框关联。
double iou(const Box &a, const Box &b);
// 在框中心 ROI 中检查有效率、MAD 和混合深度层，返回鲁棒深度。
DepthEstimate estimateDepth(const cv::Mat &depth, const Box &box,
                            const Config &config);
// 有状态的单目标流水线；同一实例须按时间顺序单线程调用。
class Pipeline {
public:
  explicit Pipeline(Config config = {});
  // 逐帧处理：时效 → 二维关联/确认 → 可选视线/深度；不含灯条提取或 PnP。
  Solution process(const Frame &frame, const std::vector<Detection> &detections,
                   double now);

private:
  Config config_;
  // 跨帧只保留关联状态，不缓存可复用的三维结果。
  std::optional<Detection> target_;
  std::optional<double> last_seen_, last_capture_;
  // track_id 是本进程关联编号；consecutive 统计连续通过确认所需的帧数。
  int track_id_{0}, consecutive_{0}, detection_consecutive_{0};
  std::string time_domain_, source_, frame_id_;
  cv::Size image_size_;
  void resetLock();
};
// 复制彩色图并叠加主点、检测框、偏差连线与状态；不修改输入图像。
cv::Mat annotate(const Frame &frame, const Solution &solution);
} // namespace aim
