// YOLOv8 检测适配接口；内部依赖 OpenCV DNN，对外统一输出原图检测框。
#pragma once
#include "demo_aim/core.hpp"
#include <opencv2/dnn.hpp>
namespace aim {
// 保存缩放后的方形图及逆变换参数；scale 是统一缩放率，left/top 是填充像素。
struct Letterbox {
  cv::Mat image;
  double scale{};
  int left{}, top{};
  cv::Size original;
};
// 等比例缩放并填充到 side×side，避免拉伸改变装甲形状。
Letterbox letterbox(const cv::Mat &image, int side);
// 解析原始 detect 张量并执行同类别 NMS；num_classes 是模型总类别数。
std::vector<Detection> decodeYoloV8(const cv::Mat &output,
                                    const Letterbox &transform, int num_classes,
                                    const Config &config, double nms_iou = .45);
// 计算权重文件指纹，供日志追溯模型版本，不代表模型准确率。
std::string sha256File(const std::string &path);
// 持有已加载网络；构造时加载本地 ONNX，逐帧 detect 不重复读取文件。
class YoloV8 {
public:
  YoloV8(const std::string &path, int num_classes, int input_size,
         Config config);
// 输入 BGR8 原图，输出反变换到原图坐标的候选框；当前使用 CPU 后端。
  std::vector<Detection> detect(const cv::Mat &image);
// 返回构造时计算的权重哈希，供每帧输出附带。
  const std::string &modelHash() const { return hash_; }

private:
  cv::dnn::Net net_;
  Config config_;
  int classes_, input_size_;
  std::string hash_;
};
} // namespace aim
