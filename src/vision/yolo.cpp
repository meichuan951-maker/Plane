// YOLOv8 ONNX 的预处理、推理和后处理；不包含训练或相机采集。
#include "demo_aim/yolo.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <openssl/evp.h>
#include <sstream>
#include <stdexcept>
namespace aim {
// 输入边长须为 32 的倍数；统一缩放后用 114 灰色填充，记录反变换参数。
Letterbox letterbox(const cv::Mat &image, int side) {
  if (image.empty() || image.type() != CV_8UC3 || side < 32 || side > 2048 ||
      side % 32)
    throw std::invalid_argument(
        "Expected BGR image and input size 32..2048, multiple of 32");
  const double scale = std::min(static_cast<double>(side) / image.cols,
                                static_cast<double>(side) / image.rows);
  const int width = std::max(1, cvRound(image.cols * scale)),
            height = std::max(1, cvRound(image.rows * scale));
  const int left = (side - width) / 2, top = (side - height) / 2;
  cv::Mat resized, padded;
  cv::resize(image, resized, {width, height});
  cv::copyMakeBorder(resized, padded, top, side - height - top, left,
                     side - width - left, cv::BORDER_CONSTANT, {114, 114, 114});
  return {padded, scale, left, top, image.size()};
}
// 仅接受 batch=1、float32 的原始检测输出；不接受分割、姿态或内置 NMS 格式。
std::vector<Detection> decodeYoloV8(const cv::Mat &raw, const Letterbox &lb,
                                    int classes, const Config &c,
                                    double threshold) {
  if (classes < 1 || classes > 1000 || !std::isfinite(threshold) ||
      threshold <= 0 || threshold > 1 || !std::isfinite(lb.scale) ||
      lb.scale <= 0 || raw.type() != CV_32F)
    throw std::invalid_argument(
        "Invalid YOLO decode configuration or tensor type");
// 每候选是 cx,cy,w,h 和 C 个类别分数；YOLOv8 此格式没有额外 objectness。
  const int features = 4 + classes;
  int a = 0, b = 0;
  if (raw.dims == 3 && raw.size[0] == 1) {
    a = raw.size[1];
    b = raw.size[2];
  } else if (raw.dims == 2) {
    a = raw.rows;
    b = raw.cols;
  } else
    throw std::invalid_argument(
        "Expected raw YOLOv8 [1,4+C,N] or [1,N,4+C], batch=1, nms=false");
// 特征维应恰好有一边等于 4+C；两边都匹配时无法可靠判定布局，直接拒绝。
  if ((a == features) == (b == features))
    throw std::invalid_argument("Wrong or ambiguous YOLO feature dimension; "
                                "check num-classes and nms=false");
  const bool channel_first = a == features;
  const int count = channel_first ? b : a;
  if (count < 1 || count > 100000)
    throw std::invalid_argument("Unexpected YOLO candidate count");
// 指针线性索引要求连续内存；兼容 [4+C,N] 和 [N,4+C] 两种布局。
  const cv::Mat output = raw.isContinuous() ? raw : raw.clone();
  const float *data = output.ptr<float>();
  auto get = [&](int n, int f) {
    return channel_first ? data[f * count + n] : data[n * features + f];
  };
  std::vector<Detection> candidates;
  for (int n = 0; n < count; ++n) {
    int cls = -1;
    double score = -1;
// 每个候选采用分数最高的类别，随后检查类别白名单及置信度阈值。
    for (int k = 0; k < classes; ++k) {
      const double p = get(n, 4 + k);
      if (std::isfinite(p) && p >= 0 && p <= 1 && p > score) {
        score = p;
        cls = k;
      }
    }
    if (score < c.confidence_min ||
        std::find(c.class_ids.begin(), c.class_ids.end(), cls) ==
            c.class_ids.end())
      continue;
    const double cx = get(n, 0), cy = get(n, 1), w = get(n, 2), h = get(n, 3);
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) ||
        !std::isfinite(h) || w <= 0 || h <= 0)
      continue;
// 先从中心宽高转为角点，再减去 padding、除以 scale，回到原图像素坐标。
    const Box box{
        (cx - w / 2 - lb.left) / lb.scale, (cy - h / 2 - lb.top) / lb.scale,
        (cx + w / 2 - lb.left) / lb.scale, (cy + h / 2 - lb.top) / lb.scale};
    // Partially cut-off plates do not become valid by clipping their boxes.
    if (box.x1 < 0 || box.y1 < 0 || box.x2 > lb.original.width ||
        box.y2 > lb.original.height)
      continue;
    candidates.emplace_back();
    auto &detection=candidates.back();
    detection.box=box; detection.confidence=score; detection.class_id=cls;
    // 普通 YOLO 没有灯条/角点输出，features 保持空。

  }
// 先按置信度排序；候选上限限制后续两两比较成本。
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const Detection &a, const Detection &b) {
                     return a.confidence > b.confidence;
                   });
  if (candidates.size() > 1000)
    candidates.resize(1000);
  std::vector<Detection> result;
// 分类 NMS：同类重叠框保留高分者，不跨类别压制；最多输出 100 个候选。
  for (const auto &candidate : candidates) {
    bool suppressed = false;
    for (const auto &accepted : result)
      if (candidate.class_id == accepted.class_id &&
          iou(candidate.box, accepted.box) > threshold) {
        suppressed = true;
        break;
      }
    if (!suppressed)
      result.push_back(candidate);
    if (result.size() >= 100)
      break;
  }
  return result;
}
// 分块读取文件，避免一次将大模型载入哈希缓冲；unique_ptr 自动释放 OpenSSL 上下文。
std::string sha256File(const std::string &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file)
    throw std::runtime_error("Missing model: " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  std::array<char, 65536> buffer{};
  while (file.read(buffer.data(), buffer.size()) || file.gcount())
    if (EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(file.gcount())) != 1)
      throw std::runtime_error("SHA256 update failed");
  if (file.bad())
    throw std::runtime_error("Model read failed");
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned length = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &length) != 1)
    throw std::runtime_error("SHA256 finalization failed");
  std::ostringstream out;
  for (unsigned i = 0; i < length; ++i)
    out << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(digest[i]);
  return out.str();
}
// 校验模型类别数量、输入尺寸和文件格式后加载网络；不自动下载或转换模型。
YoloV8::YoloV8(const std::string &path, int classes, int input_size,
               Config config)
    : config_(std::move(config)), classes_(classes), input_size_(input_size) {
  config_.validate();
  if (classes < 1 || classes > 1000 || input_size < 32 || input_size > 2048 ||
      input_size % 32)
    throw std::invalid_argument(
        "Specify model class count and valid input size");
  for (int c : config_.class_ids)
    if (c >= classes)
      throw std::invalid_argument("class_ids exceeds model class count");
  if (std::filesystem::path(path).extension() != ".onnx" ||
      !std::filesystem::is_regular_file(path))
    throw std::invalid_argument(
        "Provide existing local YOLOv8 detection .onnx weights (not .pt)");
  hash_ = sha256File(path);
  net_ = cv::dnn::readNetFromONNX(path);
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}
// 每帧执行 letterbox → blob → forward → decode，返回原图检测框。
std::vector<Detection> YoloV8::detect(const cv::Mat &image) {
  const auto lb = letterbox(image, input_size_);
// 缩放像素到 [0,1]；swapRB=true 将 OpenCV 的 BGR 转为模型需要的 RGB。
  cv::Mat blob = cv::dnn::blobFromImage(
      lb.image, 1. / 255., {input_size_, input_size_}, {}, true, false, CV_32F);
  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  if (outputs.size() != 1)
    throw std::runtime_error("Only one raw YOLOv8 detect output is supported");
  return decodeYoloV8(outputs[0], lb, classes_, config_);
}
} // namespace aim
