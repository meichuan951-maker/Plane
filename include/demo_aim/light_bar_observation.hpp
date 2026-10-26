// 将传统灯条配对结果适配为观测流水线输入。
#pragma once
#include "demo_aim/light_bars.hpp"
#include "demo_aim/observation.hpp"

namespace aim {
// confidence 使用几何配对评分；保留真实角点，米制位置由下游深度流水线求解。
std::vector<Detection> detectArmorObservations(const cv::Mat &bgr,
                                              const LightBarConfig &config);
} // namespace aim
