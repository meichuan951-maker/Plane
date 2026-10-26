#pragma once
#include "demo_aim/light_bars.hpp"
#include "demo_aim/detail/input/uvc_controls.hpp"
#include <filesystem>
namespace aim::camera_app {
void save(const std::filesystem::path &path, const cv::Mat &raw, const cv::Mat &overlay,
          const LightBarResult &result, const LightBarConfig &config,
          const CameraControls *controls);
}
