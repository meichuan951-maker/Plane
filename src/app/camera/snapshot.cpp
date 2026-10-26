#include "demo_aim/detail/app/camera/snapshot.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
namespace aim::camera_app {
namespace fs = std::filesystem;
void save(const fs::path &path, const cv::Mat &raw, const cv::Mat &overlay,
          const aim::LightBarResult &result, const aim::LightBarConfig &c,
          const CameraControls *controls) {
  if (!fs::create_directories(path))
    throw std::runtime_error("Snapshot directory already exists: " +
                             path.string());
  for (const auto &entry :
       {std::make_pair("raw.png", raw), std::make_pair("overlay.png", overlay),
        std::make_pair("red_mask.png", result.red_mask),
        std::make_pair("blue_mask.png", result.blue_mask)})
    if (!cv::imwrite((path / entry.first).string(), entry.second))
      throw std::runtime_error("Could not write snapshot");
  std::ofstream config(path / "light_bars.yaml");
  c.write(config);
  config.close();
  if (!config)
    throw std::runtime_error("Could not write snapshot thresholds");
  if (controls) {
    std::ofstream camera_file(path / "camera_controls.yaml");
    camera_file << "# Last software focus readback; may change in auto mode.\n";
    camera_file << "focus_absolute: " << controls->focus_value << '\n';
    camera_file << "focus_status: " << controls->focus_status << '\n';
    camera_file << "gain: " << controls->gain_value << '\n';
    camera_file.close();
    if (!camera_file)
      throw std::runtime_error("Could not write camera controls");
  }
  std::cout << "Saved " << path << '\n';
}
} // namespace aim::camera_app
