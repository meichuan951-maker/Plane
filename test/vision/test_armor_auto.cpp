#include "demo_aim/armor_auto.hpp"
#include "support/assertions.hpp"
#include "demo_aim/camera_calibration.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
namespace {
using test_support::check;
aim::ArmorCandidate candidate(int color, float x, float y) {
  aim::ArmorCandidate a;
  a.class_id = color;
  a.center = {x, y};
  a.box = {x - 20, y - 10, 40, 20};
  a.geometry_score = .9;
  return a;
}
} // namespace

int main(int argc, char **argv) {
  try {
    check(argc == 2, "Usage: aim_armor_auto_tests CONFIG.yaml");
    auto c = aim::ArmorAutoConfig::load(argv[1]);
    auto node = YAML::LoadFile(argv[1]);
    const auto base = std::filesystem::absolute(argv[1]).parent_path();
    check(c.target_color == 1 || c.target_color == 2, "invalid color protocol");
    c.target_color = 2; // 跟踪用例固定测红色，不依赖用户现场选色。
    c.detector.has_camera = false;
    c.calibrated = false;
    aim::ArmorTracker t(c);
    auto red = candidate(0, 330, 240), blue = candidate(1, 320, 240);
    auto r = t.update({blue, red}, {640, 480}, 1);
    check(r.target && r.target->class_id == 0 && !r.confirmed,
          "color filter/first confirmation wrong");
    r = t.update({blue, red}, {640, 480}, 1.03);
    check(!r.confirmed, "confirmed too early");
    r = t.update({blue, red}, {640, 480}, 1.06);
    check(r.confirmed, "target not confirmed");
    auto id = r.id;
    auto competing = candidate(0, 320, 240);
    red.center.x = 335;
    red.box.x += 5;
    r = t.update({competing, red}, {640, 480}, 1.09);
    check(r.target && r.target->center.x == 335 && r.id == id,
          "target switched to nearer screen-center candidate");
    r = t.update({blue}, {640, 480}, 1.12);
    check(!r.target && !r.confirmed, "lost target returned stale armor");
    r = t.update({blue}, {640, 480}, 1.5);
    check(!r.target, "wrong color reacquired");
    r = t.update({red}, {640, 480}, 1.53);
    check(r.target && !r.confirmed && r.id > id,
          "new track not reset after loss");
    t.reset();
    c.target_color = 1;
    aim::ArmorTracker tb(c);
    r = tb.update({red, blue}, {640, 480}, 2);
    check(r.target && r.target->class_id == 1, "blue=1 protocol wrong");
    r = tb.update({red, blue}, {640, 480}, 1.9);
    check(!r.target, "backward timestamp accepted");
    const auto dir = std::filesystem::temp_directory_path() /
                     ("armor-auto-test-" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    node["detector_config"] =
        (base / node["detector_config"].as<std::string>()).string();
    // Calibration fixtures below use their own 640x480 camera, independent of
    // the deployed camera's configured output resolution.
    node["camera"]["width"] = 640;
    node["camera"]["height"] = 480;
    node["rgb_calibration"] = (dir / "camera.yaml").string();
    {
      std::ofstream f(dir / "auto.yaml");
      f << node;
    }
    const auto uncalibrated =
        aim::ArmorAutoConfig::load((dir / "auto.yaml").string());
    check(!uncalibrated.calibrated && !uncalibrated.detector.has_camera,
          "missing calibration reused or fabricated camera intrinsics");
    {
      std::ofstream f(dir / "camera.yaml");
      f << "camera_width: 640\ncamera_height: 480\ncamera_matrix: "
           "[600,0,320,0,600,240,0,0,1]\ndistort_coeffs: [0,0,0,0,0]\n";
    }
    check(aim::ArmorAutoConfig::load((dir / "auto.yaml").string()).calibrated,
          "valid calibration rejected");
    {
      std::ofstream f(dir / "camera.yaml");
      f << "camera_width: 640\ncamera_height: 480\ncamera_matrix: "
           "[.nan,0,320,0,600,240,0,0,1]\ndistort_coeffs: [0,0,0,0,0]\n";
    }
    bool rejected = false;
    try {
      aim::ArmorAutoConfig::load((dir / "auto.yaml").string());
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "NaN calibration accepted");
    node["target_color"] = 0;
    {
      std::ofstream f(dir / "auto.yaml");
      f << node;
    }
    rejected = false;
    try {
      aim::ArmorAutoConfig::load((dir / "auto.yaml").string());
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "color=0 silently mapped to blue/red");
    std::filesystem::remove_all(dir);
    // 独立已知相机、多位置/倾角视图，验证标定质量门控和求出的内参。
    const cv::Matx33d K(600, 0, 320, 0, 610, 240, 0, 0, 1);
    std::vector<cv::Point3f> model;
    for (int y = 0; y < 6; ++y)
      for (int x = 0; x < 8; ++x)
        model.emplace_back(x * .025f, y * .025f, 0);
    std::vector<std::vector<cv::Point2f>> views;
    for (int i = 0; i < 30; ++i) {
      std::vector<cv::Point2f> pixels;
      const double tx = -.23 + (i % 3) * .14, ty = -.18 + ((i / 3) % 3) * .11;
      const cv::Vec3d rotation(.15 * ((i % 5) - 2), .14 * (((i / 5) % 5) - 2),
                               .025 * (i % 3));
      cv::projectPoints(model, rotation, cv::Vec3d(tx, ty, .65 + .06 * (i % 2)),
                        K, cv::noArray(), pixels);
      views.push_back(pixels);
    }
    auto camera = aim::calibrateRgbViews(views, {640, 480}, {8, 6}, 25);
    check(std::abs(camera.camera.at<double>(0, 0) - 600) < 1 &&
              std::abs(camera.camera.at<double>(1, 1) - 610) < 1,
          "calibration did not recover known intrinsics");
    check(camera.validation_max_rms_px < .01 && camera.normal_spread_deg >= 15,
          "calibration validation missing");
    views.assign(25, views[0]);
    rejected = false;
    try {
      aim::calibrateRgbViews(views, {640, 480}, {8, 6}, 25);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, "duplicate/poor-coverage calibration accepted");
    std::cout << "Automatic armor regression passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
