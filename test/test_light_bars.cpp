#include "demo_aim/light_bars.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <unistd.h>
namespace {
void check(bool value, const char *why) {
  if (!value)
    throw std::runtime_error(why);
}
template <class F> void rejects(F f) {
  bool threw = false;
  try {
    f();
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "invalid input accepted");
}
cv::Mat blank() { return cv::Mat::zeros(240, 320, CV_8UC3); }
} // namespace
int main(int argc, char **argv) {
  try {
    auto c = aim::LightBarConfig::load(argc > 1 ? argv[1]
                                                : "config/light_bars.yaml");
    auto image = blank();
    cv::rectangle(image, {30, 50, 10, 70}, {0, 0, 255}, -1);
    cv::rectangle(image, {150, 50, 10, 70}, {255, 0, 0}, -1);
    auto result = aim::detectLightBars(image, c);
    check(result.bars.size() == 2, "red and blue bars missing");
    check(result.bars[0].class_id == 0 && result.bars[1].class_id == 1,
          "BGR/color mismatch");
    auto zero_value = c;
    zero_value.min_value = 0;
    check(aim::detectLightBars(image, zero_value).bars.size() == 2,
          "zero value slider merged black background into bright components");
    check(result.armors.empty(), "different colors paired");
    check(result.bars[0].top.y < result.bars[0].bottom.y, "endpoint order");
    check(cv::norm(result.bars[0].rect.center - cv::Point2f(34.5f, 84.5f)) < 1,
          "wrong center");
    cv::Mat original = image.clone();
    aim::drawLightBars(image, result);
    check(cv::norm(original, image) == 0, "overlay mutated raw image");
    check(aim::detectLightBars(blank(), c).bars.empty(),
          "stale bars on empty scene");
    image = blank();
    cv::rectangle(image, {10, 20, 12, 70}, {0, 255, 0}, -1);
    cv::rectangle(image, {40, 20, 12, 70}, {255, 255, 255}, -1);
    cv::rectangle(image, {80, 20, 70, 12}, {0, 0, 255}, -1);
    cv::circle(image, {200, 80}, 18, {255, 0, 0}, -1);
    cv::rectangle(image, {240, 20, 2, 3}, {0, 0, 255}, -1);
    check(aim::detectLightBars(image, c).bars.empty(),
          "green/white/horizontal/round/noise not rejected");
    image = blank();
    cv::rectangle(image, {30, 50, 10, 70}, {0, 0, 100}, -1);
    check(aim::detectLightBars(image, c).bars.empty(),
          "brightness threshold ignored");
    c.min_value = 80;
    check(aim::detectLightBars(image, c).bars.size() == 1,
          "brightness adjustment ineffective");
    c.min_value = 160;
    cv::Mat hsv = blank();
    cv::rectangle(hsv, {30, 50, 10, 70}, {175, 255, 255}, -1);
    cv::cvtColor(hsv, image, cv::COLOR_HSV2BGR);
    result = aim::detectLightBars(image, c);
    check(result.bars.size() == 1 && result.bars[0].class_id == 0,
          "red hue wraparound");
    image = blank();
    cv::line(image, {100, 50}, {125, 120}, {255, 0, 0}, 8);
    check(aim::detectLightBars(image, c).bars.size() == 1,
          "tilted bar rejected");
    c.max_tilt_deg = 5;
    check(aim::detectLightBars(image, c).bars.empty(), "tilt limit ignored");
    rejects([&] { aim::detectLightBars(cv::Mat(), c); });
    rejects([&] { aim::detectLightBars(cv::Mat::zeros(30, 30, CV_8UC1), c); });
    c.min_area = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { c.validate(); });
    image = blank();
    cv::rectangle(image, {40, 40, 8, 80}, {0, 0, 255}, -1);
    cv::rectangle(image, {160, 42, 8, 78}, {0, 0, 255}, -1);
    c = aim::LightBarConfig::load(argc > 1 ? argv[1]
                                           : "config/light_bars.yaml");
    result = aim::detectLightBars(image, c);
    check(result.armors.size() == 1 && result.armors[0].reported, "one armor");
    check(result.armors[0].class_id == 0 && !result.armors[0].pose_valid,
          "uncalibrated armor");
    const auto &corners = result.armors[0].corners;
    check(corners[0].x < corners[1].x && corners[3].x < corners[2].x &&
              corners[0].y < corners[3].y && corners[1].y < corners[2].y,
          "corner order");
    auto overlay = aim::drawLightBars(image, result);
    check(cv::norm(image, overlay) > 0, "armor overlay missing");
    image = blank();
    cv::rectangle(image, {20, 30, 8, 70}, {0, 0, 255}, -1);
    cv::rectangle(image, {90, 30, 8, 70}, {0, 0, 255}, -1);
    cv::rectangle(image, {210, 30, 8, 70}, {0, 0, 255}, -1);
    cv::rectangle(image, {280, 30, 8, 70}, {0, 0, 255}, -1);
    result = aim::detectLightBars(image, c);
    check(result.armors.size() == 2, "two armors");
    const aim::ArmorCandidate *shown = nullptr;
    for (const auto &armor : result.armors)
      if (armor.reported)
        shown = &armor;
    check(shown && shown->center.x > image.cols / 2.0, "closest armor");
    // 旧评分优先选择中间较近的一对，吃掉两块正确候选。
    image = cv::Mat::zeros(480, 640, CV_8UC3);
    for (int x : {20, 155, 255, 390})
      cv::rectangle(image, {x, 200, 6, 56}, {0, 0, 255}, -1);
    result = aim::detectLightBars(image, c);
    check(result.bars.size() == 4 && result.armors.size() == 2,
          "adjacent armor conflict not resolved");
    check(std::abs(result.armors[0].corners[0].x - 22.5f) < 1 &&
              std::abs(result.armors[0].corners[1].x - 157.5f) < 1 &&
              std::abs(result.armors[1].corners[0].x - 257.5f) < 1,
          "inter-armor gap selected instead of outer pairs");
    for (const auto &armor : result.armors)
      check(armor.geometry_score >= c.pair_min_score &&
                armor.geometry_score <= 1,
            "invalid geometric score");
    auto shuffled = result.bars;
    std::reverse(shuffled.begin(), shuffled.end());
    const auto reordered = aim::pairLightBars(shuffled, c);
    check(reordered.size() == 2 &&
              reordered[0].center == result.armors[0].center,
          "pairing depends on input order");
    auto restricted = c;
    restricted.pair_max_ratio = 1;
    check(aim::pairLightBars(result.bars, restricted).empty(),
          "pair geometry config ignored");
    // 保留亮灯条白芯，但需要有效颜色支持；纯白不会被默认分为红色。
    image = blank();
    for (int x : {40, 160}) {
      cv::rectangle(image, {x, 50, 10, 56}, {0, 0, 255}, -1);
      cv::rectangle(image, {x + 2, 50, 6, 56}, {255, 255, 255}, -1);
    }
    result = aim::detectLightBars(image, c);
    check(result.bars.size() == 2 && result.armors.size() == 1,
          "overexposed red cores split into multiple bars");
    check(result.red_mask.at<unsigned char>(70, 45) == 255,
          "white core missing from red component mask");
    auto legacy = c;
    legacy.brightness_color = false;
    check(aim::detectLightBars(image, legacy).armors.empty(),
          "HSV compatibility mode not used");
    image = blank();
    for (int x : {40, 160}) {
      cv::rectangle(image, {x, 50, 10, 56}, {255, 0, 0}, -1);
      cv::rectangle(image, {x + 2, 50, 6, 56}, {255, 255, 255}, -1);
    }
    result = aim::detectLightBars(image, c);
    check(result.armors.size() == 1 && result.armors[0].class_id == 1,
          "overexposed blue cores lost or classified red");
    image = blank();
    cv::rectangle(image, {30, 50, 10, 28}, {255, 0, 0}, -1);
    cv::rectangle(image, {30, 78, 10, 28}, {0, 0, 255}, -1);
    check(aim::detectLightBars(image, c).bars.empty(),
          "mixed color component accepted");
    image = blank();
    for (int x : {40, 160})
      for (int y : {25, 150})
        cv::rectangle(image, {x, y, 8, 56}, {0, 0, 255}, -1);
    check(aim::detectLightBars(image, c).armors.size() == 2,
          "vertically separated armors suppress each other");
    // 大冲突组必须走有界后备路径，也不能重复使用灯条。
    std::vector<aim::ColorLightBar> crowded;
    for (int i = 0; i < 20; ++i) {
      const cv::Point2f center(20.0f + i * 50, 100);
      crowded.push_back({0, cv::RotatedRect(center, {6, 56}, 0),
                         center - cv::Point2f(0, 28),
                         center + cv::Point2f(0, 28)});
    }
    auto crowded_result = aim::pairLightBars(crowded, c);
    check(crowded_result.size() == 10, "large conflict group fallback");
    for (std::size_t i = 0; i < crowded_result.size(); ++i)
      for (std::size_t j = i + 1; j < crowded_result.size(); ++j)
        check(crowded_result[i].corners[0].x !=
                      crowded_result[j].corners[0].x &&
                  crowded_result[i].corners[1].x !=
                      crowded_result[j].corners[0].x &&
                  crowded_result[i].corners[1].x !=
                      crowded_result[j].corners[1].x,
              "fallback reused a light bar");
    auto slanted_pair = std::vector<aim::ColorLightBar>{crowded[0], crowded[2]};
    slanted_pair[1].rect.center.y += 20;
    slanted_pair[1].top.y += 20;
    slanted_pair[1].bottom.y += 20;
    check(aim::pairLightBars(slanted_pair, c).size() == 1,
          "moderate center misalignment rejected");
    restricted = c;
    restricted.pair_max_rectangular_error_deg = 2;
    check(aim::pairLightBars(slanted_pair, restricted).empty(),
          "rectangular geometry config ignored");
    image = blank();
    for (int x : {40, 80})
      cv::rectangle(image, {x, 50, 4, 18}, {0, 0, 255}, -1);
    cv::GaussianBlur(image, image, {3, 3}, 0.6);
    check(aim::detectLightBars(image, c).armors.size() == 1,
          "small slightly blurred bars lost");
    aim::LightBarConfig pose = c;
    pose.has_camera = true;
    pose.fx = pose.fy = 500;
    pose.cx = 160;
    pose.cy = 120;
    pose.camera_width = 320;
    pose.camera_height = 240;
    pose.armor_width_m = 0.135;
    pose.lightbar_length_m = 0.056;
    pose.validate();
    const cv::Matx33d camera(500, 0, 160, 0, 500, 120, 0, 0, 1);
    const cv::Matx33d facing(0, -1, 0, 0, 0, -1, 1, 0, 0);
    const auto rotation_z = [](double deg) {
      const double a = deg * CV_PI / 180, cs = std::cos(a), sn = std::sin(a);
      return cv::Matx33d(cs, -sn, 0, sn, cs, 0, 0, 0, 1);
    };
    const auto rotation_y = [](double deg) {
      const double a = deg * CV_PI / 180, cs = std::cos(a), sn = std::sin(a);
      return cv::Matx33d(cs, 0, sn, 0, 1, 0, -sn, 0, cs);
    };
    const float half_w = 0.135f / 2, half_h = 0.056f / 2;
    const std::vector<cv::Point3f> object{{0, half_w, half_h},
                                          {0, -half_w, half_h},
                                          {0, -half_w, -half_h},
                                          {0, half_w, -half_h}};
    const auto project = [&](const cv::Matx33d &rotation, const cv::Vec3d &t) {
      cv::Mat rvec;
      cv::Rodrigues(cv::Mat(rotation), rvec);
      std::vector<cv::Point2f> pixels;
      cv::projectPoints(object, rvec, t, camera, cv::noArray(), pixels);
      std::array<cv::Point2f, 4> points{};
      for (int i = 0; i < 4; ++i)
        points[i] = pixels[i];
      return points;
    };
    const auto near_angle = [](double value, double expected) {
      return std::abs(value - expected) < 1.0;
    };
    auto facing_pose =
        aim::solveArmorPose(project(facing, {0.3, 0, 2}), pose, {320, 240});
    check(facing_pose && near_angle(facing_pose->yaw_deg, 0) &&
              near_angle(facing_pose->pitch_deg, 0),
          "facing pose");
    check(cv::norm(facing_pose->position_camera_m - cv::Vec3d(.3, 0, 2)) < 1e-3,
          "PnP translation or metric scale wrong");
    auto yawed = aim::solveArmorPose(
        project(facing * rotation_z(20), {0, 0, 2}), pose, {320, 240});
    check(yawed && near_angle(yawed->yaw_deg, 20) &&
              near_angle(yawed->pitch_deg, 0),
          "positive yaw");
    auto pitched = aim::solveArmorPose(
        project(facing * rotation_y(-15), {0, 0, 2}), pose, {320, 240});
    check(pitched && near_angle(pitched->pitch_deg, -15) &&
              near_angle(pitched->yaw_deg, 0),
          "negative pitch");
    check(pitched->reprojection_error_px < 0.1, "reprojection quality missing");
    const auto distant_pose = aim::solveArmorPose(
        project(facing * rotation_z(20), {0, 0, 8}), pose, {320, 240});
    check(distant_pose && distant_pose->ambiguous,
          "nearly equivalent planar pose hypotheses not flagged");
    auto invalid_corners = project(facing, {0, 0, 2});
    std::swap(invalid_corners[1], invalid_corners[2]);
    check(!aim::solveArmorPose(invalid_corners, pose, {320, 240}),
          "crossed quad accepted by pose solver");
    invalid_corners = project(facing, {0, 0, 2});
    invalid_corners[0].x += 10;
    auto precise_pose = pose;
    precise_pose.pose_max_error_px = 0.05;
    check(!aim::solveArmorPose(invalid_corners, precise_pose, {320, 240}),
          "pose reprojection threshold ignored");
    auto invalid_pose = pose;
    invalid_pose.fx = std::numeric_limits<double>::infinity();
    rejects([&] { invalid_pose.validate(); });
    auto invalid_pair = c;
    invalid_pair.pair_min_score = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { invalid_pair.validate(); });
    // 截图配置必须保留全部新增阈值，避免离线重测与现场不一致。
    const auto config_copy =
        std::filesystem::temp_directory_path() /
        ("armor_config_" + std::to_string(getpid()) + ".yaml");
    auto saved = pose;
    saved.brightness_color = false;
    saved.min_color_fraction = 0.2;
    saved.pair_min_score = 0.4;
    saved.pose_max_error_px = 1.5;
    {
      std::ofstream stream(config_copy);
      saved.write(stream);
    }
    const auto loaded = aim::LightBarConfig::load(config_copy.string());
    std::filesystem::remove(config_copy);
    check(loaded.has_camera && loaded.fx == saved.fx &&
              loaded.camera_width == saved.camera_width &&
              loaded.brightness_color == saved.brightness_color &&
              loaded.min_color_fraction == saved.min_color_fraction &&
              loaded.pair_min_score == saved.pair_min_score &&
              loaded.pose_max_error_px == saved.pose_max_error_px,
          "snapshot config round trip changed recognition settings");
    pose.has_camera = false;
    check(!aim::solveArmorPose(project(facing, {0, 0, 2}), pose, {320, 240}),
          "intrinsics required");
    std::cout << "Light bar regression passed: color/white cores, geometry, "
                 "pair conflicts/fallback, small blurred bars, empty scene, "
                 "raw preservation, pose quality/ambiguity, config round trip, "
                 "invalid input\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
