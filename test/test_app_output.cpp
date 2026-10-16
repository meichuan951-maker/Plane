#include "../src/app/armor_auto/output.hpp"
#include "../src/app/common/frame_rate.hpp"
#include "../src/app/common/parse.hpp"
#include "../src/app/depth/output.hpp"
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using nlohmann::json;
namespace rgb = aim::armor_auto_app;
namespace depth = aim::depth_app;

void check(bool ok, const std::string &why) {
  if (!ok)
    throw std::runtime_error(why);
}

void near(const json &value, double expected, const char *why) {
  check(value.is_number() && std::abs(value.get<double>() - expected) < 1e-8,
        why);
}

void nullFields(const json &packet,
                std::initializer_list<const char *> fields) {
  for (const auto *field : fields)
    check(packet.at(field).is_null(),
          std::string("cached value leaked: ") + field);
}

void noMeasurement(const json &packet) {
  check(!packet.at("valid").get<bool>(),
        "invalid frame reported a measurement");
  nullFields(packet, {"z_m", "range_m", "normal_yaw_deg", "normal_pitch_deg",
                      "bearing_yaw_deg", "bearing_pitch_deg"});
}

aim::TrackedArmor armorFixture() {
  aim::ArmorCandidate armor;
  armor.pose_valid = true;
  // Optical frame: right 2 m, up 1 m, forward 2 m. Range is 3 m;
  // bearing is right 45 degrees, up 19.471... degrees, independent of normal.
  armor.position_camera_m = {2, -1, 2};
  armor.yaw_deg = -12;
  armor.pitch_deg = 8;
  armor.reprojection_error_px = .4;
  armor.corners = {cv::Point2f(10, 20), cv::Point2f(30, 20),
                   cv::Point2f(30, 40), cv::Point2f(10, 40)};
  return {armor, 7, true};
}

aim::DepthPlane planeFixture() {
  aim::DepthPlane plane;
  plane.centroid_m = {2, -1, 2};
  plane.range_m = 3;
  plane.normal_yaw_deg = -12;
  plane.normal_pitch_deg = 8;
  plane.bearing_yaw_deg = 45;
  plane.bearing_pitch_deg = 19.47122063449069;
  plane.rmse_m = .002;
  return plane;
}

void checkAnglesAndDistance(const json &packet) {
  near(packet.at("range_m"), 3, "range must differ from optical Z");
  near(packet.at("z_m"), 2, "optical Z changed");
  near(packet.at("normal_yaw_deg"), -12, "normal yaw lost its meaning");
  near(packet.at("normal_pitch_deg"), 8, "normal pitch lost its meaning");
  near(packet.at("bearing_yaw_deg"), 45, "bearing yaw sign/units changed");
  near(packet.at("bearing_pitch_deg"), 19.47122063449069,
       "bearing pitch sign/units changed");
}

void rgbReports() {
  aim::ArmorAutoConfig config;
  config.calibrated = config.dimensions_confirmed = true;
  auto target = armorFixture();
  rgb::FrameContext context{11, 3, 30, true, false, 10, .1};
  auto packet = rgb::makeReport(config, target, context);
  check(packet.at("valid") && packet.at("reason") == "ok",
        "current confirmed RGB pose rejected");
  check(packet.at("coordinate_frame") == "rgb_optical" &&
            packet.at("position_camera_m") == json::array({2., -1., 2.}),
        "RGB optical coordinates changed");
  checkAnglesAndDistance(packet);
  near(packet.at("sample_time_monotonic_s"), 9.9, "RGB sample time changed");
  near(packet.at("valid_until_monotonic_s"), 10.15, "RGB expiry changed");
  check(packet.at("armor_corners_px").size() == 4,
        "current armor corners disappeared");

  auto rejected = [&](const char *reason) {
    const auto report = rgb::makeReport(config, target, context);
    noMeasurement(report);
    nullFields(report, {"position_camera_m", "reprojection_error_px",
                        "valid_until_monotonic_s"});
    check(report.at("reason") == reason, "RGB rejection reason changed");
  };
  config.calibrated = false;
  rejected("rgb_calibration_missing");
  config.calibrated = true;
  config.dimensions_confirmed = false;
  rejected("armor_dimensions_unconfirmed");
  config.dimensions_confirmed = true;
  target.confirmed = false;
  rejected("confirming_target");
  target.confirmed = true;
  target.target->pose_valid = false;
  rejected("pose_rejected");
  target.target->pose_valid = true;
  target.target->pose_ambiguous = true;
  rejected("pose_ambiguous");
  target.target.reset();
  rejected("target_lost_or_absent");
  target = armorFixture();

  // The cached pose remains valid: frame freshness must independently block it.
  for (double age : {.251, .3, -.01, std::numeric_limits<double>::quiet_NaN()}) {
    context.age_s = age;
    rejected("stale_or_missing_frame");
    const auto report = rgb::makeReport(config, target, context);
    check(!report.at("detection_valid").get<bool>() &&
              !report.at("tracking_confirmed").get<bool>(),
          "RGB cached detection remained current");
    nullFields(report, {"armor_corners_px"});
    if (!std::isfinite(age) || age < 0)
      nullFields(report, {"frame_age_ms", "sample_time_monotonic_s"});
  }
  context.age_s = .1;
  context.has_image = false;
  rejected("stale_or_missing_frame");
  context.has_image = true;
  context.failed = true;
  rejected("camera_failed");

  for (bool failed : {false, true}) {
    auto stopped = packet;
    rgb::invalidateReport(stopped, 12, failed);
    noMeasurement(stopped);
    nullFields(stopped, {"position_camera_m", "reprojection_error_px",
                         "armor_corners_px", "frame_age_ms",
                         "sample_time_monotonic_s", "valid_until_monotonic_s"});
    check(stopped.at("frame") == 12 &&
              stopped.at("reason") == (failed ? "camera_failed" : "stopped") &&
              !stopped.at("detection_valid").get<bool>() &&
              !stopped.at("tracking_confirmed").get<bool>(),
          "RGB stop report retained a live target");
  }
}

void depthReports() {
  depth::FrameContext context{11, 3, 30, true, false, 10, .1, "ok"};
  const cv::Rect roi(10, 20, 30, 40);
  const auto measurement = planeFixture();
  auto packet = depth::makePacket(context, roi, false, true, measurement);
  check(packet.at("valid") && packet.at("reason") == "ok",
        "current depth measurement rejected");
  checkAnglesAndDistance(packet);
  check(packet.at("xyz_depth_optical_m") == json::array({2., -1., 2.}) &&
            packet.at("position_local_world_m") == json::array({2., -2., 1.}),
        "fixed-camera optical-to-local axes changed");
  check(packet.at("world_frame") == "fixed_depth_camera_anchor" &&
            packet.at("world_requires_stationary_camera") &&
            !packet.at("armor_identity_verified").get<bool>() &&
            !packet.at("tracking").get<bool>(),
        "manual depth output gained unverified tracking/world semantics");
  near(packet.at("sample_time_monotonic_s"), 9.9, "depth sample time changed");
  const auto optical =
      depth::makePacket(context, roi, true, false, measurement);
  nullFields(optical, {"position_local_world_m", "world_frame"});
  check(optical.at("source") == "center_depth_surface" &&
            packet.at("source") == "manual_depth_roi",
        "manual and center-depth sources became indistinguishable");

  auto rejected = [&](const char *reason) {
    const auto report =
        depth::makePacket(context, roi, false, true, measurement);
    noMeasurement(report);
    nullFields(report, {"xyz_depth_optical_m", "position_local_world_m",
                        "plane_rmse_m"});
    check(report.at("reason") == reason, "depth rejection reason changed");
  };
  for (double age : {.251, .3, -.01, std::numeric_limits<double>::quiet_NaN()}) {
    context.age_s = age;
    rejected("stale_or_missing_depth");
    if (!std::isfinite(age) || age < 0)
      nullFields(depth::makePacket(context, roi, false, true, measurement),
                 {"frame_age_ms", "sample_time_monotonic_s"});
  }
  context.age_s = .1;
  context.has_image = false;
  rejected("stale_or_missing_depth");
  context.has_image = true;
  context.failed = true;
  rejected("depth_stream_failed");
  context.failed = false;
  context.reason = "plane_rejected";
  const auto absent =
      depth::makePacket(context, roi, false, true, std::nullopt);
  noMeasurement(absent);
  nullFields(absent, {"xyz_depth_optical_m", "position_local_world_m"});
  check(absent.at("reason") == "plane_rejected", "depth failure reason lost");

  for (bool failed : {false, true}) {
    const auto stopped = depth::stoppedPacket(packet, 12, failed);
    noMeasurement(stopped);
    nullFields(stopped,
               {"xyz_depth_optical_m", "position_local_world_m", "plane_rmse_m",
                "frame_age_ms", "sample_time_monotonic_s"});
    check(stopped.at("frame") == 12 &&
              stopped.at("reason") ==
                  (failed ? "depth_stream_failed" : "stopped"),
          "depth stop report retained live status");
  }
}

void commonHelpers() {
  using namespace std::chrono;
  const aim::app::FrameRate::Clock::time_point start{};
  aim::app::FrameRate fps(start);
  check(fps.update(start, false) == 0, "initial FPS fabricated a frame");
  for (int frame = 1; frame <= 30; ++frame)
    fps.update(start + milliseconds(frame * 30), true);
  check(fps.update(start + seconds(1), false) == 30,
        "waiting polls counted as completed frames");
  check(fps.update(start + milliseconds(1500), false) == 30,
        "FPS changed before the next interval");
  check(fps.update(start + seconds(2), false) == 0,
        "stalled stream retained its old FPS");
  fps.update(start + milliseconds(2250), true);
  fps.update(start + milliseconds(2500), true);
  check(fps.update(start + seconds(3), true) == 3,
        "FPS did not recover after new frames");

  check(aim::app::parseInteger("42") == 42 &&
            aim::app::parseInteger("-2") == -2,
        "valid integer rejected");
  for (const char *value : {"42x", "42 ", "3.5", "", "99999999999999999999"}) {
    bool rejected = false;
    try {
      aim::app::parseInteger(value);
    } catch (const std::exception &) {
      rejected = true;
    }
    check(rejected, std::string("invalid integer accepted: ") + value);
  }
}
} // namespace

int main() {
  try {
    rgbReports();
    depthReports();
    commonHelpers();
    std::cout << "Application output regression passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
