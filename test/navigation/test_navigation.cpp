// 无硬件回归：重点检验坐标含义、图像时刻插值、失效和任务边界。
#include "support/assertions.hpp"
#include "support/runner.hpp"
#include "demo_aim/core.hpp"
#include "demo_aim/mission.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace {
using test_support::check;
void near(const cv::Vec3d &a, const cv::Vec3d &b) {
  check(cv::norm(a - b) < 1e-7, "vector mismatch");
}
template <class F> void throws(F function) {
  test_support::expectThrows<std::invalid_argument>(function, "expected rejection");
}
aim::PoseSample pose(double t, cv::Vec3d p = {}) {
  aim::PoseSample s;
  s.valid = true;
  s.reason = "test";
  s.capture_s = t;
  s.received_s = t + .01;
  s.position_m = p;
  return s;
}
aim::WorldCalibration calibration() {
  aim::WorldCalibration c;
  c.epoch = 1;
  c.odom_frame = "odom_enu";
  c.map_T_odom = aim::RigidTransform{cv::Matx33d::eye(), {10, 20, 0}};
  c.body_T_camera =
      aim::RigidTransform{{0, 0, 1, -1, 0, 0, 0, -1, 0}, {.1, 0, 0}};
  return c;
}
aim::Solution observation(double t) {
  aim::Solution s;
  s.valid = s.position_valid = s.detection_valid = true;
  s.capture_s = t;
  s.emitted_s = t + .02;
  s.frame_id = "camera_color_optical_frame";
  s.point_camera = cv::Vec3d{0, 0, 2};
  s.track_id = 1;
  return s;
}
struct MissionFixture {
  aim::MissionConfig c;
  aim::MissionInput i;
  MissionFixture() {
    c.home_map_m = {0, 0, 0};
    c.takeoff_map_m = {0, 0, 1};
    c.recovery_map_m = {1, 0, 1};
    c.return_route_map_m = {{.5, 0, 1}, {0, 0, 1}};
    i.vehicle_map = pose(1);
    i.vehicle_map.odom_frame = "map";
    i.flight_healthy = true;
    i.landed = true;
    i.start_requested = true;
    i.segment_clear = [](const cv::Vec3d &, const cv::Vec3d &) { return true; };
    advance(1, {0, 0, 0});
  }
  void advance(double t, cv::Vec3d p) {
    i.now = t;
    i.flight_state_s = t;
    i.vehicle_map.capture_s = t;
    i.vehicle_map.received_s = t;
    i.vehicle_map.position_m = p;
  }
  void target(double t) {
    i.target.valid = true;
    i.target.time_domain = "host_monotonic";
    i.target.epoch = 1;
    i.target.track_id = 1;
    i.target.point_map_m = cv::Vec3d{4, 0, 1};
    i.target.capture_s = t;
  }
  void approach(aim::MissionMachine &m) {
    check(m.step(i).phase == aim::MissionPhase::Takeoff, "takeoff");
    advance(1.1, {0, 0, 1});
    i.landed = false;
    check(m.step(i).phase == aim::MissionPhase::Search, "search");
    advance(1.2, {0, 0, 1});
    target(1.2);
    check(m.step(i).phase == aim::MissionPhase::Approach, "approach");
  }
};
} // namespace
int main() {
  test_support::Runner run;
  run("ENU/NED and FLU/FRD axes", [] {
    near(aim::enuFromNed() * cv::Vec3d{1, 2, 3}, {2, 1, -3});
    near(aim::fluFromFrd() * cv::Vec3d{1, 2, 3}, {1, -2, -3});
  });
  run("rigid composition/inverse and lever arm", [] {
    auto c = calibration();
    const auto t = *c.map_T_odom * *c.body_T_camera;
    near(t.apply({0, 0, 2}), {12.1, 20, 0});
    near(t.inverse().apply(t.apply({1, 2, 3})), {1, 2, 3});
  });
  run("reflection and nonfinite transform rejected", [] {
    throws([] {
      aim::RigidTransform{{1, 0, 0, 0, 1, 0, 0, 0, -1}, {}}.validate();
    });
    throws([] {
      aim::RigidTransform t;
      t.translation[1] = NAN;
      t.validate();
    });
  });
  run("quaternion 180 degrees and SLERP shortest path", [] {
    const cv::Vec4d q{0, 0, 0, 1};
    near(aim::quaternionRotation(
             aim::rotationQuaternion(aim::quaternionRotation(q))) *
             cv::Vec3d{1, 0, 0},
         {-1, 0, 0});
    near(aim::quaternionRotation(aim::slerp({1, 0, 0, 0}, q, .5)) *
             cv::Vec3d{1, 0, 0},
         {0, 1, 0});
    check(std::abs(aim::slerp(q, -q, .5).dot(q)) > 0.99999, "antipodal");
    throws([] { aim::quaternionRotation({0, 0, 0, 0}); });
  });
  run("pose history interpolates translation and attitude", [] {
    aim::PoseHistory h;
    auto a = pose(1, {0, 0, 0}), b = pose(1.05, {2, 0, 0});
    b.body_flu_to_odom_wxyz = {0, 0, 0, 1};
    h.push(a);
    h.push(b);
    const auto s = h.at(1.025, "host_monotonic");
    check(bool(s), "bracket");
    near(s->position_m, {1, 0, 0});
    near(s->odomFromBody().rotation * cv::Vec3d{1, 0, 0}, {0, 1, 0});
  });
  run("pose history never extrapolates or crosses gaps/domain", [] {
    aim::PoseHistory h;
    h.push(pose(1));
    h.push(pose(1.3));
    check(!h.at(.9, "host_monotonic") && !h.at(1.4, "host_monotonic") &&
              !h.at(1.1, "host_monotonic") && !h.at(1, "replay_monotonic"),
          "unsafe history query");
  });
  run("history duplicate/reordering and capacity", [] {
    aim::PoseHistory h({3, 1, .1, .01});
    for (int n = 0; n < 10; ++n)
      check(h.push(pose(1 + n * .025)), "push");
    check(h.size() == 3, "bounded");
    check(!h.push(pose(1.1)) && !h.push(pose(1.225)), "out of order");
  });
  run("invalid pose clears history and rejects old epoch", [] {
    aim::PoseHistory h;
    h.push(pose(1));
    auto s = pose(1.1);
    s.epoch = 2;
    h.push(s);
    s.valid = false;
    h.push(s);
    check(h.size() == 0, "invalid clear");
    check(!h.push(pose(1.2)), "old epoch revived");
    s = pose(1.3);
    s.epoch = 2;
    s.time_error_s = .1;
    check(!h.push(s), "clock error");
  });
  run("world target uses capture time not newest vehicle pose", [] {
    aim::PoseHistory h;
    h.push(pose(1, {0, 0, 0}));
    h.push(pose(1.05, {2, 0, 0}));
    const auto t =
        aim::projectTargetToMap(observation(1.025), h, calibration(), 1.08);
    check(t.valid, "projection");
    near(*t.point_map_m, {13.1, 20, 0});
  });
  run("world target requires distance, calibration and correct clocks", [] {
    aim::PoseHistory h;
    h.push(pose(1));
    auto o = observation(1);
    auto c = calibration();
    o.point_camera.reset();
    check(!aim::projectTargetToMap(o, h, c, 1.02).valid,
          "bearing is not position");
    o = observation(1);
    c.body_T_camera.reset();
    check(!aim::projectTargetToMap(o, h, c, 1.02).valid, "missing extrinsic");
    check(!aim::projectTargetToMap(o, h, calibration(), 1.4).valid, "stale");
    o.time_domain = "replay_monotonic";
    check(!aim::projectTargetToMap(o, h, calibration(), 1.02).valid,
          "wrong clock");
  });
  run("world target invalid after EKF reset and foreign camera", [] {
    aim::PoseHistory h;
    auto p = pose(1);
    p.epoch = 2;
    h.push(p);
    check(aim::projectTargetToMap(observation(1), h, calibration(), 1.02)
                  .reason == "map_alignment_expired",
          "reset alignment");
    auto c = calibration();
    c.epoch = 2;
    auto o = observation(1);
    o.frame_id = "other_camera";
    check(!aim::projectTargetToMap(o, h, c, 1.02).valid, "wrong camera");
  });
  run("self pose map transform includes translation and velocity rotation", [] {
    auto p = pose(1, {1, 2, 3});
    p.velocity_odom_mps = cv::Vec3d{1, 0, 0};
    const auto m = aim::poseInMap(p, calibration());
    check(m.valid && m.odom_frame == "map", "map self pose");
    near(m.position_m, {11, 22, 3});
    p.epoch = 2;
    check(!aim::poseInMap(p, calibration()).valid, "expired self alignment");
  });
  run("map transform rejects malformed pose metadata and optional velocity", [] {
    const std::vector<std::function<void(aim::PoseSample &)>> corrupt = {
        [](auto &p) { p.capture_s = -.1; },
        [](auto &p) { p.received_s = p.capture_s - .1; },
        [](auto &p) { p.received_s = INFINITY; },
        [](auto &p) { p.time_error_s = NAN; },
        [](auto &p) { p.time_error_s = -.1; },
        [](auto &p) { p.time_domain.clear(); },
        [](auto &p) { p.velocity_odom_mps = cv::Vec3d{NAN, 0, 0}; }};
    for (const auto &change : corrupt) {
      auto p = pose(1);
      change(p);
      const auto mapped = aim::poseInMap(p, calibration());
      check(!mapped.valid && mapped.reason == "invalid_pose",
            "malformed source became valid map pose");
    }
    auto p = pose(1);
    auto c = calibration();
    p.epoch = c.epoch = 0;
    check(!aim::poseInMap(p, c).valid, "uninitialized epoch became valid");
  });
  run("pose integrity does not override consumer clock error limits", [] {
    auto p = pose(1);
    p.time_error_s = .02;
    check(aim::poseInMap(p, calibration()).valid,
          "coordinate transform imposed clock quality policy");
    aim::PoseHistory strict;
    check(!strict.push(p), "default history ignored clock error limit");
    aim::PoseHistory tolerant({256, 2, .1, .03});
    check(tolerant.push(p), "configured history limit not respected");
  });
  run("mission waits for localization and metric target", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    f.i.vehicle_map.valid = false;
    check(m.step(f.i).phase == aim::MissionPhase::Idle,
          "no takeoff without pose");
    f.advance(1.1, {0, 0, 0});
    f.i.vehicle_map.valid = true;
    m.step(f.i);
    f.advance(1.2, {0, 0, 1});
    f.i.landed = false;
    m.step(f.i);
    f.advance(1.3, {0, 0, 1});
    check(m.step(f.i).phase == aim::MissionPhase::Search,
          "search without metric target");
  });
  run("mission refuses unvalidated entire motion segment", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    bool called = false;
    f.i.segment_clear = [&](const cv::Vec3d &a, const cv::Vec3d &b) {
      called = true;
      near(a, {0, 0, 0});
      near(b, {0, 0, 1});
      return false;
    };
    auto r = m.step(f.i);
    check(called && r.action == "hold" && !r.waypoint_map_m, "blocked segment");
  });
  run("mission does not equate target loss with contact", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    f.approach(m);
    f.advance(1.3, {3, 0, 1});
    f.target(1.3);
    f.i.contact_authorized = f.i.contact_region_verified = true;
    check(m.step(f.i).phase == aim::MissionPhase::Contact, "contact request");
    f.advance(1.4, {3, 0, 1});
    f.i.target.valid = false;
    const auto r = m.step(f.i);
    check(r.phase == aim::MissionPhase::Recover &&
              r.reason == "contact_target_lost",
          "lost != contact");
  });
  run("mission stale target holds then recovery", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    f.approach(m);
    f.advance(1.5, {0, 0, 1});
    check(m.step(f.i).action == "hold", "no pursuit stale target");
    f.advance(1.8, {0, 0, 1});
    check(m.step(f.i).phase == aim::MissionPhase::Recover, "recovery timeout");
  });
  run("mission aborts when an airborne task lands unexpectedly", [] {
    for (bool contact : {false, true}) {
      MissionFixture f;
      aim::MissionMachine m(f.c);
      f.approach(m);
      if (contact) {
        f.advance(1.3, {3, 0, 1});
        f.target(1.3);
        f.i.contact_authorized = f.i.contact_region_verified = true;
        check(m.step(f.i).phase == aim::MissionPhase::Contact, "contact");
      }
      f.advance(1.4, {1, 0, 0});
      f.target(1.4);
      f.i.landed = true;
      const auto r = m.step(f.i);
      check(r.phase == aim::MissionPhase::Abort &&
                r.reason == "unexpected_landing" && !r.waypoint_map_m,
            "landed task still planned motion/contact");
      f.advance(1.5, {1, 0, 0});
      f.i.landed = false;
      check(m.step(f.i).phase == aim::MissionPhase::Abort,
            "unexpected landing did not latch");
    }
  });
  run("mission rejects malformed optional velocity", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    f.approach(m);
    f.advance(1.3, {0, 0, 1});
    f.target(1.3);
    f.i.vehicle_map.velocity_odom_mps = cv::Vec3d{INFINITY, 0, 0};
    const auto r = m.step(f.i);
    check(r.phase == aim::MissionPhase::Abort &&
              r.reason == "localization_unavailable",
          "mission accepted malformed map pose");
  });
  run("takeoff landing after liftoff aborts until explicit reset", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    check(m.step(f.i).phase == aim::MissionPhase::Takeoff, "takeoff");
    f.advance(1.1, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Takeoff,
          "normal waiting for liftoff rejected");
    f.advance(1.2, {0, 0, .4});
    f.i.landed = false;
    check(m.step(f.i).phase == aim::MissionPhase::Takeoff, "climbing");
    f.advance(1.3, {0, 0, 0});
    f.i.landed = true;
    const auto r = m.step(f.i);
    check(r.phase == aim::MissionPhase::Abort &&
              r.reason == "unexpected_landing" && !r.waypoint_map_m,
          "landing during climb retried takeoff");
    f.advance(1.4, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "abort latch");
    m.reset();
    f.advance(1.5, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Takeoff, "restart after reset");
    f.advance(1.6, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Takeoff,
          "reset retained previous airborne state");
  });
  run("mission resets and manual override latch abort", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    f.approach(m);
    f.advance(1.3, {0, 0, 1});
    f.i.vehicle_map.epoch = 2;
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "reset abort");
    f.advance(1.4, {0, 0, 1});
    f.i.vehicle_map.epoch = 1;
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "abort latch");
    m.reset();
    f.i.manual_override = true;
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "manual");
  });
  run("mission state freshness and time reversal", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    m.step(f.i);
    f.advance(2, {0, 0, 1});
    f.i.flight_state_s = 1;
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "stale health");
    m.reset();
    f.i.flight_state_s = 2;
    m.step(f.i);
    f.i.now = 1.9;
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "time reversal");
  });
  run("full mission return route and confirmed landing", [] {
    MissionFixture f;
    aim::MissionMachine m(f.c);
    f.approach(m);
    f.advance(1.3, {3, 0, 1});
    f.target(1.3);
    f.i.contact_authorized = f.i.contact_region_verified = true;
    m.step(f.i);
    f.advance(1.4, {3, 0, 1});
    f.i.contact_confirmed_s = 1.4;
    check(m.step(f.i).phase == aim::MissionPhase::Recover, "confirmed contact");
    f.advance(1.5, {1, 0, 1});
    check(m.step(f.i).phase == aim::MissionPhase::Return, "recovery");
    f.advance(1.6, {.5, 0, 1});
    auto r = m.step(f.i);
    near(*r.waypoint_map_m, {0, 0, 1});
    check(!r.executable, "must not fly");
    f.advance(1.7, {0, 0, 1});
    check(m.step(f.i).phase == aim::MissionPhase::Land, "land intent");
    f.advance(1.8, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Land,
          "needs landed confirmation");
    f.advance(1.9, {0, 0, 0});
    f.i.landed = true;
    check(m.step(f.i).phase == aim::MissionPhase::Complete, "mission complete");
    f.advance(2, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Complete,
          "no automatic restart");
  });
  run("mission configuration and stage timeout", [] {
    MissionFixture f;
    auto c = f.c;
    c.return_route_map_m.clear();
    throws([&] { aim::MissionMachine m(c); });
    f.c.stage_timeout_s = .1;
    aim::MissionMachine m(f.c);
    m.step(f.i);
    f.advance(1.2, {0, 0, 0});
    check(m.step(f.i).phase == aim::MissionPhase::Abort, "timeout");
  });
  return run.finish();
}
