// 可重复离线演示：所有位姿和反馈都是脚本数据，不模拟真实飞行动力学。
#include "demo_aim/observation.hpp"
#include "demo_aim/mission.hpp"
#include <iostream>
int main() {
  try {
    aim::MissionConfig c;
    c.home_map_m = {0, 0, 0};
    c.takeoff_map_m = {0, 0, 1};
    c.recovery_map_m = {1, 0, 1};
    c.return_route_map_m = {{0, 0, 1}};
    c.time_domain = "replay_monotonic";
    aim::MissionMachine mission(c);
    aim::PoseHistory history;
    aim::WorldCalibration calibration;
    // 此外参仅为“前视相机与机体共点”的合成夹具，不是真机标定参数。
    calibration.body_T_camera =
        aim::RigidTransform{{0, 0, 1, -1, 0, 0, 0, -1, 0}, {}};
    calibration.map_T_odom = aim::RigidTransform{};
    calibration.epoch = 1;
    calibration.odom_frame = "odom_enu";
    const std::vector<cv::Vec3d> path = {
        {0, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {3, 0, 1},
        {3, 0, 1}, {3, 0, 1}, {1, 0, 1}, {0, 0, 1}, {0, 0, 0}};
    for (std::size_t k = 0; k < path.size(); ++k) {
      const double t = 1. + .1 * k;
      aim::PoseSample pose;
      pose.valid = true;
      pose.reason = "synthetic";
      pose.capture_s = t;
      pose.received_s = t;
      pose.time_domain = "replay_monotonic";
      pose.position_m = path[k];
      history.push(pose);
      aim::Solution target;
      target.valid = target.detection_valid = target.position_valid = true;
      target.capture_s = t;
      target.emitted_s = t;
      target.time_domain = "replay_monotonic";
      target.track_id = 1;
      target.frame_id = "camera_color_optical_frame";
      // 固定地图点 (4,0,1) 的相机观测；通过实际变换模块恢复同一个世界位置。
      target.point_camera =
          (*calibration.body_T_camera)
              .inverse()
              .apply(pose.odomFromBody().inverse().apply({4, 0, 1}));
      aim::MissionInput input;
      input.now = t;
      input.vehicle_map = aim::poseInMap(pose, calibration);
      input.target = aim::projectTargetToMap(target, history, calibration, t,
                                             .2, "replay_monotonic");
      input.flight_state_s = t;
      input.flight_healthy = true;
      input.start_requested = k == 0;
      input.landed = k == 0 || k == 9;
      input.contact_authorized = true;
      input.contact_region_verified = true;
      if (k == 5)
        input.contact_confirmed_s = t;
      // 离线夹具没有障碍；真实接入必须替换为整段机体碰撞检查。
      input.segment_clear = [](const cv::Vec3d &, const cv::Vec3d &) {
        return true;
      };
      const auto intent = mission.step(input);
      std::cout << aim::Json{{"synthetic", true},
                             {"time_s", t},
                             {"target", input.target.json()},
                             {"vehicle", input.vehicle_map.json()},
                             {"intent", intent.json()}}
                       .dump()
                << '\n';
      if (k == path.size() - 1 && intent.phase != aim::MissionPhase::Complete)
        return 2;
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
