// 独立台架诊断入口：不会启动 YOLO、解锁或执行任务状态机。
#include "demo_aim/observation.hpp"
#include "demo_aim/mavlink_link.hpp"
#include "demo_aim/serial_port.hpp"
#include <cmath>
#include <csignal>
#include <iostream>
#include <yaml-cpp/yaml.h>
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void stop(int) { stop_requested = 1; }
} // namespace
int main(int argc, char **argv) {
  try {
    std::string path = "config/px4_link.yaml";
    double seconds = 30;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "px4_localization_cpp --config config/px4_link.yaml "
                     "--seconds 30\n"
                     "MicoVIO -> PX4 -> Orange Pi. JSON diagnostics; only "
                     "TIMESYNC TX.\n";
        return 0;
      }
      if ((arg == "--config" || arg == "--seconds") && i + 1 < argc) {
        const std::string value = argv[++i];
        if (arg == "--config")
          path = value;
        else {
          std::size_t used = 0;
          seconds = std::stod(value, &used);
          if (used != value.size())
            throw std::invalid_argument("Invalid --seconds");
        }
      } else
        throw std::invalid_argument("Unknown/incomplete option: " + arg);
    }
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 86400)
      throw std::invalid_argument("--seconds must be in (0,86400]");
    const auto y = YAML::LoadFile(path);
    aim::LinkConfig c;
    c.system_id = y["system_id"].as<int>();
    c.component_id = y["component_id"].as<int>();
    c.own_system_id = y["own_system_id"].as<int>();
    c.own_component_id = y["own_component_id"].as<int>();
    c.px4_odometry_contract_confirmed =
        y["px4_odometry_contract_confirmed"].as<bool>();
    c.max_pose_age_s = y["max_pose_age_s"].as<double>();
    c.estimator_timeout_s = y["estimator_timeout_s"].as<double>();
    c.heartbeat_timeout_s = y["heartbeat_timeout_s"].as<double>();
    c.max_position_std_m = y["max_position_std_m"].as<double>();
    c.max_angle_std_rad = y["max_angle_std_rad"].as<double>();
    const bool timesync = y["timesync_enabled"].as<bool>();
    aim::MavlinkLink link(c);
    aim::SerialPort port(y["device"].as<std::string>(), y["baud"].as<int>());
    aim::PoseHistory history;
    std::optional<aim::PoseSample> latest;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    const double start = aim::monotonicNow();
    double next_probe = start, next_log = start;
    while (!stop_requested && aim::monotonicNow() - start < seconds) {
      double now = aim::monotonicNow();
      if (timesync && now >= next_probe) {
        port.write(link.timesyncProbe(now));
        next_probe = now + .2;
      }
      const auto bytes = port.read(20);
      now = aim::monotonicNow();
      for (const auto &pose : link.feed(bytes.data(), bytes.size(), now)) {
        history.push(pose);
        if (pose.valid)
          latest = pose;
        else
          latest.reset();
      }
      const auto d = link.diagnostics(now);
      if (latest && (!d.clock_ready || !d.estimator_ready || d.reset_required ||
                     now - latest->capture_s > c.max_pose_age_s)) {
        latest.reset();
        history.clear();
      }
      if (now >= next_log) {
        const auto state = link.flightState(now);
        std::cout << aim::Json{{"schema_version", 1},
                               {"time_domain", "host_monotonic"},
                               {"now", now},
                               {"link", d.json()},
                               {"flight_state", state.json()},
                               {"pose",
                                latest ? latest->json() : aim::Json(nullptr)},
                               {"history_size", history.size()},
                               {"flight_commands_enabled", false}}
                         .dump()
                  << '\n';
        next_log = now + .1;
      }
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << aim::Json{{"valid", false},
                           {"error", e.what()},
                           {"flight_commands_enabled", false}}
                     .dump()
              << '\n';
    return 1;
  }
}
