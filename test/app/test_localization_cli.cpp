// Exercise the actual CLI through an isolated Linux PTY; no flight hardware.
#include "support/assertions.hpp"
#include "support/runner.hpp"
#include "demo_aim/observation.hpp"
#include <chrono>
#include <cmath>
#include <common/mavlink.h>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <pty.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char **environ;

namespace {
using test_support::check;
using Clock = std::chrono::steady_clock;
enum class Traffic { Silent, Foreign, Heartbeat, Pose, ExpiredPose, Interrupt };

struct Bench {
  int master{-1}, slave{-1};
  pid_t child{-1};
  std::filesystem::path directory;
  Bench() {
    char pattern[] = "/tmp/aim-localization-cli-XXXXXX";
    const char *created = mkdtemp(pattern);
    check(created, "create CLI test directory");
    directory = created;
  }
  ~Bench() {
    if (child > 0) {
      kill(child, SIGKILL);
      waitpid(child, nullptr, 0);
    }
    if (master >= 0)
      close(master);
    if (slave >= 0)
      close(slave);
    std::error_code error;
    std::filesystem::remove_all(directory, error);
  }
};

struct Result {
  int code{};
  std::vector<aim::Json> samples;
  aim::Json error;
};

Result runCli(const std::string &executable, Traffic traffic,
              bool require_pose = false) {
  Bench bench;
  char device[256]{};
  check(openpty(&bench.master, &bench.slave, device, nullptr, nullptr) == 0,
        "create CLI test PTY");
  check(fcntl(bench.master, F_SETFD, FD_CLOEXEC) == 0 &&
            fcntl(bench.slave, F_SETFD, FD_CLOEXEC) == 0 &&
            fcntl(bench.master, F_SETFL, O_NONBLOCK) == 0,
        "configure CLI test PTY");
  const bool pose_traffic =
      traffic == Traffic::Pose || traffic == Traffic::ExpiredPose;
  const auto config = bench.directory / "link.yaml";
  const auto output = bench.directory / "stdout.jsonl";
  const auto errors = bench.directory / "stderr.jsonl";
  {
    std::ofstream file(config);
    file << "device: " << device
         << "\nbaud: 921600\nsystem_id: 1\ncomponent_id: 1\n"
            "own_system_id: 42\nown_component_id: 191\n"
            "px4_odometry_contract_confirmed: true\n"
            "timesync_enabled: "
         << (pose_traffic ? "true" : "false")
         << "\nmax_pose_age_s: 0.2\nestimator_timeout_s: 0.5\n"
            "heartbeat_timeout_s: 1.0\nmax_position_std_m: 0.5\n"
            "max_angle_std_rad: 0.35\n";
    check(bool(file), "write CLI test configuration");
  }
  std::vector<std::string> arguments{
      executable, "--config", config.string(), "--seconds",
      pose_traffic ? "2.5" : "0.4"};
  if (require_pose)
    arguments.push_back("--require-pose");
  std::vector<char *> argv;
  for (auto &argument : arguments)
    argv.push_back(argument.data());
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  check(posix_spawn_file_actions_init(&actions) == 0, "spawn actions");
  int error = posix_spawn_file_actions_addopen(
      &actions, STDOUT_FILENO, output.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (!error)
    error = posix_spawn_file_actions_addopen(
        &actions, STDERR_FILENO, errors.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (!error)
    error = posix_spawn(&bench.child, executable.c_str(), &actions, nullptr,
                        argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  check(error == 0, "start localization CLI");

  mavlink_status_t tx{}, parser{}, parser_output{};
  mavlink_message_t rx{}, decoded{};
  auto send = [&](const mavlink_message_t &message) {
    std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(bytes, &message);
    check(write(bench.master, bytes, length) == length, "send test telemetry");
  };
  const auto started = Clock::now();
  const double remote_offset = aim::monotonicNow() - 1.;
  auto next_send = started;
  bool interrupted = false;
  int status = 0;
  while (true) {
    const auto waited = waitpid(bench.child, &status, WNOHANG);
    check(waited >= 0, "wait for localization CLI");
    if (waited == bench.child) {
      bench.child = -1;
      break;
    }
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - started).count();
    check(elapsed < 8., "localization CLI timed out");
    // Repeated telemetry removes any dependency on process startup timing.
    if (now >= next_send && traffic != Traffic::Silent &&
        traffic != Traffic::Interrupt) {
      mavlink_heartbeat_t heartbeat{};
      heartbeat.autopilot = MAV_AUTOPILOT_PX4;
      heartbeat.type = MAV_TYPE_QUADROTOR;
      mavlink_message_t message{};
      mavlink_msg_heartbeat_encode_status(
          traffic == Traffic::Foreign ? 2 : 1, 1, &tx, &message, &heartbeat);
      send(message);
      if (pose_traffic) {
        const auto remote_us = static_cast<std::uint64_t>(
            std::llround((aim::monotonicNow() - remote_offset - .01) * 1e6));
        mavlink_estimator_status_t estimator{};
        estimator.time_usec = remote_us;
        estimator.flags = ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_VELOCITY_VERT | ESTIMATOR_POS_HORIZ_REL |
                          ESTIMATOR_POS_VERT_ABS;
        mavlink_msg_estimator_status_encode_status(1, 1, &tx, &message,
                                                    &estimator);
        send(message);
        if (traffic != Traffic::ExpiredPose || elapsed < 1.7) {
          mavlink_odometry_t odometry{};
          odometry.time_usec = remote_us;
          odometry.frame_id = MAV_FRAME_LOCAL_NED;
          odometry.child_frame_id = MAV_FRAME_LOCAL_NED;
          odometry.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;
          odometry.q[0] = 1;
          for (const int i : {0, 6, 11, 15, 18, 20})
            odometry.pose_covariance[i] = .001;
          mavlink_msg_odometry_encode_status(1, 1, &tx, &message, &odometry);
          send(message);
        }
      }
      next_send = now + std::chrono::milliseconds(15);
    }
    if (pose_traffic) {
      std::uint8_t bytes[4096];
      const auto count = read(bench.master, bytes, sizeof(bytes));
      for (ssize_t i = 0; i < count; ++i) {
        if (mavlink_frame_char_buffer(&rx, &parser, bytes[i], &decoded,
                                     &parser_output) != MAVLINK_FRAMING_OK ||
            decoded.msgid != MAVLINK_MSG_ID_TIMESYNC)
          continue;
        mavlink_timesync_t sync{};
        mavlink_msg_timesync_decode(&decoded, &sync);
        sync.tc1 = std::llround((aim::monotonicNow() - remote_offset) * 1e9);
        sync.target_system = 42;
        sync.target_component = 191;
        mavlink_message_t response{};
        mavlink_msg_timesync_encode_status(1, 1, &tx, &response, &sync);
        send(response);
      }
    }
    if (traffic == Traffic::Interrupt && !interrupted) {
      // First diagnostic output confirms the child installed its signal handler.
      if (std::filesystem::file_size(output) > 0) {
        check(kill(bench.child, SIGINT) == 0, "interrupt localization CLI");
        interrupted = true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check(WIFEXITED(status), "localization CLI terminated unexpectedly");
  Result result;
  result.code = WEXITSTATUS(status);
  for (const auto &path : {output, errors}) {
    std::ifstream file(path);
    std::string line;
    while (std::getline(file, line)) {
      if (path == output)
        result.samples.push_back(aim::Json::parse(line));
      else
        result.error = aim::Json::parse(line);
    }
  }
  check(!result.samples.empty(), "missing diagnostic output");
  return result;
}
} // namespace

int main(int argc, char **argv) {
  if (argc != 2)
    return 1;
  const std::string executable = argv[1];
  test_support::Runner run;
  run("silent serial stream fails", [&] {
    const auto r = runCli(executable, Traffic::Silent);
    check(r.code == 2 && r.error["reason"] == "no_configured_source_packets",
          "silent stream reported success");
  });
  run("foreign telemetry cannot satisfy configured source", [&] {
    const auto r = runCli(executable, Traffic::Foreign);
    const auto &link = r.samples.back()["link"];
    check(link["packets"].get<int>() > 0 &&
              link["packets"] == link["foreign_packets"],
          "foreign telemetry was not exercised");
    check(r.code == 2, "foreign-only stream reported success");
  });
  run("source telemetry without pose is valid diagnostic run", [&] {
    const auto r = runCli(executable, Traffic::Heartbeat);
    check(r.code == 0 && r.samples.back()["pose"].is_null(),
          "diagnostic run incorrectly requires a pose");
  });
  run("explicit pose acceptance fails without pose", [&] {
    const auto r = runCli(executable, Traffic::Heartbeat, true);
    check(r.code == 3 && r.error["reason"] == "required_pose_unavailable",
          "pose requirement was ignored");
  });
  run("explicit pose acceptance succeeds for current pose", [&] {
    const auto r = runCli(executable, Traffic::Pose, true);
    check(r.code == 0 && !r.samples.back()["pose"].is_null() &&
              r.samples.back()["pose"]["valid"] == true,
          "current synchronized pose was rejected");
  });
  run("earlier valid pose cannot satisfy final acceptance", [&] {
    const auto r = runCli(executable, Traffic::ExpiredPose, true);
    bool saw_valid_pose = false;
    for (const auto &sample : r.samples)
      saw_valid_pose = saw_valid_pose || !sample["pose"].is_null();
    check(saw_valid_pose, "test never established an earlier valid pose");
    check(r.code == 3 && r.samples.back()["pose"].is_null(),
          "expired pose satisfied final acceptance");
  });
  run("SIGINT reports interruption", [&] {
    const auto r = runCli(executable, Traffic::Interrupt);
    check(r.code == 130, "SIGINT reported success or acceptance failure");
  });
  return run.finish();
}
