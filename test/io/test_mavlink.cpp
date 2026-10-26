// 真实 MAVLink 编解码 + Linux 伪终端；不访问飞控硬件，不发送飞行指令。
#include "support/assertions.hpp"
#include "support/runner.hpp"
#include "demo_aim/mavlink_link.hpp"
#include "demo_aim/serial_port.hpp"
#include <cmath>
#include <common/mavlink.h>
#include <iostream>
#include <limits>
#include <pty.h>
#include <stdexcept>
#include <unistd.h>
namespace {
using test_support::check;
void near(const cv::Vec3d &a, const cv::Vec3d &b) {
  check(cv::norm(a - b) < 1e-6, "vector mismatch");
}
std::vector<std::uint8_t> bytes(const mavlink_message_t &m) {
  std::vector<std::uint8_t> b(MAVLINK_MAX_PACKET_LEN);
  b.resize(mavlink_msg_to_send_buffer(b.data(), &m));
  return b;
}
mavlink_message_t decode(const std::vector<std::uint8_t> &b) {
  mavlink_message_t rx{}, m{};
  mavlink_status_t s{}, out{};
  bool good = false;
  for (auto c : b)
    if (mavlink_frame_char_buffer(&rx, &s, c, &m, &out) == MAVLINK_FRAMING_OK)
      good = true;
  check(good, "decode test frame");
  return m;
}
struct Fixture {
  aim::LinkConfig config;
  aim::MavlinkLink link;
  mavlink_status_t tx{};
  explicit Fixture(bool confirmed = true, double estimator_timeout_s = .5)
      : config(makeConfig(confirmed, estimator_timeout_s)), link(config) {}
  static aim::LinkConfig makeConfig(bool v, double estimator_timeout_s) {
    aim::LinkConfig c;
    c.px4_odometry_contract_confirmed = v;
    c.estimator_timeout_s = estimator_timeout_s;
    return c;
  }
  std::vector<aim::PoseSample> send(const mavlink_message_t &m, double now) {
    const auto b = bytes(m);
    return link.feed(b.data(), b.size(), now);
  }
  mavlink_message_t syncResponse(const std::vector<std::uint8_t> &probe,
                                 double remote, int sys = 1) {
    mavlink_timesync_t s{};
    const auto m = decode(probe);
    mavlink_msg_timesync_decode(&m, &s);
    s.tc1 = std::llround(remote * 1e9);
    s.target_system = 42;
    s.target_component = 191;
    mavlink_message_t r{};
    mavlink_msg_timesync_encode_status(sys, 1, &tx, &r, &s);
    return r;
  }
  void sync() {
    for (int n = 0; n < 5; ++n) {
      const double t = 10 + n * .1;
      send(syncResponse(link.timesyncProbe(t), t + .001 - 9), t + .002);
    }
    check(link.diagnostics(10.403).clock_ready, "time sync did not converge");
  }
  mavlink_message_t heartbeat(int sys = 1,
                              int autopilot = MAV_AUTOPILOT_PX4) {
    mavlink_heartbeat_t h{};
    h.autopilot = autopilot;
    h.type = MAV_TYPE_QUADROTOR;
    mavlink_message_t m{};
    mavlink_msg_heartbeat_encode_status(sys, 1, &tx, &m, &h);
    return m;
  }
  void localPosition(double host, double received, float x = 1) {
    mavlink_local_position_ned_t p{};
    p.time_boot_ms = std::llround((host - 9) * 1000);
    p.x = x;
    mavlink_message_t m{};
    mavlink_msg_local_position_ned_encode_status(1, 1, &tx, &m, &p);
    send(m, received);
  }
  void health(double now = 10.41, std::uint16_t flags = 47) {
    mavlink_estimator_status_t e{};
    e.time_usec = std::llround((now - 9) * 1e6);
    e.flags = flags;
    mavlink_message_t m{};
    mavlink_msg_estimator_status_encode_status(1, 1, &tx, &m, &e);
    send(m, now + .001);
  }
  void ready() {
    sync();
    send(heartbeat(), 10.405);
    health();
  }
  mavlink_odometry_t odom(double host = 10.45) {
    mavlink_odometry_t o{};
    o.time_usec = std::llround((host - 9) * 1e6);
    o.frame_id = MAV_FRAME_LOCAL_NED;
    o.child_frame_id = MAV_FRAME_LOCAL_NED;
    o.estimator_type = MAV_ESTIMATOR_TYPE_AUTOPILOT;
    o.q[0] = 1;
    o.x = 1;
    o.y = 2;
    o.z = -3;
    o.vx = 4;
    o.vy = 5;
    o.vz = -6;
    for (int i : {0, 6, 11, 15, 18, 20})
      o.pose_covariance[i] = .001;
    return o;
  }
  mavlink_message_t message(const mavlink_odometry_t &o, int sys = 1) {
    mavlink_message_t m{};
    mavlink_msg_odometry_encode_status(sys, 1, &tx, &m, &o);
    return m;
  }
  aim::PoseSample pose(const mavlink_odometry_t &o, double now = 10.455) {
    const auto events = send(message(o), now);
    check(!events.empty(), "missing pose event");
    return events.front();
  }
};
struct Pty {
  int master = -1, slave = -1;
  char name[256]{};
  Pty() {
    if (openpty(&master, &slave, name, nullptr, nullptr) < 0)
      throw std::runtime_error("openpty failed");
  }
  ~Pty() {
    if (master >= 0)
      close(master);
    if (slave >= 0)
      close(slave);
  }
};
} // namespace
int main() {
  test_support::Runner run;
  run("TIMESYNC is the only emitted packet", [] {
    Fixture f;
    const auto m = decode(f.link.timesyncProbe(10));
    check(m.msgid == MAVLINK_MSG_ID_TIMESYNC && m.sysid == 42 &&
              m.compid == 191,
          "probe identity/type");
    check(f.link.timesyncProbe(10.01).empty(), "bounded outstanding requests");
  });
  run("synchronization rejects foreign and slow responses", [] {
    Fixture f;
    auto probe = f.link.timesyncProbe(10);
    f.send(f.syncResponse(probe, 1.001, 2), 10.002);
    check(!f.link.diagnostics(10.002).clock_ready, "foreign sync");
    probe = f.link.timesyncProbe(10.2);
    f.send(f.syncResponse(probe, 1.21), 10.25);
    check(!f.link.diagnostics(10.25).clock_ready, "high RTT");
    check(f.link.diagnostics(10.25).timesync_bad_rtt == 1,
          "slow sync response not diagnosed");
  });
  run("TIMESYNC reports wrong targets without trusting them", [] {
    Fixture f;
    const auto probe = f.link.timesyncProbe(10);
    auto reply = f.syncResponse(probe, 1.001);
    mavlink_timesync_t response{};
    mavlink_msg_timesync_decode(&reply, &response);
    response.target_system = 64;
    response.target_component = 131;
    mavlink_msg_timesync_encode_status(1, 1, &f.tx, &reply, &response);
    f.send(reply, 10.002);
    auto d = f.link.diagnostics(10.003);
    check(d.timesync_responses == 1 && d.timesync_wrong_target == 1 &&
              d.timesync_accepted == 0 && !d.clock_ready &&
              d.timesync_target_system == 64 &&
              d.timesync_target_component == 131,
          "wrong target response trusted or hidden");
    // 被拒绝的回包不能消耗 token；随后正确的响应仍可匹配。
    f.send(f.syncResponse(probe, 1.002), 10.004);
    d = f.link.diagnostics(10.005);
    check(d.timesync_accepted == 1 && d.clock_sync_samples == 1 &&
              d.timesync_target_system == 42 &&
              d.timesync_target_component == 191,
          "corrected response did not recover");
    f.send(f.syncResponse(probe, 1.003), 10.006);
    check(f.link.diagnostics(10.007).timesync_unmatched == 1,
          "duplicate response was not diagnosed");
  });
  run("legacy TIMESYNC responses without target extensions still synchronize", [] {
    Fixture f;
    for (int n = 0; n < 5; ++n) {
      const double t = 10 + n * .1;
      const auto probe = f.link.timesyncProbe(t);
      auto reply = f.syncResponse(probe, t + .001 - 9);
      mavlink_timesync_t response{};
      mavlink_msg_timesync_decode(&reply, &response);
      response.target_system = response.target_component = 0;
      mavlink_msg_timesync_encode_status(1, 1, &f.tx, &reply, &response);
      check(reply.len <= MAVLINK_MSG_ID_TIMESYNC_MIN_LEN,
            "legacy test unexpectedly includes target extensions");
      f.send(reply, t + .002);
    }
    const auto d = f.link.diagnostics(10.403);
    check(d.clock_ready && d.timesync_accepted == 5 &&
              d.timesync_wrong_target == 0,
          "legacy responses no longer synchronize");
  });
  run("fragmented MAVLink packets and concatenated telemetry", [] {
    Fixture f;
    f.ready();
    auto b = bytes(f.message(f.odom()));
    std::vector<aim::PoseSample> events;
    for (auto c : b) {
      auto out = f.link.feed(&c, 1, 10.455);
      events.insert(events.end(), out.begin(), out.end());
    }
    check(events.size() == 1 && events[0].valid, "bytewise decode");
    b = bytes(f.heartbeat());
    auto tail = bytes(f.message(f.odom(10.5)));
    b.insert(b.end(), tail.begin(), tail.end());
    events = f.link.feed(b.data(), b.size(), 10.505);
    check(events.size() == 1 && events[0].valid, "concatenated decode");
  });
  run("NED and FRD pose conversion uses PX4 quaternion contract", [] {
    Fixture f;
    f.ready();
    const auto p = f.pose(f.odom());
    check(p.valid && p.odom_frame == "odom_enu", "pose validity");
    near(p.position_m, {2, 1, 3});
    near(*p.velocity_odom_mps, {5, 4, 6});
    near(p.odomFromBody().rotation * cv::Vec3d{1, 0, 0}, {0, 1, 0});
    check(std::abs(p.capture_s - 10.45) < 1e-8, "sample clock mapping");
  });
  run("local FRD arbitrary heading does not pretend north", [] {
    Fixture f;
    f.ready();
    auto o = f.odom();
    o.frame_id = MAV_FRAME_LOCAL_FRD;
    o.child_frame_id = MAV_FRAME_LOCAL_FRD;
    const auto p = f.pose(o);
    check(p.valid && p.odom_frame == "odom_flu", "frame name");
    near(p.position_m, {1, -2, 3});
  });
  run("body-frame velocity rotates into world", [] {
    Fixture f;
    f.ready();
    auto o = f.odom();
    o.child_frame_id = MAV_FRAME_BODY_FRD;
    o.q[0] = std::sqrt(.5);
    o.q[3] = std::sqrt(.5);
    o.vx = 1;
    o.vy = o.vz = 0;
    const auto p = f.pose(o);
    check(p.valid, "rotated velocity validity");
    near(*p.velocity_odom_mps, {1, 0, 0});
  });
  run("corrupt CRC rejected and parser recovers", [] {
    Fixture f;
    f.ready();
    auto b = bytes(f.message(f.odom()));
    b[20] ^= 0x40;
    check(f.link.feed(b.data(), b.size(), 10.455).empty(), "CRC acceptance");
    check(f.pose(f.odom(10.5), 10.505).valid, "parser recovery");
    check(f.link.diagnostics(10.505).bad_frames > 0, "CRC diagnostics");
  });
  run("foreign IDs cannot create pose", [] {
    Fixture f;
    f.ready();
    check(f.send(f.message(f.odom(), 2), 10.455).empty(), "foreign odom");
    check(f.link.diagnostics(10.455).foreign_packets == 1, "foreign counter");
  });
  run("unconfirmed firmware contract never yields navigation pose", [] {
    Fixture f(false);
    f.ready();
    const auto p = f.pose(f.odom());
    check(!p.valid && p.reason == "px4_odometry_contract_unconfirmed",
          "unknown profile");
  });
  run("no clock synchronization no valid pose", [] {
    Fixture f;
    f.send(f.heartbeat(), 10.405);
    f.health();
    check(!f.pose(f.odom()).valid, "unsynced pose");
  });
  run("estimator flags are independent of position arrival", [] {
    Fixture f;
    f.ready();
    f.health(10.43, 1);
    check(!f.pose(f.odom()).valid, "bad health");
  });
  run("unknown covariance and invalid quality rejected", [] {
    Fixture f;
    f.ready();
    auto o = f.odom();
    o.pose_covariance[0] = NAN;
    check(!f.pose(o).valid, "unknown covariance");
    o = f.odom(10.5);
    o.quality = -1;
    check(!f.pose(o, 10.505).valid, "failed quality");
  });
  run("PX4 diagonal covariance accepts missing cross terms only", [] {
    Fixture f;
    f.ready();
    auto px4 = [&](double host) {
      auto o = f.odom(host);
      for (auto &value : o.pose_covariance)
        value = NAN;
      for (int i : {0, 6, 11, 15, 18, 20})
        o.pose_covariance[i] = .001;
      return o;
    };
    check(f.pose(px4(10.45)).valid, "PX4 covariance layout rejected");
    auto o = px4(10.5);
    o.pose_covariance[0] = .26;
    check(f.pose(o, 10.505).reason == "pose_covariance_unknown_or_large",
          "missing cross terms bypassed accuracy limit");
    o = px4(10.55);
    o.pose_covariance[6] = NAN;
    check(f.pose(o, 10.555).reason == "pose_covariance_unknown_or_large",
          "unknown diagonal accepted");
    o = px4(10.6);
    o.pose_covariance[1] = 0;
    check(f.pose(o, 10.605).reason == "pose_covariance_invalid",
          "partially missing covariance accepted");
    o = px4(10.65);
    o.pose_covariance[1] = std::numeric_limits<float>::infinity();
    check(f.pose(o, 10.655).reason == "pose_covariance_invalid",
          "infinite cross term accepted");
    o = px4(10.7);
    o.pose_covariance[11] = -.001;
    check(f.pose(o, 10.705).reason == "pose_covariance_unknown_or_large",
          "negative diagonal variance accepted");
  });
  run("one Hz bench health timeout is explicit and still bounded", [] {
    Fixture strict;
    strict.ready();
    check(!strict.link.diagnostics(10.92).estimator_ready,
          "operational health timeout relaxed");
    Fixture bench(true, 1.5);
    bench.ready();
    check(bench.link.diagnostics(10.92).estimator_ready,
          "bench profile rejected one Hz estimator cadence");
    check(bench.pose(bench.odom(10.9), 10.925).valid,
          "current pose rejected between one Hz status messages");
    bench.send(bench.heartbeat(), 11.90);
    check(!bench.link.diagnostics(11.92).estimator_ready &&
              bench.link.diagnostics(11.92).clock_ready,
          "bench timeout retained stale estimator health");
  });
  run("bad covariance cross terms and quaternion rejected", [] {
    Fixture f;
    f.ready();
    auto o = f.odom();
    o.pose_covariance[1] = 10;
    check(!f.pose(o).valid, "non PSD covariance");
    o = f.odom(10.5);
    o.q[0] = 2;
    check(!f.pose(o, 10.505).valid, "non unit quaternion");
  });
  run("unsupported parent and velocity frames rejected", [] {
    Fixture f;
    f.ready();
    auto o = f.odom();
    o.frame_id = MAV_FRAME_GLOBAL;
    check(!f.pose(o).valid, "global frame");
    o = f.odom(10.5);
    o.child_frame_id = MAV_FRAME_BODY_NED;
    f.pose(o, 10.505);
    f.health(10.52);
    o = f.odom(10.55);
    o.child_frame_id = MAV_FRAME_BODY_NED;
    check(f.pose(o, 10.555).reason == "unsupported_velocity_frame",
          "unsupported twist frame");
  });
  run("stale pose cannot be refreshed by receive time", [] {
    Fixture f;
    f.ready();
    auto o = f.odom(10.1);
    check(!f.pose(o, 10.455).valid, "stale remote sample");
  });
  run("cached local position expires by remote sample age", [] {
    Fixture f;
    f.ready();
    f.health(10.60);
    // Arrives 160 ms after sampling: only 40 ms of the 200 ms budget remains.
    f.localPosition(10.45, 10.61);
    const auto current = f.link.flightState(10.64);
    check(current.local_position && current.local_position->value.xy_valid &&
              current.local_position->value.z_valid &&
              current.local_position->value.velocity_valid,
          "current delayed position rejected");
    const auto stale = f.link.flightState(10.66);
    check(stale.local_position && !stale.local_position->value.xy_valid &&
              !stale.local_position->value.z_valid &&
              !stale.local_position->value.velocity_valid,
          "receive time extended remote sample validity");
    check(f.link.diagnostics(10.66).clock_ready &&
              f.link.diagnostics(10.66).estimator_ready,
          "position expiry altered clock/estimator readiness");
  });
  run("local position requires a current PX4 heartbeat", [] {
    auto invalid = [](const aim::FlightStateSnapshot &state) {
      check(state.local_position && !state.local_position->value.xy_valid &&
                !state.local_position->value.z_valid &&
                !state.local_position->value.velocity_valid,
            "position valid without a current PX4 heartbeat");
    };
    Fixture missing;
    missing.sync();
    missing.health();
    missing.localPosition(10.42, 10.425);
    invalid(missing.link.flightState(10.43));
    check(!missing.link.diagnostics(10.43).estimator_ready,
          "missing heartbeat became estimator ready");

    Fixture foreign_autopilot;
    foreign_autopilot.sync();
    foreign_autopilot.send(
        foreign_autopilot.heartbeat(1, MAV_AUTOPILOT_ARDUPILOTMEGA), 10.405);
    foreign_autopilot.health();
    foreign_autopilot.localPosition(10.42, 10.425);
    invalid(foreign_autopilot.link.flightState(10.43));

    Fixture expired;
    expired.ready();
    // Refresh the estimator and position without refreshing the heartbeat.
    expired.health(11.38);
    expired.localPosition(11.39, 11.395);
    check(expired.link.flightState(11.40).local_position->value.xy_valid,
          "fresh PX4 heartbeat rejected");
    invalid(expired.link.flightState(11.41));
    check(!expired.link.diagnostics(11.41).estimator_ready,
          "expired heartbeat became estimator ready");

    Fixture changed;
    changed.ready();
    changed.localPosition(10.44, 10.445);
    check(changed.link.flightState(10.45).local_position->value.xy_valid,
          "initial PX4 position rejected");
    changed.send(changed.heartbeat(1, MAV_AUTOPILOT_ARDUPILOTMEGA), 10.46);
    invalid(changed.link.flightState(10.47));
  });
  run("EKF reset changes epoch and requires new health", [] {
    Fixture f;
    f.ready();
    const auto a = f.pose(f.odom());
    auto o = f.odom(10.5);
    o.reset_counter = 1;
    const auto b = f.pose(o, 10.505);
    check(!b.valid && b.epoch > a.epoch, "epoch update");
    f.health(10.52);
    o = f.odom(10.55);
    o.reset_counter = 1;
    const auto c = f.pose(o, 10.555);
    check(c.valid && c.epoch == b.epoch, "new epoch revalidates");
  });
  run("new estimator health cannot revive a previous epoch position", [] {
    Fixture f;
    f.ready();
    f.localPosition(10.44, 10.445, 123);
    const auto before = f.pose(f.odom());
    check(before.valid, "initial pose rejected");
    auto o = f.odom(10.5);
    o.reset_counter = 1;
    const auto reset = f.pose(o, 10.505);
    check(reset.epoch > before.epoch, "missing EKF epoch change");
    f.health(10.52);
    const auto old = f.link.flightState(10.525);
    check(old.heartbeat && old.local_position &&
              old.local_position->value.position_ned_m[0] == 123 &&
              !old.local_position->value.xy_valid &&
              !old.local_position->value.z_valid &&
              !old.local_position->value.velocity_valid,
          "new estimator health revived an old epoch position");
    check(f.link.diagnostics(10.525).estimator_ready,
          "EKF reset discarded the current heartbeat");
    // A rejected duplicate must not assign the new epoch to the old cache.
    f.localPosition(10.44, 10.526, 999);
    const auto duplicate = f.link.flightState(10.527);
    check(duplicate.local_position &&
              duplicate.local_position->value.position_ned_m[0] == 123 &&
              !duplicate.local_position->value.xy_valid,
          "duplicate packet relabeled the previous epoch cache");
    f.localPosition(10.53, 10.535, 456);
    const auto fresh = f.link.flightState(10.54);
    check(fresh.local_position && fresh.local_position->value.xy_valid &&
              fresh.local_position->value.z_valid &&
              fresh.local_position->value.velocity_valid &&
              fresh.local_position->value.position_ned_m[0] == 456,
          "new epoch local position failed to recover");
  });
  run("boot reversal latches reset until explicit new session", [] {
    Fixture f;
    f.ready();
    f.pose(f.odom());
    auto o = f.odom();
    o.time_usec = 10;
    const auto p = f.pose(o, 10.5);
    check(!p.valid && f.link.diagnostics(10.5).reset_required,
          "clock reversal");
    check(f.link.timesyncProbe(10.6).empty(), "no continue after reboot");
    const auto epoch = p.epoch;
    f.link.resetSession();
    check(!f.link.diagnostics(11).clock_ready &&
              f.link.diagnostics(11).epoch > epoch,
          "new session");
  });
  run("independent parser instances and synchronization expiry", [] {
    Fixture a, b;
    a.ready();
    check(!b.link.diagnostics(10.455).clock_ready, "global parser state");
    check(!a.link.diagnostics(13).clock_ready, "sync timeout");
  });
  run("bad estimator status immediately invalidates history", [] {
    Fixture f;
    f.ready();
    aim::PoseHistory history;
    history.push(f.pose(f.odom()));
    mavlink_estimator_status_t e{};
    e.time_usec = 1470000;
    e.flags = 1;
    mavlink_message_t m{};
    mavlink_msg_estimator_status_encode_status(1, 1, &f.tx, &m, &e);
    const auto events = f.send(m, 10.475);
    check(!events.empty() && !events.front().valid,
          "missing invalidation event");
    for (const auto &event : events)
      history.push(event);
    check(history.size() == 0 && !f.link.diagnostics(10.475).estimator_ready,
          "old pose survives failure");
  });
  run("clock offset jump latches a new session requirement", [] {
    Fixture f;
    f.ready();
    const auto probe = f.link.timesyncProbe(10.6);
    f.send(f.syncResponse(probe, .001), 10.602);
    const auto d = f.link.diagnostics(10.603);
    check(d.reset_required && !d.clock_ready && d.epoch > 1,
          "clock jump ignored");
  });
  run("serial pty partial packets and outbound bytes", [] {
    Pty p;
    aim::SerialPort port(p.name);
    Fixture f;
    const auto b = bytes(f.heartbeat());
    const std::size_t split = b.size() / 2;
    check(::write(p.master, b.data(), split) == static_cast<ssize_t>(split),
          "master write");
    auto data = port.read(100);
    check(data.size() == split &&
              f.link.feed(data.data(), data.size(), 1).empty(),
          "first fragment");
    check(::write(p.master, b.data() + split, b.size() - split) ==
              static_cast<ssize_t>(b.size() - split),
          "second write");
    data = port.read(100);
    f.link.feed(data.data(), data.size(), 1.1);
    check(bool(f.link.flightState(1.1).heartbeat), "pty heartbeat");
    const auto probe = f.link.timesyncProbe(2);
    port.write(probe);
    std::uint8_t rx[512];
    const auto n = ::read(p.master, rx, sizeof(rx));
    check(n == static_cast<ssize_t>(probe.size()) &&
              std::equal(probe.begin(), probe.end(), rx),
          "outbound roundtrip");
    check(port.read(1).empty(), "read timeout");
  });
  run("serial disconnect and placeholder are explicit errors", [] {
    bool caught = false;
    try {
      aim::SerialPort p("/dev/ttyS?");
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    check(caught, "placeholder accepted");
    Pty p;
    aim::SerialPort port(p.name);
    close(p.master);
    p.master = -1;
    caught = false;
    try {
      port.read(10);
    } catch (const std::runtime_error &) {
      caught = true;
    }
    check(caught, "disconnect ignored");
  });
  return run.finish();
}
