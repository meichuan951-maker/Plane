#include "demo_aim/mavlink_link.hpp"
#include <algorithm>
#include <cmath>
#include <common/mavlink.h>
#include <limits>
#include <stdexcept>
namespace aim {
void LinkConfig::validate() const {
  for (int id : {system_id, component_id, own_system_id, own_component_id})
    if (id < 1 || id > 255)
      throw std::invalid_argument("MAVLink IDs must be 1..255");
  if (system_id == own_system_id && component_id == own_component_id)
    throw std::invalid_argument("Duplicate MAVLink identity");
  for (double v : {max_pose_age_s, estimator_timeout_s, heartbeat_timeout_s,
                   max_position_std_m, max_angle_std_rad})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Invalid link limits");
}
Json LinkDiagnostics::json() const {
  return {{"packets", packets},
          {"bad_frames", bad_frames},
          {"foreign_packets", foreign_packets},
          {"rejected_odometry", rejected_odometry},
          {"timesync_responses", timesync_responses},
          {"timesync_accepted", timesync_accepted},
          {"timesync_unmatched", timesync_unmatched},
          {"timesync_wrong_target", timesync_wrong_target},
          {"timesync_bad_rtt", timesync_bad_rtt},
          {"clock_sync_samples", clock_sync_samples},
          {"timesync_target_system",
           timesync_target_system ? Json(*timesync_target_system) : Json(nullptr)},
          {"timesync_target_component",
           timesync_target_component ? Json(*timesync_target_component)
                                     : Json(nullptr)},
          {"epoch", epoch},
          {"clock_ready", clock_ready},
          {"estimator_ready", estimator_ready},
          {"last_message_id", last_message_id},
          {"odometry_frame_id",
           odometry_frame_id ? Json(*odometry_frame_id) : Json(nullptr)},
          {"velocity_frame_id",
           velocity_frame_id ? Json(*velocity_frame_id) : Json(nullptr)},
          {"reset_required", reset_required},
          {"reason", reason},
          {"clock_error_s", clock_error_s}};
}
struct MavlinkLink::Impl {
  LinkConfig config;
  FlightStateCache flight;
  // 每个对象独立解析器状态；不用库的进程全局 channel 状态，避免多串口互相干扰。
  mavlink_message_t rx{};
  mavlink_status_t parse{}, tx{};
  LinkDiagnostics diag;
  std::optional<mavlink_estimator_status_t> estimator;
  std::optional<std::uint64_t> last_odom_us;
  std::optional<std::uint8_t> reset_counter;
  std::optional<int> parent_frame, velocity_frame;
  // LOCAL_POSITION_NED 没有 reset_counter，只能记录接收时已知的 epoch。
  // EKF/父坐标重置后必须收到新位置，不能用新健康状态重新激活旧缓存；
  // 这不能识别所有在网络中迟到的重置前报文。
  std::uint64_t local_position_epoch{};
  double estimator_received{}, last_feed{-1};
  int autopilot{-1}, sync_samples{};
  double offset{}, sync_error{}, last_sync{-1};
  std::int64_t pending_token{};
  explicit Impl(LinkConfig c)
      : config(c), flight(c.system_id, c.component_id, c.heartbeat_timeout_s,
                          c.max_pose_age_s) {}
  bool clockReady(double now) const {
    return sync_samples >= 5 && last_sync >= 0 && now >= last_sync &&
           now - last_sync <= 2. && sync_error <= .01 && !diag.reset_required;
  }
  std::optional<double> hostTime(std::uint64_t remote_us, double now,
                                 double max_age) const {
    if (!clockReady(now) || !remote_us)
      return {};
    const double stamp = static_cast<double>(remote_us) * 1e-6 + offset;
    // 不用接收时刻冒充采样时刻，不把 epoch 时间猜成 boot 时间。
    if (!std::isfinite(stamp) || stamp < 0 || stamp > now ||
        now - stamp > max_age)
      return {};
    return stamp;
  }
  bool healthy(double now) const {
    if (!estimator || now < estimator_received ||
        now - estimator_received > config.estimator_timeout_s ||
        !hostTime(estimator->time_usec, now, config.estimator_timeout_s))
      return false;
    constexpr std::uint16_t needed =
        ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ |
        ESTIMATOR_VELOCITY_VERT | ESTIMATOR_POS_HORIZ_REL |
        ESTIMATOR_POS_VERT_ABS;
    return (estimator->flags & needed) == needed &&
           !(estimator->flags &
             (ESTIMATOR_CONST_POS_MODE | ESTIMATOR_ACCEL_ERROR));
  }
  bool estimatorReady(double now) const {
    return healthy(now) && flight.snapshot(now).heartbeat.has_value() &&
           autopilot == MAV_AUTOPILOT_PX4;
  }
  std::vector<std::uint8_t> encode(mavlink_message_t &m) {
    std::vector<std::uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    bytes.resize(mavlink_msg_to_send_buffer(bytes.data(), &m));
    return bytes;
  }
  void clearClock() {
    sync_samples = 0;
    last_sync = -1;
    pending_token = 0;
  }
  void timesync(const mavlink_message_t &m, double now) {
    mavlink_timesync_t s{};
    mavlink_msg_timesync_decode(&m, &s);
    if (s.tc1 <= 0)
      return; // 飞控主动探针不是对本机请求的响应。
    ++diag.timesync_responses;
    if (!pending_token || s.ts1 != pending_token) {
      ++diag.timesync_unmatched;
      return;
    }
    diag.timesync_target_system = s.target_system;
    diag.timesync_target_component = s.target_component;
    const bool addressed = (s.target_system == config.own_system_id &&
                            s.target_component == config.own_component_id);
    // 兼容旧版 PX4 的广播响应，但仍必须匹配来源与唯一的本机待决 token。
    const bool legacy = s.target_system == 0 && s.target_component == 0;
    if (!addressed && !legacy) {
      ++diag.timesync_wrong_target;
      return;
    }
    const double sent = static_cast<double>(pending_token) * 1e-9;
    pending_token = 0;
    const double rtt = now - sent;
    if (rtt < 0 || rtt > .02) {
      ++diag.timesync_bad_rtt;
      return;
    }
    const double candidate =
        (now + sent) / 2 - static_cast<double>(s.tc1) * 1e-9;
    if (sync_samples && std::abs(candidate - offset) > .05) {
      // 大偏移变化可能是飞控重启；锁住直到重新建立会话，防止旧 map
      // 原点继续使用。
      diag.reset_required = true;
      ++diag.epoch;
      clearClock();
      estimator.reset();
      diag.reason = "clock_jump_reset_required";
      return;
    }
    if (sync_samples && std::abs(candidate - offset) > .005) {
      sync_samples = 0;
    }
    const double deviation = sync_samples ? std::abs(candidate - offset) : 0;
    offset = sync_samples ? .8 * offset + .2 * candidate : candidate;
    sync_error = rtt / 2 + deviation;
    sync_samples = std::min(sync_samples + 1, 100);
    last_sync = now;
    ++diag.timesync_accepted;
  }
  PoseSample odometry(const mavlink_message_t &m, double now) {
    mavlink_odometry_t o{};
    mavlink_msg_odometry_decode(&m, &o);
    PoseSample p;
    p.received_s = now;
    p.epoch = diag.epoch;
    auto reject = [&](const char *why) {
      ++diag.rejected_odometry;
      diag.reason = why;
      p.reason = why;
      return p;
    };
    if (diag.reset_required)
      return reject("reset_required");
    if (last_odom_us && o.time_usec <= *last_odom_us) {
      if (*last_odom_us - o.time_usec > 1000000) {
        diag.reset_required = true;
        ++diag.epoch;
        p.epoch = diag.epoch;
        clearClock();
        estimator.reset();
        return reject("remote_clock_reversed_reset_required");
      }
      return reject("odometry_duplicate_or_reordered");
    }
    // EKF 复位、参考坐标改变都必须使旧 map 对齐失效。child 仅为速度的表达坐标。
    if ((reset_counter && *reset_counter != o.reset_counter) ||
        (parent_frame && *parent_frame != o.frame_id)) {
      ++diag.epoch;
      estimator.reset();
    }
    p.epoch = diag.epoch;
    last_odom_us = o.time_usec;
    reset_counter = o.reset_counter;
    parent_frame = o.frame_id;
    velocity_frame = o.child_frame_id;
    if (!config.px4_odometry_contract_confirmed)
      return reject("px4_odometry_contract_unconfirmed");
    if (o.estimator_type != MAV_ESTIMATOR_TYPE_AUTOPILOT)
      return reject("not_px4_fused_odometry");
    if (!flight.snapshot(now).heartbeat || autopilot != MAV_AUTOPILOT_PX4)
      return reject("px4_heartbeat_missing");
    if (!healthy(now))
      return reject("estimator_unhealthy_or_stale");
    const auto stamp = hostTime(o.time_usec, now, config.max_pose_age_s);
    if (!stamp)
      return reject("pose_clock_unsynced_or_stale");
    p.capture_s = *stamp;
    p.time_error_s = sync_error;
    if (o.quality < 0 || o.quality > 100)
      return reject("odometry_quality_failed");
    // quality=0 在 MAVLink 表示未知；靠独立 ESTIMATOR_STATUS
    // 与协方差判定，不当成满分。
    const int diagonal[] = {0, 6, 11, 15, 18, 20};
    for (int i = 0; i < 6; ++i) {
      const double variance = o.pose_covariance[diagonal[i]];
      const double limit =
          i < 3 ? config.max_position_std_m : config.max_angle_std_rad;
      if (!std::isfinite(variance) || variance < 0 || variance > limit * limit)
        return reject("pose_covariance_unknown_or_large");
    }
    // PX4 只发布六个对角方差，十五个非对角项全部填 NaN。
    // 接受此明确布局，但不将缺失的相关项当成零协方差。
    // 完整矩阵仍检查 PSD；混合缺失布局与无穷值均拒绝。
    cv::Matx<double, 6, 6> covariance;
    bool missing_cross_terms = false, known_cross_terms = false;
    int index = 0;
    for (int row = 0; row < 6; ++row)
      for (int col = row; col < 6; ++col) {
        const double value = o.pose_covariance[index++];
        if (row != col && std::isnan(value)) {
          missing_cross_terms = true;
          continue;
        }
        if (!std::isfinite(value))
          return reject("pose_covariance_invalid");
        if (row != col)
          known_cross_terms = true;
        covariance(row, col) = covariance(col, row) = value;
      }
    if (missing_cross_terms && known_cross_terms)
      return reject("pose_covariance_invalid");
    if (!missing_cross_terms) {
      cv::Mat eigenvalues;
      cv::eigen(cv::Mat(covariance), eigenvalues);
      if (eigenvalues.at<double>(5) < -1e-8)
        return reject("pose_covariance_invalid");
    }
    cv::Matx33d basis;
    if (o.frame_id == MAV_FRAME_LOCAL_NED) {
      basis = enuFromNed();
      p.odom_frame = "odom_enu";
    } else if (o.frame_id == MAV_FRAME_LOCAL_FRD) {
      basis = fluFromFrd();
      p.odom_frame = "odom_flu";
    } else
      return reject("unsupported_pose_frame");
    try {
      // 已核对 PX4 ODOMETRY.hpp：q 始终为 body-FRD 到父坐标，child_frame_id
      // 只描述速度。
      const auto parent_R_frd =
          quaternionRotation({o.q[0], o.q[1], o.q[2], o.q[3]});
      p.position_m = basis * cv::Vec3d{o.x, o.y, o.z};
      p.body_flu_to_odom_wxyz =
          rotationQuaternion(basis * parent_R_frd * fluFromFrd());
      const cv::Vec3d velocity{o.vx, o.vy, o.vz};
      if (o.child_frame_id == o.frame_id)
        p.velocity_odom_mps = basis * velocity;
      else if (o.child_frame_id == MAV_FRAME_BODY_FRD)
        p.velocity_odom_mps = basis * parent_R_frd * velocity;
      else
        return reject("unsupported_velocity_frame");
      if (!finiteVector(p.position_m) || !finiteVector(*p.velocity_odom_mps))
        return reject("nonfinite_odometry");
    } catch (const std::invalid_argument &) {
      return reject("invalid_odometry_quaternion");
    }
    p.valid = true;
    p.reason = "ok";
    diag.reason = "ok";
    return p;
  }
};
MavlinkLink::MavlinkLink(LinkConfig c) {
  c.validate();
  impl_ = std::make_unique<Impl>(c);
}
MavlinkLink::~MavlinkLink() = default;
std::vector<PoseSample> MavlinkLink::feed(const std::uint8_t *data,
                                          std::size_t size, double now) {
  auto &s = *impl_;
  if (size > 4096 || (!data && size) || !std::isfinite(now) || now < 0 ||
      now < s.last_feed)
    throw std::invalid_argument("Invalid MAVLink chunk/time");
  s.last_feed = now;
  std::vector<PoseSample> poses;
  for (std::size_t i = 0; i < size; ++i) {
    mavlink_message_t m{};
    mavlink_status_t status{};
    const auto result =
        mavlink_frame_char_buffer(&s.rx, &s.parse, data[i], &m, &status);
    if (result == MAVLINK_FRAMING_BAD_CRC ||
        result == MAVLINK_FRAMING_BAD_SIGNATURE) {
      ++s.diag.bad_frames;
      continue;
    }
    if (result != MAVLINK_FRAMING_OK)
      continue;
    // 未配置签名密钥，不把可解析的签名包宣称为已认证数据。
    if (m.incompat_flags & MAVLINK_IFLAG_SIGNED) {
      ++s.diag.bad_frames;
      continue;
    }
    ++s.diag.packets;
    s.diag.last_message_id = m.msgid;
    if (m.sysid != s.config.system_id || m.compid != s.config.component_id) {
      ++s.diag.foreign_packets;
      continue;
    }
    switch (m.msgid) {
    case MAVLINK_MSG_ID_TIMESYNC:
      s.timesync(m, now);
      break;
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t h{};
      mavlink_msg_heartbeat_decode(&m, &h);
      s.autopilot = h.autopilot;
      s.flight.updateHeartbeat(
          m.sysid, m.compid,
          {bool(h.base_mode & MAV_MODE_FLAG_SAFETY_ARMED), h.custom_mode},
          {now, {}});
      break;
    }
    case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
      mavlink_estimator_status_t e{};
      mavlink_msg_estimator_status_decode(&m, &e);
      if (!s.estimator || e.time_usec > s.estimator->time_usec) {
        s.estimator = e;
        s.estimator_received = now;
        if (!s.healthy(now)) {
          PoseSample invalid;
          invalid.received_s = now;
          invalid.epoch = s.diag.epoch;
          invalid.reason = "estimator_unhealthy_or_stale";
          poses.push_back(invalid);
        }
      }
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
      mavlink_attitude_quaternion_t a{};
      mavlink_msg_attitude_quaternion_decode(&m, &a);
      try {
        s.flight.updateAttitude(m.sysid, m.compid, {{a.q1, a.q2, a.q3, a.q4}},
                                {now, a.time_boot_ms});
      } catch (const std::invalid_argument &) {
        ++s.diag.bad_frames;
      }
      break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_local_position_ned_t p{};
      mavlink_msg_local_position_ned_decode(&m, &p);
      const bool valid =
          s.estimatorReady(now) &&
          s.hostTime(static_cast<std::uint64_t>(p.time_boot_ms) * 1000, now,
                     s.config.max_pose_age_s)
              .has_value();
      try {
        if (s.flight.updateLocalPosition(
                m.sysid, m.compid,
                {{p.x, p.y, p.z}, {p.vx, p.vy, p.vz}, valid, valid, valid},
                {now, p.time_boot_ms}))
          s.local_position_epoch = s.diag.epoch;
      } catch (const std::invalid_argument &) {
        ++s.diag.bad_frames;
      }
      break;
    }
    case MAVLINK_MSG_ID_ODOMETRY:
      poses.push_back(s.odometry(m, now));
      break;
    default:
      break; // 不认识的消息不假定为位置数据
    }
  }
  // 即使没有 ODOMETRY 到来，时钟跳变也通知下游清空历史。
  if (s.diag.reset_required) {
    PoseSample invalid;
    invalid.received_s = now;
    invalid.epoch = s.diag.epoch;
    invalid.reason = "reset_required";
    poses.push_back(invalid);
  }
  return poses;
}
std::vector<std::uint8_t> MavlinkLink::timesyncProbe(double now) {
  auto &s = *impl_;
  if (!std::isfinite(now) || now <= 0 || now > 9e9)
    throw std::invalid_argument("Invalid TIMESYNC host time");
  if (s.diag.reset_required)
    return {};
  // 一个待决请求最多保留 100ms，避免无界队列和响应误匹配。
  if (s.pending_token && now - static_cast<double>(s.pending_token) * 1e-9 < .1)
    return {};
  s.pending_token = static_cast<std::int64_t>(std::llround(now * 1e9));
  mavlink_timesync_t t{};
  t.ts1 = s.pending_token;
  t.target_system = s.config.system_id;
  t.target_component = s.config.component_id;
  mavlink_message_t m{};
  mavlink_msg_timesync_encode_status(s.config.own_system_id,
                                     s.config.own_component_id, &s.tx, &m, &t);
  return s.encode(m);
}
FlightStateSnapshot MavlinkLink::flightState(double now) const {
  auto snapshot = impl_->flight.snapshot(now);
  if (snapshot.local_position) {
    auto &position = *snapshot.local_position;
    // 缓存按接收时间超时，位置有效性还必须按远端采样时间检查；
    // 延迟到达的样本不能在接收后再获得完整的一段有效期。
    const bool current =
        impl_->estimatorReady(now) &&
        impl_->local_position_epoch == impl_->diag.epoch &&
        position.stamp.remote_boot_ms &&
        impl_->hostTime(*position.stamp.remote_boot_ms * 1000, now,
                        impl_->config.max_pose_age_s)
            .has_value();
    if (!current) {
      position.value.xy_valid = false;
      position.value.z_valid = false;
      position.value.velocity_valid = false;
    }
  }
  return snapshot;
}
LinkDiagnostics MavlinkLink::diagnostics(double now) const {
  if (!std::isfinite(now) || now < 0)
    throw std::invalid_argument("Invalid diagnostic time");
  auto d = impl_->diag;
  d.clock_ready = impl_->clockReady(now);
  d.estimator_ready = impl_->estimatorReady(now);
  d.odometry_frame_id = impl_->parent_frame;
  d.velocity_frame_id = impl_->velocity_frame;
  d.clock_error_s = impl_->sync_error;
  d.clock_sync_samples = impl_->sync_samples;
  return d;
}
void MavlinkLink::resetSession() {
  const auto epoch = impl_->diag.epoch + 1;
  const auto c = impl_->config;
  impl_ = std::make_unique<Impl>(c);
  impl_->diag.epoch = epoch;
}
} // namespace aim
