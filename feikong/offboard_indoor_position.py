#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""室内 Offboard 航线脚本：PX4 SITL / 实机均可"""
import time, signal, sys
from pymavlink import mavutil

# ========== 配置区（只改这里）==========
CONNECTION = "udp:127.0.0.1:14540"   # PX4 SITL 默认端口
AUTO_ARM   = True                     # 仿真中可直接自动解锁
TARGET_ALT = 1.0                      # 初始高度 1 m
WP_SPEED   = 0.5                      # m/s
WAYPOINTS  = [
    (0.0, 0.0, -TARGET_ALT),
    # (1.0, 0.0, -TARGET_ALT),
    # (1.0, 1.0, -TARGET_ALT),
    # (0.0, 1.0, -TARGET_ALT),
    # (0.0, 0.0, -TARGET_ALT),
]
# ======================================

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

master = None
running = True


def handler(sig, frame):
    global running
    running = False
    print("\n[中断] 尝试降落...")
    signal.signal(signal.SIGINT, handler)


def connect():
    global master
    print(f"连接飞控 {CONNECTION} ...")
    master = mavutil.mavlink_connection(CONNECTION, source_system=255)
    print("等待心跳...")
    hb = master.wait_heartbeat(timeout=15)
    if hb is None:
        print("❌ 超时，未收到心跳"); sys.exit(1)
    master.target_system = hb.get_srcSystem()
    master.target_component = hb.get_srcComponent()
    print(f"连接成功 sys={master.target_system} comp={master.target_component}")


def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack is None:
        print("⚠️ 未收到 ARM 确认")
        return
    if hasattr(ack, 'result') and ack.result != 0:
        print(f"❌ 解锁失败，result={ack.result}")
    else:
        print("✅ 解锁命令已接受")


def disarm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )


def set_offboard():
    # PX4 的 OFFBOARD 是主模式值 6，不能左移 16 位
    for _ in range(20):
        if not running:
            break
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            0, 0, 0, 0, 0
        )
        send_setpoint(0, 0, -TARGET_ALT)
        time.sleep(0.05)


def send_setpoint(x, y, z, yaw=0.0):
    # MAVLink time_boot_ms 必须是 uint32，不能用 Unix 时间戳
    t_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        t_ms,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,   # 机体坐标系       MAV_FRAME_LOCAL_NED
        0b100111111000,   # 只启用位置 + yaw
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw, 0
    )


def pos():
    m = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
    if m is None:
        return None
    return (m.x, m.y, m.z)


def wait_state(offboard_ok, armed_ok, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout and running:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            print("heartbeat None")
            continue
        if hb.get_srcComponent() != 1:
            continue

        mode = hb.custom_mode
        off = (mode & 0xFFFF) == PX4_CUSTOM_MAIN_MODE_OFFBOARD
        arm = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        print(f"custom_mode={mode} off={off} armed={arm} base_mode={hb.base_mode}")

        if off and arm:
            return True
    return False


def main():
    connect()
    time.sleep(0.5)

    p = pos()
    print(f"当前位置 x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}" if p else "⚠ 未收到 LOCAL_POSITION_NED")

    print("\n预发送 setpoint (2s)...")
    t0 = time.time()
    while time.time() - t0 < 2.0 and running:
        send_setpoint(0, 0, -TARGET_ALT)
        time.sleep(0.05)

    if AUTO_ARM:
        print("[自动] 解锁 + 切 Offboard")
        arm()
        time.sleep(0.3)
    else:
        print("\n【手动模式】请用遥控器解锁，并把模式开关拨到 Offboard")

    ok = wait_state(offboard_ok=True, armed_ok=True, timeout=90)
    if not ok:
        print("❌ 超时：未检测到 OFFBOARD + ARMED，退出")
        sys.exit(1)

    print("✅ 检测到 OFFBOARD + ARMED\n========== 开始航线 ==========")

    for i, (x, y, z) in enumerate(WAYPOINTS, 1):
        if not running:
            break
        prev = WAYPOINTS[i-2] if i > 1 else (0, 0, -TARGET_ALT)
        dist = ((x-prev[0])**2 + (y-prev[1])**2 + (z-prev[2])**2) ** 0.5
        steps = max(int(dist / WP_SPEED / 0.05), 1)
        print(f"航段 {i}: 目标=({x:.2f},{y:.2f},{z:.2f}) 距离 {dist:.2f}m / {steps}步")
        for s in range(steps):
            if not running:
                break
            sx = prev[0] + (x-prev[0]) * s / steps
            sy = prev[1] + (y-prev[1]) * s / steps
            sz = prev[2] + (z-prev[2]) * s / steps
            send_setpoint(sx, sy, sz)
            if s % 10 == 0:
                cur = pos()
                h = -cur[2] if cur else None
                print(f"  实际 x={cur[0]:.2f} y={cur[1]:.2f} 高={h:.2f}m" if cur else "  (无位置数据)")
            time.sleep(0.05)
        send_setpoint(x, y, z)
        time.sleep(0.5)

    print("========== 航线结束 ==========")
    print("切回 Position 并 disarm（遥控器或脚本）")


if __name__ == "__main__":
    main()