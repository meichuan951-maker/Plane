#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
室内无 GPS Offboard 飞行脚本
硬件前提：光流 + 测距仪已接入 EKF，遥控器可定点定高
链路：笔记本 USB 直连 Pixhawk（/dev/ttyACM0）
"""

from pymavlink import mavutil
import time
import math


# ============================================================
# CONFIG：真机只需改这里
# ============================================================

CONNECTION = "/dev/ttyACM0"   # USB直连 Pixhawk；数传电台改成 "/dev/ttyUSB0:57600"
FC_ID      = 1                      # MAV_COMP_ID_AUTOPILOT1

SETPOINT_HZ = 20
DT          = 1.0 / SETPOINT_HZ

TARGET_ALT  = 1.0                   # ★ 室内先 1~1.5 m，别超过 2 m（NED，实际发 -TARGET_ALT）
SPEED       = 0.4                   # m/s，室内慢一点
AUTO_ARM    = False                 # False=遥控器解锁并切Offboard（首飞推荐）；True=全自动

OFFBOARD_CUSTOM_MODE = 6 << 16      # PX4 主模式 6

# ============================================================
# 连接
# ============================================================

print("连接飞控 %s ..." % CONNECTION)
master = mavutil.mavlink_connection(CONNECTION)
master.target_system    = 1         # ★ 真机必须写死
master.target_component = 1         # ★ 别用广播 0

print("等待心跳...")
hb = master.wait_heartbeat(timeout=15)
if hb is None:
    raise SystemExit("❌ 15秒内没收到心跳：检查 /dev/ttyACM*、dialout 权限、ModemManager、QGC是否占口")
print("连接成功  sys=%d comp=%d" % (master.target_system, master.target_component))

# ============================================================
# 工具函数
# ============================================================

_prog_start = time.monotonic()

def time_boot_ms():
    """单调递增的毫秒时间戳，脚本重启不会跳变"""
    return int((time.monotonic() - _prog_start) * 1000) & 0xFFFFFFFF


def is_fc(msg):
    """只认飞控发的包，过滤 QGC/mavlink-router 的杂用心跳"""
    if msg is None:
        return False
    try:
        if msg.get_srcComponent() != FC_ID:
            return False
    except Exception:
        pass
    return True


def send_setpoint(x, y, z):
    """位置型 setpoint：只保留位置，速度/加速度/yaw 全部忽略"""
    mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        time_boot_ms(), master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0
    )


def pump(x, y, z):
    """发一个周期 setpoint（双份抗抖动）+ 非阻塞收一条消息"""
    send_setpoint(x, y, z)
    send_setpoint(x, y, z)
    return master.recv_match(blocking=False)


def poll_diag():
    """打印飞控的原话：arm/offboard 被拒的原因全在这里"""
    ack = master.recv_match(type='COMMAND_ACK', blocking=False)
    if ack:
        names = {0: 'ACCEPTED', 2: 'DENIED', 3: 'UNSUPPORTED',
                 4: 'FAILED', 7: 'IN_PROGRESS'}
        print("  [ACK] cmd=%d result=%s" % (ack.command, names.get(ack.result, ack.result)))
    txt = master.recv_match(type='STATUSTEXT', blocking=False)
    if txt:
        print("  [飞控] %s" % txt.text.strip())


def get_local_pos(timeout=12):
    """读取当前 LOCAL_POSITION_NED，作为航线起点（无 GPS 时必须相对当前位置）"""
    t0 = time.monotonic()
    best = None
    while time.monotonic() - t0 < timeout:
        p = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if p and getattr(p, 'eph', 999) < 10.0:
            best = (p.x, p.y, p.z)
            break
        pump(0, 0, -TARGET_ALT)
        time.sleep(DT)
        poll_diag()
    if best is None:
        raise SystemExit("❌ 拿不到本地位置估计：检查光流/测距数据、EKF2_HGT_MODE=Range")
    print("当前位置 x=%.2f  y=%.2f  z=%.2f (高度 %.2f m)" % (best[0], best[1], best[2], -best[2]))
    return best


def wait_flag(check_fn, label, timeout=30, setpoint=(0, 0, 0)):
    """通用等待：持续保活 setpoint，直到 check_fn(msg) 返回 True"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        msg = pump(*setpoint)
        poll_diag()
        if is_fc(msg) and check_fn(msg):
            print("✅ %s" % label)
            return True
        time.sleep(DT)
    print("❌ 超时：%s" % label)
    return False


def request_offboard():
    print("\n请求 OFFBOARD ...")
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             OFFBOARD_CUSTOM_MODE)


def arm():
    print("\n发送 ARM ...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 1, 0, 0, 0, 0, 0, 0)


def disarm():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 0, 0, 0, 0, 0, 0, 0)


def land():
    print("\n请求 LAND ...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_LAND,
                                 0, 0, 0, 0, 0, 0, 0, 0)


# ============================================================
# 航线：相对当前位置生成
# ============================================================

origin = get_local_pos()
ox, oy, oz = origin

waypoints = [
    (ox,      oy,      -TARGET_ALT),          # 悬停点（就在头顶）
    (ox+2.0,  oy,      -TARGET_ALT),          # 前 2 m
    (ox+2.0,  oy+2.0,  -TARGET_ALT),          # 右 2 m（室内别飞太大）
    (ox,      oy,      -TARGET_ALT),          # 回原点
]


def lerp(a, b, t):
    t = max(0.0, min(1.0, t))
    return (a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t)


def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


# ============================================================
# 主流程
# ============================================================

try:
    # ---------- 阶段 0：预填充 ----------
    print("\n预发送 setpoint (2s)...")
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        pump(0, 0, -TARGET_ALT)
        poll_diag()
        time.sleep(DT)

    # ---------- 阶段 1：OFFBOARD ----------
    if AUTO_ARM:
        request_offboard()
        if not wait_flag(lambda m: m.custom_mode == OFFBOARD_CUSTOM_MODE,
                         "已进入 OFFBOARD", timeout=30, setpoint=(0, 0, -TARGET_ALT)):
            raise SystemExit
        # ---------- 阶段 2：ARM ----------
        arm()
        if not wait_flag(lambda m: bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                         "已 ARMED", timeout=30, setpoint=(0, 0, -TARGET_ALT)):
            raise SystemExit
    else:
        print("\n【手动模式】请用遥控器解锁，并把模式开关拨到 Offboard")
        print("          （脚本只做给定目标，失效即安全）")
        if not wait_flag(lambda m: (m.custom_mode == OFFBOARD_CUSTOM_MODE) and
                                   bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                         "检测到 OFFBOARD + ARMED", timeout=90, setpoint=(0, 0, -TARGET_ALT)):
            raise SystemExit

    # ---------- 阶段 3：航线 ----------
    print("\n========== 开始航线 ==========")
    last_log = time.monotonic()

    for i in range(len(waypoints) - 1):
        p0, p1 = waypoints[i], waypoints[i + 1]
        L = max(dist(p0, p1), 0.01)
        steps = max(1, int(round(L / (SPEED * DT))))
        print("\n航段 %d -> %d : 距离 %.2f m, %d 步" % (i + 1, i + 2, L, steps))

        for k in range(steps + 1):
            tgt = lerp(p0, p1, k / steps)
            msg = pump(*tgt)
            poll_diag()
            if is_fc(msg) and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                raise SystemExit("\n⚠️ 已 disarm，中止航线")

            if time.monotonic() - last_log > 1.0:
                pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                if pos:
                    print("  目标=(%.2f,%.2f,%.2f) | 实际 x=%.2f y=%.2f 高=%.2f m vz=%.2f"
                          % (tgt[0], tgt[1], tgt[2], pos.x, pos.y, -pos.z, pos.vz))
                last_log = time.monotonic()
            time.sleep(DT)

        # 每个航点悬停 1.5 s
        t_hold = time.monotonic() + 1.5
        while time.monotonic() < t_hold:
            pump(*p1)
            poll_diag()
            time.sleep(DT)

    print("\n航线完成，悬停 3 s 后降落 ...")
    t_hold = time.monotonic() + 3.0
    while time.monotonic() < t_hold:
        pump(*waypoints[-1])
        time.sleep(DT)

except KeyboardInterrupt:
    print("\n收到 Ctrl+C")

# ---------- 阶段 4：降落（期间持续保活）----------
land()
print("降落中，保持 setpoint ...")
t0 = time.monotonic()
ok = False
while time.monotonic() - t0 < 45:
    pump(*waypoints[-1])
    msg = master.recv_match(type='HEARTBEAT', blocking=False)
    if is_fc(msg) and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        ok = True
        break
    time.sleep(DT)
print("✅ 已 disarm，程序结束" if ok else "⚠️ 未检测到 disarm，请手动接管（拨回 Position 并上锁）")