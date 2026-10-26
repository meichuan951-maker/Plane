from pymavlink import mavutil
import time


# ============================================================
# 参数
# ============================================================
#
CONNECTION = "udp:127.0.0.1:14540"      #pymavlink 的连接串。14540 是 PX4（gazebo）MAVLink 的端口，127.0.0.1 表示仿真跑在本机。真机飞的话要改成 udpout:飞控IP:14550 之类。
#世界坐标系
TARGET_X = 2.0
TARGET_Y = 3.0
TARGET_Z = -2.0       # LOCAL_NED: -0.5 = 上方 0.5 m

SETPOINT_HZ = 20   #频率为20hz发给飞控
SETPOINT_DT = 1.0 / SETPOINT_HZ

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
OFFBOARD_CUSTOM_MODE = PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
#pX4 的 custom_mode 字段是 32 位：高 16 位是主模式，低 16 位是子模式


# ============================================================
# 连接 PX4
# ============================================================

master = mavutil.mavlink_connection(CONNECTION)

print("等待 PX4 心跳...")
master.wait_heartbeat()

print("连接成功")
print("System:", master.target_system)
print("Component:", master.target_component)


# ============================================================
# 时间
# ============================================================

program_start = time.monotonic()


#是上电后的毫秒数，真机上要改
def time_boot_ms():
    """
    MAVLink 的 time_boot_ms
    不使用 time.time() 的 Unix 时间戳
    """
    return int(
        (time.monotonic() - program_start) * 1000
    ) & 0xFFFFFFFF


# ============================================================
# 发送位置 setpoint
#
# LOCAL_NED
#
# X = North
# Y = East
# Z = Down
#SET_POSITION_TARGET_LOCAL_NED 这条消息同时携带位置、速度、加速度、偏航角、偏航角速度。type_mask 用来告诉飞控哪些字段忽略。这里把速度、加速度、偏航角速率全部忽略，只保留 位置 + yaw。
# z = -0.5
# 上方 0.5 m
# ============================================================

def send_position_setpoint(x, y, z):

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |

        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |

        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    master.mav.set_position_target_local_ned_send(

        time_boot_ms(),

        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        type_mask,

        x,
        y,
        z,

        0,
        0,
        0,

        0,
        0,
        0,

        0,
        0
    )


# ============================================================
# 请求 OFFBOARD
# ============================================================

def request_offboard():

    print()
    print("请求 OFFBOARD...")

    master.mav.set_mode_send(

        master.target_system,

        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,

        OFFBOARD_CUSTOM_MODE
    )


# ============================================================
# ARM
# ============================================================

def arm():

    print()
    print("发送 ARM...")

    master.mav.command_long_send(

        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

        0,

        1,
        0,
        0,
        0,
        0,
        0,
        0
    )


# ============================================================
# LAND
# ============================================================

def land():

    print()
    print("================================")
    print("请求 LAND")
    print("================================")

    master.mav.command_long_send(

        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_CMD_NAV_LAND,

        0,

        0,
        0,
        0,
        0,
        0,
        0,
        0
    )


# ============================================================
# 1. 预发送 setpoint
#
# 20 Hz
#
# 注意：
# 这里不能只发送一次。
# ============================================================

print()
print("开始预发送 Offboard setpoint...")

for i in range(100):

    send_position_setpoint(
        TARGET_X,
        TARGET_Y,
        TARGET_Z
    )

    time.sleep(SETPOINT_DT)

print("已经发送 100 个 setpoint")


# ============================================================
# 2. 请求 OFFBOARD
# ============================================================

request_offboard()


# ============================================================
# 3. 等待 OFFBOARD
#
# 关键：
# 等待过程中仍然持续发送 setpoint
# ============================================================

offboard = False

start_time = time.monotonic()

while time.monotonic() - start_time < 5.0:

    # ★ 始终保持 Offboard 心跳
    send_position_setpoint(
        TARGET_X,
        TARGET_Y,
        TARGET_Z
    )

    msg = master.recv_match(
        type="HEARTBEAT",
        blocking=False
    )

    if msg is not None:

        print(
            "custom_mode:",
            msg.custom_mode,
            "base_mode:",
            msg.base_mode
        )

        if msg.custom_mode == OFFBOARD_CUSTOM_MODE:

            offboard = True
            break

    time.sleep(SETPOINT_DT)


if not offboard:

    print()
    print("❌ OFFBOARD 没有成功进入")

    raise SystemExit


print()
print("================================")
print("✅ OFFBOARD 已进入")
print("================================")


# ============================================================
# 4. ARM
#
# 注意：
# ARM 等待过程中仍然发送 setpoint
# ============================================================

arm()

armed = False

start_time = time.monotonic()

while time.monotonic() - start_time < 5.0:

    # ★ ARM 等待期间也不能停止 setpoint
    send_position_setpoint(
        TARGET_X,
        TARGET_Y,
        TARGET_Z
    )

    msg = master.recv_match(
        type="HEARTBEAT",
        blocking=False
    )

    if msg is not None:

        armed = bool(
            msg.base_mode &
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        print(
            "Armed:",
            armed,
            "custom_mode:",
            msg.custom_mode
        )

        if armed:
            break

    time.sleep(SETPOINT_DT)


if not armed:

    print()
    print("❌ ARM 失败")

    raise SystemExit


print()
print("================================")
print("✅ 已 Armed")
print("================================")


# ============================================================
# 5. 正式飞行
#
# 持续 20 Hz 发送
# ============================================================

print()
print("开始保持 2 m 高度")
print("LOCAL_NED Z =", TARGET_Z)
print("按 Ctrl+C → 自动 LAND")
print()


try:

    last_print = time.monotonic()

    while True:

        # ★ 核心：
        # 永远持续发送 setpoint
        send_position_setpoint(
            TARGET_X,
            TARGET_Y,
            TARGET_Z
        )

        # 每 1 秒打印一次状态
        if time.monotonic() - last_print > 1.0:

            msg = master.recv_match(
                type="LOCAL_POSITION_NED",
                blocking=False
            )

            if msg is not None:

                height = -msg.z

                print(
                    "高度: %.2f m   "
                    "x: %.2f   "
                    "y: %.2f   "
                    "vz: %.2f"
                    % (
                        height,
                        msg.x,
                        msg.y,
                        msg.vz
                    )
                )

            last_print = time.monotonic()

        time.sleep(SETPOINT_DT)


# ============================================================
# 6. Ctrl+C
#
# 不直接退出！
# 请求 LAND
# ============================================================

except KeyboardInterrupt:

    print()
    print("收到 Ctrl+C")

    try:

        land()

        print("等待 PX4 执行 LAND...")
        time.sleep(5)

    except Exception as e:

        print("LAND 命令发送失败:", e)

    print("程序结束")
