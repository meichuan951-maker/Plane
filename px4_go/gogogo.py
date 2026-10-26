from pymavlink import mavutil
import time
import math

# ======================== 参数 ========================   位置控制

#CONNECTION = "udp:127.0.0.1:14540"      #SITL 14540，QGC 14550    ttyACM0串口号+波特率57600
connection_string = "/dev/ttyACM0"  # 串口连接 PX4，波特率 57600      会变0123
baud_rate =57600


SETPOINT_HZ = 20
DT = 1.0 / SETPOINT_HZ

HOME_Z = -2.0          # 悬停高度 2 m（NED 下为正，所以是负）
SPEED = 0.5            # m/s，航点之间的平移速度

OFFBOARD_CUSTOM_MODE = 6 << 16   # 393216

# ======================== 连接 ========================
#master = mavutil.mavlink_connection(CONNECTION)  #仿真时发的      
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)  #真实无人机上连接px4飞控然后识别
print("等待 PX4 心跳...")
# 固定目标为实际 PX4，避免 QGC/router 的心跳成为命令目标。
connect_deadline = time.monotonic() + 30
while True:
    heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if (heartbeat is not None
            and heartbeat.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            and heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4):
        master.target_system = heartbeat.get_srcSystem()
        master.target_component = heartbeat.get_srcComponent()
        break
    if time.monotonic() >= connect_deadline:
        raise SystemExit("❌ 30 秒内未收到 PX4 飞控心跳，请检查串口及 MAVLink 配置")
print(f"连接成功：system={master.target_system}, component={master.target_component}")

prog_start = time.monotonic()


def time_boot_ms():
    return int((time.monotonic() - prog_start) * 1000) & 0xFFFFFFFF


# ======================== 心跳过滤（关键修复）========================

FC_ID = 1   # MAV_COMP_ID_AUTOPILOT1


def is_fc_hb(hb):
    """
    只接受来自飞控 autopilot 组件的心跳。
    链路里 QGC / mavlink-router / 其他节点也会广播 HEARTBEAT，
    它们的 custom_mode 通常是 0，会误触发"脱离 OFFBOARD"。
    """
    if hb is None:
        return False
    try:
        if hb.get_srcSystem() != master.target_system or hb.get_srcComponent() != FC_ID:
            return False
    except Exception:
        return False
    # 有些桥接/转发节点发来的包 type==0，也过滤掉
    if getattr(hb, "type", 0) == 0:
        return False
    return True


# ======================== 发送 setpoint ========================

last_position = None
arm_ack = None
status_history = []


def receive_messages():
    """统一读取消息，避免按 HEARTBEAT 筛选时丢弃 ACK 和拒绝原因。"""
    global last_position, arm_ack
    latest_hb = None
    # 限制处理时间，防止遥测繁忙时阻塞 setpoint 发送。
    deadline = time.monotonic() + DT / 2
    for _ in range(200):
        if time.monotonic() >= deadline:
            break
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        if (msg.get_srcSystem() != master.target_system
                or msg.get_srcComponent() != FC_ID):
            continue
        kind = msg.get_type()
        if kind == "HEARTBEAT" and is_fc_hb(msg):
            latest_hb = msg
        elif kind == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            text = text.rstrip("\x00")
            status_history.append(text)
            del status_history[:-20]
            print(f"[PX4 severity={msg.severity}] {text}")
        elif kind == "COMMAND_ACK":
            entry = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result)
            result = entry.name if entry else str(msg.result)
            print(f"[COMMAND_ACK] command={msg.command}, result={result} ({msg.result}), "
                  f"result_param2={getattr(msg, 'result_param2', 0)}")
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                arm_ack = msg.result
        elif kind == "LOCAL_POSITION_NED":
            last_position = msg
    return latest_hb


def send_pos(x, y, z):#发送目标位置，在本地坐标系下，NED坐标系，x北，y东，z下
    mask = ( #选择控制模式，位置，速度，加速度或力，偏航角yaw，偏航角速度yaw_rate，忽略的位为1，使用的位为0
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )#都忽略
    master.mav.set_position_target_local_ned_send( #持续发送位置 setpoint，PX4 需要持续发送，否则会自动退出 OFFBOARD
        time_boot_ms(),                           #    消息时间辍
        master.target_system,                     #目标系统ID：发给那个飞机
        master.target_component,                  #目标组件ID：发给那个组件
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,       #MAV_FRAME_LOCAL_NED        MAV_FRAME_BODY_NED机体坐标系
        mask,                                      #控制模式掩码
        x, y, z,                                      #位置
        0, 0, 0,                              
        0, 0, 0,
        0, 0                 #yaw, yaw_rate
    )

def pump(x, y, z):     #循环
    """
    每周期发送 setpoint 并处理所有类型的遥测，返回本周期最新飞控心跳。
    """
    send_pos(x, y, z)
    return receive_messages()


# ======================== 模式 / ARM / LAND ========================
#让px4飞控切换到offboard模式
def request_offboard(): 
    print("请求 OFFBOARD...")
    master.mav.set_mode_send(
        master.target_system,           #目标系统ID：发给那个飞机
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,   #发送模式类型：自定义模式
        OFFBOARD_CUSTOM_MODE,                                #发送的自定义模式值
    )


def arm():           #起起起飞
    global arm_ack
    arm_ack = None
    print("发送 ARM...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )


def land():
    print("请求 LAND...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )


# ======================== 状态监护 ========================

_bad_hb_count = 0     # 连续异常心跳计数
_BAD_HB_LIMIT = 3     # ★ 抗抖动：连坏 N 次才认定退出


def check_hb(hb, label=""):
    """
    返回 True = 一切正常；False = 确认退出，调用方应中止航线。
    非飞控心跳静默忽略；飞控心跳异常则打印真实 custom_mode 并计数。
    """
    global _bad_hb_count
    if not is_fc_hb(hb):
        return True                     # 不是飞控的包，直接忽略（不再误报）

    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    cm = hb.custom_mode

    if not armed:
        print(f"\n⚠️ [{label}] 检测到已 disarm (custom_mode={cm})，中止航线")
        return False
    if cm != OFFBOARD_CUSTOM_MODE:
        _bad_hb_count += 1
        print(f"\n⚠️ [{label}] 飞控 custom_mode={cm} (期望 {OFFBOARD_CUSTOM_MODE})，"
              f"第 {_bad_hb_count}/{_BAD_HB_LIMIT} 次")
        if _bad_hb_count >= _BAD_HB_LIMIT:
            print("   → 确认脱离 OFFBOARD，中止航线")
            return False
    else:
        _bad_hb_count = 0               # 恢复正常就清零

    return True


# ======================== 等待 OFFBOARD ========================

def wait_offboard(timeout=20):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        hb = pump(0, 0, HOME_Z)
        if is_fc_hb(hb) and hb.custom_mode == OFFBOARD_CUSTOM_MODE:
            return True
        time.sleep(DT)
    # 打印最后一次看到的值，方便排错
    print(f"   最后收到的 custom_mode={hb.custom_mode if hb else None}, "
          f"srcComp={hb.get_srcComponent() if hb else '?'}")
    return False


# ======================== 1. 预填充 ========================

print("预发送 setpoint ...")
for _ in range(100):
    pump(0, 0, HOME_Z)
    time.sleep(DT)

# ======================== 2. OFFBOARD ========================

request_offboard()
if not wait_offboard():
    raise SystemExit("❌ 未进入 OFFBOARD（检查 SITL/QGC 状态、home 是否设定）")
print("✅ OFFBOARD 已进入")

# ======================== 3. ARM ========================

arm()
t0 = time.monotonic()
armed = False
while time.monotonic() - t0 < 20:
    hb = pump(0, 0, HOME_Z)
    if is_fc_hb(hb) and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
    time.sleep(DT)
if not armed:
    if arm_ack is None:
        print("未收到 ARM 的 COMMAND_ACK：检查命令目标、串口是否被其他程序占用及链路状态。")
    elif arm_ack == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("ARM 命令已接受，但未观察到已解锁心跳；检查自动上锁、心跳链路及上方 PX4 消息。")
    else:
        print(f"ARM 未成功，ACK result={arm_ack}；具体检查项见上方 [PX4] 消息。")
    if status_history:
        print("最近 PX4 提示：\n  " + "\n  ".join(status_history[-10:]))
    print("若没有文字原因，请查看 QGC 的飞行前检查/通知，或 MAVLink Console 执行 commander check。")
    raise SystemExit("❌ 未确认解锁，停止执行航线")
print("✅ 已 ARMED")

# ======================== 4. 航线 ========================
# LOCAL_NED：X 北(=机头朝北时的前方)，Y 东(=右侧)，Z 下
waypoints = [
    (0.0, 0.0, HOME_Z),   # 起点，悬停 2 m
    # (2.0, 0.0, HOME_Z),   # 前飞 2 m
    # (2.0, 3.0, HOME_Z),   # 右飞 3 m
]


def dist3(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def lerp(a, b, t):
    t = max(0.0, min(1.0, t))
    return (a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


try:
    last_log = time.monotonic()

    for i in range(len(waypoints) - 1):
        p0, p1 = waypoints[i], waypoints[i + 1]
        L = dist3(p0, p1)
        steps = max(1, int(round(L / (SPEED * DT))))
        print(f"\n航段 {i + 1} -> {i + 2}: {p0}  ->  {p1}   距离 {L:.2f} m, 共 {steps} 步")

        for k in range(steps + 1):
            tgt = lerp(p0, p1, k / steps)
            hb = pump(*tgt)
            if not check_hb(hb, label=f"航段{i+1}"):
                raise SystemExit

            if time.monotonic() - last_log > 2.0:      # ★ 降频打印，减少阻塞
                pos = last_position
                if pos:
                    print(f"  目标=({tgt[0]:5.2f},{tgt[1]:5.2f},{tgt[2]:5.2f}) | "
                          f"实际 x={pos.x:6.2f} y={pos.y:6.2f} 高度={-pos.z:5.2f} m vz={pos.vz:6.2f}")
                last_log = time.monotonic()

            time.sleep(DT)

        # 到达航点后悬停 2 s
        hold_until = time.monotonic() + 2.0
        while time.monotonic() < hold_until:
            hb = pump(*p1)
            if not check_hb(hb, label="悬停"):
                raise SystemExit
            time.sleep(DT)

    print("\n航线完成，悬停 5 s 后自动降落 ...")
    hold_until = time.monotonic() + 5.0
    while time.monotonic() < hold_until:
        pump(*waypoints[-1])
        time.sleep(DT)

except KeyboardInterrupt:
    print("\n收到 Ctrl+C")

# ======================== 5. 降落（期间继续保活）========================

land()
print("降落中，保持 setpoint ...")
t0 = time.monotonic()
disarmed = False
while time.monotonic() - t0 < 40:
    hb = pump(*waypoints[-1])
    if is_fc_hb(hb) and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        disarmed = True
        break
    time.sleep(DT)
print("已 disarm，程序结束" if disarmed else "超时未检测到 disarm，请手动接管")

