from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

print("等待 PX4 心跳...")
master.wait_heartbeat()
print("连接成功")

while True:
    msg = master.recv_match(type=['HEARTBEAT', 'ATTITUDE'], blocking=True, timeout=1)

    if msg is None:
        continue

    if msg.get_type() == 'HEARTBEAT':
        print("收到 HEARTBEAT，系统ID:", master.target_system)

    elif msg.get_type() == 'ATTITUDE':
        print("姿态 -> roll:", msg.roll, "pitch:", msg.pitch, "yaw:", msg.yaw)

    time.sleep(0.2)
