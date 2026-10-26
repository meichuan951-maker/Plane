from pymavlink import mavutil

print("正在等待 PX4 SITL 心跳...")

master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

master.wait_heartbeat()

print("连接成功")
print("目标系统ID:", master.target_system)
print("目标组件ID:", master.target_component)
