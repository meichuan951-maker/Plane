from pymavlink import mavutil
import time
import math







#==========================参数设置==========================    速度控制
# 设置串口参数
connection_string ="/dev/ttyACM0"  # 串口连接 PX4，波特率 57600
baud_rate = 57600 #波特率57600   或者115200

SETPOINT_HZ=20    # 间隔20
DT=1.0/SETPOINT_HZ
HOME_Z =2.0 #z轴高度是2m
SPEED = 0.5 # 0.5m/s   

OFFBOARD_CUSTOM_MODE = 6 << 16


#=========================连接PX4==========================
master = mavuitl.mavlink_connection(connection_string, baud=baud_rate) #练级飞控
print("正在等待 px4 心跳... ")
#固定目标为实际px4,而不是地面站
connection
connect_deadline = time.time() + 30
while True:
    heartbeat =master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if (heartbeat is not None)

