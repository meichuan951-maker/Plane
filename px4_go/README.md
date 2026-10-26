# PX4 电控、离线联测与视觉偏航

本目录保存 Python 电控原稿、可导入的离线控制模块，以及独立的无桌面视觉偏航运行器。
它与 [src/](../src/README.md) 中的 C++ 感知、定位和任务实现并列；Python 模块不参加 CMake 编译。
下面两条运行链路用途不同：离线联测只连接内存模拟 peer，`yaw_follow` 则会连接真实串口并执行自动起飞任务。

## 文件分工

- [gogogo.py](gogogo.py)：电控提供的正式位置控制原稿。
- [offboard_go_1.py](offboard_go_1.py)、[body_go1.py](body_go1.py)：保留的未完成方案；[heartbeat_test.py](heartbeat_test.py) 是原始链路检查。
- [control.py](control.py)：不访问设备的位置控制状态机、MAVLink 内存编解码和模拟 peer，供离线 worker 和回归使用。
- [offline_control.py](offline_control.py)：离线命令行与联测 worker，驱动 `control.py` 并输出控制周期统计。
- [yaw_policy.py](yaw_policy.py)：纯计算模块，校验 `armor_auto` 的二维检测、跟踪和时效字段，将目标水平像素误差变成有限幅和死区的偏航角速度。
- [yaw_mission.py](yaw_mission.py)：不访问设备的单次任务状态机，接收飞行/视觉状态，返回保持位置、切换模式、解锁或降落等动作。
- [yaw_follow.py](yaw_follow.py)：运行编排与设备适配，启动无窗口 `armor_auto`、读取其标准输出 JSONL，独占 PX4 串口，将任务动作转成 MAVLink 消息。
- [__init__.py](__init__.py)：Python 包入口；`__pycache__/` 是自动生成的解释器缓存。

## 与其他目录的联系

- [tools/benchmark_vision_control.py](../tools/benchmark_vision_control.py) 将 `offline_control` 与 C++ 录像视觉 worker 并行启动，统一测量窗口并采样资源；视觉结果不传给离线控制器。协议和统计口径见[联测指南](../docs/vision_control_benchmark.md)。
- [src/app/armor_auto/](../src/app/armor_auto/) 生成 `yaw_follow` 消费的视觉 JSONL；其采集和识别分别复用 [src/input/](../src/input/README.md) 和 [src/vision/](../src/vision/README.md)。二维跟踪可以在 PnP 米制 `valid=false` 时使用，不能据此获得目标距离或视线角度。
- [config/armor_auto.yaml](../config/armor_auto.yaml) 和 [config/light_bars.yaml](../config/light_bars.yaml) 配置视觉算法；偏航任务自己的运行配置、板端相机配置和 systemd 模板在 [deploy/rk3588/](../deploy/rk3588/README.md)，不读取 C++ 定位接收器的 `px4_link.yaml`。
- [src/io/](../src/io/README.md) 的 C++ MAVLink 定位诊断链路与本目录的 Python 串口控制相互独立；[src/navigation/](../src/navigation/README.md) 的离线任务意图也未接入这里的执行器。同一串口应由一个进程拥有，不能同时启动 `yaw_follow` 和 `px4_localization_cpp` 或原稿。
- [test/io/](../test/io/) 中的 `test_px4_go_control.py`、`test_yaw_policy.py`、`test_yaw_mission.py` 和 `test_yaw_follow.py` 分别覆盖离线协议、像素策略、任务状态和运行器；[test/CMakeLists.txt](../test/CMakeLists.txt) 统一登记可用回归。

## 原稿与离线控制

`gogogo.py` 是电控提供的正式位置控制原稿；`offboard_go_1.py`、`body_go1.py` 是未完成方案，`heartbeat_test.py` 是原始链路检查。四份原稿保留原样。原稿会在顶层执行设备操作，不要将它们 import 到视觉程序或资源采样器。

新增 `control.py` 将正式原稿的 LOCAL_NED 位置掩码、20 Hz 发送、100 周期预填充、Offboard / ARM / 保持 / LAND 消息处理整理为可导入模块。新增 `offline_control.py` 仅使用 pymavlink 的 common v2 编解码器，控制器与内存 peer 之间传递真实 MAVLink 字节。它不打开串口、UDP 或 socket，不运行 PX4、EKF 或飞行动力学，也不接管真实无人机。

默认本地 NED 原点明确为 `[0,0,0]`，固定目标为 `[0,0,-2]`；配置其他原点时，目标 Z 为原点 Z 减去高度。内存 peer 的位置遥测始终为静态原点，不把收到 setpoint 当作飞机已到达。模块包含单一消息读取入口、system/component 和消息类型检查、心跳超时、有限队列及状态历史。

依赖是 Python 3 和 `pymavlink`；不需要飞控、ROS、Gazebo 或相机。离线调试（6 秒预热后测量 10 秒）：

```bash
python3 -m px4_go.offline_control --seconds 10
python3 -m unittest discover -s test/io -p 'test_px4_go_control.py' -v
```

联测启动协议：

```bash
python3 -m px4_go.offline_control --benchmark-worker
```

进程加载完成输出一行 `{"event":"ready","role":"control",...}`。父进程通过 stdin 发送一行 JSON，字段为 `start_monotonic_s`、`warmup_s`、`measure_s`，时间来自同一台机器的 `time.monotonic()`。建议未来 0.5 秒共同启动、预热至少 6 秒，确保测量开始前完成原稿要求的 5 秒预填充与握手。所有进程应使用相同窗口。

测量结束输出一行 `event=result`，包括成功标志、握手状态、发送数量、实际 Hz、周期 P95/最大值、调度延迟和错过的周期。20 Hz 调度使用单调时钟和绝对 deadline；延误时跳过错过的周期，不补发突发目标。周期 P95 最多保留最后 8192 个测量周期，最大值和错过周期数量覆盖整个测量窗口。结束后的模拟 LAND 握手不属于测量窗口。

输出结果后，worker 阻塞等待 stdin 任意 release 行或 EOF，供父进程完成最终内存采样，然后成功退出 0。`success=true` 要求握手完成、测量阶段实际处于保持状态且完整跑完窗口；短于预填充时间的测试会报告失败。控制资源值包含 CPython、pymavlink、状态机与模拟 peer，不等于真实飞控固件或真实闭环系统的开销。

上述 `control` / `offline_control` 链路没有将识别结果转换为真实飞行指令。其模拟位置不是真实定位，不能将联测成功视为真机闭环验收。

## 无桌面视觉偏航入口

`yaw_follow` 依赖 Python 3.10 及以上、`pymavlink`、PyYAML，以及已在目标板构建好的 `armor_auto`。
按 [RK3588 部署说明](../deploy/rk3588/README.md) 填写本地运行配置和相机 YAML 后，可先只校验配置：

```bash
python3 -m px4_go.yaw_follow --config deploy/rk3588/yaw_follow.json --check-config
python3 -m unittest discover -s test/io -p 'test_yaw_*.py' -v
```

`--check-config` 检查配置、文件及所需 Python 包，不打开硬件，也不启动视觉子进程；示例文件中的占位值必须先替换。
完整启动、安装和自启动步骤见部署说明。

运行时自身位置来自 PX4 `LOCAL_POSITION_NED`，由估计器状态和遥测时效门控。
任务单次起飞后保持起飞位置的本地 XY 和配置高度，只让目标水平居中，不向目标平移；没有目标接近、场地导航或接触动作。
`yaw_follow` 不实现 C++ 定位链路中的 TIMESYNC、外部定位融合或场地坐标对齐。
当前仅完成无硬件回归，尚未实机验收；离线联测和原稿也不能替代这一验收。
