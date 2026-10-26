# C++ 源码目录

本目录存放当前工程的 C++ 实现，负责相机输入、视觉测量、定位通信、坐标变换和应用编排。
公共头文件统一放在 [include/demo_aim](../include/demo_aim/)，应用和采集的内部声明放在
[include/demo_aim/detail](../include/demo_aim/detail/)；修改接口时应同时查看对应头文件。

## 一级目录各自负责什么

- [app](app/README.md)：程序入口和运行循环，负责解析参数、组合各模块、显示预览及输出 JSON。
  查找某个可执行程序从哪里启动、如何使用配置、如何处理退出时，从这里开始。
- [input](input/README.md)：输入适配，包括合成数据、可选 RealSense，以及普通 USB/UVC
  相机的采集子进程、共享帧和硬件控制。负责拿到图像，不负责识别目标。
- [vision](vision/README.md)：视觉算法和观测契约，包括传统红蓝灯条/装甲配对、跟踪、标定/PnP、
  观测流水线和通用深度平面测量。
- [io](io/README.md)：遥测缓存、串口和可选 MAVLink 接收与时间同步，将飞控消息整理为状态和定位输入。
- [navigation](navigation/README.md)：几何变换、定位历史、相机目标到地图的投影及离线任务状态机。
  生成任务意图，不直接向飞控发送运动指令。

## 模块怎样连接

当前代码保留几条独立入口，不能把它们理解成已经连通的完整飞行系统：

1. **普通 USB 视觉**：`app/camera_light_test.cpp` 或 `app/armor_auto_main.cpp` 启动应用，
   通过 `input` 的 `CameraStream` 获取图像，再调用 `vision` 的灯条检测、装甲跟踪和 PnP。
   `app` 负责画面、截图与测量输出。这条链路不经过 `vision/core.cpp` 的 `Pipeline`。
2. **既有观测流水线**：`app/main.cpp` → `app/application.cpp`，从 `input` 选择
   synthetic/synthetic-rgb/realsense；合成输入直接注入检测框，RealSense 图像经 `vision/light_bar_observation.cpp` 复用灯条/装甲候选检测，
   再交给 `vision/core.cpp`。`--detector-config` 默认读取 `config/light_bars.yaml`。
   `aim_core` 保留对遥测缓存库 `aim_flight_state` 的公共链接依赖，但当前 `Pipeline` 未使用飞控遥测，
   此应用也没有启动 MAVLink 串口接收器。
3. **PX4 定位诊断**：`app/localization_main.cpp` 调用 `io` 的串口/MAVLink 实现，
   借助 `navigation` 整理定位、时钟和复位状态。需要开启 `AIM_WITH_MAVLINK`。
4. **离线任务演示**：`app/mission_demo.cpp` 向 `navigation` 提供合成位姿、目标和反馈，
   验证地图投影及任务阶段；输出 `executable=false`，没有飞控执行器或真实飞行动力学。

因此，`armor_auto` 的 JSON、`demo_aim_cpp` 的观测和 PX4 位姿仍需适配时间、坐标与有效性，
才能组合为新的闭环入口。接口细节见 [数据与坐标契约](../docs/interfaces.md)。

## 与源码目录之外的联系

- [CMakeLists.txt](../CMakeLists.txt) 将本目录编译为 `aim_*` 库和应用；
  [scripts/build.sh](../scripts/build.sh) 是统一构建入口，产物位于根目录 `build/`。
- [config](../config/) 存放视觉阈值、装甲尺寸/标定引用和 PX4 链路配置，由应用加载后交给对应模块。
- [scripts](../scripts/) 包装构建和启动命令；[tools](../tools/README.md) 复用视觉库做资源测试、
  录像联测和相机诊断，不是本目录应用的替代实现。
- [px4_go](../px4_go/README.md) 是独立的 Python 控制与离线联测代码。
  其中 `yaw_follow` 以子进程方式读取 `armor_auto` 输出并自行管理 MAVLink，未接入 C++ 任务状态机。
- [test](../test/) 按 core、vision、input、app、navigation、io 等职责验证这里的模块；
  测试目标集中登记在 [test/CMakeLists.txt](../test/CMakeLists.txt)。
- [runs](../runs/) 保存运行输出；[archive](../archive/) 保存已退出主线的相机实现和历史资料，
  不应据归档入口判断当前功能。

运行命令、可选依赖与当前验收边界见 [项目 README](../README.md)。
