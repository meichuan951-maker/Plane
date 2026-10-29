# 应用入口与运行编排

本目录把输入、视觉、定位和任务模块组合为可执行程序，负责命令行参数、运行循环、
预览/截图、JSON 输出以及退出处理。具体采集由 [input](../input/README.md) 完成，
图像算法放在 [vision](../vision/README.md)，坐标与任务逻辑放在
[navigation](../navigation/README.md)，串口和遥测放在 [io](../io/README.md)。

## 程序从哪里开始

- [main.cpp](main.cpp) → [application.cpp](application.cpp)：`demo_aim_cpp`。
  由 [options.cpp](options.cpp) 解析参数，选择 RGB-D／纯彩色合成或可选 RealSense 输入，
  合成输入使用注入框，RealSense 图像经传统灯条/装甲候选检测转换为 `Detection`，再调用 `Pipeline` 输出观测。
  `--detector-config` 默认读取 `config/light_bars.yaml`。
  [json_log.cpp](json_log.cpp) 写 JSONL，默认每份最多 10 MiB、保留当前文件及 4 份历史，使用同路径排他锁；
  [ros_output.cpp](ros_output.cpp) 提供可选 ROS 2 JSON 发布。日志轮转不影响 `armor_auto` 的独立追加输出。
- [camera_light_test.cpp](camera_light_test.cpp) → [camera/application.cpp](camera/application.cpp)：
  `camera_light_test`。MaixCAM Lite 的 UVC 预览、红蓝灯条/装甲候选、阈值调整和截图入口；默认跳过主机相机控制，通用 UVC 对焦/增益控制可用 `--controls` 显式启用。
- [armor_auto_main.cpp](armor_auto_main.cpp) → [armor_auto/application.cpp](armor_auto/application.cpp)：
  `armor_auto`。读取相机或单张图片，检测、选择和确认目标，输出跟踪与 PnP 测量；
  `--calibrate` 转入 [armor_calibrate.cpp](armor_calibrate.cpp) 的棋盘格采集与标定流程。
- [localization_main.cpp](localization_main.cpp)：可选的 `px4_localization_cpp`。
  读取 PX4 链路 YAML，调用 `io` 处理串口、TIMESYNC、估计器与位姿诊断；
  需在构建时开启 `AIM_WITH_MAVLINK`。
- [mission_demo.cpp](mission_demo.cpp)：`demo_mission_cpp`。
  使用合成位姿、观测和反馈调用 `navigation` 的任务状态机，不连接相机或真实飞控。

两个 UVC 主入口还处理内部 `--camera-worker` 分支，让同一可执行文件作为采集子进程运行；
这部分实现位于 `input/uvc_worker.cpp`，正常使用时由 `CameraStream` 启动。

## 子目录怎样分工

- [camera](camera/)：`application.cpp` 组织相机测试循环、预览绘制和交互，`options.cpp` 解析 CLI，
  `preview.cpp` 实现清晰度评价与对焦扫描，`snapshot.cpp` 保存画面和控制状态。
- [armor_auto](armor_auto/)：`application.cpp` 组织自动识别，`options.cpp` 解析 CLI，
  `output.cpp` 统一判断帧新鲜度和测量有效性、生成/清空 JSON，
  `preview.cpp` 使用同一帧状态绘制预览并保存截图。
- [common](common/)：共享的整数参数解析与 FPS 统计，编译为 `aim_app_common`；
  不承载检测、跟踪或飞控业务。
- [depth](depth/)：目前只有 `output.cpp`，将通用深度平面结果转换为带有效性和坐标说明的 JSON，
  编译为 `aim_depth_output` 供回归使用；当前没有对应的实时深度应用入口。

内部声明与这些路径对应，位于
[include/demo_aim/detail/app](../../include/demo_aim/detail/app/)，而不是混放在源码目录。
新增参数、输出字段或函数时，需要同步检查对应内部头文件和调用方。

## 与其他文件夹的联系

- [config/aim.yaml](../../config/aim.yaml) 配置既有观测链路；
  [config/light_bars.yaml](../../config/light_bars.yaml) 配置灯条测试；
  [config/armor_auto.yaml](../../config/armor_auto.yaml) 配置自动装甲与标定路径；
  [config/px4_link.yaml](../../config/px4_link.yaml) 配置定位通信。
- [tools/maixcam/run_light_test.py](../../tools/maixcam/run_light_test.py) 按 MaixCAM Lite 的 USB 身份查找视频节点并启动相机测试，
  [scripts/armor_auto.sh](../../scripts/armor_auto.sh) 启动自动装甲/标定；
  二者使用根目录 `build/` 下的可执行程序。
- [px4_go/yaw_follow.py](../../px4_go/yaw_follow.py) 启动无窗口 `armor_auto` 子进程并读取它的 JSON。
  Python 入口独立管理飞控连接，C++ `armor_auto` 自身只提供视觉结果；
  板端配置和自启动说明见 [泰山派 RK3566 部署](../../deploy/rk3588/README.md)。
- [test/app](../../test/app/) 验证输出有效性、截图一致性和可选定位 CLI；
  [test/input](../../test/input/) 覆盖相机监管/控制及相关预览逻辑。
  构建与测试登记分别见 [根 CMakeLists.txt](../../CMakeLists.txt) 和
  [test/CMakeLists.txt](../../test/CMakeLists.txt)。
- [docs/interfaces.md](../../docs/interfaces.md) 说明输出字段、时钟、坐标和 [JSONL 轮转约定](../../docs/interfaces.md#demo_aim_cpp-jsonl-文件管理)，
  [runs](../../runs/) 存放日志/预览；[tools](../../tools/README.md) 提供独立的资源测量入口。

## 阅读和修改建议

先找对应主入口，再读相应 `application.cpp`，然后沿调用进入 `input`、`vision`、`io` 或
`navigation`。调试有框但无距离、目标丢失后输出、预览与 JSON 不一致等问题时，
同时查看 `armor_auto/output.cpp` 的 `evaluateFrame`、`makeReport` 和停止清空逻辑。

普通 UVC 识别与 `demo_aim_cpp` 的 `Pipeline` 是独立链路；
`demo_mission_cpp` 只生成 `executable=false` 的离线意图。
不要将任一程序的视觉有效标志理解为已经具备实机导航或接触授权。
构建与运行方法见 [项目 README](../../README.md)，源码全局关系见 [src 总览](../README.md)。
