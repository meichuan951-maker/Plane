# navigation：坐标、定位历史与任务意图

本目录实现“相机中的目标如何落到地图中，以及下一步任务应做什么”。它接收已有的自身位姿和米制视觉观测，不负责相机采集、目标检测、MAVLink 收发或飞控执行。在根 [CMakeLists.txt](../../CMakeLists.txt) 中编译为 `aim_navigation`，依赖公共观测库 `aim_observation`，无需真实飞控即可使用和测试。

## 主要文件

- [geometry.cpp](geometry.cpp)：刚体变换的校验、组合和求逆，四元数与旋转矩阵转换、姿态插值，以及 NED→ENU、FRD→FLU 基变换。接口见 [geometry.hpp](../../include/demo_aim/geometry.hpp)。
- [localization.cpp](localization.cpp)：实现 `PoseSample`、有界 `PoseHistory`、地图对齐和目标投影。历史按观测时刻插值，不向时间范围外推算；无效定位、时间域或定位会话变化会打断历史。接口见 [localization.hpp](../../include/demo_aim/localization.hpp)。
- [mission.cpp](mission.cpp)：根据最新位姿、目标和外部状态，生成起飞、搜索、接近、接触请求、恢复、返航、降落等阶段的意图；处理目标丢失、定位复位、超时和人工接管。接口见 [mission.hpp](../../include/demo_aim/mission.hpp)。

## 与其他目录的联系

- [src/io](../io/README.md) 将 PX4 融合定位转换为 `PoseSample`。本目录用 `PoseHistory` 在图像采样时刻取得对应位姿，并通过 `poseInMap()` 转成地图坐标；定位的 `epoch` 改变后必须重新提供地图对齐。
- [src/vision](../vision/README.md) 的公共 `Solution` 提供目标在相机光学系中的米制点。`projectTargetToMap()` 采用 `map_T_odom × odom_T_body(图像时刻) × body_T_camera × p_camera` 得到 `WorldTarget`。这要求有效距离、安装外参、地图对齐及相容的时钟；二维框或视线方向本身不能生成地图目标点。
- [src/app/mission_demo.cpp](../app/mission_demo.cpp) 组合坐标变换、定位历史和状态机，生成 `demo_mission_cpp`。其中位姿、授权、接触反馈及“路径畅通”都是合成夹具，不是真实传感器或场地校验结果。[src/app/localization_main.cpp](../app/localization_main.cpp) 则仅使用定位历史做串口诊断，未将真实定位接入任务执行。
- [include/demo_aim](../../include/demo_aim/) 保存本目录的公共类型与接口；[docs/interfaces.md](../../docs/interfaces.md) 说明时间域、坐标系、标定和有效性约定。任务参数目前由调用方构造 `MissionConfig`，本目录不直接读取 [config](../../config/) 的 YAML。
- [test/navigation/test_navigation.cpp](../../test/navigation/test_navigation.cpp) 覆盖坐标变换、历史插值、地图投影和任务状态转换，登记入口为 [test/CMakeLists.txt](../../test/CMakeLists.txt)。
- [px4_go](../../px4_go/README.md) 是独立的 Python 控制代码。当前没有将本目录的 `MissionIntent` 自动交给它执行，也没有把独立地图工程的规划器接成 `segment_clear` 校验器。

## 当前边界与阅读顺序

`MissionIntent` 及其 JSON 的 `executable` 始终为 `false`。状态机要求调用方提供健康与落地状态、路径整段校验、接触授权、区域确认和接触反馈；它没有实现真实 Offboard 执行器、接触控制器或比赛规则数据接入。`WorldTarget` 的协方差输出目前仍为空。

建议先读公共接口 [geometry.hpp](../../include/demo_aim/geometry.hpp) → [localization.hpp](../../include/demo_aim/localization.hpp) → [mission.hpp](../../include/demo_aim/mission.hpp)，再对照本目录实现，最后查看 [mission_demo.cpp](../app/mission_demo.cpp) 和导航回归。工程构建、离线运行命令见 [根 README](../../README.md)。
