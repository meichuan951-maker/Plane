# 视觉无人机 Demo（C++17）

面向 Orange Pi 5 8GB / Ubuntu 22.04 的视觉与导航开发工程。当前开发主线是
**YLR4K USB/UVC 相机 + 传统红蓝灯条检测 + 装甲板跟踪 + 相机标定/PnP**，通过普通 UVC 接口采集。
定位接入方案为 **MicoVIO → PX4 → 香橙派**，不使用 GPS；目前仍需独立完成实际融合与导航集成验收。

仓库：[meichuan951-maker/Plane](https://github.com/meichuan951-maker/Plane)。文档更新：2026-10-26。

运行命令见本页。开发顺序与验收目标见 [视觉无人机路线图](docs/visual_drone_roadmap.md)，
数据/坐标/时间契约见 [接口说明](docs/interfaces.md)，选型与安装标定见 [硬件说明](docs/hardware.md)。
2027 场地、仿真入口与两个工程的接口边界见 [RMUC 2027 集成说明](docs/rmuc2027_integration.md)。
Astra Pro 专用程序、配置、启动脚本和历史记录已移至 [相机归档](archive/astra_pro_20261023/README.md)，
原使用说明见 [历史用法与实验记录](archive/astra_pro_20261023/USAGE_legacy.md)。
MaixCAM 预览、装甲识别、UVC 脚本及专用测试已移至 [MaixCAM 归档](archive/maixcam_20261025/README.md)，不再作为当前入口或参加默认回归。

## 规则依据与当前边界

当前空中系统的规则依据是 `test_1/` 中 2026-10-15 发布的
[《RoboMaster 2027 超级对抗赛比赛规则手册 V1.0.0》](test_1/RoboMaster%202027%20机甲大师超级对抗赛比赛规则手册V1.0.0（20261015）.pdf)。
同目录的高校联盟赛**参赛手册**涉及参赛与技术评审，不替代超级对抗赛的比赛规则；`test_1/` 中旧版变更前瞻只作历史资料。

- §3.2.1，印刷第 27 页：空中机器人全自动运行，配备急停遥控器；多机通信仅通过机库间接进行。当前 PX4 串口是机内诊断链路，尚未实现机库多机通信。
- 表 5-2，印刷第 65 页：空中机器人冲击对基地装甲、前哨站中部装甲有对应的伤害项目，机器人装甲和飞镖检测模块不应混为同一类目标。灯条检测尚未识别这些物理模块身份。
- R47/R48，印刷第 131 页：准备阶段不得启动桨叶，飞行不得出战场。`no_demo` 尚未接入比赛阶段输入、场地围栏或禁区地图，不能将任务模拟的 `segment_clear=true` 夹具用于真实场地。独立地图工程的静态边界也不代替比赛禁区和授权。
- R62/R64，印刷第 134 页：能量机关、机库不是允许冲击的目标，并有飞镖发射站禁区约束。当前红蓝颜色和临时跟踪号不能证明目标类别或区域合法。
- §5.1.1，印刷第 64 页：空中机器人冲击的接触时间、面累积冲量仍标“待定”。不根据旧前瞻或图纸填入确定的冲击阈值。

任务模块目前只生成 `executable=false` 的离线意图，外部授权、目标区域确认和路径验证仍是调用方输入，
并非已经接入裁判系统。继续以地面感知、定位和目标前停住为本阶段验收目标。

## 当前目标与开发顺序

第一阶段目标：**导航到目标附近 → 搜索 → 确认装甲板 → 精细对准 → 在目标前稳定停住。**
接触、恢复、返航和降落放到后续阶段。

1. 地面验证真实画面、帧率、灯条配对和自动跟踪。
2. 标定当前相机，分别验证目标中心偏角、板面朝向和米制距离。
3. 独立验证自身定位与 PX4 融合；这一步可与地面视觉并行。
4. 联合离线验证与闭环仿真，验证目标丢失、观测延迟、定位失效和退出行为。
5. 再进入真机有限对准与接近，逐步验收到目标前停住。

绿色灯辅助搜索、短焦广角镜头仍处于方案讨论阶段，**尚无绿色灯检测实现或可靠最远距离结论**。
图纸中的绿色灯属于飞镖检测模块，不等同于装甲板中心或接触点。
视觉引导阶段由视觉改变导航参考，PX4 仍负责飞行控制，不能跳过自身状态估计。

## 当前能做什么

- `camera_light_test`：普通 USB/UVC 相机真实画面、实测处理帧率、传统红蓝灯条候选提取、阈值调节和截图；不需要厂商 SDK。
- `armor_auto`：按颜色自动选择装甲板、连续确认和临时目标跟踪；提供棋盘格标定入口，标定与尺寸有效时输出 RGB 相机系 PnP 位置、距离、板面朝向和目标中心偏角。
- `demo_aim_cpp`：合成和可选 D435i 输入的观测流水线；实时图像复用传统灯条/装甲候选检测，再进行关联与深度质量检查。
- `px4_localization_cpp`：可选 MAVLink UART 接收心跳、姿态、位置、融合 ODOMETRY，完成 TIMESYNC 与估计器状态检查。
- `demo_mission_cpp`：合成位姿和目标经过实际坐标变换、任务状态机，走完起飞、搜索、接近、接触反馈、返航和降落的离线流程。
- `benchmark_vision_control.py`：指定录像上的传统视觉与 `px4_go` 离线控制并行测量 CPU、PSS/RSS、帧率和控制周期；不连接真实飞控。
- `px4_go.yaw_follow`：新增待实机验收的无桌面视觉偏航入口，接收 PX4 本地定位，单次起飞后保持位置/高度并使目标水平居中；部署见下文。

任务输出 `executable=false`，`demo_aim_cpp` 观测输出 `contact_permitted=false`。既有 C++ 任务尚未接入实机 Offboard 执行器、
接触控制器或精确进库降落。新增 Python 偏航入口独立管理视觉和 MAVLink，不执行目标接近或场地导航，尚未实机验收。
MicoVIO 软件适配尚未经过真实模块/PX4 融合验收；C++ 定位诊断程序唯一发送消息为 TIMESYNC。

独立的 [rmuc_gazebo_map](../rmuc_gazebo_map/README_zh-CN.md) 已切换到完整 2027 地图，
现用163 g微型机直接停在所选红／蓝方原机库表面，提供键盘操控与100 mm低空完整航线；
旧X500/PX4仿真入口已归档，现用入口不加载旧机型。
此前地面起飞和人工托台的静态绕障结果属于历史任务，均未包含本工程的视觉观测或任务意图；
`demo_mission_cpp` 仍是独立离线夹具，`executable=false` 的含义保持不变。

普通 USB 视觉由 `camera_light_test` 和 `armor_auto` 独立入口提供，尚未接入 `demo_aim_cpp` 的观测流水线。
`track_id` 仅为临时跟踪编号；装甲数字身份识别和多目标身份保证尚未实现。
PnP 测距已有代码，但尚未完成实机精度和 8～10 m 可靠性验收；RGB-D ROI 点也不等于指定接触点。
仓库未附带当前相机的 RGB 标定结果。没有本相机标定时仍可画框，数值角度/距离不可用，不能套用其它相机参数。

## 目录和阅读顺序

目录职责与相互关系见 [src 源码总览](src/README.md)、[px4_go 电控说明](px4_go/README.md)
和 [tools 工具说明](tools/README.md)；`src` 各一级模块的独立 README 可从源码总览进入。

```text
no_demo/
├── README.md                 # 运行入口和能力边界
├── CMakeLists.txt            # 应用/库构建和可选依赖，回归登记委托给 test/
├── cmake/                   # 唯一构建目录约束和禁用目标归档
├── include/demo_aim/         # 稳定公共 *.hpp 接口
│   └── detail/              # 内部 *.hpp，按对应源码的模块路径组织
│       ├── app/             # 应用编排、参数、输出、预览与公共辅助
│       └── input/           # UVC 采集与内部进程协议
├── src/                     # 仅存放 *.cpp 实现，目录仍按职责归组
│   ├── vision/              # 检测、观测、关联和可选测距
│   ├── input/               # UVC 采集/控制/进程与 synthetic/realsense
│   ├── navigation/          # 坐标、定位历史和任务状态机
│   ├── io/                  # 遥测缓存、MAVLink、UART
│   └── app/                 # 薄入口、camera/、armor_auto/ 与通用输出模块
├── config/                  # 灯条、自动装甲、观测与 PX4 链路配置
├── scripts/                 # 统一构建、UVC 视觉启动与外部地图入口
├── tools/                   # 视觉/电控资源测试与相机诊断工具
├── px4_go/                  # 电控原稿、整理后的控制模块与无硬件离线 worker
├── test/                    # 自动化回归，构建入口为 test/CMakeLists.txt
│   ├── core/                # 观测流水线、合成输入与遥测缓存
│   ├── vision/              # 灯条/装甲、标定/跟踪与深度平面
│   ├── input/               # UVC 控制与模拟取流进程
│   ├── app/                 # 输出有效性、预览与截图一致性
│   ├── navigation/          # 坐标、定位历史与任务状态机
│   ├── io/                  # 可选 MAVLink 编解码和伪终端串口
│   ├── build/               # 构建脚本与禁用目标归档回归
│   └── support/             # Release 下仍执行的断言和具名用例统计
├── docs/                    # 开发路线、接口、硬件与联测说明
├── archive/                 # 退出主线的旧相机程序与历史用法
├── test_1/                  # 规则手册和硬件参考资料，非自动化测试
├── runs/                    # 日志、预览及旧构建归档，不提交生成输出
└── build/               # 统一构建目录，不提交生成文件
```

从 `src/app/camera_light_test.cpp` → `src/app/camera/` 阅读 USB 预览和阈值测试；
从 `src/app/armor_auto_main.cpp` → `src/app/armor_auto/application.cpp` →
`src/vision/armor_tracker.cpp`、`armor_pose.cpp` 阅读自动装甲链路；
从 `src/vision/depth_plane.cpp` 和 `src/app/depth/output.cpp` 阅读通用深度平面算法与输出契约；
从 `src/app/armor_calibrate.cpp` → `src/vision/camera_calibration.cpp` 阅读相机标定；
从 `src/app/main.cpp` → `application.cpp` → `src/vision/core.cpp` 阅读既有观测链路；
从 `src/app/localization_main.cpp` → `src/io/mavlink_link.cpp` 阅读定位接收；
从 `src/app/mission_demo.cpp` 阅读坐标和任务模块的组合方式。
公共 include、可执行名称、CLI 和配置路径保持兼容，源码物理目录按职责归组。
头文件与实现分离的整理范围为当前 `no_demo` 应用工程；PX4 和第三方 SDK/生成库沿用各自布局。
公共接口继续使用 `demo_aim/core.hpp` 等路径；内部头位于 `include/demo_aim/detail/`，
保留 `app/`、`input/` 及子模块路径，例如 `demo_aim/detail/app/camera/options.hpp`，避免同名头混淆。
`include/` 是统一头文件搜索根；`detail/` 供工程内部使用，不属于稳定公共接口，C++ 命名空间保持原有划分。
`camera/`、`armor_auto/` 分别负责 UVC 预览和自动装甲的采集循环、参数与预览；`armor_auto/output.cpp`
集中生成/清空测量 JSON，`common/` 只复用整数解析和 FPS 统计。通用深度平面算法、`depth/output.cpp`
及其无硬件回归保留，RGB 与深度的坐标、标定和有效性规则独立维护。
`aim_camera_stream` 复用进程监管和 UVC 控制，`aim_uvc_worker` 提供 RGB 采集。
完整规则和旧版变更前瞻 PDF 均位于 `test_1/`；此目录存放参考资料，自动化回归源码在 `test/`。
测试依赖和 CTest 登记集中在 `test/CMakeLists.txt`；生产代码不依赖 `test/support/`。
直接 CMake 构建的测试程序输出到 `build/aim_*_tests`；`colcon` 构建时位于 `build/no_demo_aim_cpp/`，
原有 CTest 名称和可执行文件名保持兼容。
本轮迁移将旧 `build/`、`build_cpp/` 备份到 `runs/single_build_migration_*/build_directories.tar.gz`，
同目录保存文件校验清单和迁移前的功能配置。
后续本地构建只使用根目录 `build/` 及其子目录；`colcon` 使用 `build/no_demo_aim_cpp/`，
不再创建并列的 `build_cpp/`、`build_debug/` 等目录。关闭可选功能时，旧产物保留在对应构建目录的 `disabled/`。
CMake 缓存包含构建目录的绝对路径，因此迁移时重新配置，不能直接把旧目录改名后沿用缓存。
独立地图工程 `../rmuc_gazebo_map` 不属于本项目 CMake 构建。

## RMUC 2027 地图与仿真

现用无人机统一为 **163 g、130 × 130 × 130 mm** 微型机，含护桨、起落架与接触头，地图使用完整 `RMUC2027_V1.0.0.STL`。
旧 X500 尺寸检查机及 `inspect / px4 / navigation` 专用启动入口已归档，详见[归档说明](../rmuc_gazebo_map/archive/legacy_aircraft_20261019/README.md)。本项目真实飞控的台架通信功能不受此仿真模型整理影响。

先按[地图工程 README](../rmuc_gazebo_map/README_zh-CN.md)构建 Gazebo 插件，然后在 `no_demo` 根目录运行：

```bash
./scripts/rmuc2027_sim.sh
./scripts/rmuc2027_sim.sh keyboard --team blue
./scripts/rmuc2027_sim.sh route --team both
./scripts/rmuc2027_sim.sh check
```

不加命令时显示菜单：`1` 键盘操控、`2` 航线导航、`q` 退出。键盘模式按 R/F 起降、WASD 移动、Q/E 转向；升高100 mm后开放横移，松键减速。1/2/3切换速度，C仅瞬时复位。
航线模式自动完成100 mm起飞、向己方基地侧退让、低空绕中央机关、对方前哨顶部接触（10 N、3 s）、退出安全距离、重新规划返航和原位物理降落。接触失败或超时也会安全返航。

统一入口只允许同时打开一个仿真窗口，切换前先关闭当前窗口。两种模式支持 `--check` 和 `--start-paused`；航线支持 `--headless`、`--team both` 与 `--case contact-timeout`。地图不在同级目录时设置 `RMUC_MAP_ROOT`。

微型机显示对象由208个合并至12个，保留全部208个物理碰撞体、质量、重心、惯量和七颗阵营灯。模型与两种模式共享同一份参数，不加载归档机型。
统一入口默认自动检测OpenGL，在CPU软件渲染时尝试现有NVIDIA显卡；可显式加 `--renderer nvidia`。实际结果见输出目录的 `renderer.json`。地图查询缓存限额64 MiB，规划结束后回收驻留页，保持原碰撞几何和物理步长。
这些是独立仿真：键盘采用速度控制，航线平移采用运动学回放，接触采用外力与物理求解，尚未接入本项目视觉目标或真实飞控闭环。
完整输出与模型维护见[集成说明](docs/rmuc2027_integration.md)和[航线接触说明](../rmuc_gazebo_map/docs/OUTPOST_CONTACT.md)。地图工程的回归数量独立于下文 `no_demo` CTest。

## 编译和回归

需要 CMake ≥3.16、C++17 编译器、OpenCV 4（core/imgproc/imgcodecs/highgui/videoio/calib3d）、
yaml-cpp、nlohmann-json 和线程支持。RealSense SDK、ROS 2、MAVLink 为可选依赖。
全部本地构建统一使用 `build`，推荐通过统一脚本构建并运行回归：

```bash
cd /home/user/nohuman_plane_std/no_demo
./scripts/build.sh --test
./build/demo_aim_cpp --help
./build/camera_light_test --help
./build/armor_auto --help
```

已有 `build/CMakeCache.txt` 时保留其中的功能开关与依赖路径；若原配置已开启 ROS 2，
执行前先运行 `source /opt/ros/humble/setup.bash`。首次配置默认 Release，
三个可选后端 RealSense、ROS 2、MAVLink 全部关闭。`JOBS=2` 是默认并行度，
可用 `JOBS=4 ./scripts/build.sh --test` 调整；不加 `--test` 只构建。

明确切回普通 USB 视觉与无硬件基础配置：

```bash
./scripts/build.sh --test -- \
  -DAIM_WITH_REALSENSE=OFF -DAIM_WITH_ROS2=OFF \
  -DAIM_WITH_MAVLINK=OFF
```

`--` 后只接受 CMake 的 `-D`、`-U` 缓存选项；不接受 `-S`、`-B`、`-G`，避免再次生成第二套构建目录。
直接运行 CMake 允许项目根目录的 `build/` 或其子目录；Debug、Release 和可选功能在各自入口的原目录重新配置，
例如 `./scripts/build.sh -- -DCMAKE_BUILD_TYPE=Debug`，不另外创建 `build_debug/` 或验证目录。
完整可选功能验证也使用同一个 `build`，按后文提供 SDK／MAVLink 路径并开启相应开关；
ROS 2 开启前先加载对应环境。关闭功能时，CMake 配置阶段将已知的旧可执行文件和库按内容哈希归档到
各自构建目录的 `disabled/<SHA256>/`，避免入口误用旧版本；再次开启会在正常构建位置重新生成。

手动构建统一使用 `cmake -S . -B build`、`cmake --build build -j2` 和
`ctest --test-dir build --output-on-failure` 命令；推荐脚本以统一配置、构建和回归入口。手动 CMake 配置也会归档关闭功能的残留产物。
直接调用 CMake 时 `AIM_WITH_REALSENSE` 的原生默认值仍为 ON，SDK 缺失时跳过实时后端；
其余可选功能默认 OFF。YLR4K 预览和检测使用普通 UVC 接口，无需厂商深度 SDK。
后文按需重新启用相关选项；`demo_aim_cpp --help` 显示实际 RealSense/ROS 编译能力。
x86_64 开发机程序不能直接复制到 ARM64 香橙派运行；`scripts/` 的库搜索路径也需按板端环境调整。

也可以直接使用 `colcon` 构建本 CMake 包，不需要开启 ROS 2 功能：

```bash
colcon build
ctest --test-dir build/no_demo_aim_cpp --output-on-failure
./build/no_demo_aim_cpp/demo_aim_cpp --help
```

`colcon` 将该包的缓存、目标文件和程序放在 `build/no_demo_aim_cpp/`，顶层仍只有一个 `build/`。
它默认另外生成 `install/`（安装产物）和 `log/`（构建日志），这两者已加入 Git 忽略规则。
上述回归命令要求 `BUILD_TESTING=ON`（默认开启）。`colcon` 与 `scripts/build.sh` 使用不同缓存，
功能开关需分别配置；需要 ROS 2 时先加载环境，再执行 `colcon build --cmake-args -DAIM_WITH_ROS2=ON`。
`scripts/` 中的应用启动入口仍使用直接 CMake 构建的 `build/` 路径；使用 `colcon` 时按上例从包子目录启动程序。
所有构建目录均检查真实路径，不允许通过符号链接将构建输出移到项目 `build/` 之外。

`BUILD_TESTING=ON` 时登记基础回归，启用 MAVLink 时还会登记协议、串口和定位 CLI 回归。
当前数量以 `ctest --test-dir build -N` 为准，不等于测试程序内部断言数量。
另外，有 Python 时增加资源采样和联测工具 2 项；有 pymavlink 时增加离线控制 1 项，
有 Python OpenCV / NumPy 时增加录像 worker 1 项，均使用 `benchmark` 标签筛选。
覆盖灯条提取/配对、姿态多解、自动装甲配置/跟踪/标定、深度平面、观测、坐标/任务、CLI 和 UVC 取流/控制。
取流回归用独立进程模拟打开设备卡住、帧发布卡住、控制卡住、持锁进程退出、断流后恢复，以及扫焦期间持续预览；无需真实相机。
`aim_build_tools_regression` 检查统一构建入口的缓存保留、参数约束和禁用目标归档。
可用 `ctest --test-dir build -L vision --output-on-failure` 只运行视觉回归；
其他分组标签为 `core`、`input`、`app`、`navigation`、`io`、`build`，`cli` 可筛选帮助页、离线演示和伪终端 CLI 验收。
`aim_localization_cli_regression` 用伪终端验证静默/其他设备报文退出码、诊断与位姿验收、最终位姿过期和 Ctrl+C；不访问飞控。
测试通过 `include/` 搜索根引用 `demo_aim/detail/...` 内部头；不再把 `src/` 加入头文件搜索路径。
这些是开发机软件回归，不是相机识别准确率、板端实时性或真实飞行验收。

`aim_app_output_regression` 无需相机或厂商 SDK，覆盖 RGB/深度输出时效、停止清空、角度含义和坐标映射。
JSON 与预览使用处理完成后的帧龄；即使缓存中保留有效姿态，帧超过 250 ms 后也不输出有效测量。
`aim_armor_snapshot_regression` 检查退出截图使用同一帧的原图与重新生成的叠加图，并检查过期/故障状态及覆盖保护。
定位历史、地图变换与任务层复用 `PoseSample::validate()` 检查元数据和几何完整性；任务在非降落阶段意外着陆后中止并锁存。

切换前的构建验证与旧相机现场记录保存在[历史用法与实验记录](archive/astra_pro_20261023/USAGE_legacy.md)。
软件回归不作为相机测量精度、板端性能或飞行验收结论。

## YLR4K USB 相机：实时画面与红蓝灯条

参考 `sp_vision_25` 的独立相机/传统检测测试分层，以及传统识别器中亮度提取、颜色判断和装甲几何检查的分层，
自行实现 UVC 输入、亮度与 HSV 颜色投票、几何候选配对和独立姿态估计。
相机通过普通 UVC 接口采集，目标检测使用灯条颜色与几何关系。
普通 UVC 相机通过 Linux `uvcvideo`、V4L2 和 OpenCV 采集；
驱动接口见 [Linux UVC 文档](https://cdn.kernel.org/doc/html/latest/userspace-api/media/drivers/uvcvideo.html)。
当前使用 YLR4K 相机模块，通过普通 UVC 彩色接口接入。通用 CLI 默认请求 640×480、30 FPS、MJPG；
2026-10-25 已重新核验本机 Lombo `27c4:0531`，彩色节点为 `/dev/video4`，专用启动脚本使用稳定的 by-id 路径。
历史 YLR4K（Lombo `27c4:0531`）测试记录保留在本节末尾，日期和适用范围保持原记录。

本机当前已验证的预览配置（60 FPS 目标尚未实现）：

```bash
./scripts/ylr4k_live.sh          # 1920×1080、MJPG、30 FPS，只开主预览
./scripts/ylr4k_live.sh --mask   # 需要调灯条阈值时再开掩膜窗口
```

用户提供的商品参数表将 YLR800-4K、YLR1200-4K 都标为“最高 60 帧/秒”，
但未注明对应分辨率、固件或启用步骤。本机 USB 名称仅为 `YLR4Kusb camera`，
尚不能据此区分这两个具体型号，也不能把当前 UVC 枚举上限当作传感器的硬件上限。
该设备当前 MJPG、YUYV、H264 的 UVC 枚举模式均最高 30 FPS，降低分辨率也未枚举出 60 FPS 档位。
640×480 请求 60 FPS 后驱动实际协商为 30 FPS，300 帧纯采集实测 30.01 FPS；
1280×720 和 1920×1080 MJPG 的 300 帧纯采集均约 30.02 FPS。
补充绕过 OpenCV、检测与显示的驱动直采：640×360 下分别用 MJPG、YUYV、H264
请求 60 FPS，每种跳过 15 个初始缓冲后采集 210 个缓冲，驱动均返回 30 FPS，未得到 60 FPS 输出。
USB 原始描述符最短帧间隔也是 333333（100 ns 单位，约 30 FPS），没有 166666/166667 的 60 FPS 档。
进一步核对 [Linux UVC 驱动源码](https://github.com/torvalds/linux/blob/v6.8/drivers/media/usb/uvc/uvc_v4l2.c)：
`uvc_v4l2_set_streamparm` 会先用 `uvc_try_frame_interval` 按描述符调整请求周期，之后才向设备发送 PROBE。
因此单凭上述 V4L2 测试还不能判断设备自身如何响应直接的 60 FPS 协商。
随后已由用户执行管理员权限下的标准 UVC PROBE 测试，绕过 Linux 的帧间隔调整：
MJPG 的 640×360、1280×720、1920×1080 都直接请求 166667（约 60 FPS），
设备均返回 333333（约 30 FPS）；相应格式与分辨率索引正确，没有接受 60 FPS 请求。
原 PROBE 已恢复，两个接口均重新绑定驱动，恢复报告没有错误，详见本机 `direct_probe.json`。
为排除取整边界，又按 Linux `v4l2_fraction_to_interval(1, 60)` 的整数除法结果补测 166666，
与前一轮 166667 相差 100 ns。640×360、720p、1080p 的设备响应仍全部为 333333，
原 PROBE 与两个驱动接口再次恢复成功，没有恢复错误。
两份完整结果分别保存在 `direct_probe_166667.json`、`direct_probe_166666.json`，均位于上述本机测试目录。
这排除了仅由 Linux 描述符调整或这两个帧周期取整值造成回退的解释，
确认本机当前配置在这三档 MJPG 标准协商下均未接受 60 FPS；不能据此断言传感器硬件最高只有 30 FPS。
尚未找到与本机型号匹配的官方高帧率固件或扩展控制协议；需卖家明确该订单型号的
60 FPS 分辨率、输出格式、开启步骤及匹配本机的原厂工具/固件。
1920×1080 的主窗口加灯条检测连续验证 600 帧，稳定阶段 29.8–30.2 FPS，
计入启动等待的全程平均 29.20 FPS，首帧等待 0.53 秒。
目前暂用已验证的 1080p/30 FPS 预览；V4L2 以及上述直接标准 UVC PROBE 请求均回退到 30，60 FPS 目标尚未实现。
原始枚举、测试日志和截图位于本机 `runs/ylr4k_tuning_20261025/`。
这次只验证当前场景的吞吐与预览，光学对焦、照明和真实灯条识别效果仍需现场摆靶确认。

`tools/probe_ylr4k_60fps.py` 用于上述直接标准 UVC 帧率协商测试，默认仅输出只读计划。
166667 和 166666 两轮诊断均已完成，目前无需重复执行。
后续取得新固件或模式信息后需重测时，先按 q 关闭相机预览，再在项目目录运行：

```bash
sudo /usr/bin/python3 -I -B tools/probe_ylr4k_60fps.py --apply \
  | tee runs/ylr4k_tuning_20261025/direct_probe_166666.json
```

该诊断需要管理员 USB 访问，限当前已校验布局的 `27c4:0531` 相机，测试 MJPG 的 640×360、720p、1080p。
它只做临时 PROBE 协商，随后恢复原协商与驱动，不提交或启动视频传输。
脚本会拒绝仍被占用的设备；接受 60 FPS 协商仍需后续真实采集验证，不能当作已经达到 60 FPS。
诊断结束后可用 `./scripts/ylr4k_live.sh` 恢复已验证的预览。

编译后在**连接相机的桌面终端**执行，先关闭 ffplay/其他预览程序：

```bash
cd /home/user/nohuman_plane_std/no_demo
./build/camera_light_test --list
# /dev/video0 仅为示例，替换成上一步找到的彩色 Video Capture 节点。
./build/camera_light_test --device /dev/video0 \
  --width 640 --height 480 --fps 30 --format MJPG --config config/light_bars.yaml --mask
```

同一相机可能暴露多个节点，插拔后编号可能变化。安装了 `v4l2-ctl` 时可用
`v4l2-ctl --list-devices` 和 `v4l2-ctl -d /dev/video0 --all` 确认 Video Capture；
优先使用 `/dev/v4l/by-id/` 下实际存在的稳定路径。
默认灯条参数为 `config/light_bars.yaml`，应根据 YLR4K 的真实曝光与画面重新调整。

默认请求 MJPG、640×480、30 FPS；使用 `--width`、`--height`、`--fps`、`--format MJPG|YUYV` 调整。
设备不一定接受请求参数，启动日志显示请求值和驱动协商值，低于请求帧率时明确警告，叠加显示实际图像尺寸。
FPS 是最近约一秒完成的“采集+检测+显示”吞吐，首次统计前显示 0，不是传感器帧率或端到端延迟。
`detect ms` 只统计灯条算法耗时。带控制的预览在第一帧后应用设备支持的预览默认值：自动曝光、增益、数字锐度和连续自动对焦；不等待亮度稳定或自动扫描整个焦点范围。第一帧到达即开始预览，终端报告首帧等待时间。`--no-controls` 和无窗口模式跳过这些控制写入。不自动修改 USB 电源设置，USB 超时也不能直接认定为自动休眠。

取流和控制在独立受监控子进程中执行，主进程负责窗口、检测及按键。共享内存只保留最新一帧，慢消费者不会积压旧帧。连续 3 秒没有新帧会终止本轮采集并重试，默认恢复预算为 15 秒（加上至多一次 3 秒设备等待）；`--reconnect-seconds N` 可调整，0 表示失败即退出。相机物理掉线或固件无响应时，仍可能需要重新插接/换线，软件重试不保证恢复设备。等待超过 250 ms 时显示等待提示并清空旧轮廓/掩膜；等待、重连和驱动卡住期间均可按 q/Esc 或 Ctrl+C 退出。不会无限等待子进程释放失效的设备。

主窗口有 Focus 和 Gain。把装甲放在工作距离上，按 `f` 才开始手动扫焦，按 `a` 启用相机自身连续自动对焦。扫焦由正常预览的新帧逐步驱动，窗口持续刷新，拖动 Focus 或按 a/e 可取消扫描。每个位置读回确认后等待至少 350 ms，再取三帧中心清晰度的中值；扫描包含范围两端。拖动滑块的请求节流到最多约每 100 ms 一次；失败给出警告并恢复读回值。对焦读回正确只证明驱动接受参数，不证明镜头运动或已经对准目标。

历史 YLR4K 冷启动时曾出现黑帧。如确实全黑，可按 `e` 尝试最长快门后恢复自动曝光；该操作按需执行，不再拖慢默认启动。`--no-controls` 可运行带窗口的纯取流预览，跳过曝光/焦点/增益查询和写入，用于区分取流与控制故障。无窗口测试也不改相机控制。

画面发虚时，先固定相机、增加目标照明，检查镜头保护膜和污渍。比较约 1 m 和实际工作距离下的黑白文字，缓慢扫描 Focus 的完整范围；若读回数值变化而画面始终不变，继续核对镜头是否支持电动调焦，排查调焦机构、模组固件或机械焦点，不能仅凭滑块认定已对焦。Gain 和灯条阈值不能恢复失焦细节。
历史 YLR4K 驱动报告 MJPG 支持 1920×1080/30 FPS。设备确实支持该模式时，可用以下命令比较原始细节；实际协商尺寸仍以启动日志为准，提高分辨率不解决光学失焦：

```bash
./build/camera_light_test --device /dev/video0 --width 1920 --height 1080 --fps 30 --format MJPG
```

- 正常取流时显示真实画面，红/蓝轮廓和端点表示相应颜色的细长候选，顶部给出灯条和装甲数量；断流期间显示等待提示，不沿用旧检测结果。
- 同色、近似平行的两根灯条经长度比、中心距、纵向偏移和矩形一致性筛选，绿色四边形表示几何装甲候选（左上、右上、右下、左下）。只有最靠近画面中心的一块显示 yaw/pitch。
- yaw 为正表示装甲法线偏向画面右侧（左侧灯条更近），pitch 为正表示法线偏向上方（下沿更近）。正对相机且板面平行于像平面时两者为 0。这是装甲朝向，不是飞控指令。
- `--mask` 显示二值图（左红、右蓝），Min V 调亮度门限，Min S 调颜色投票的饱和度门限；变化即时生效。
  默认亮度模式的掩膜展示通过颜色和灯条形状筛选的完整亮轮廓；兼容 HSV 模式展示颜色阈值掩膜。
  `min_saturation`（Min S，常简称 min_s）范围 0～255，越高越要求颜色鲜艳；
  它不表示亮度或最小面积。`min_value`（Min V）才控制亮度门限。
- `s` 保存原图、叠加图、两张掩膜和当时的 `light_bars.yaml` 到 `runs/camera_snapshots/` 下的新目录。
- `q`、Esc 或 Ctrl+C 退出；支持可见性查询的窗口后端也可关闭主窗口退出。取流失败会在预算内重试，预算耗尽后报错退出。
- 无桌面验收：`./build/camera_light_test --device /dev/video0 --no-show --frames 120 --save-dir runs/usb_check_01`。
  保存目录必须尚不存在；正常退出且仍有新鲜画面时保存最后一帧，断流时不保存旧帧。
- 离线重测截图：`./build/camera_light_test --image runs/usb_check_01/raw.png --mask`；
  加 `--no-show` 只处理一次，或 `--frames N` 限制处理次数。静态图片不显示相机 FPS。

`config/light_bars.yaml` 控制 HSV 色相范围、最小面积/长度、长宽比、偏离竖直方向的倾角和填充率，
以及颜色支持、配对几何和姿态误差门限；也可 `--config` 加载截图保存的全部参数。
装甲尺寸默认使用小装甲灯条模型：中心距 0.135 m、灯条长 0.056 m，不会自动判断大/小装甲类型。
`camera_matrix`、`distort_coeffs` 必须来自当前这台相机和对应分辨率；缺省时四边形仍会画出，角度显示 `yaw n/a`。
先摆放真实发光红/蓝灯条，检查空场景、分别出现、同时出现、
移动与移出画面的结果；倾斜超过默认 40° 会被过滤。只改滑块不会覆盖项目配置文件。
默认 `brightness_color: true` 从 HSV 的 V 通道提取亮轮廓，用轮廓内部的红/蓝有色像素投票。
`min_color_fraction` 要求有色像素占亮轮廓的比例，`min_color_dominance` 要求主色在有色像素中的比例；
有足够彩色边缘时可保留过曝白芯，完全失去颜色的纯白灯条仍不能可靠判断红蓝。
`brightness_color: false` 可切回原 HSV 提取方式；暗灯条可适当降低 Min V，降低 Min S 会增加白色物体误检。

配对评分综合长度比、纵向偏移、灯条夹角、矩形一致性和装甲模型比例；`pair_min_score` 是几何评分门限，
不是数字分类置信度。灯条共用构成的冲突组不超过 12 根时，求总评分最大的无共用灯条配对；
更大冲突组退回按评分贪心，避免指数规模的耗时。中间灯条阻挡通过排序范围查询实现，替代第三层全量扫描。
这改善相邻两块装甲之间误配的情况，但纯灯条几何仍不能证明数字区域存在，真实误检/漏检需要现场录像验证。

姿态同时要求凸四边形、所有模型点在相机前方、平均重投影误差不超过 `pose_max_error_px` 和
`pose_max_error_ratio × 图像中较短灯条长度`。显示的 `err` 单位为像素；
当不同朝向的 IPPE 解误差十分接近时标记 `ambiguous`，此时低像素误差仍不代表朝向唯一。
实现分为 `light_bars.cpp`（提取/流程）、`armor_geometry.cpp`（几何/冲突配对）、
`armor_pose.cpp`（位姿/质量检查）和 `light_bar_config.cpp`（参数校验/完整保存）。
红蓝细长物体也可能通过筛选。角度依赖灯条端点和上述尺寸；这个入口不输出米制距离、装甲数字身份或飞控指令。
本程序不接串口、不输出飞行控制指令。

2026-10-08 本轮识别优化的离线耗时对比：640×480、OpenCV 两线程、每场景预热 20 次并测量 300 次。
常规 8 根灯条/4 个候选平均 0.67 ms（原版 0.62 ms）；白芯场景平均 0.67 ms，提取 8 根灯条/4 个候选，
原版 0.63 ms 但未提取到灯条；108 根灯条的合成杂乱场景平均 1.02 ms（原版 0.76 ms）。
新增颜色和几何质量检查带来少量额外耗时，这些数字只统计未配置相机内参的检测，不包含取流、显示或真实场景准确率。

2026-10-07 开发机实测：该 YLR4K 相机分别通过无窗口和带窗口/掩膜模式的 120 帧测试，
640×480 MJPG 下稳定处理约 30 FPS，包含启动等待的全程平均约 27.7 FPS。
当时画面偏暗且没有可见灯条；这只验证取流/显示和处理性能，真实红蓝灯条识别效果仍需现场摆靶验证。

2026-10-08 优化后实测：无窗口和虚拟桌面带窗口/掩膜模式分别连续处理 2700 帧，
运行约 90.5 秒，全程平均分别为 29.83 / 29.88 FPS，首帧等待分别为 0.54 / 0.50 秒；
两轮测试期间未出现新的 USB 错误或采集子进程遗留。带窗口测试保留相机当前参数，未触发扫焦或曝光恢复。
这些结果验证启动与持续取流，不代表光学对焦、灯条识别或物理掉线后的恢复已经验收。

## 自动装甲识别、跟踪与 RGB 相机标定

当前自动装甲视觉入口使用 `armor_auto`，**使用 YLR4K 的 UVC 彩色帧，不需要鼠标框选**：每个新彩色帧提取灯条、组合装甲候选，
按配置筛选颜色，连续 3 帧确认后选定目标。在关联距离/尺寸门限内保持同一目标，检测丢失立即使当前结果无效；
超过 0.25 s 后重新搜索。`track_id` 是临时跟踪编号，不是装甲数字身份，当前未接入数字分类器。

配置文件为 `config/armor_auto.yaml`，颜色按用户约定：

```yaml
target_color: 1 # 蓝色；改为 2 选择红色。修改后重启程序。
```

当前默认 `1`（蓝色），不识别板上的数字。先根据 `camera_light_test --list` 的结果修改配置中的
`camera.device`、`width`、`height`、`fps` 和 `format`，不要直接照搬别人的设备编号。
默认配置面向 YLR4K：`/dev/video0` 为待核验的设备占位，640×480、30 FPS、MJPG 为请求模式。
本次未验证当前硬件是否接受这些参数；驱动实际尺寸不一致时，`armor_auto` 会报错退出。

当前测试目标已确认灯条中心距 0.135 m、发光长度 0.056 m；换装甲板必须更新实际尺寸，
核对后才可设置 `armor.dimensions_confirmed: true`。
`detector_config` 使用通用 `config/light_bars.yaml`；`rgb_calibration` 指向尚未提供的
`config/ylr4k_rgb_calibration.yaml`，需为当前 YLR4K、镜头、对焦状态和分辨率独立标定，不能复用旧相机参数。
两个路径在主配置中均相对于主配置所在目录解析。

```bash
# 完成上面的基础构建并核对 config/armor_auto.yaml 后执行。
cmake --build build --target armor_auto -j2
mkdir -p runs
./scripts/armor_auto.sh --output runs/armor_auto.jsonl
# 无窗口验证；结果以默认 10 Hz 追加到 JSONL。
./scripts/armor_auto.sh --no-show --seconds 90 --output runs/armor_auto.jsonl
```

`q`/Esc 退出，`r` 清除当前目标并重新搜索。主窗口显示装甲候选、所选目标框、状态原因，
有效时显示 Range/Z 和 **Surface（板面朝向）**；**目标中心 Bearing 偏角目前通过 JSON 输出，未叠加到窗口**。
窗口和 JSON 均显示最近约一秒的新帧处理 FPS。

JSON 默认以 10 Hz 输出，`--hz 1..60` 控制报告频率，不改变相机 FPS。
未指定 `--output` 时写标准输出；指定时**追加**到文件，不自动编号，父目录需事先创建。
使用 `--save-dir NEW_DIR` 保存退出时的原图/叠加图，目录必须尚不存在。
可用 `--image FILE --no-show` 检查单张同分辨率图片；它只处理一帧，不用于验证连续确认或动态跟踪。

距离来源为**已标定彩色相机 + 实际装甲尺寸的 PnP**。
缺少 `config/ylr4k_rgb_calibration.yaml` 时仍可自动识别/跟踪，但 `valid=false`，距离、XYZ、角度为 `null`。
标定文件尺寸必须与采集分辨率一致，矩阵/畸变须有限且合理；不能填入网上同型号相机或其它相机的焦距。
板面 IPPE 两种姿态接近同样可信时，返回 `pose_ambiguous`，不输出唯一朝向。

**已有纸质棋盘格时先完成一次 RGB 标定**。当前用户确认的标定板为 9×12 个方格，
对应 8×11 个内角点，单格实际边长为 23.1 mm。更换标定板时必须按实际数值修改，
不能把方格数当成内角点数。启动前先在其它相机预览窗口按 `q` 退出，释放彩色相机：

```bash
./scripts/armor_auto.sh --calibrate --board-cols 8 --board-rows 11 --square-mm 23.1
```

移动纸质棋盘格到画面不同位置、距离，并绕两个方向倾斜；检测到完整角点后按空格采集，
至少采集 25 个不同视图，再按 `c` 求解。默认写入主配置指定的标定路径，拒绝覆盖已有文件；
可用 `--calibration-output NEW.yaml` 输出到新文件，再修改配置路径。
标定会检查画面覆盖度、倾角多样性、标定 RMS ≤0.8 px，及未参与求内参视图的最大 RMS ≤1.5 px；
未通过则要求继续采集，不生成假标定。完成后重新启动自动识别程序。

JSON 的 `position_camera_m` 是装甲中心在 **RGB 光学系**中的坐标，X 向右、Y 向下、Z 向前，单位米；
`range_m` 是直线距离，`z_m` 是前向距离。`normal_yaw_deg`/`normal_pitch_deg` 为板面法线朝向，
`bearing_yaw_deg`/`bearing_pitch_deg` 为**目标中心相对相机的左右/上下偏角**，角度单位度。yaw 正向右、pitch 正向上。
两组角度含义不同：板面正对相机时 Surface 可接近零，即使目标中心位于画面边缘，Bearing 仍可明显非零。
当前入口将两组角度都与完整 PnP 有效性绑定；不是有框就一定有角度。
`detection_valid` 表示当前存在所选候选，`tracking_confirmed` 表示通过连续确认，
`valid` 还要求标定、尺寸、帧新鲜度和姿态质量全部通过；具体失败原因见 `reason`。
`sample_time_monotonic_s` 和 `valid_until_monotonic_s` 使用本机单调时钟；下游必须检查 `valid` 和时效，
不能把停止/丢失前的最后一条有效数据一直当作当前目标。正常停止会追加一条 `reason=stopped` 的无效记录；
断流或未通过质量检查时距离/角度为 `null`。
这是视觉结果接口，输出机体/地图坐标仍需安装外参和机体位姿；本入口没有接入飞行控制。

## RK3588 开机自启与视觉偏航

Ubuntu 22.04 无桌面启动入口及部署步骤见 [RK3588 自启动说明](deploy/rk3588/README.md)。
新增 `python3 -m px4_go.yaw_follow` 管理无窗口视觉及唯一的 PX4 串口，
执行单次自动起飞、原地保持位置/高度与目标水平居中任务；原始 `gogogo.py` 保留。
相机未标定时只用二维像素误差，不提供目标距离或视线角度。
示例中的板端路径、接口、安装确认与飞行策略须填写后才通过配置检查；当前仅完成无硬件回归，尚未实机验收。

## 无硬件运行

```bash
# RGB-D 合成：直接注入检测框，不运行图像检测。
./build/demo_aim_cpp --source synthetic --frames 60 --output runs/rgbd.jsonl
# 纯彩色合成：可以有二维结果，三维位置始终为空。
./build/demo_aim_cpp --source synthetic-rgb --frames 60 --output runs/rgb.jsonl
# 坐标和任务离线演示；不发送指令，不包含真实动力学。
./build/demo_mission_cpp
```

60 帧合成预期 51 帧二维有效，RGB-D 为 44 帧三维有效、纯彩色为 0 帧三维有效。
任务演示最终 `intent.phase="complete"`，固定目标世界点始终为 `[4,0,1]`。

`demo_aim_cpp --output FILE` 默认每份 JSONL 最多 10 MiB，最多保留 5 份：当前 `FILE` 和历史 `FILE.1`～`FILE.4`，其中 `.1` 最新。
每次启动及运行中即将超限时轮转，超大单行直接报错，不拆分记录。同一路径同时只允许一个进程写入；
当前或备份路径被占用时明确失败，可换用不重叠的其他 `--output` 路径。
对应的固定 `.lock` 文件默认最多 5 个，永久留用且不计入 5 份日志。
此策略仅用于 `demo_aim_cpp`，`armor_auto` 仍按前文所述独立追加；完整边界见[日志管理约定](docs/interfaces.md#demo_aim_cpp-jsonl-文件管理)。

需要预览时附加 `--preview-dir runs/新目录` 或有桌面时 `--show`；
预览目录必须是新目录，按 q/Esc 退出窗口。绿色框表示二维有效，三维另查 `position_valid`。
常用运行只输出必要日志；需要保留的测试输出存入 `runs/`，不必每次另建验证报告。

## 视觉与电控离线 CPU / 内存联测

`px4_go/gogogo.py` 的正式控制逻辑已整理为独立模块，与传统视觉录像入口并行测试。
默认只连接内存中的 MAVLink 模拟 peer，不连接真实飞控，也不把视觉结果转换成飞行指令。
原始电控脚本保留；使用说明见 [px4_go](px4_go/README.md)，完整测量范围和统计口径见[联测指南](docs/vision_control_benchmark.md)。

```bash
./scripts/build.sh --test -- -DCMAKE_BUILD_TYPE=Release
python3 tools/benchmark_vision_control.py \
  --video /home/user/AUTO_std/sp_vision_25/assets/demo/demo.avi \
  --duration 30 --warmup 6 --repeats 3 \
  --output runs/vision_control_mytest/report.json
```

默认测试 `control_only`、`traditional_only`、`traditional_joint` 三种场景，输出 JSON、CSV 和原始采样。
视觉统一为 640×480 / 目标 30 FPS / OpenCV 2 线程，控制 20 Hz；100% CPU 表示一个逻辑核。
结果不包含真实 PX4/EKF、相机、GUI 或整板系统开销。

## 传统视觉内存测量

`vision_memory_probe` 复用项目的传统灯条/装甲算法。
`compare` 对同一张 BGR 图重复测量传统视觉，统一线程数、预热帧与测量帧。
它测的是算法进程，未包含相机采集、窗口、ROS、深度、导航和整板系统开销。

先配置工程，再单独构建探针：

```bash
cmake --build build --target vision_memory_probe -j2
python3 tools/benchmark_vision_memory.py compare \
  --probe build/vision_memory_probe --image /实际路径/frame_640x480.png \
  --threads 2 --frames 300 --warmup 30 --repeats 3 \
  --output runs/memory_comparison.json
```

`--output` 必须为新文件，同时生成 `.cases/` 原始日志。采样器只用 Python 标准库和 Linux `/proc`。
需要传统跟踪时可加 `--armor-config config/armor_auto.yaml`，在结果中检查 `tracking_enabled`、`pose_enabled`。
未标定时不伪造 PnP 内参；这次离线图测试不作为识别率、相机或目标距离验收。

结果口径：

- `probe.peak_rss_kib`：内核记录的该算法进程生命周期 RSS 峰值，包含解码、初始化、预热及检测，KiB 转 MiB 除以 1024。
- `sampled_tree_pss_peak_kib`：按间隔采样的父子进程合计 PSS 峰值，共享页按比例分摊；短峰可能漏采，不能称为内核峰值。
- `measurement_pss_p95_kib`：探针预热结束后样本的 P95；结合 `measurement_samples` 和读取失败数量解读。
- `sampled_tree_rss_peak_kib`：同一时刻 RSS 求和，共享内存会重复计数，不用于直接推算整机所需容量。
- `sampled_tree_swap_pss_peak_kib`：检查被测进程是否已换出；不要把这台开发电脑的全局已用内存当成视觉程序开销。

采集链另用 `profile` 测量整个父子进程组，确保配置中的相机设备、分辨率及界面开关正确：

```bash
python3 tools/benchmark_vision_memory.py profile --duration 60 \
  --output runs/armor_live_memory.json -- \
  ./build/armor_auto --config config/armor_auto.yaml --no-show
```

达到时限会请求退出并有界清理本次启动的进程组。`profile` 的 `success` 表示采集完成，
应用是否正常完成另看 `application_success`、`returncode`、`duration_limit_reached` 和 `cleanup`。
若相机重连/失败或仅预览空画面，本次内存值不能代表正常工作的采集链。

采购时将算法结果与目标板的系统、采集、ROS/定位等实际峰值合并，另留余量。
PC OpenCV CPU 测量不等于香橙派 ARM64 实测；增加内存容量也不能保证处理帧率。

## 可选 RealSense RGB-D 观测

D435i 已连接且 SDK 已编译时，`demo_aim_cpp` 使用现有传统灯条检测提取装甲候选，
通过 `detectArmorObservations` 转换为 `Detection` 后进入 `Pipeline`。检测阈值由 `--detector-config` 指定，默认 `config/light_bars.yaml`；
`config/aim.yaml` 继续配置观测过滤、连续确认和深度质量。
红蓝类别保持 `0=target_red`、`1=target_blue`，允许两类参与筛选不代表同时跟踪两个目标；
候选 `confidence` 使用几何评分，不是类别概率。

```bash
# 基础构建显式关闭过 RealSense，使用前需重新启用并构建。
./scripts/build.sh -- -DAIM_WITH_REALSENSE=ON
./build/demo_aim_cpp --source realsense \
  --detector-config config/light_bars.yaml \
  --frames 0 --threads 2 --output runs/camera.jsonl
```

按需增加 `--serial 实际序列号`、`--show`。默认 CPU 后端和两个 OpenCV 工作线程，摄像头起始设置为 640×480/30 Hz；
这不是机载处理帧率保证。可用 `uname -m`、`lsusb`、`lsusb -t` 检查平台和实际 USB 协商。
相机深度对齐到彩色，使用 SDK 反投影；未知标定或无有效深度时不伪造三维点。

输出 schema=2：二维看 `detection_valid`，视线看 `bearing_valid`，米制位置看 `position_valid`；
旧 `valid` 保持三维含义。光学系为 x右/y下/z前，`point_body_m` 仅在外参已标定时提供 FLU 坐标。
其余字段与时钟约定集中在接口说明中。

## PX4 / MicoVIO 台架通信

使用已有官方 MAVLink v2 生成头文件，路径必须包含 `common/mavlink.h`，不要只复制单个头文件。
以下路径存在于当前开发机；换机需提供对应头文件目录：

```bash
./scripts/build.sh --test -- -DCMAKE_BUILD_TYPE=Release -DAIM_WITH_MAVLINK=ON \
  -DAIM_MAVLINK_INCLUDE_DIR=/home/user/PX4-Autopilot/build/px4_sitl_default/mavlink
./build/px4_localization_cpp --help
```

本实现使用带 TIMESYNC 目标地址和 `*_encode_status` 的生成接口，不自动下载依赖或修改 PX4。
编辑 `config/px4_link.yaml`：将 `/dev/ttyS?` 改为真实设备，核对 921600 baud、system/component ID；
依据实际固件核对融合 ODOMETRY 语义后将 `px4_odometry_contract_confirmed` 设为 true。
默认占位设备会被拒绝；约定未确认时仅提供报文诊断，不输出可导航定位。

MicoVIO 接飞控的传感器链路，香橙派接另一条遥测链路。PX4 应在实际连接香橙派的 MAVLink 实例中提供：

- `HEARTBEAT`：身份与状态，默认 1 s 超时。
- `ODOMETRY`：融合位置、姿态、速度、协方差、复位计数；建议从 30～50 Hz 台架观察。
- `ESTIMATOR_STATUS`：估计有效性，建议从 10 Hz 台架观察。
- `TIMESYNC` 请求响应：程序每 0.2 s 请求一次。
- 可选 `ATTITUDE_QUATERNION`、`LOCAL_POSITION_NED`：用于状态缓存诊断。

先在 PX4 控制台使用 `mavlink status`、`mavlink help` 核对实例与该固件的 stream 配置方法。
不要将所需数据流误配到 QGC 的另一端口；接收器不会自行修改消息率或飞控参数。

```bash
./build/px4_localization_cpp --config config/px4_link.yaml --seconds 30
```

当前连接的 MicoAir743v2 USB 可先使用独立台架配置：

```bash
./build/px4_localization_cpp --config config/px4_usb_bench.yaml --seconds 30
```

该配置使用本机已核对的 `/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00`，
为当前约 1 Hz 的 `ESTIMATOR_STATUS` 与心跳保留 1.5 秒诊断超时。
它保持 `px4_odometry_contract_confirmed: false`，因此仍不会输出可导航位姿；
运行配置 `px4_link.yaml` 的 0.5 秒估计器超时及所有几何阈值保持不变。
换设备时先重新核对 USB 标识，完成正式融合/时效验收后使用对应的运行配置。

默认模式只要求本次运行收到配置 system/component 的 MAVLink 报文，
允许 `pose=null`，便于区分串口通信和定位验收。
需要把当前定位作为验收条件时，在已确认固件语义的配置上增加 `--require-pose`：
只有结束时仍存在有效且未过期的位姿才成功，曾经有效的旧位姿不算通过。
退出码为 `0` 成功、`1` 配置/运行错误、`2` 未收到配置来源的报文
（静默或只有其他设备的报文）、`3` 未满足 `--require-pose`；
Ctrl+C 返回 `130`，SIGTERM 返回 `143`。诊断与定位条件失败也会在标准错误输出 JSON 原因。

约每 0.1 s 一行 JSON，检查 `packets` 增长，`bad_frames` 不持续增长，`clock_ready` 和 `estimator_ready` 有效，
并且 `pose.valid=true`。原地静止及按轴平移/旋转时，检查坐标符号与实物一致。
`pose=null` 表示当前没有可用定位；`link.reason` 是最近事件原因，不能代替当前时效标志。
`timesync_responses=0` 表示未见配置来源的同步响应；响应增加但 `timesync_wrong_target`
增加时，先核对响应的目标地址与本机 system/component ID。
`timesync_unmatched` 表示未匹配当前请求，`timesync_bad_rtt` 表示往返延迟不合格；
`timesync_accepted` 与 `clock_sync_samples` 用于确认同步是否推进。
这些诊断不跳过来源、目标地址或请求 token 检查。
`reset_required=true` 时旧场地对齐失效，重建会话后重新确认原点，不能继续追踪旧坐标。
串口断连报错退出，不自动沿用旧原点重连；可用 Ctrl+C 结束。

坐标完整链路为 `map_T_odom × odom_T_body(图像时刻) × body_T_camera × p_camera`。
MicoVIO 安装/融合、相机外参、相机时钟和场地原点均需实测；USB/UART 不产生“开发板坐标系”。
具体 NED/ENU、FRD/FLU、复位和插值约定见接口说明。

## 可选 ROS 2 诊断

```bash
source /opt/ros/humble/setup.bash
./scripts/build.sh -- -DAIM_WITH_ROS2=ON -DAIM_WITH_REALSENSE=ON
./build/demo_aim_cpp --source realsense \
  --detector-config config/light_bars.yaml \
  --frames 0 --output runs/ros.jsonl --ros
# 另一个加载相同 ROS 环境的终端：
ros2 topic echo /demo/aim/solution
```

实际话题 `/demo/aim/solution`，类型 `std_msgs/msg/String`，内容为同一观测 JSON，
QoS reliable/volatile、队列 1；仅支持实时相机发布。没有 tf、其他拟议 ROS 包或飞控控制话题。
消息可靠投递不代表观测新鲜，消费端仍需自己的超时检查。

## 下一步工作

1. 固定现有 USB 相机/镜头/曝光配置，保存真实装甲板录像，完成空场景、颜色、移动和丢失测试。
2. 完成该成像配置的 RGB 标定，核对装甲实际尺寸；先验收目标中心偏角，再核对 PnP 距离和板面朝向。
3. 做距离、视角、光照测试；绿色灯先作为独立辅助搜索实验，依据实测决定是否采用和如何切换。
4. 按 MicoVIO 手册独立验证 PX4 安装外参、融合、实际消息与精度，标定相机安装外参及场地原点。
5. 统一 `armor_auto` 与既有观测/定位的数据契约、时间和坐标，接成有时效检查的实时流水线。
6. 复用 [2027 地图微型机仿真](docs/rmuc2027_integration.md)，另行实现视觉/任务到规划目标的适配；在合成数据验证与闭环仿真中验证控制权、命令时效、整段轨迹碰撞和失效行为，再验收真机对准与停住。
7. 前述通过后，再独立增加接触区域、接触反馈、恢复/返航和精确降落。

分阶段通过条件见 [视觉无人机路线图](docs/visual_drone_roadmap.md)。
当前世界目标协方差仍为空；PnP 测距精度、多目标身份、传统检测准确率、室外深度、
ROS 实际传输、香橙派持续性能和完整实机闭环均未由软件回归证明。所有真机几何和控制阈值都需实测。
