# RK3566 的 ROS 2 Humble 环境与构建

适用目标：RK3566-M2101、4GB 内存、32GB 存储、ARM64。宿主保留厂家 Ubuntu 20.04.4、Linux 4.19.219 和 ROS 1 Noetic，在独立的 Ubuntu 22.04.5 ARM64 根文件系统中安装 ROS 2 Humble 与 Python 3.10，无需刷机。

2026-10-23 初次部署时，Humble 已安装，板端已确认 Python 3.10.12、`ROS_VERSION=2`、`ROS_DISTRO=humble`。所选 9 个 ROS 2 包与 `no_demo` 的 4 个视觉目标已完成 ARM64 原生构建。DDS 收发、实际 EGO 规划和只读控制节点检查三个阶段均通过，见[该轮 ROS 2 总报告](../../runs/ros2_humble_rk3566_20261023/runtime_01/report.json)；[Python 控制模块导入](../../runs/ros2_humble_rk3566_20261023/installation/install-python.log)与[视觉加载、合成输入检查](../../runs/ros2_humble_rk3566_20261023/vision_02/report.json)也已通过。这些属于相机切换前的部署及离线软件验证，不能证明本次 YLR4K 程序已经同步或真实硬件已经验收。

2026-10-23 本次 YLR4K 迁移已通过 USB ADB 同步到实际 RK3566：归档并移除 16 个旧 Astra 活动源文件，完成 4 个视觉程序的 ARM64 原生重编译，加载、合成确认和灯条检测均通过[板端离线报告](../../runs/rk3566_ylr4k_20261023_01/validation/report.json)。[最终源码校验](../../runs/rk3566_ylr4k_20261023_01/migration/final_source_verification.json)核对本地与板端文件哈希及新校准界面的链接结果；真实 USB 相机取流仍未验收。

## 目录与隔离方式

- `/opt/rk3566-humble/rootfs`：Ubuntu Jammy 根文件系统，Humble 位于其中的 `/opt/ros/humble`。
- `/opt/rk3566-humble/workspace`：源码、板端构建目录和安装目录；进入环境后映射为 `/workspace`。
- `/usr/local/bin/rk-ros2`：本目录 [rk-ros2](rk-ros2) 启动器的板端安装位置。

启动器通过私有 mount namespace 挂载根文件系统内的 `proc`、`sys`、`dev` 和工作区，再进入 chroot；挂载不向宿主传播。它清除外层 Noetic 的 Python、动态库及 ROS 环境变量，加载 Humble，并在存在时加载 `/workspace/install/local_setup.bash`。

Jammy 与 ROS 2 软件源只配置在新根文件系统内，不混入宿主 Focal。chroot 用于分开用户空间依赖，仍共用宿主内核和设备。默认不安装开机飞行服务，不启动相机、串口、飞控控制或解锁任务。

## YLR4K 相机切换

当前相机通过普通 USB/UVC 彩色接口接入，共用主工程的 `config/armor_auto.yaml`。默认请求 MJPG、640×480、30 FPS，`/dev/video0` 只是示例节点；应先列出设备，再改为板端实际彩色 Video Capture 节点或稳定的 `/dev/v4l/by-id/` 路径，并核验设备实际支持的模式。灯条参数使用 `config/light_bars.yaml`；`config/ylr4k_rgb_calibration.yaml` 尚未提供，必须为这台相机、当前镜头/对焦状态和分辨率独立标定。未标定时可检测和跟踪，距离、XYZ 和角度仍无效，不能复用旧相机内参。

Astra Pro 专用采集、诊断、控制和启动文件已退出主工程构建，历史文件及用法位于[相机归档](../../archive/astra_pro_20261023/README.md)。YLR4K 入口不依赖 OpenNI 或 Astra SDK；通用深度平面算法和输出回归仍保留在主工程。

2026-10-23 本轮检查中，板端只枚举到 `rkcif`/`rkisp` 内部视频节点，尚未检测到 USB YLR4K；`/dev/video0` 当时是内部 MIPI 输入。接入 USB 相机后应重新列出设备，不能沿用这个示例编号作为已确认的 YLR4K 路径。

在板端切换前，将最新 `src/`、`include/`、`config/`、`cmake/`、`CMakeLists.txt`、`scripts/`、`deploy/` 与 `archive/astra_pro_20261023/` 同步到 `/opt/rk3566-humble/workspace/no_demo`，在 `rk-ros2` 内对应 `/workspace/no_demo`。现场参数和标定应先单独备份。仅覆盖新源码会遗留板端旧文件：按归档 `manifest.json` 中 `action=moved` 的原路径，将板端仍存在的 Astra 专用文件归档并核对；内容不一致时另行备份，不覆盖已有历史文件。`action=snapshot` 是共用集成文件的旧版快照，不应据此移走活动的 CMake、构建脚本或 `armor_auto.yaml`。

不要复制开发电脑的 x86 `build/`、`install/` 或动态库到板端。完成源码与配置同步后，按下节仅重编视觉，再运行新一轮离线验证。旧的 `vision_02/report.json` 保留为 2026-10-23 初次部署记录，不作为本次迁移完成的凭据。

## 2026-10-23 初次部署的离线依赖来源

该轮板端仅通过 USB ADB 连接，没有可用网络。依赖在电脑端以独立 `APT_CONFIG`、ARM64 架构和独立包状态解析、下载，再通过 USB 传入；不会向开发电脑安装 ARM64 软件包。首批下载 1140 个 Debian 包，约 684MB；包含 APT 索引和缓存的 `offline-packages.tar` 为 820674560 字节，约 783MiB。[packages.txt](packages.txt) 记录所需顶层包，部署时生成的 `offline-package-manifest.json` 记录实际版本、架构和 SHA256。

基础文件已按官方发布值核对 SHA256：

- [Ubuntu Base 22.04.5 ARM64](https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/)：`ubuntu-base-22.04.5-base-arm64.tar.gz`，对照同目录官方 `SHA256SUMS`，SHA256 为 `075d4abd2817a5023ab0a82f5cb314c5ec0aa64a9c0b40fd3154ca3bfdae979f`。
- [ros2-apt-source 1.2.0 Jammy](https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb)：对照 [ROS 官方镜像 Dockerfile](https://github.com/osrf/docker_images/blob/master/ros/humble/ubuntu/jammy/ros-core/Dockerfile) 中的校验值，SHA256 为 `767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433`。

Python 控制依赖使用 `pymavlink==2.4.50` 的 ARM64、CPython 3.10 离线 wheel，并同时准备 `lxml`、`fastcrc`；文件与校验值记录在 `python-wheel-manifest.json`。不能复用电脑上的 x86 wheel，也不能将此 CP310 wheel 安装到宿主 Python 3.8。

## 进入环境

以下所有 shell 命令均在 **RK3566 板端** 执行，前提是根文件系统及启动器安装已经完成。启动器需要 root；普通用户运行时会通过 `sudo` 提升权限。

```bash
rk-ros2
```

无参数进入交互 shell，工作目录为 `/workspace`，`exit` 返回宿主。也可以直接执行一条命令：

```bash
rk-ros2 ros2 pkg list
rk-ros2 python3 -c 'import sys, rclpy; print(sys.version); print(rclpy.__file__)'
```

默认 `ROS_DOMAIN_ID=66`、`ROS_LOCALHOST_ONLY=1`，用于板内验证。显式设置的这两个变量以及可选 `RMW_IMPLEMENTATION` 会保留，例如：

```bash
ROS_DOMAIN_ID=67 rk-ros2 ros2 pkg list
```

多机网络联调时需显式设置 `ROS_LOCALHOST_ONLY=0`，并让参与同一 ROS 2 网络的节点使用相同 domain；实际 Wi-Fi、发现和消息传输效果需要另外验证。启动器不保留临时下载代理。其他应用变量可通过 `rk-ros2 env NAME=value command ...` 传入。

## 在板端构建

```bash
rk-ros2 bash /workspace/no_demo/deploy/rk3566/build_workspace.sh
```

[build_workspace.sh](build_workspace.sh) 先构建选定的 ROS 2 消息、EGO 规划、轨迹桥接和控制包，再构建视觉目标。4GB 内存下，含 PCL/Eigen 模板的单个 C++ 编译单元也可能占用较多内存，因此 colcon 按包顺序执行，并设置 `MAKEFLAGS=-j1` 限制单包 GNU Make 并行度；同时设置 `CMAKE_BUILD_PARALLEL_LEVEL=1`，视觉构建显式使用 `--parallel 1`。仅设置 CMake 环境变量不足以限制 colcon 自动传入的 GNU Make 任务数，不能省略 `MAKEFLAGS=-j1`。

Humble 和既有 `/workspace/install` 已准备好，仅切换相机或更新视觉程序时，跳过 9 个 ROS 2 包，只重新配置并构建视觉：

```bash
rk-ros2 bash /workspace/no_demo/deploy/rk3566/build_workspace.sh --vision-only
```

已完成的 9 个 ROS 2 包为 `quadrotor_msgs`、`traj_utils`、`plan_env`、`path_searching`、`bspline_opt`、`ego_planner`、`rosmsg_tcp_bridge`、`volans_msgs`、`px4_control`。本次单任务首次完整构建的日志汇总耗时为 **1 小时 5 分 53 秒**，不含随后进行的视觉构建；这是一次部署构建记录，不是程序运行周期。

EGO 相关包传入 `-DEGO_DEBUG_SYMBOLS=OFF`，保留 `-O3` 并使用 `-g0`，不生成额外调试符号；该选项默认仍为 `ON`，保持其他环境的既有行为。ROS 2 安装产物位于 `/workspace/install`，视觉可执行文件位于 `/workspace/no_demo/build/rk3566-humble`。首次编译可能较慢，不能复制电脑上的 x86 `build`、`install` 或动态库代替板端构建。

2026-10-23 初次部署时及本次相机切换后，视觉目标 `demo_aim_cpp`、`armor_auto`、`camera_light_test`、`vision_video_worker` 均已构建成功；本次重新确认 4 个可执行文件为 AArch64，`--help` 加载检查全部通过。

本次发现板端时钟回退到 2022 年，导致增量构建跳过新对象文件的链接；已用电脑当前时间同步板端，并清理重编、重新检查链接。旧源码、配置、四个原程序和原缓存已备份到板端 `/workspace/no_demo/runs/ylr4k_migration_20261023_01/backup/`；同步与构建日志保存在同一迁移目录。后续开机若再次回退，应先核对板端时间，再进行增量构建。

## YLR4K 启动与离线复核

[vision.sh](vision.sh) 从自身位置确定项目根目录，使用 `build/rk3566-humble` 内的板端程序。使用以下入口，不调用包含开发机 x86 库目录的 `scripts/armor_auto.sh`：

```bash
# 只列出设备，不开始预览。
rk-ros2 bash /workspace/no_demo/deploy/rk3566/vision.sh list
# 核对主配置中的实际设备节点后，进行无窗口取流检查。
rk-ros2 bash /workspace/no_demo/deploy/rk3566/vision.sh camera --no-show --frames 120
# 自动装甲识别与临时目标跟踪；未标定时只使用二维结果。
rk-ros2 bash /workspace/no_demo/deploy/rk3566/vision.sh armor --no-show --seconds 30
```

需要预览时，必须显式配置 chroot 的图形访问，再去掉 `--no-show`。`rk-ros2` 默认清除 `DISPLAY`、`WAYLAND_DISPLAY` 和 `XAUTHORITY`，仅有宿主桌面并不足够；可通过 `rk-ros2 env DISPLAY=<实际显示地址> XAUTHORITY=<chroot内可读的授权文件> bash .../vision.sh ...` 传入实际值，并确认显示套接字在该环境可访问，不自动放宽宿主显示权限。下列标定示例须在已配置这些变量的 `rk-ros2` 交互环境中执行，且需要真实棋盘格及独占相机。示例沿用已确认的 8×11 内角点、23.1 mm 方格边长，换板时按实物修改：

```bash
bash /workspace/no_demo/deploy/rk3566/vision.sh calibrate \
  --board-cols 8 --board-rows 11 --square-mm 23.1
```

不打开相机或飞控的部署复核使用 [validate_vision.py](validate_vision.py)。输出目录必须尚不存在，重跑时更换编号：

```bash
rk-ros2 python3 /workspace/no_demo/deploy/rk3566/validate_vision.py \
  --output-dir /workspace/no_demo/runs/ylr4k_validation_20261023_01
```

脚本检查 4 个视觉目标的 ELF 架构与加载、3 帧合成观测、按当前目标颜色生成的单帧灯条检测，以及旧 Astra 活动路径和二进制是否已退役；记录配置哈希、请求相机参数和标定路径状态，并保存 `report.json` 与进程日志。它不打开相机、串口或飞控，也不启动控制节点。

默认检查 `build/rk3566-humble` 和 AArch64 程序，另一个构建目录可通过 `--build-dir PATH` 指定；开发机离线复核可显式指定 `--expect-arch x86_64`，其结果不能代替板端验收。检查通过也不代表相机支持请求模式、取流稳定、PnP 精度或真实识别效果已验收，仍需分别保留实际画面和测量记录。

## 2026-10-23 初次部署验证结果与范围

统一使用 [validate_runtime.py](validate_runtime.py) 验证 ROS 2 运行链路；所有包由同一个 `/workspace/install` 提供，启动器自动加载。重新验证时使用以下命令。输出目录必须是 `/workspace` 下尚不存在的新目录，重复执行时更换末尾编号：

```bash
rk-ros2 python3 /workspace/no_demo/deploy/rk3566/validate_runtime.py \
  --output-dir /workspace/runs/ros2_validation_20261023_01
```

脚本顺序运行三个阶段，全部限制为板内回环通信，并分别使用 domain 65、66、67：

1. 官方 C++ `talker` 向 Python `listener` 实际发送 DDS 消息。本次收到 3 条序号不同的消息，检查通过。
2. 启动实际 EGO 规划器与 [validate_ego_ros2.py](validate_ego_ros2.py)，由实际输出的 B 样条驱动理想跟随器。本次 30 秒测量窗口收到 32 条不同编号的新轨迹，三路模拟从机轨迹各在本地发布 50Hz，见[规划记录](../../runs/ros2_humble_rk3566_20261023/runtime_01/planner/planner_validation.json)。这些频率是本地发布统计，不代表三架实机的消息接收率；未核验规划器内部轨迹缓存的逐条接受情况。
3. 启动 `position_to_mavros_node`，两次只读查询均取得 `output_rate=20.0`。本地观察到解锁和切模式请求各 0 次，未发现对应客户端端点，检查通过。此阶段未启动 MAVROS、未连接飞控，参数值不等于实测 20Hz 控制输出。

输出目录保存总报告 `report.json`、各阶段 JSON 与进程日志；脚本只清理本次启动的子进程。本次三个阶段均已清理完成，没有本次验证进程残留。

视觉检查见[视觉报告](../../runs/ros2_humble_rk3566_20261023/vision_02/report.json)。`demo_aim_cpp` 的 3 帧合成输入依次为 `acquiring`、`acquiring`、`ok`，第三帧 `valid=true`，符合 `confirm_frames=3` 的确认规则；不能要求前两帧已经确认。`armor_auto` 在单帧 640×480 合成蓝色灯条图上得到 `detection_valid=true`，同时 `tracking_confirmed=false`、`valid=false`，符合单帧尚未确认、没有相机标定的测试条件。本次没有验证相机标定、PnP 距离或连续跟踪能力。

当前 `armor_auto` 输出 JSON，不发布 ROS 话题。`AIM_WITH_ROS2=ON` 只为 `demo_aim_cpp` 添加 ROS 2 诊断输出；本次合成运行未启用 ROS 发布，未验收视觉 ROS 消息输出，也未验证视觉到规划器的完整接入链路。

这些检查验证部署和软件接口，没有验收障碍或机间碰撞净空、动力学约束、速度上限、无线链路或实飞。尽管启动参数 `max_vel=1.5`，理想跟随器记录的最大速度约为 **2.12m/s**；不能据此宣称速度上限合格或轨迹满足动力学约束。

后续需接入真实相机、MicoVIO、无线链路和飞控，验证实际输入下的跟踪、定位、规划及控制闭环。本轮未连接真实飞控，未执行解锁或飞行；编译时的内存负载也不能代替实际运行负载的测试。
