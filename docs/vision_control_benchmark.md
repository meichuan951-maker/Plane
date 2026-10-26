# 视觉与电控离线资源联测

本入口将 `px4_go/gogogo.py` 正式版本中的位置指令与消息处理整理为独立 Python 控制模块，和 C++ 视觉程序并行运行。它用于测量本机 CPU、内存及两侧调度情况，不形成视觉控制闭环。

## 本次测量范围

- 视频：`/home/user/AUTO_std/sp_vision_25/assets/demo/demo.avi`，原始 1440×1080、MJPEG、30 FPS。
- 视觉：流式解码并缩放为 640×480，OpenCV 算法线程数 2，目标处理频率 30 FPS；没有整段视频缓存、GUI、相机或 ROS。
- 传统模式：复用现有灯条提取、装甲配对和 `ArmorTracker`；录像相机内参未知，因此禁用 PnP，不套用其他相机的标定。
- 控制：复用正式版本的 NED 位置掩码、20 Hz、100 周期预填充及 OFFBOARD / ARM / 保持 / LAND 状态处理。通过 pymavlink 真实编码、解码字节，连接内存中的模拟协议 peer；不打开串口、UDP 或 socket。
- 控制资源包含 CPython、pymavlink、状态机和模拟 peer；没有运行 PX4 固件、EKF 或飞行动力学。原始四份电控脚本保留，未完成的脚本不参与联测。

## 构建与运行

所有构建产物继续保留在唯一的根目录 `build/` 中。性能测量使用 Release；这会更新现有缓存的优化级别，并保留其他功能开关。

```bash
cd /home/user/nohuman_plane_std/no_demo
./scripts/build.sh --test -- -DCMAKE_BUILD_TYPE=Release
python3 tools/benchmark_vision_control.py \
  --video /home/user/AUTO_std/sp_vision_25/assets/demo/demo.avi \
  --duration 30 --warmup 6 --repeats 3 \
  --output runs/vision_control_mytest/report.json
```

控制 worker 需要 Python 3.8 及以上和 `pymavlink`；采样器只使用标准库与 Linux `/proc`，不依赖 psutil。报告时间使用固定 UTC+08:00，不依赖 `zoneinfo`。缺少 pymavlink 时，应在自己的 Python 环境安装该依赖后重跑。使用 colcon 产物时增加 `--worker build/no_demo_aim_cpp/vision_video_worker`，并在对应 CMake 缓存中使用 Release。

RK3566 等 ARM64 板上可先只构建录像 worker，关闭硬件后端并用单个编译任务减少构建内存需求：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DAIM_WITH_REALSENSE=OFF -DAIM_WITH_ROS2=OFF \
  -DAIM_WITH_MAVLINK=OFF
cmake --build build --target vision_video_worker --parallel 1
python3 tools/benchmark_vision_control.py --video /实际路径/demo.avi \
  --scenarios traditional_joint --threads 1 --warmup 6 --duration 10 --repeats 1 \
  --output runs/rk3566_traditional_smoke_01/report.json
```

需先准备 C++17、CMake、OpenCV 4、yaml-cpp、nlohmann-json 和上述 Python 依赖；不复制 x86 可执行文件到 ARM64。这个离线入口不打开相机或飞控，也不包含 EGO、ROS 和机间通信；板端结果需运行后另行记录。

默认测试三种情况，各独立重复三次：`control_only` 控制单独、`traditional_only` 传统单独、`traditional_joint` 传统加控制。每次预热 6 秒、测量 30 秒，轮次之间反转执行顺序；总时长约 5.5 分钟，另加加载及退出时间。预热包含正式版本约 5 秒的预填充。需要仅重测联合运行时可增加 `--scenarios traditional_joint`。

输出 JSON、同名 CSV 和 `.cases/` 必须使用新路径，工具不会覆盖已有结果。每个 case 保存工作进程的 stdout/stderr、逐次采样及完整结果；总报告记录视频、程序、控制源码和配置的哈希，以及 CPU 型号、有效逻辑核数、线程与测量参数。

## 如何解读结果

- `workload_valid`：两侧均就绪、完成同一测量窗口、确实处理了帧/指令、采样完整且正常退出，才为 true。仅有资源采样、进程空转或提前退出不会被判为有效联测。
- `metrics.vision`、`metrics.control`：各自进程树；`metrics.total`：同一采样批次的两者合计，不含采样器、桌面和其他系统进程。单独测试时 total 等于该侧。
- `cpu_percent_one_core_mean`：100% 表示占满一个逻辑核，200% 表示两个。CPU 平均值来自窗口两端 CPU 秒数差除以实际墙钟时间。
- `cpu_percent_host_mean`：上述值除以有效逻辑核数。32 核电脑上的整机百分比较小，不能照搬到 4 核或 8 核开发板。
- `sampled_pss_peak_kib`：测量窗口内的 PSS 采样峰值，共享页按比例分摊。它适合观察联合运行的实际物理内存归属。
- `sampled_rss_peak_kib`：同窗口 RSS 求和，共享页可能重复计算；不能将其直接当作整板内存需求。
- `whole_run_sampled_peaks`：包含启动、预热与测量的采样峰值。各 worker 的 `result.peak_rss_kib` 则是内核记录的单进程生命周期峰值，包含初始化，二者口径不同。
- `workers.vision.result.actual_fps`：窗口内开始处理的帧数除以测量秒数；最后一帧可能稍晚完成。过慢时跳过错过的调度时隙，不积压补算；录像仍按顺序读取，因此不等于实时视频丢帧策略。
- `workers.control.result.actual_hz`、`period_p95_ms`、`period_max_ms`、`missed_deadlines`：用于判断 20 Hz 指令发送是否受到视觉负载影响，不能仅看低 CPU 占用。

KiB 除以 1024 为 MiB。采样默认间隔 0.1 秒，短暂峰值仍可能漏采；报告同时保留生命周期 RSS 峰值。树统计要求根进程存活并回收子进程；脱离进程树的守护进程不在统计范围。联测 worker 完成后会保持常驻，待最终采样后再由父进程释放。

这份结果是本机 x86_64 CPU 实测；相机驱动、实际串口、GUI、ROS 及系统余量应另测，不能据此宣称香橙派或其他 ARM 开发板已经达到相同帧率。

## 回归与后续接入

```bash
ctest --test-dir build -L benchmark --output-on-failure
python3 -m px4_go.offline_control --seconds 10
```

回归覆盖进程 CPU 计数、子进程回收、PSS/RSS 采样、失败/零工作量拒绝、共同窗口、进程清理、MAVLink 编解码与状态检查、小录像 EOF 和握手协议。

后续真机接入需新增明确的传输与执行器适配，定义视觉输出时效、相机外参、机体/本地坐标、目标距离和丢目标处理。本次保持原有 C++ 遥测接收器语义，不向它添加实际飞行指令。

## 2026-10-20 本机实测

AMD Ryzen 9 8940HX、32 个有效逻辑核，C++ Release / OpenCV 4.6 CPU。以下保留该次传统视觉与控制测量的结果；当时功能配置的 21/21 项 CTest 通过。原始报告在 `runs/vision_control_20261020/report.json`，同目录 CSV 与 `.cases/` 保存各次数据。

以下 CPU 为三轮平均占用的中位数，100% 等于一个逻辑核；内存为三轮中最大的测量窗口采样峰值，PSS 与 RSS 分开列出：

- 控制单独：CPU 0.27%，PSS 14.5 MiB，RSS 21.3 MiB；20 Hz。
- 传统视觉单独：CPU 17.1%，PSS 41.7 MiB，RSS 53.6 MiB；视觉帧率中位数 30.0 FPS。
- 传统加控制：总 CPU 17.4%（本机整机约 0.54%），总 PSS 56.3 MiB，总 RSS 74.9 MiB；视觉约 29.8 FPS，控制 20 Hz。联合时视觉 CPU 约 17.0%，控制约 0.33%。

所有控制 case 在各自测量窗口内发送 600 个设定点，错过周期为 0；全部控制 case 中最大周期约 50.79 ms，标称周期 50 ms。被测进程 SwapPss 始终为 0，全部工作进程正常退出，无清理残留。

这表明本次传统视觉联合运行增加的主机控制模块资源较小。上述 MiB 是进程负载数据，不是整机内存需求或 ARM 开发板性能；不能将本机的整机 CPU 百分比换成开发板占用率。

## 2026-10-22 RK3566 板端联合实测

通过 USB ADB 在实际 RK3566 四核、4 GB 板上原生编译运行。板端为 Ubuntu 20.04.4、ROS Noetic、OpenCV 4.2.0、Python 3.8.10，因此本轮另行构建官方 ROS1 EGO-Swarm；没有运行当前工程的 ROS2 移植版。原始报告、日志、测试脚本及补丁保存在 `runs/rk3566_20261022/`，其中 `joint_02/combined.json` 为通过的联合测试，`joint_02/compact.json` 为摘要，`provenance.json` 记录来源和限制。

共同测量窗口为 30 秒，之前预热 6 秒。负载为一个真实 EGO 规划器、传统灯条/装甲识别录像流水线、内存中的 MAVLink 控制模拟接口，以及三路各目标 50 Hz 的静止从机 B 样条消息。合成里程计理想跟随规划器实际输出，10×10×3 m 地图含中心固定障碍及地面，0.1 m 网格、12457 个点以 2 Hz 输入。没有其他编译任务与本轮并行。

- 录像输入和算法输入均为 640×480，OpenCV 算法线程数 1：实测 29.9 FPS，30 秒处理 897 帧；算法平均 14.53 ms，解码缩放平均 6.65 ms，跳过 3 个调度时隙。
- 控制模拟接口：20 Hz、600 条设定点、错过周期 0；周期 P95 为 50.17 ms，最大 54.30 ms。此处没有运行飞控固件或姿态控制环。
- 共同窗口内收到 31 条不同 ID 的实际规划轨迹；全程没有无效 B 样条消息。完整日志的 44 次成功规划耗时中位数约 1.55 ms、P95 约 57 ms、最大约 74 ms，这些耗时包含启动阶段，不能直接当作共同窗口内的统计。
- 三路合成从机消息各实测约 49.33 Hz，合计 148 Hz；确认发布端存在规划器订阅连接，尚未对内部轨迹缓存逐条验收计数。消息走板内回环，没有测量 Wi-Fi。
- 整棵测试进程树平均占用 236.86% 单核，即四核总算力的 59.21%；采样 PSS 峰值 314.04 MiB、RSS 求和峰值 433.42 MiB、SwapPss 为 0。该口径包含 ROS、输入生成器和采样器，不含其余系统进程。
- 测量窗口内 SoC 温度最高约 51.25℃，采样 CPU 频率保持 1.8 GHz；30 秒结果不能代表长期热稳定性。测试退出后没有遗留其拥有的子进程。

原始 1440×1080 MJPEG 录像在一次短测中仅约 14.8 FPS，主要时间花在解码缩放。最终通过的测试先在主机转换出 640×480 录像，再交给板端解码；不能据此宣称板端 1080p 相机采集和缩放也能达到 30 FPS。录像内参未知，PnP 关闭；这次只测识别负载，没有评估识别准确率。

首轮 `joint_01` 因共同窗口内没有持续规划轨迹而判为失败，原始失败报告保留。排查发现官方 `getLocalTarget()` 在循环时间恰好等于全局轨迹终点时，`>` 判断遗漏了最终目标赋值；隔离测试源码将该处改为 `>=`，同时给合成室内地图补上地面后重测。86 个传输源码文件已逐项核对，只有这一处算法源文件与原包不同；补丁位于 `harness/get_local_target_endpoint.patch`。两项修改一起进入复测，因此不是单变量对照实验。

板端测试文件保留在 `/tmp/codex-rk3566-trial-20261022`。已有该隔离工作区时可运行以下命令，输出目录必须使用尚不存在的新名称：

```bash
. /opt/ros/noetic/setup.sh
. /tmp/codex-rk3566-trial-20261022/ego_ws/devel/setup.sh
python3 /tmp/codex-rk3566-trial-20261022/run_combined.py \
  --video /tmp/codex-rk3566-trial-20261022/demo_640x480.avi \
  --output-dir /tmp/codex-rk3566-trial-20261022/results/joint_03
```

板端系统时间未校准，日志墙钟显示 2022 年；采集日期为主机的 2026-10-22，测量窗口全部使用板端单调时钟。本次结果支持继续在该板上验证简化原型；真实相机、MicoVIO、串口飞控、无线时延、定位漂移及四机协同闭环仍需接入后分别测量。
