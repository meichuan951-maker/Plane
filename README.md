# 视觉无人机 Demo（C++17）

面向 Orange Pi 5 8GB / Ubuntu 22.04 的视觉与导航开发工程。当前开发主线是
**普通 USB 相机 + 传统红蓝灯条检测 + 装甲板跟踪 + 相机标定/PnP**，不要求 YOLO 或海康 SDK。
定位接入方案为 **MicoVIO → PX4 → 香橙派**，不使用 GPS；目前仍需独立完成实际融合与导航集成验收。

仓库：[meichuan951-maker/Plane](https://github.com/meichuan951-maker/Plane)。文档更新：2026-10-12。

运行命令见本页。开发顺序与验收目标见 [视觉无人机路线图](docs/visual_drone_roadmap.md)，
数据/坐标/时间契约见 [接口说明](docs/interfaces.md)，选型与安装标定见 [硬件说明](docs/hardware.md)。
后续需要 YOLO 时再阅读 [红蓝训练指南](docs/training.md)。

## 当前目标与开发顺序

第一阶段目标：**导航到目标附近 → 搜索 → 确认装甲板 → 精细对准 → 在目标前稳定停住。**
接触、恢复、返航和降落放到后续阶段。

1. 地面验证真实画面、帧率、灯条配对和自动跟踪。
2. 标定当前相机，分别验证目标中心偏角、板面朝向和米制距离。
3. 独立验证自身定位与 PX4 融合；这一步可与地面视觉并行。
4. 联合回放与闭环仿真，验证目标丢失、观测延迟、定位失效和退出行为。
5. 再进入真机有限对准与接近，逐步验收到目标前停住。

绿色灯辅助搜索、短焦广角镜头仍处于方案讨论阶段，**尚无绿色灯检测实现或可靠最远距离结论**。
图纸中的绿色灯属于飞镖检测模块，不等同于装甲板中心或接触点。
视觉引导阶段由视觉改变导航参考，PX4 仍负责飞行控制，不能跳过自身状态估计。

## 当前能做什么

- `camera_light_test`：普通 USB/UVC 相机真实画面、实测处理帧率、传统红蓝灯条候选提取、阈值调节和截图；不需要 YOLO 或厂商 SDK。
- `armor_auto`：按颜色自动选择装甲板、连续确认和临时目标跟踪；提供棋盘格标定入口，标定与尺寸有效时输出 RGB 相机系 PnP 位置、距离、板面朝向和目标中心偏角。
- `astra_live_measure`：可选 Astra 深度图手动 ROI/平面测量；不自动识别或跟踪装甲板，不提供 RGB-D 自动配准。
- `demo_aim_cpp`：合成、PNG 回放和可选 D435i 输入的既有观测流水线；真实检测使用可选 YOLOv8 ONNX CPU 后端。
- `px4_localization_cpp`：可选 MAVLink UART 接收心跳、姿态、位置、融合 ODOMETRY，完成 TIMESYNC 与估计器状态检查。
- `demo_mission_cpp`：合成位姿和目标经过实际坐标变换、任务状态机，走完起飞、搜索、接近、接触反馈、返航和降落的离线流程。

任务输出 `executable=false`，`demo_aim_cpp` 观测输出 `contact_permitted=false`。当前没有实机 Offboard 执行器、
接触控制器或精确进库降落，也没有将独立视觉与串口诊断程序组成实时导航进程。
MicoVIO 软件适配尚未经过真实模块/PX4 融合验收；串口唯一发送消息为 TIMESYNC。

普通 USB 视觉由 `camera_light_test` 和 `armor_auto` 独立入口提供，尚未接入 `demo_aim_cpp` 的观测流水线。
`track_id` 仅为临时跟踪编号；装甲数字身份识别、多目标身份保证和 RKNN/NPU 推理尚未实现。
PnP 测距已有代码，但尚未完成实机精度和 8～10 m 可靠性验收；RGB-D ROI 点也不等于指定接触点。
仓库未附带当前相机的 RGB 标定结果。没有本相机标定时仍可画框，数值角度/距离不可用，不能套用其它相机参数。

## 目录和阅读顺序

```text
no_demo/
├── README.md                 # 运行入口和能力边界
├── CMakeLists.txt            # 全部构建目标和可选依赖
├── include/demo_aim/         # 稳定公共头文件
├── src/
│   ├── vision/              # 检测、观测、关联和可选测距
│   ├── input/               # UVC 采集/控制/进程、深度、synthetic/replay/realsense
│   ├── navigation/          # 坐标、定位历史和任务状态机
│   ├── io/                  # 遥测缓存、MAVLink、UART
│   └── app/                 # 独立入口、camera/ 应用层、标定、日志、ROS
├── config/                  # 灯条、自动装甲、YOLO/观测与 PX4 链路配置
├── scripts/                 # armor_auto、Astra 深度的开发机启动脚本
├── tools/                   # 独立 Astra 深度诊断源码
├── test/                    # 视觉、标定、取流、深度、导航、通信回归
├── examples/cpp_replay/      # 最小回放输入夹具
├── docs/                    # 开发路线、接口、硬件与训练说明
├── picture/、test_1/         # 原始目标与模块资料
├── models/                  # 用户模型，不提交权重
├── runs/                    # 日志和预览，不提交生成输出
└── build_cpp/               # 统一构建目录，不提交生成文件
```

从 `src/app/camera_light_test.cpp` → `src/app/camera/` 阅读 USB 预览和阈值测试；
从 `src/app/armor_auto_main.cpp` → `src/vision/armor_tracker.cpp`、`armor_pose.cpp` 阅读自动装甲链路；
从 `src/app/armor_calibrate.cpp` → `src/vision/camera_calibration.cpp` 阅读相机标定；
从 `src/app/main.cpp` → `application.cpp` → `src/vision/core.cpp` 阅读既有观测链路；
从 `src/app/localization_main.cpp` → `src/io/mavlink_link.cpp` 阅读定位接收；
从 `src/app/mission_demo.cpp` 阅读坐标和任务模块的组合方式。
公共 include、可执行名称、CLI 和配置路径保持兼容，源码物理目录按职责归组。
原规则 PDF 留在项目根目录。独立地图工程 `../rmuc_gazebo_map` 不属于本项目构建。

## 编译和回归

需要 CMake ≥3.16、C++17 编译器、OpenCV 4（core/imgproc/imgcodecs/highgui/videoio/dnn/calib3d）、
yaml-cpp、nlohmann-json、OpenSSL 和线程支持。RealSense SDK、ROS 2、MAVLink、Astra OpenNI 为可选依赖。
全部本地构建统一使用 `build_cpp`，通过 CMake 选项切换功能。

先构建普通 USB 视觉与无硬件演示，显式关闭可选后端，避免沿用旧 CMake 缓存：

```bash
cd /home/user/nohuman_plane_std/no_demo
cmake -S . -B build_cpp -DCMAKE_BUILD_TYPE=Release \
  -DAIM_WITH_REALSENSE=OFF -DAIM_WITH_ROS2=OFF \
  -DAIM_WITH_MAVLINK=OFF -DAIM_WITH_ASTRA=OFF
cmake --build build_cpp -j2
ctest --test-dir build_cpp --output-on-failure
./build_cpp/demo_aim_cpp --help
./build_cpp/camera_light_test --help
./build_cpp/armor_auto --help
```

仅预览和检测彩色帧不需要 Astra SDK，即使相机来自 Astra 也可使用其 UVC 接口。
`AIM_WITH_REALSENSE` 默认 ON，SDK 缺失时跳过实时后端；其余可选功能默认 OFF。
后文按需重新启用相关选项；`demo_aim_cpp --help` 显示实际 RealSense/ROS 编译能力。
x86_64 开发机程序不能直接复制到 ARM64 香橙派运行；`scripts/` 的库搜索路径也需按板端环境调整。

`BUILD_TESTING=ON` 时，基础配置登记 **11 项 CTest**；启用 Astra 增加 1 项，启用 MAVLink 增加 2 项，
两者同时启用共 14 项。数量以 `ctest --test-dir build_cpp -N` 为准，不等于测试程序内部断言数量。
覆盖灯条提取/配对、姿态多解、自动装甲配置/跟踪/标定、深度平面、观测、坐标/任务、CLI 和 UVC 取流/控制。
取流回归用独立进程模拟打开设备卡住、帧发布卡住、控制卡住、持锁进程退出、断流后恢复，以及扫焦期间持续预览；无需真实相机。
这些是开发机软件回归，不是相机识别准确率、板端实时性或真实飞行验收。

2026-10-12 项目首次推送前验证：开发机构建成功，Astra 开启、MAVLink 关闭，12 项 CTest 全部通过。

## 普通 USB 相机：实时画面与红蓝灯条（无 YOLO）

参考 `sp_vision_25` 的独立相机/传统检测测试分层，以及传统识别器中亮度提取、颜色判断和装甲几何检查的分层，
自行实现 UVC 输入、亮度与 HSV 颜色投票、几何候选配对和独立姿态估计。
不引入海康 SDK、OpenVINO、数字分类模型或云台通信。
普通 UVC 相机通过 Linux `uvcvideo`、V4L2 和 OpenCV 采集；
驱动接口见 [Linux UVC 文档](https://cdn.kernel.org/doc/html/latest/userspace-api/media/drivers/uvcvideo.html)。
仓库的自动装甲配置目前指向 Astra Pro 的 RGB UVC 接口。历史 YLR4K（Lombo `27c4:0531`）
测试记录保留在本节末尾，不代表每台机器当前都连接该设备。

编译后在**连接相机的桌面终端**执行，先关闭 ffplay/其他预览程序：

```bash
cd /home/user/nohuman_plane_std/no_demo
./build_cpp/camera_light_test --list
# /dev/video0 仅为示例，替换成上一步找到的彩色 Video Capture 节点。
./build_cpp/camera_light_test --device /dev/video0 --mask
```

同一相机可能暴露多个节点，插拔后编号可能变化。安装了 `v4l2-ctl` 时可用
`v4l2-ctl --list-devices` 和 `v4l2-ctl -d /dev/video0 --all` 确认 Video Capture；
优先使用 `/dev/v4l/by-id/` 下实际存在的稳定路径。
使用仓库中这台 Astra 的配置时，示例为：

```bash
./build_cpp/camera_light_test \
  --device /dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0 \
  --format YUYV --config config/astra_light_bars.yaml --mask
```

默认请求 MJPG、640×480、30 FPS；使用 `--width`、`--height`、`--fps`、`--format MJPG|YUYV` 调整。
设备不一定接受请求参数，启动日志显示驱动协商值，叠加显示实际图像尺寸。
FPS 是最近约一秒完成的“采集+检测+显示”吞吐，首次统计前显示 0，不是传感器帧率或端到端延迟。
`detect ms` 只统计灯条算法耗时。启动保留相机当前曝光、对焦和增益，不再等待亮度稳定或自动扫描整个焦点范围；第一帧到达即开始预览，终端报告首帧等待时间。不自动修改 USB 电源设置，USB 超时也不能直接认定为自动休眠。

取流和控制在独立受监控子进程中执行，主进程负责窗口、检测及按键。共享内存只保留最新一帧，慢消费者不会积压旧帧。连续 3 秒没有新帧会终止本轮采集并重试，默认恢复预算为 15 秒（加上至多一次 3 秒设备等待）；`--reconnect-seconds N` 可调整，0 表示失败即退出。相机物理掉线或固件无响应时，仍可能需要重新插接/换线，软件重试不保证恢复设备。等待超过 250 ms 时显示等待提示并清空旧轮廓/掩膜；等待、重连和驱动卡住期间均可按 q/Esc 或 Ctrl+C 退出。不会无限等待子进程释放失效的设备。

主窗口有 Focus 和 Gain。把装甲放在工作距离上，按 `f` 才开始手动扫焦，按 `a` 启用相机自身连续自动对焦。扫焦由正常预览的新帧逐步驱动，窗口持续刷新，拖动 Focus 或按 a/e 可取消扫描。每个位置读回确认后等待至少 350 ms，再取三帧中心清晰度的中值；扫描包含范围两端。拖动滑块的请求节流到最多约每 100 ms 一次；失败给出警告并恢复读回值。对焦读回正确只证明驱动接受参数，不证明镜头运动或已经对准目标。

历史 YLR4K 冷启动时曾出现黑帧。如确实全黑，可按 `e` 尝试最长快门后恢复自动曝光；该操作按需执行，不再拖慢默认启动。`--no-controls` 可运行带窗口的纯取流预览，跳过曝光/焦点/增益查询和写入，用于区分取流与控制故障。无窗口测试也不改相机控制。

画面发虚时，先固定相机、增加目标照明，检查镜头保护膜和污渍。比较约 1 m 和实际工作距离下的黑白文字，缓慢扫描 Focus 的完整范围；若读回数值变化而画面始终不变，继续核对镜头是否支持电动调焦，排查调焦机构、模组固件或机械焦点，不能仅凭滑块认定已对焦。Gain 和灯条阈值不能恢复失焦细节。
历史 YLR4K 驱动报告 MJPG 支持 1920×1080/30 FPS。设备确实支持该模式时，可用以下命令比较原始细节；实际协商尺寸仍以启动日志为准，提高分辨率不解决光学失焦：

```bash
./build_cpp/camera_light_test --device /dev/video0 --width 1920 --height 1080 --fps 30 --format MJPG
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
- 无桌面验收：`./build_cpp/camera_light_test --device /dev/video0 --no-show --frames 120 --save-dir runs/usb_check_01`。
  保存目录必须尚不存在；正常退出且仍有新鲜画面时保存最后一帧，断流时不保存旧帧。
- 离线重测截图：`./build_cpp/camera_light_test --image runs/usb_check_01/raw.png --mask`；
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
本程序不读取模型、不接串口、不输出飞行控制指令。

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

当前自动装甲视觉入口使用 `armor_auto`，**不需要鼠标框选或 Astra 深度 SDK**：每个新彩色帧提取灯条、组合装甲候选，
按配置筛选颜色，连续 3 帧确认后选定目标。在关联距离/尺寸门限内保持同一目标，检测丢失立即使当前结果无效；
超过 0.25 s 后重新搜索。`track_id` 是临时跟踪编号，不是装甲数字身份，当前未接入数字分类器。

配置文件为 `config/armor_auto.yaml`，颜色按用户约定：

```yaml
target_color: 1 # 蓝色；改为 2 选择红色。修改后重启程序。
```

当前默认 `2`（红色）。先根据 `camera_light_test --list` 的结果修改配置中的
`camera.device`、`width`、`height`、`fps` 和 `format`，不要直接照搬别人的设备编号。
默认配置使用此前 Astra Pro UVC 的固定路径、640×480、30 FPS、YUYV。
这些参数来自配置，不代表另一台相机也支持同样模式。驱动实际尺寸不一致时，`armor_auto` 会报错退出。

当前测试目标已确认灯条中心距 0.135 m、发光长度 0.056 m；换装甲板必须更新实际尺寸，
核对后才可设置 `armor.dimensions_confirmed: true`。
`config/astra_light_bars.yaml` 只针对本次 Astra 过曝灯条适当放宽饱和度门限，不修改其它相机配置。
更换相机时同时核对 `detector_config` 和 `rgb_calibration`；它们的相对路径以主配置所在目录为基准。

```bash
# 完成上面的基础构建并核对 config/armor_auto.yaml 后执行。
cmake --build build_cpp --target armor_auto -j2
mkdir -p runs
./scripts/armor_auto.sh --output runs/armor_auto.jsonl
# 无窗口验证；结果以默认 10 Hz 追加到 JSONL。
./scripts/armor_auto.sh --no-show --seconds 90 --output runs/armor_auto.jsonl
```

`q`/Esc 退出，`r` 清除当前目标并重新搜索。主窗口显示装甲候选、所选目标框、状态原因，
有效时显示 Range/Z 和 **Surface（板面朝向）**；**目标中心 Bearing 偏角目前通过 JSON 输出，未叠加到窗口**。
窗口 FPS 调试用 `camera_light_test`；`armor_auto` 的 `fps` 位于 JSON 中，是从启动起计算的平均处理吞吐。

JSON 默认以 10 Hz 输出，`--hz 1..60` 控制报告频率，不改变相机 FPS。
未指定 `--output` 时写标准输出；指定时**追加**到文件，不自动编号，父目录需事先创建。
使用 `--save-dir NEW_DIR` 保存退出时的原图/叠加图，目录必须尚不存在。
可用 `--image FILE --no-show` 检查单张同分辨率图片；它只处理一帧，不用于验证连续确认或动态跟踪。

距离来源为**已标定彩色相机 + 实际装甲尺寸的 PnP**，不是 Astra 深度传感器距离。
缺少 `config/astra_rgb_calibration.yaml` 时仍可自动识别/跟踪，但 `valid=false`，距离、XYZ、角度为 `null`。
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
标定会检查画面覆盖度、倾角多样性、训练 RMS ≤0.8 px，及未参与求内参视图的最大 RMS ≤1.5 px；
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

2026-10-10 缺少 RGB 标定时现场验证：连续 120 帧全部检测到红色候选，前 2 帧确认后，118 帧保持目标，
相应距离/角度均为空；另一次带窗口 90 s 测试处理 1696 帧、1693 帧确认目标，未卡死。
这验证自动识别/关联和失效门控，不是实际测距精度、移动靶跟踪或飞行验收。

## Astra Pro：独立深度测距检查

开发机接入的 Astra Pro 深度接口为 `2bc5:0403`，彩色 UVC 接口为 `2bc5:0501`。
2026-10-10 对约 1 m 普通墙面实测：OpenNI 2.3.0.63 的默认 320×240 模式连续 120 帧全零，
显式改为 640×480、30 FPS、毫米深度后，连续 120 帧获得有效深度，中心 7×7 区域约 1.15～1.17 m。
保留的诊断程序复测 120 帧：预热后 90/90 帧中心 ROI 坐标有效，Z 中位数 1.161 m、范围 1.161～1.165 m。
该结果验证有距离输出，不是与尺量真值比较的精度标定，也不是装甲板定位验收。

在项目根目录运行：

```bash
./scripts/check_astra_depth.sh --frames 120
# 指定的是深度图像素，不是彩色图像素；用于检查偏离中心点的 XYZ。
./scripts/check_astra_depth.sh --u 400 --v 240 --frames 120
```

脚本单独构建 `build_cpp/astra_depth_check`，不改变现有 CMake 配置或主识别程序。
默认使用同工作区的 x86_64 OpenNI 2.3.0.63；可通过 `ASTRA_OPENNI2_ROOT` 指定安装目录。
运行时显式选择这一版 OpenNI 和系统 libusb，避免误加载系统 OpenNI 或海康 SDK 同名库。
先退出其他深度预览程序，将相机对准约 1 m 的普通墙面/纸板。

诊断默认使用 640×480、30 FPS 的 1 mm 深度流，关闭本次流的镜像及彩色配准，明确输出原始深度相机系坐标；前 30 帧用于预热。
中心 ROI 至少 30/49 像素有效、MAD 不超过 40 mm、主深度层占比至少 80%，才输出 XYZ。
无效数据输出 `null`，不会把 0 当成目标位置；测试结束打印有效帧数及 ROI 深度范围。
退出码 0 表示预热后有有效 ROI 坐标且完成指定帧数，4 表示未通过，1 表示设备/参数错误。

`xyz_depth_optical_m` 是以深度相机光心为原点的坐标，单位米，X 向右、Y 向下、Z 向前；
`range_m` 是三维直线距离，Z 是轴向深度，两者不能混用。
SDK 的 `convertDepthToWorld` 在这里指相机坐标；程序已将其 Y 向上转换成工程使用的 Y 向下。

这不是自动装甲板三维定位程序。现有 `camera_light_test` 只处理彩色帧；
要输出装甲板坐标，必须先建立彩色与深度的像素对应并验证时序、筛选装甲区域的有效深度，
再反投影，不能将彩色坐标按分辨率比例直接套到深度图。
输出机体/地图坐标还需要相机安装外参和相应时刻的机体位姿。

## Astra Pro：实时手动框选深度诊断

此前 Astra Pro 的 RGB-D 出厂标定读回 NaN，尚未完成彩色识别框到深度的有效映射。
`astra_live_measure` 因此采用独立的**深度图手动框选模式**：没有彩色自动配准，也没有目标跟踪。
角度和 XYZ 使用 SDK 深度反投影，完整几何精度尚未标定验证，窗口和 JSON 均标注为初步估计。

```bash
cmake -S . -B build_cpp -DAIM_WITH_ASTRA=ON
cmake --build build_cpp --target astra_live_measure -j2
./scripts/astra_live.sh
# 把测量结果追加写入 JSONL，窗口继续显示。
./scripts/astra_live.sh --output runs/astra_live.jsonl
```

退出其他深度预览后运行。在深度窗口中按住鼠标左键，框选装甲板**内部平整区域**，
避开外轮廓、灯条、底座和背景；至少 8×8 像素且应具有足够实际面积。
实时显示 `Range`（三维直线距离）、`Z`（前向深度）、`Surface`（板面法线 yaw/pitch）
和 `Bearing`（区域中心所在方向 yaw/pitch）。按 `r` 清除区域，`q`/Esc 或关闭窗口退出。
**装甲板移动后需要重新框选；固定框不会自动跟随或验证目标身份，即使框内变成墙面也可能得到平面测量。**

坐标为深度相机光学系：X 向右、Y 向下、Z 向前，XYZ/距离单位米、角度单位度。
Surface yaw 正表示法线偏向画面右侧（左侧更近），pitch 正表示法线向上（下沿更近）；
Bearing yaw 正表示目标在右侧，pitch 正表示目标在上方。两组角度不相同。
JSON 中 `xyz_depth_optical_m` 是所选有效点的三维均值，不保证等于装甲板几何中心；
`source=manual_depth_roi`、`tracking=false` 和 `calibration=sdk_depth_projection_unverified` 明示测量范围。

```bash
# 无窗口取固定深度 ROI；x y width height 是深度图像素，不能使用彩色坐标。
./scripts/astra_live.sh --no-show --roi 435 380 45 40 --frames 120 --hz 10
```

JSON 默认以 10 Hz 输出到标准输出，状态信息写入标准错误。可用 `--hz 1..30` 修改输出频率；
`--seconds N` 或 `--frames N` 限制运行时间/接收帧数。未选区域、有效深度不足、区域混合/不平整、
实际覆盖面积太小或帧超过 250 ms 未更新时，`valid=false`，距离/角度为 `null`。
`valid` 仅表示当前所选区域通过深度/平面质量检查，不表示已确认装甲板身份或标定精度。
启动预热 30 帧；USB 采集在独立受监控进程中执行，驱动阻塞不阻塞窗口；重连后清除旧区域，需重新选取。
`AIM_WITH_ASTRA` 默认关闭；脚本使用本工作区 OpenNI 2.3.0.63，SDK 安装位置可通过构建参数
`AIM_ASTRA_SDK_ROOT` 和运行时 `ASTRA_OPENNI2_ROOT` 指定，二者应一致。

2026-10-10 现场验证：深度窗口连续运行 90 s 收到 2635 帧（约 29.3 FPS，包含启动预热），
没有选区域时不输出测量值。正常退出后再次打开成功；上一轮人工选定的深度内部区域
`435 380 45 40` 连续 120/120 帧通过深度/平面检查，Range 约 1.135 m、Z 约 1.065 m，
板面 yaw 约 -1.4°、pitch 约 +14.9°。这些是当时摆放位置的初步估计，不能硬编码为其他目标的取样坐标。
当日回归包含已知倾角平面、无效深度、混合深度层和 XYZ 进程间传输检查；当前 CTest 数量见前文。

## 无硬件运行

```bash
# RGB-D 合成：注入检测框，不运行 YOLO。
./build_cpp/demo_aim_cpp --source synthetic --frames 60 --output runs/rgbd.jsonl
# 纯彩色合成：可以有二维结果，三维位置始终为空。
./build_cpp/demo_aim_cpp --source synthetic-rgb --frames 60 --output runs/rgb.jsonl
# 保留的 PNG + JSON 输入夹具。
./build_cpp/demo_aim_cpp --source replay \
  --manifest examples/cpp_replay/manifest.json --frames 5 --output runs/replay.jsonl
# 坐标和任务离线演示；不发送指令，不包含真实动力学。
./build_cpp/demo_mission_cpp
```

60 帧合成预期 51 帧二维有效，RGB-D 为 44 帧三维有效、纯彩色为 0 帧三维有效；
5 帧回放预期 3 帧三维有效。任务演示最终 `intent.phase="complete"`，固定目标世界点始终为 `[4,0,1]`。

`demo_aim_cpp` 的 JSONL 重名会自动加编号，终端给出实际路径。需要预览时附加 `--preview-dir runs/新目录` 或有桌面时 `--show`；
预览目录必须是新目录，按 q/Esc 退出窗口。绿色框表示二维有效，三维另查 `position_valid`。
常用运行只输出必要日志；不再把测试输出复制进 `examples/` 或每次生成另一份验证报告。

## 可选 YOLO 和 RealSense

这一分支用于后续模型检测与 RGB-D 观测实验；前面的普通 USB/传统灯条流程不依赖它。

自定义权重的类别固定 `0=target_red`、`1=target_blue`；`config/aim.yaml` 已允许 `[0,1]`。
候选类别过滤不改变模型类别总数，检测两类不等于同时跟踪两个目标。训练、标注与 ONNX 导出见训练指南。

当前检测器仅支持 YOLOv8 detect 的单个 float32 原始输出 `[1,4+C,N]` 或 `[1,N,4+C]`，
程序处理 letterbox 逆映射、类别过滤和 NMS。YOLOv5、分割、姿态或已融合 NMS 的输出不适用。

D435i 已连接、SDK 已编译且双类别模型已准备好时：

```bash
# 基础构建显式关闭过 RealSense，使用前需重新启用并构建。
cmake -S . -B build_cpp -DAIM_WITH_REALSENSE=ON
cmake --build build_cpp -j2
./build_cpp/demo_aim_cpp --source realsense \
  --weights models/target_red_blue_v1.onnx --num-classes 2 --input-size 640 \
  --frames 0 --threads 2 --output runs/camera.jsonl
```

按需增加 `--serial 实际序列号`、`--show`。默认 CPU 后端和两个 OpenCV 工作线程，摄像头起始设置为 640×480/30 Hz；
这不是机载推理帧率保证。可用 `uname -m`、`lsusb`、`lsusb -t` 检查平台和实际 USB 协商。
相机深度对齐到彩色，使用 SDK 反投影；未知标定或无有效深度时不伪造三维点。

输出 schema=2：二维看 `detection_valid`，视线看 `bearing_valid`，米制位置看 `position_valid`；
旧 `valid` 保持三维含义。光学系为 x右/y下/z前，`point_body_m` 仅在外参已标定时提供 FLU 坐标。
其余字段、回放格式与时钟约定集中在接口说明中。

## PX4 / MicoVIO 台架通信

使用已有官方 MAVLink v2 生成头文件，路径必须包含 `common/mavlink.h`，不要只复制单个头文件。
以下路径存在于当前开发机；换机需提供对应头文件目录：

```bash
cmake -S . -B build_cpp -DCMAKE_BUILD_TYPE=Release -DAIM_WITH_MAVLINK=ON \
  -DAIM_MAVLINK_INCLUDE_DIR=/home/user/PX4-Autopilot/build/px4_sitl_default/mavlink
cmake --build build_cpp -j2
ctest --test-dir build_cpp --output-on-failure
./build_cpp/px4_localization_cpp --help
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
./build_cpp/px4_localization_cpp --config config/px4_link.yaml --seconds 30
```

约每 0.1 s 一行 JSON，检查 `packets` 增长，`bad_frames` 不持续增长，`clock_ready` 和 `estimator_ready` 有效，
并且 `pose.valid=true`。原地静止及按轴平移/旋转时，检查坐标符号与实物一致。
`pose=null` 表示当前没有可用定位；`link.reason` 是最近事件原因，不能代替当前时效标志。
`reset_required=true` 时旧场地对齐失效，重建会话后重新确认原点，不能继续追踪旧坐标。
串口断连报错退出，不自动沿用旧原点重连；可用 Ctrl+C 结束。

坐标完整链路为 `map_T_odom × odom_T_body(图像时刻) × body_T_camera × p_camera`。
MicoVIO 安装/融合、相机外参、相机时钟和场地原点均需实测；USB/UART 不产生“开发板坐标系”。
具体 NED/ENU、FRD/FLU、复位和插值约定见接口说明。

## 可选 ROS 2 诊断

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build_cpp -DAIM_WITH_ROS2=ON -DAIM_WITH_REALSENSE=ON
cmake --build build_cpp -j2
./build_cpp/demo_aim_cpp --source realsense \
  --weights models/target_red_blue_v1.onnx --num-classes 2 \
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
6. 在回放与闭环仿真中验证控制执行、控制权、命令时效、整段轨迹碰撞和失效行为，再验收真机对准与停住。
7. 前述通过后，再独立增加接触区域、接触反馈、恢复/返航和精确降落；YOLO/RKNN 优化按需求后置。

分阶段通过条件见 [视觉无人机路线图](docs/visual_drone_roadmap.md)。
当前世界目标协方差仍为空；PnP 测距精度、多目标身份、真实 ONNX 准确率、室外深度、
ROS 实际传输、香橙派持续性能和完整实机闭环均未由软件回归证明。所有真机几何和控制阈值都需实测。
