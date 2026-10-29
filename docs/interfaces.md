# C++ 接口与数据契约

本页统一维护视觉、定位、坐标与任务接口；构建和运行见 [主 README](../README.md)。
代码为 C++17。以下描述当前保留的公共接口、库和输出约定。更新：2026-10-26。
2027 场景与外部仿真入口见 [RMUC 2027 集成说明](rmuc2027_integration.md)。

## 模块与依赖

- `src/vision/`：core.cpp 处理观测确认/可选测距，observation.cpp 序列化，light_bars.cpp 等实现传统灯条与装甲候选检测。
- `src/input/`：合成、可选 RealSense，以及 UVC 采集进程与控制。
- `src/navigation/`：geometry.cpp 刚体变换，localization.cpp 位姿历史和世界目标，mission.cpp 任务决策。
- `src/io/`：flight_state.cpp 遥测缓存，mavlink_link.cpp 协议与同步，serial_port.cpp Linux UART。
- `src/app/`：程序入口、标定、日志与可选 ROS 输出；`camera/`、`armor_auto/`
  各自组织参数、循环和预览，`depth/` 保留通用测量输出，`common/` 复用严格整数解析与 FPS 统计。

`aim_observation` 提供观测数据契约与序列化，`aim_flight_state` 提供遥测缓存。
`aim_core` 保留视觉流水线及对这两个库的公共依赖，现有视觉调用者仍可链接它。
`aim_input` 和 `aim_navigation` 直接依赖观测契约，应用复用 `aim_light_bars` 提供实时检测；
`aim_mavlink` 依赖导航和遥测缓存，应用负责装配。
`aim_app_common` 集中编译严格整数解析与帧率统计，供相关应用和回归目标链接。

`aim_camera_stream` 集中构建共享帧、进程监管与 UVC 控制；MaixCAM Lite 的主机 UVC RGB 程序额外链接 `aim_uvc_worker`。
内部共享内存文件描述符和流状态常量定义在
`include/demo_aim/detail/input/uvc_stream_protocol.hpp`。
共享时钟函数的实现位于 `src/input/uvc_stream_protocol.cpp`，由 `aim_camera_stream` 编译。
`aim_armor_output` 供自动装甲应用及输出回归使用；`aim_depth_output` 与通用深度平面算法保留供无硬件回归和后续调用，
该库当前没有对应的独立实时采集应用。两套坐标和有效性契约保持独立。
核心与导航不链接相机 SDK 或 ROS。生产目标位于根 `CMakeLists.txt`，
回归目标和测试分组位于 `test/CMakeLists.txt`；测试助手仅供回归目标使用。

自动装甲在耗时处理结束后取得时间/帧龄快照，JSON 和预览使用同一快照；通用深度输出保留独立的时效检查。
帧龄非有限、负值、超过 250 ms，或采集失败/缺帧时，输出测量无效且距离/角度为 `null`。
停止记录清空测量值与观测时刻，消费端仍须独立检查时效，避免沿用停止前的最后有效记录。
自动装甲退出保存截图时，重新用最后原图、对应检测和刷新后的时效状态生成叠加图，不复用限频窗口缓存。
过期/故障时仍保存原图用于诊断，叠加图显示无效状态；已有截图目录拒绝覆盖。

当前头文件与实现分离的范围为 `no_demo` 应用工程；PX4 与第三方 SDK/生成库保留各自布局。
`src/` 仅保存 `.cpp` 实现；稳定公共接口位于 `include/demo_aim/`，
内部头统一位于 `include/demo_aim/detail/`，按对应源码保留 `app/`、`input/` 和子模块路径。
例如应用头使用 `demo_aim/detail/app/armor_auto/output.hpp`，采集头使用
`demo_aim/detail/input/uvc_stream.hpp`；同名 `options.hpp`、`application.hpp` 由模块路径区分。
所有这些头均从 `include/` 搜索根引用；测试也使用该根，不再添加 `src/` 搜索路径。
`detail/` 不作为稳定公共 API，C++ 命名空间沿用现有模块划分。
`Pipeline`、`PoseHistory`、`FlightStateCache`、`MavlinkLink`、`MissionMachine` 均按单线程串行访问设计。
实时集成时应把串口接收与耗时图像处理解耦，使用有界队列和显式同步，不能积压旧观测。

## 输入与时间

`Frame.color_bgr` 是原图分辨率 CV_8UC3。`depth_m` 可为空；存在时是对齐到彩色图的
CV_32FC1 米制轴向深度，不允许用常数假深度替代单目输入。

内参未知保持全零；有效内参必须匹配图像大小。只有上游已去畸变的图像才设
`rectified=true`，否则需提供 SDK/标定反投影回调 `deproject(u,v,z)`。
回调应支持任意正轴向深度，尤其 z=1 的视线反投影。
没有有效标定时仍可检测，但主点误差、视线角和三维点为空。

`capture_s` 是采集时刻；`received_s` 是可选的图像到达时刻；`process(..., now)`
是处理入口时刻，三者必须处于同一时钟域并满足 capture ≤ received ≤ now。
`host_monotonic` 表示本机 steady_clock，`replay_monotonic` 表示合成输入和离线测试使用的虚拟时间。
不要直接填入 Unix 时间或 PX4 time_boot_ms。`clock_valid=false` 会拒绝该帧。
旧字段 `emitted_monotonic_s` 仍表示处理入口，不包含编码、写盘和发送完成时间。

```cpp
#include "demo_aim/core.hpp"
aim::Config cfg;
cfg.class_ids = {0, 1}; // 0=target_red，1=target_blue
// pipeline 应跨帧保留，默认需连续三帧确认。
aim::Pipeline pipeline(cfg);
aim::Frame f;
f.color_bgr = bgr_image;
f.source = "usb_color";
f.sequence = sequence;
f.capture_s = mapped_capture_time;
f.received_s = receive_time;
// 单目输入不填写 depth_m；内参未知时不伪造焦距。
aim::Detection d{{100, 80, 160, 140}, .9, 0};
auto result = pipeline.process(f, {d}, aim::monotonicNow());
auto payload = result.json();
```

代码片段中的图像、sequence 和时间由未来采集适配器提供，不是已实现 USB 启动命令。

`demo_aim_cpp --source realsense` 复用传统灯条检测，通过 `src/vision/light_bar_observation.cpp` 的 `detectArmorObservations` 提取装甲候选并转换为 `Detection`，
再交给 `Pipeline` 做关联和深度测量。`--detector-config` 默认读取 `config/light_bars.yaml`；
`--config` 默认读取 `config/aim.yaml`，分别管理图像检测阈值与观测有效性条件。
合成输入直接注入检测框，不运行图像检测。类别 `0=target_red`、`1=target_blue` 表示红蓝颜色，
不表示敌我或装甲数字身份。候选的 `confidence` 来自几何评分，不是类别概率。

## 灯条与装甲板几何

`Detection.features` 是可选项。灯条顺序左、右，每条有 top/bottom 两端点；
角点顺序左上、右上、右下、左下，按装甲板对应关系定义。
`geometry_layout_id` 标识物理几何版本；不会自动把外壳 135×125 mm 当作灯条端点尺寸。

所有特征使用原图像素坐标，必须有限、位于图像内且非退化；四角还需构成上述顺序的凸四边形。
非法特征丢弃并报告 `features_reason=invalid_features`，有效检测框可独立保留。
数值校验不证明特征确实来自灯条。预览只绘制上游实际提供且通过校验的特征，
不会仅凭边界框生成角点；青色为灯条，紫色为角点。

## schema=2 输出契约

- `detection_valid`：本帧二维观测经过时效、过滤、关联和连续确认。
- `bearing_valid`：确认的二维观测有可用反投影模型。
- `position_valid`：深度/三维确认通过；旧 `valid` 保持同样的三维含义。
- `tracking_state`：searching、acquiring、tracking、lost。
- `position_reason`：三维不可用的具体原因，不覆盖二维确认结果。
- `light_bars_uv`、`corners_uv`：像素特征，缺失为 null。
- `bearing_camera_unit`：光学系单位视线向量，没有距离。
- `point_camera_m`：光学系 x右/y下/z前；`point_body_m` 需已标定外参，机体系 FLU。
- `optical_depth_m`：轴向 Z；`range_m`：欧氏距离，两者不能混用。
- `image_velocity_px_s`：相邻匹配帧框中心的像素差分，包含相机运动，不是世界目标速度。
- `position_covariance_m2`：当前 null，未把 MAD 或检测置信度冒充位置协方差。
- `contact_permitted` 固定 false；`aligned` 保留三维有效后判断主点对准的旧行为。

二维确认和三维确认分开计数。深度故障不使已确认二维框失效，三维恢复仍需重新确认。
丢目标时只在保持窗口内保留 track_id，框、特征、视线、速度和三维点全部清空。
重获目标后不跨丢失间隔计算速度。来源、时钟域、分辨率或 frame_id 改变会重新确认。
track_id 只在进程内唯一。当前仍选定一个目标，同类 IoU 关联不等于多目标跟踪，
不包含自运动补偿、光流、运动预测或目标交叉时的身份保证。

旧消费者不要再用 `valid=false` 推断没有框；二维消费者使用检测看门狗：

```cpp
if (aim::isDetectionCurrent(result, now, 0.2)) { /* 二维数据 */ }
if (aim::isCurrent(result, now, 0.2)) { /* 三维数据 */ }
```

实时看门狗默认 host_monotonic。消费合成输入等虚拟时间观测时，须显式传第四参数 replay_monotonic。
跨域、过期、未来或非有限时间均拒绝。停止发布时消费端仍需自己的超时机制。
异常诊断也使用 schema=2，所有有效标志为 false，未知采集时间与时钟域为 null。

## demo_aim_cpp JSONL 文件管理

`src/app/json_log.cpp` 的 `JsonLog` 管理 `demo_aim_cpp --output FILE`：默认单份最多 10 MiB，
最多保留 5 个日志文件，包括当前 `FILE` 和 `FILE.1`～`FILE.4`；`.1` 为最近历史，`.4` 为最旧历史。
每次启动先轮转已有当前文件；运行中下一条记录会使当前文件超限时，再轮转后写入。
超过单份限额的记录直接报错，不拆行。CLI 没有新增限额参数，内部构造可调整单份大小和保留数量。

`JsonLog` 生命周期内对当前文件和各备份路径持有 `flock` 排他锁，避免不同输出名与备份名重叠时并发写入。
默认对应 `FILE.lock`、`FILE.1.lock`～`FILE.4.lock`，最多 5 个零字节锁文件，永久留用且不计入 5 个数据日志。
任一对应路径被其他进程占用时构造明确失败，可改用与这些路径不重叠的其他 `--output`。
以 `.lock` 结尾的路径保留给锁元数据，不能作为日志输出。
轮转只操作精确的当前/历史路径，不扫描或清除旧版 `stem_N.ext`、其他输出前缀或 `runs/` 中的历史实验；
新逻辑不再递增创建 `_N` 编号。`armor_auto` 不使用这套 `JsonLog`，仍独立追加到指定输出文件。

## 飞控状态缓存

```cpp
#include "demo_aim/flight_state.hpp"
aim::FlightStateCache cache(1, 1, 1.0, 0.2); // system, component, 心跳/其他超时秒数
cache.updateHeartbeat(1, 1, {false, 0}, {receive_time, std::nullopt});
cache.updateAttitude(1, 1, attitude, {receive_time, remote_boot_ms});
auto snapshot = cache.snapshot(aim::monotonicNow());
```

`MavlinkLink` 解码器可将真实遥测填入：姿态为 body-FRD 到 local-NED 的单位四元数 wxyz；
位置、速度分别为 NED 米与米/秒。缓存拒绝其他 system/component、非有限值和重复/乱序消息。
心跳、姿态和位置分别过期；没有收到或超时的字段是 null。
位置的 xy/z/velocity 有效标志默认 false，需接收端依据实际估计器状态赋值。
不能仅凭收到 LOCAL_POSITION_NED 报文证明定位有效。

飞控重启、boot 时间回绕或换设备需显式 reset，缓存不会静默接受远端时间倒退。
这是接收器之后的缓存；`px4_localization_cpp` 已接入串口接收循环，原 `demo_aim_cpp` 仍独立运行。缓存类本身不负责连接。
当前仅依据接收年龄判断时效，不保证遥测在飞控端的真实测量年龄。
新增定位模块已提供时间同步、有限状态历史和拍摄时刻插值；世界坐标仍需实测相机外参及当前原点对齐。
不得直接把最新姿态与旧图像组合为目标世界位置。

## 坐标、同步与世界目标

本实现专门针对核对过的 **PX4 fused ODOMETRY**：

- `frame_id=LOCAL_NED`：位置 `(N,E,D)` 变为内部 `odom_enu=(E,N,U)`。
- `frame_id=LOCAL_FRD`：世界参考的任意航向 FRD，转为 Z 向上的 `odom_flu`，不能把它叫地理东北天。
- PX4 的 `q` 是 **机体 FRD 到 parent** 的旋转，转换为 `body_flu_to_odom_wxyz`。
- `child_frame_id` 描述**速度表达坐标**，不能用它推断姿态四元数的机体系。
  支持与 parent 相同的速度系或 `BODY_FRD`；后者先由姿态转入 parent 再变换坐标轴。
- 不支持的坐标、非单位四元数、非有限位置、未知/非法/超过阈值的位姿协方差均拒绝。
  `quality=0` 表示未知；以独立估计器状态与协方差共同检查，不当成质量满分。
- 输出单位米、米/秒、秒，四元数 wxyz。位置已由 PX4 表示在车辆参考点；模块安装杆臂应先在 PX4 正确配置。

相机光学系为 x 右/y 下/z 前，机体系为 FLU（x 前/y 左/z 上）。目标变换：

```text
p_map = map_T_odom × odom_T_body(图像采集时刻) × body_T_camera × p_camera
```

`body_T_camera` 是实测相机到机体外参，包含旋转和安装偏移。
`map_T_odom` 是当前定位原点到场地/机库世界系的对齐。它们默认缺失；不会用单位阵假装已经标定。
USB 和串口只是传输，不产生“香橙派坐标系”。没有 GPS 时的 map 是本地米制世界系，不是经纬度。

对接独立 2027 地图时，`map` 采用地图工程的米制、XY 居中、Z 向上坐标；Z=0 是底板下表面，
不是各处地面或飞机落稳后的机体原点高度。当前机库停车的共享配置 XYZ 指 `base_link`，
PX4 生成 pose 指模型根节点；红蓝方由 `config/flight_start.yaml` 选择，导航读取当前 SITL 快照。
原始 STL 的毫米转换和平移、机体与根节点高度、出生/起飞/目标配置，见
[2027 坐标与配置](rmuc2027_integration.md#地图与坐标基准)。不得把原始 STL 毫米坐标直接填入任务意图。

地图工程现有控制桥在已知 Gazebo/MAVROS ENU 轴向约定下，用机库上落稳静止的仿真真值标定原点平移；
该仿真专用标定不证明实机 `map_T_odom` 已知，也不证明 `no_demo` 的定位和视觉已经接入。
实机和本工程定位链路仍须提供当前会话的 `WorldCalibration`，校验 `time_domain`、拍摄时刻插值与 epoch。
ROS 仿真 `/clock`、本机单调时间及串口 TIMESYNC 的时间不能直接混用；尚无两工程间的时钟适配器。

TIMESYNC 通过往返时间估计飞控到本机的时钟偏移：至少 5 次稳定响应才启用；
拒绝 RTT 超过 20 ms 的样本、检查偏移变化，2 s 无新同步即失效。时间误差指标是估计值，
不是硬件同步精度保证。收到正确串口包不代表图像与飞控已经同步；相机采集时钟也必须映射到同一本机时钟。

`PoseHistory` 默认最多 256 点、2 s 历史，仅在间隔 ≤0.1 s 的有效位姿间线性插值位置、SLERP 插值姿态。
不外推，不跨定位失效、时钟域、坐标名或 epoch 插值。图像太新、尚无后一帧位姿时返回 `pose_not_bracketed`，
集成进程应有界等待后一帧或丢弃图像，不能拿最新姿态替代。

`PoseSample::validate()` 统一检查非负有限时间、接收不早于采集、非空时钟域/坐标系、正 epoch、
有限的可选速度及合法刚体姿态；`PoseHistory`、`poseInMap()` 与任务入口复用它。
该方法检查完整性，不证明定位健康或新鲜：上游 `valid`、当前时效、最大时钟误差仍须分别检查。
`poseInMap()` 不把非法元数据变成有效地图位姿；坐标映射本身不强加历史/任务各自的时间误差阈值。

`reset_counter` 或参考坐标变化会增加 epoch，使 `WorldCalibration.epoch` 不匹配、旧世界坐标失效。
时间大幅倒退会锁定 `reset_required`。epoch 仅在当前接收器进程会话内使用，不能把旧进程的 epoch=1
和新进程的 epoch=1 当成同一个原点；每次启动/重连必须重新确认 `map_T_odom`，不持久化“已对齐”状态。

## 视觉与定位的装配

以下为应用装配顺序，具体完整合成代码见 `src/app/mission_demo.cpp`：

```cpp
// 接收线程/事件循环：收到无效事件也要 push，让历史立即清空。
for (const auto &pose : link.feed(bytes.data(), bytes.size(), receive_time))
    history.push(pose);
// 每轮还需检查 diagnostics；同步/估计器失效或断连时 history.clear()。

// 视觉线程：solution 来自原 Pipeline，必须是真实米制目标点。
auto target = aim::projectTargetToMap(solution, history, calibration, now);
// 自身位置和目标位置使用同一 map 对齐：
auto vehicle = aim::poseInMap(current_pose, calibration);
```

这段展示接口组合，线程共享 history 需要加锁或由单一事件循环串行访问。
`projectTargetToMap()` 不从像素框估计距离。`demo_aim_cpp` 的纯彩色观测路径没有 PnP，仍只能输出二维/视线；
独立 `armor_auto` 已有基于标定和已知灯条尺寸的 PnP，但尚未接入此世界坐标流水线。
8～10 m 目标的有效三维观测、目标几何对应与实机标定仍需验证。
旧 RGB-D 管线默认深度阈值也不是 10 m 的有效测距保证。
输出世界目标协方差仍为 null，未把检测评分或 PX4 自身协方差冒充完整目标误差。

## 任务决策

`MissionMachine` 接收：同一 map 下的有效自身位姿、世界目标点、飞控健康/着陆状态、
显式开始/人工接管、接触授权与接触区域确认、独立接触反馈，以及整段碰撞验证回调。

典型路径：`idle → takeoff → search → approach → contact → recover → return → land → complete`。
它会检查时间、定位 epoch、目标身份、超时与健康；终止/完成状态锁存，须显式 reset 才能开始新任务。
目标丢失不等于接触成功；恢复/返航也要求定位和飞行健康仍可靠。
定位无效时输出中止/交接意图，不能假设一定有能力自动悬停或返航。
曾离地后在非 `land` 阶段收到新鲜的已着陆状态，会锁存 `abort / unexpected_landing`，
包括起飞爬升途中落回地面的情况；不会继续接近、请求接触或自动再次起飞。

接近阶段输出距离目标一定距离的站位点；到位后仅请求未来的接触控制器，
没有写入高速撞击轨迹、加速度控制律或机体碰撞后的自动恢复控制器。
`segment_clear(start,end)` 必须验证整段运动和机体包络；缺少回调或返回 false 时只输出 hold。
不能只检查两端，也不能把离线示例中“无障碍”的恒 true 回调用于真机。
返航路线、恢复点、机库位置由 `MissionConfig` 显式传入，当前示例全部是虚构测试坐标。
`mission_demo.cpp` 中的单位地图对齐、1 m 起飞高度、固定目标和恒 true 碰撞回调保留为离线夹具；
不能只替换成 2027 航点就宣称接通场地碰撞检查。真实适配应从完整静态占用图检查整段路径和机体包络，
并分别验证指定机库点的实体接触停放和完整飞行净空；停车通过不构成自动离库许可。

`no_demo` 尚无任务意图到外部规划器的目标适配器。独立地图工程已有只连接其本机 SITL 的 PX4 Offboard 控制桥，
当前默认用于原机库停车监控：静止标定后进入 READY，自动离库尚未认证，`auto_fly:=true` 进入 HOLD，
不请求 OFFBOARD、不解锁。该桥不消费本工程的观测 JSON 或 `MissionIntent`。
后续集成需定义目标接受/取消、时效与状态反馈，让规划器生成绕障轨迹，并保持唯一飞控控制出口；
不能并行启动第二个 Offboard 执行器。真机还需验证 ENU/map 到飞控当前局部系的逆变换、
限速/限加速度、受控接近及接触判断、接触后健康确认和精确进库降落；仅将 `x/y` 写进 JSON 不会自动控制飞行。
目标观测不是自身定位消息，不应放进 ODOMETRY 让飞控误认为飞机到了目标位置。

## MAVLink 传输边界

本实现读取用户本机 PX4 `ODOMETRY.hpp` 所描述的融合 `vehicle_odometry`，
不直接解析 MicoVIO 的未确认私有消息；固件约定需在实机确认。
MicoVIO 原始定位已经送入 PX4 后，香橙派不将 PX4 融合结果再次回灌为外部视觉测量。

接收器使用官方 MAVLink common 生成头文件和对象独立的解析状态；校验 CRC、来源 ID、分包与粘包。
未配置签名密钥，带签名报文会被拒绝，不宣称已经提供签名认证。
一次最多解码 4096 bytes；串口默认 8N1，读等待/写截止各 20 ms，断连报错退出。
时间同步只保留一个待决请求。唯一发送接口为 TIMESYNC，无参数修改或飞行命令。

姿态、位置和心跳是独立超时的缓存；导航插值使用经过时间同步与估计器质量检查的 ODOMETRY。
定位质量未知、协方差无效/过大、估计器健康失效、时钟异常都不能静默输出有效位置。
`LOCAL_POSITION_NED` 的有效标志还要求当前 PX4 心跳，按远端采样时刻而非仅接收时刻复查帧龄。
定位 epoch 改变后，之前接收的位置只保留为无效诊断，必须收到新位置才能恢复；新的健康报文或被拒收的重复位置包不能激活旧缓存。
该报文本身不含 reset_counter，接收端不能识别所有网络中迟到的重置前数据，导航仍使用有明确重置语义的 ODOMETRY。
`PoseHistory` 默认最多 256 点/2 s，不跨失效间隙插值。

后续飞控执行器还需定义唯一控制出口、命令截止时间、控制模式确认和失效交接。
独立接触反馈应包含采样时刻、触发锁存、健康及卡住/断线诊断，不能从目标消失推断成功。
这些是真机执行阶段的接口要求，不是当前已发布的自定义 ROS/MAVLink 消息。

参考：[MAVLink ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)、
[TIMESYNC](https://mavlink.io/en/services/timesync.html)、
[PX4 外部视觉定位](https://docs.px4.io/main/en/ros/external_position_estimation)。

## 验证记录口径

当前可重复回归源码位于 `test/`，推荐用 `./scripts/build.sh --test` 构建并执行；
已有构建也可直接用 `ctest --test-dir build --output-on-failure`。`test_1/` 是参考资料目录。
旧 2026-09-16 迁移摘要记录：当时 26 项 C++、22 项 Python 回归通过，60 帧合成共有字段逐帧一致
（容差 1e-8、差异 0），44 帧有效。
当时记录了默认、无 RealSense、ROS 构建检查，但没有证明实际相机、ROS 传输、检测精度或飞行能力。
旧 Python 实现与历史生成 JSON 已退出当前输入夹具；这些历史数字不替代当前 CTest 回归结果。
