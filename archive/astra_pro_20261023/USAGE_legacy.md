# Astra Pro 历史用法与实验记录

归档日期：2026-10-23。当前项目已切换为 YLR4K USB/UVC 相机。
本文件从切换前的主 README 原样复制 Astra Pro 使用章节、相关配置说明和历史验证记录，供追溯。
以下命令和相对路径对应归档前的项目布局；Astra 专用程序、配置和脚本现保存在本归档目录的原相对路径下，未接入当前主线构建。
原独立深度相机参考仓库：[meichuan951-maker/shendu_camera](https://github.com/meichuan951-maker/shendu_camera)。

## 原 README：历史构建验证

2026-10-12 项目首次推送前验证：开发机构建成功，Astra 开启、MAVLink 关闭，12 项 CTest 全部通过。
2026-10-14 结构整理验证：所有可选后端关闭时 12 项通过；RealSense、ROS 2、MAVLink、Astra
全部启用时 15 项通过。合成和 PNG 回放的逐帧 JSON 与整理前的已构建程序一致，安装入口包含六个程序。
2026-10-16 规则与异常路径复查：基础配置 13 项、全部可选后端开启 16 项 CTest 全部通过。
新增/扩展截图同帧、IPC 描述符冲突、非法地图位姿、意外落地以及 MAVLink 采样过期/心跳/epoch 回归；
合成、纯 RGB 合成和 PNG 回放在基础/全可选构建中的逐帧 JSON 一致。Astra SDK 缺厂商驱动时在配置阶段明确报错。
2026-10-17 构建整理验证：基础配置 14 项、全部可选后端开启 17 项、恢复原有配置后 15 项 CTest 全部通过。
当前保留 RealSense／ROS 2／Astra 开启、MAVLink 关闭；统一入口、禁用旧产物归档及缓存保留回归通过。
以上未打开真实相机或飞控，不作为赛事执行、测量精度或飞行验收结论。

## 原 README：Astra RGB UVC 预览示例

使用仓库中这台 Astra 的配置时，示例为：

```bash
./build/camera_light_test \
  --device /dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0 \
  --format YUYV --config config/astra_light_bars.yaml --mask
```

## 原 README：自动装甲章节（含当时的 Astra 配置和现场记录）

## 自动装甲识别、跟踪与 RGB 相机标定

当前自动装甲视觉入口使用 `armor_auto`，**不需要鼠标框选或 Astra 深度 SDK**：每个新彩色帧提取灯条、组合装甲候选，
按配置筛选颜色，连续 3 帧确认后选定目标。在关联距离/尺寸门限内保持同一目标，检测丢失立即使当前结果无效；
超过 0.25 s 后重新搜索。`track_id` 是临时跟踪编号，不是装甲数字身份，当前未接入数字分类器。

配置文件为 `config/armor_auto.yaml`，颜色按用户约定：

```yaml
target_color: 1 # 蓝色；改为 2 选择红色。修改后重启程序。
```

当前默认 `1`（蓝色），匹配现场蓝色靶；不识别板上的数字。先根据 `camera_light_test --list` 的结果修改配置中的
`camera.device`、`width`、`height`、`fps` 和 `format`，不要直接照搬别人的设备编号。
默认配置使用此前 Astra Pro UVC 的固定路径、640×480、30 FPS、YUYV。
这些参数来自配置，不代表另一台相机也支持同样模式。驱动实际尺寸不一致时，`armor_auto` 会报错退出。

当前测试目标已确认灯条中心距 0.135 m、发光长度 0.056 m；换装甲板必须更新实际尺寸，
核对后才可设置 `armor.dimensions_confirmed: true`。
`config/astra_light_bars.yaml` 只针对本次 Astra 过曝灯条适当放宽饱和度门限，不修改其它相机配置。
更换相机时同时核对 `detector_config` 和 `rgb_calibration`；它们的相对路径以主配置所在目录为基准。

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

脚本单独构建 `build/astra_depth_check`，不改变现有 CMake 配置或主识别程序。
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

同时启动 Astra 彩色装甲候选与深度测量窗口：

```bash
cmake --build build --target armor_auto astra_live_measure -j2
bash ./scripts/astra_armor_live.sh
```

彩色硬件参数在 `config/astra_rgb_controls.yaml`。双窗口启动脚本使用
Python 3/PyYAML 和 `v4l2-ctl` 应用配置，并检查范围、步长和驱动读回值。
`backlight_compensation: 0` 关闭背光补偿，已实测比默认 1 更暗、灯条光晕减轻，
但白芯未完全消除；该参数不是曝光时间。2026-10-12 本机 Astra Pro RGB
标准 UVC AE/曝光请求均返回 PIPE，尚未取得可用手动曝光接口。
不要把识别阈值或软件调暗当成恢复已饱和的颜色/边缘。
运行中修改并立即应用：`python3 tools/astra_rgb_controls.py --apply`；
省略 `--apply` 只检查当前值，不写入。驱动读回成功不等于图像质量验收。

双窗口脚本默认启用**中心准星测距**：把深度图中心白色小框完全放在装甲板内部平整区域，
有有效深度且通过平面质量检查时直接显示距离，无需鼠标拖框；
中心测到墙面也会显示距离，`source=center_depth_surface`、`armor_identity_verified=false`，
不能将它当成已经自动关联到彩色检测框的装甲中心。
按 `c` 回到中心探针，拖框或按 `r` 切换手动区域；
启动时加 `--manual-depth` 可关闭默认中心探针。
单独运行可用 `./scripts/astra_live.sh --center-depth --local-world-fixed`。

两个窗口分别显示最近约一秒的新帧 FPS。深度窗口对中心小框或手动选择的装甲内部平整区域，
显示三维直线距离、前向深度、所选表面有效点均值的相机 XYZ 和固定局部世界 XYZ。
局部世界原点为深度相机光心，X 向前、Y 向左、Z 向上；
由相机光学系转换为 `[Z, -X, -Y]`，相机必须固定不动。
这是用户定义的局部世界系，不是场地地图或 VIO 定位；所选点均值不保证是装甲几何中心。
手动模式未选区域、无效深度或断流时显示 `n/a`、JSON 坐标为 `null`。
彩色候选与深度区域仍未自动配准，手动区域不跟踪目标，目标移动后应重新框选。
任一窗口按 q/Esc 退出双窗口；结果保存在启动时打印的 `runs/astra_armor_live_*/`。
单独深度入口可使用 `./scripts/astra_live.sh --local-world-fixed` 启用相同局部世界坐标。

此前 Astra Pro 的 RGB-D 出厂标定读回 NaN，尚未完成彩色识别框到深度的有效映射。
`astra_live_measure` 因此采用独立的**深度图手动框选模式**：没有彩色自动配准，也没有目标跟踪。
角度和 XYZ 使用 SDK 深度反投影，完整几何精度尚未标定验证，窗口和 JSON 均标注为初步估计。

```bash
./scripts/build.sh -- -DAIM_WITH_ASTRA=ON
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

