# MaixCAM 画面预览与 C++ 识别接入

本模块于 2026-10-25 退出当前开发入口并归档，以下保留实验说明。归档位置和恢复方式见 [归档说明](../../README.md)。下列电脑端命令均从主工程根目录执行。

## MaixVision 预览与装甲板识别

在 MaixVision 连接设备，停止原先的 `stream_uvc.py`，打开本目录的 `preview_maixvision.py`，点击运行当前文件，在右侧图像预览区观看。确认预览没有暂停；需要换脚本时先停止当前程序，避免同时打开相机。

当前已在同一入口加入 MaixCAM 本机传统装甲板识别，顶部 `ENABLE_ARMOR_DETECTION=True` 默认开启；改为 `False` 并重新运行即可恢复纯预览。全部代码保留在一个脚本中，MaixVision 运行当前文件时无需上传其他 Python 模块或安装 OpenCV/NumPy。

当前按用户桌面上的红色装甲进行高速验证：`TARGET_COLOR="red"`，原图保持 640×360，只在中央白框内每帧检测。`DETECTION_WINDOW=(.40,.40,.20,.20)` 为归一化坐标，对应全图坐标 `[256,144,128,72]`；必须将两根灯条和整个装甲完整放进白框。框外不参与搜索，接近窗口边界也可能截短灯条。可改 `TARGET_COLOR` 为 `"blue"` 或 `"both"`，将 `DETECTION_WINDOW=None` 恢复全画面；这些更大搜索范围需重新测帧率。

曝光默认 `EXPOSURE_US=1500`（1.5 ms，手动）。启动先打印请求值，第 5 帧后用独立 getter 打印 `Exposure readback_us=...`；setter 返回请求值，不能据此认定硬件已读回。灯条依然过曝可试 1000，过暗可试 2000～3000；设为 `None` 恢复自动曝光，修改后重新运行。曝光调整用于保留灯条颜色和减轻白芯饱和，不能单靠它保证恢复帧率。

用户设备上上一版全图红蓝检测实测 `detect_ms=84～91`，而编码/提交约 14～15 ms，是从 40 FPS 降到约 9 FPS 的主要开销。当前 ROI 直接传给原生 `find_blobs()`，保留窗口内原始像素，不通过缩小整幅图像或跳过检测提高计数。角点仍为全图坐标，不额外叠加窗口偏移。最小灯条长度调为 6 像素，并为单像素宽竖直色块补充像素包围框处理，减少小灯因退化旋转矩形而丢失的情况；实际识别效果待现场验证。

相应将原生色块的像素数/包围框面积门槛调为 6/8，避免一像素宽的小灯在生成旋转矩形之前就被较大的面积门槛删除；随后仍须通过长宽比和装甲配对过滤。

对平均长度不超过 16 像素的候选，评分增加 1 像素的量化容差；长度比、纵向差、角度和间距的原始硬门槛仍须通过。它用于减少小灯条因整数角点而积累过大评分惩罚的情况，不代表已确认图中漏检就是这个原因。`small_target_tolerance_px=0` 可关闭该修正，真实识别率和误检率仍需现场测试。

识别流程是 SDK 原生 LAB 色块检测 → 灯条旋转矩形与长轴过滤 → 合并白芯两侧的近距同色碎片 → 同色灯条几何配对。两条灯之间画绿色装甲四边形，并显示红/蓝候选标记；画面上方显示 FPS、红/蓝灯条数和装甲候选数。只画当前帧的结果，空场景不会保留上一帧装甲框。候选数设有上限，避免杂乱背景拖慢配对。

这些框表示灯条几何候选；没有数字分类或相机标定解算，因此当前入口不输出装甲身份、距离或 yaw/pitch。设备端使用 MaixPy LAB 阈值，不能直接抄电脑端 `light_bars.yaml` 的 OpenCV HSV 数值。

现场先将真实发光灯条放入画面并调好镜头焦点。若灯条数为 0，使用 MaixVision 的阈值工具检查灯条颜色，调整脚本顶部 `LAB_THRESHOLDS`；顺序固定为红、蓝，每组为 `(L最小, L最大, A最小, A最大, B最小, B最大)`，L 范围 0～100，A/B 范围 -128～127。当前阈值是起始值，曝光过强导致整根灯条发白时，应先改善曝光或照明以保留颜色。若能画出两根同色灯条但没有装甲框，再按画面调整 `ARMOR_GEOMETRY` 的长度比、倾角、间距等配对限制。

此脚本通过 `display.send_to_maixvision(frame)` 直接发送真实画面，不创建 UVC 推流器或物理屏幕对象；无需启动 `ffplay`、`camera_light_test`，也无需重新编译 C++。它默认请求 640×360 / 60 FPS，左上角以黑底绿字显示 `FPS: xx.x`，约每半秒更新；首个统计窗口完成前显示 `FPS: --`。帧率按相邻取图时刻的间隔统计，包含循环中的检测、绘制、编码和预览提交耗时；该数值不保证电脑图像面板以同样帧率显示。保留 WARN/ERROR 日志，控制台每两秒另输出一次循环帧率及 `read_ms/detect_ms/draw_ms/encode_send_ms` 平均耗时，同时将检测拆为 `blob_ms`（原生颜色扫描）和 `geometry_ms`（Python 几何）。有灯条但未配对时，每两秒最多记录 4 根灯条的中心、长宽和方向，便于定位漏检。加入识别后能否维持至少 40 FPS，需在设备上重新实测。

参数集中在脚本顶部：`WIDTH=640`、`HEIGHT=360`、`REQUESTED_FPS=60`、`JPEG_QUALITY=80`，以及目标颜色、检测窗口、曝光参数。MaixPy 4.12.5 的 GC4653 实现对不超过 1280×720 且请求大于 30 FPS 的配置会选择高帧率传感器模式，当前先使用官方支持的 60 FPS，不降低输出分辨率或跳过预览来提高计数。若模式不稳定，可将 `REQUESTED_FPS` 改回 `30` 并重新运行。不同传感器模式的画面可能有轻微偏移，后续标定应使用实际运行模式。

画质目前按帧率优先调整：保留 640×360 输出，JPEG 质量由 85 回到 80，先降低预览传输负担。环境变量 `MAIXVISION_IMG_QUALITY` 在首次提交预览前设置，需重新运行脚本才生效；该设置针对 JPEG 预览。它不会增加像素数量，也不能修复镜头失焦。帧率余量足够后，可将质量调回 85，或另测更高分辨率；这些设置不能保证仍达到 40 FPS。

60 FPS 是请求值，能否持续达到 40 FPS 以上需要在设备与电脑端实测。MaixVision 预览发送队列满时可能丢帧，设备提交 FPS 达标不代表面板显示 FPS 达标。若未达到目标，用控制台分段耗时判断取图、绘制或编码/提交中的瓶颈。

必须在 MaixVision 点击运行，让设备建立预览连接。电脑终端或普通 SSH 执行此脚本会给出提示并退出。当前已检查语法及 SDK API，实际预览帧率仍需在设备上确认。

宿主机回归不访问相机，归档后可运行 `python3 -B -m unittest discover -s archive/maixcam_20261025/test/input -p 'test_maixcam_*.py'`。这些检查覆盖几何和 SDK 调用路由，不能代替真实灯条、现场阈值及设备性能测试；已从主工程 CTest 登记中移除。

## 通过 UVC 接入电脑端 C++ 识别

2026-10-25 初次检查：新设备为 `359f:2120 sipeed maixcam`，当时仅提供 RNDIS/NCM USB 网卡。后续已确认 UVC 启用成功：`/dev/video4` 为 MaixCAM 图像采集节点，`/dev/video5` 为元数据，640×360 / MJPG / 30 FPS 已列入支持模式；`/dev/video0–3` 都是内置 ASUS 摄像头。编号可能随插拔变化，以启动器实际检查为准。

下面两段设备端程序需要通过 **MaixVision 在 MaixCAM 上运行**，不要在电脑终端执行。实现按旧版 `sys.device_id() == "maixcam"` 官方系统配置方式编写；其他型号会明确停止，避免误改配置。

1. MaixVision 左下角连接，扫描并选择 MaixCAM。`10.191.124.1` / `10.191.125.1` 在本次连接检查中均已 ping 通；电脑自身地址为对应的 `.100`，不要将其填成设备地址。
2. 在 MaixVision 打开 `enable_uvc.py`，运行当前文件一次。仅新增 `/boot/usb.uvc`，保留现有 USB 网卡配置；不切 HOST，不自动重启 USB，不更改开机应用。若报告系统标识或文件布局不符，请保留报错以核对固件。
3. 显示启用成功后，停止脚本，断电重新连接 USB，等待启动完成并重新连接 MaixVision。UVC 枚举会改变 USB 连接，原 MaixVision 连接可能断开。
4. 在 MaixVision 打开并运行 `stream_uvc.py`，保持运行。它将每帧真实图像编码为质量 80 的 JPEG 后送入 UVC。默认关闭 MaixVision 同步预览，减少重复编码和网络传输；此时 MaixVision 预览不更新是预期行为，以电脑端识别窗口为准。只开启 UVC 或只运行电脑端程序不会自动启动此取图循环。
5. 在 Ubuntu 电脑终端执行：

```bash
cd /home/user/nohuman_plane_std/no_demo
python3 archive/maixcam_20261025/tools/maixcam/run_light_test.py
```

启动器按 USB 归属查找 MaixCAM，查询并确认 MJPG 采集节点与 640×360 模式，再启动现有 `build/camera_light_test`。它设置系统 GSettings 路径，避开 Snap Code 的 GTK schema 冲突；跳过普通 UVC 曝光/对焦控制查询。它不会用 ASUS 摄像头兜底；没有节点或模式未确认时会输出诊断并退出。实际协商 FPS 以启动日志为准。

只检查节点和模式：

```bash
python3 archive/maixcam_20261025/tools/maixcam/run_light_test.py --check
```

按 `s` 保存截图，按 `q` / `Esc` 退出。新相机与分辨率需要独立标定；当前使用无内参的 `light_bars.yaml`，只验证画面、灯条及装甲候选框。

## 黑屏检查

电脑端的 `run_light_test.py` 只接收 UVC 视频。每次通过 MaixVision 重新运行本地最新版 `stream_uvc.py` 后，才会将新代码发送到相机；编辑本地文件不会自动更新正在运行的设备端程序。

2026-10-25 实测进展：设备为 MaixPy 4.12.5 / GC4653；用户设备日志的源 RGB 抽样均值为 116～154，主机限时采集也已取得真实的 640×360 彩色图像。但该次采集仅约 2～4 FPS，并出现 `Corrupt JPEG data: premature end of data segment`。这说明链路能够出图，仍需验证传输稳定性；不能据此宣称已稳定达到 30 FPS。

新版脚本先显式编码一次 JPEG，再交给 UVC；默认不向 MaixVision 发送第二路预览，也不复制整帧 RGB 到 Python 做采样。它会输出各环节的平均耗时：

```text
MaixCAM stream: ... FPS produced=... read_ms=... jpeg_ms=... uvc_ms=... preview_ms=...
```

后续日志已确认：关闭 RGB 诊断后，前约 5 秒可达到 29～30 FPS，随后第 180 帧附近骤降到不足 1 FPS；未启动电脑播放器也能出现。因此尚不能将其归因于分辨率、播放器或硬件性能上限。

当前脚本新增 `log.set_log_level(log.LogLevel.LEVEL_WARN, False)`，屏蔽 SDK 在每次 `UvcStreamer.show()` 时同步打印的 `_using_jpeg: 1` INFO 行，保留 WARN/ERROR 和每秒 Python 统计。启动行应包含 `sdk_log=WARN`，逐帧 INFO 行应消失。此改动用于验证输出缓冲或 MaixVision 日志转发反压是否造成持续运行后的降速；尚未通过设备实测确认根因。重新运行后需观察至少 60 秒，并对比开始与结束时的帧率，不能仅以启动几秒达到 30 FPS 判定稳定。

`read_ms` 是取图时间，`jpeg_ms` 是编码时间，`uvc_ms` 是 SDK 提交时间，`preview_ms` 是可选预览调用时间。这些是设备端耗时，`uvc_ms` 不代表 USB 传输已完成；RGB 抽样、日志及其他循环开销不计入这四项，因此不能仅用它们的和换算总 FPS。若日志停在 `MaixCAM init: opening camera...`，则是打开相机尚未返回，还没有开始推流。

- 在 MaixVision 控制台确认持续出现 `MaixCAM stream: ... FPS produced=...`，且 produced 持续增长。低于 1 FPS 时日志也会变慢，因为循环返回后才能统计。若有异常，先保留完整 Traceback；不要只启动 `enable_uvc.py` 后就直接运行电脑端接收器。
- 需要确认源图像是否为黑图时，临时将 `RGB_DIAGNOSTICS` 改为 `True` 并重新运行，日志才会出现 `sample_rgb_mean/min/max`。它会复制完整 RGB 图像，可能显著影响测得的帧率，检查完应关闭。
- 源 RGB 抽样正常但电脑端黑屏：继续检查 UVC 发送与主机端日志。此时调整灯条的 Min S / Min V 不会恢复原始画面。若确需对比 MaixVision 预览，可把脚本顶部 `MAIXVISION_PREVIEW` 改为 `True`，重新运行；预览最多 5 Hz，并复用已编码的 JPEG。
- 两边都黑且 `sample_rgb_mean`、`max` 持续接近 0：采样到的源图像很暗，检查镜头遮挡、环境照明及相机曝光，再依据日志排查传感器；这些数值是 RGB 字节抽样，不是完整图像统计或曝光测量。
- 启用 MaixVision 预览后，预览不更新时也要确认没有暂停。设备端 FPS 表示取图、编码与提交循环吞吐；电脑端 FPS 是接收处理吞吐。后台 UVC 服务可以重复发送缓冲中的旧 JPEG，因此电脑端高 FPS 不等于传感器产生了同样多的新画面。停止设备脚本后显示旧图，也不能证明仍在取图。

新版单次编码路径通过了 6 项宿主机模拟接口检查；上述实际彩色图来自修改前的限时采集，新版在设备上的帧率与稳定性仍待验证。无需重新编译 C++，但必须在 MaixVision 重新运行更新后的设备脚本。

依据：[官方相机 API](https://wiki.sipeed.com/maixpy/api/maix/camera.html)、[原生色块与绘图 API](https://wiki.sipeed.com/maixpy/api/maix/image.html)、[官方 UVC 文档](https://wiki.sipeed.com/maixpy/doc/zh/video/uvc_streaming.html)、[USB 配置示例](https://github.com/sipeed/MaixPy/blob/main/examples/tools/maixcam_switch_usb_mode.py)、[UVC 串流示例](https://github.com/sipeed/MaixPy/blob/main/examples/vision/streaming/uvc_stream.py)、[UvcStreamer 实现](https://github.com/sipeed/MaixCDK/blob/main/components/vision/port/maixcam/maix_uvc_stream.cpp)、[无物理屏幕的 MaixVision 预览 API](https://wiki.sipeed.com/maixpy/api/maix/display.html#send_to_maixvision)。
