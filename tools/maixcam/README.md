# MaixCAM Lite 相机入口

当前项目选择 **MaixCAM Lite（无屏幕版本）**。本目录已于 2026-10-28 从历史快照恢复为开发入口；设备端脚本使用 MaixPy，在 MaixVision 中运行，电脑端启动器使用项目现有的 C++ 检测程序。

## 设备端预览与传统灯条检测

1. 用数据线连接 MaixCAM Lite，在 MaixVision 左下角扫描并连接设备。连接地址以当前扫描结果为准。
2. 停止其他占用设备相机的脚本，打开本目录 `preview_maixvision.py`，点击运行当前文件。
3. 在 MaixVision 图像区观看预览；脚本通过 `display.send_to_maixvision(frame)` 发送画面，不创建物理屏幕对象，适用于 Lite 的无屏幕使用方式。

预览默认请求 `640×360 / 60 FPS`、JPEG 质量 `80`，顶部显示循环实测 FPS，控制台输出取图、检测、绘制和编码/提交耗时。**60 FPS 是请求值，本次恢复未进行 MaixCAM Lite 的真实帧率验收。** 传感器、系统版本、曝光、检测区域和预览传输都会影响实际吞吐；设备提交帧率也不等同于电脑图像区刷新率。

当前保留已有灯条检测参数：`ENABLE_ARMOR_DETECTION=True`、`TARGET_COLOR="red"`，`DETECTION_WINDOW=(.40,.40,.20,.20)` 只搜索中央白框。要检测其他位置，修改区域或设为 `None` 搜索全画面；颜色可选 `"red"`、`"blue"`、`"both"`。全画面检测的帧率需重新实测。设 `ENABLE_ARMOR_DETECTION=False` 可先验证纯预览。

`EXPOSURE_US=1500` 为手动曝光请求，设为 `None` 使用自动曝光。脚本会在第 5 帧后尝试独立读回曝光；曝光和 `LAB_THRESHOLDS` 需要根据 Lite 的镜头、照明和真实发光灯条调整。MaixPy 的 LAB 阈值不能直接复制电脑端 OpenCV 的 HSV 数值。

检测使用原生 LAB 色块、灯条几何和同色配对，只给出当前帧候选框。它不输出装甲数字、米制距离或相机姿态；这些功能需要另行实现或通过下述 C++ 路径结合本机标定使用。修改本地脚本后需要在 MaixVision 重新运行，才会更新设备端代码。

## 可选：通过 UVC 接入电脑端 C++ 检测

只有需要把视频送给电脑端 `camera_light_test` / `armor_auto` 时才使用此路径。它与上面的设备端检测预览是两种入口；同时只能运行一个打开相机的设备端脚本。

1. 在 MaixVision 上运行 `enable_uvc.py` 一次。它按官方第一代 MaixCAM 的 USB 配置布局检查 SDK ID 和 `/boot/usb.dev`，仅创建 `/boot/usb.uvc`，保留现有 USB 网络配置；不自动重启 USB。若打印的 ID 或布局不匹配，会停止并要求核对系统镜像。
2. 停止脚本，手动断电重新连接，等待设备启动，再连接 MaixVision。
3. 在 MaixVision 上运行 `stream_uvc.py` 并保持运行。它请求 `640×360 / 30 FPS`，每帧编码一次 JPEG 后交给 `UvcStreamer`；默认关闭额外的 MaixVision 同步预览以减少负担。设备端真实视频需要这个持续取图循环，单独启用 UVC 不能代替推流。
4. 在电脑项目目录先检查 UVC 节点，再启动检测：

```bash
python3 tools/maixcam/run_light_test.py --check
python3 tools/maixcam/run_light_test.py
```

启动器按 USB 归属查找 MaixCAM 家族的采集节点，要求 MJPG 的 `640×360` 模式；有多个候选或没有匹配节点时停止，不使用笔记本内置相机兜底。USB 家族标识不能区分 Lite 和带屏幕版本，应确认物理连接的是本项目选定的 MaixCAM Lite。启动器使用 `--no-controls`，设备端曝光由 MaixPy 脚本设置，电脑端不套用普通 UVC 相机的增益、锐度或对焦默认值。

UVC 路径目前请求 30 FPS，与设备端 MaixVision 预览的 60 FPS 请求分开；实际协商参数和处理帧率以启动日志为准。按 `s` 保存截图，按 `q` / `Esc` 退出。更换到 Lite 后，应在最终运行分辨率和镜头状态下独立标定，不能复用其他相机内参。

## 无硬件回归

```bash
python3 -B -m unittest discover -s test/input -p 'test_maixcam_*.py'
```

这些回归检查几何配对、SDK 调用路由和单次编码复用，不连接相机、不读取 MaixPy SDK、不验证真实识别率或帧率。设备端实时性能和主机 UVC 持续取流仍需连接 MaixCAM Lite 后验收。

## 官方依据

- [MaixCAM Lite / 无屏幕版快速开始](https://wiki.sipeed.com/maixpy/doc/zh/README_no_screen.html)
- [无需初始化物理屏幕的 MaixVision 预览 API](https://wiki.sipeed.com/maixpy/api/maix/display.html#send_to_maixvision)
- [UVC 串流与 UvcStreamer](https://wiki.sipeed.com/maixpy/doc/zh/video/uvc_streaming.html)
- [按 SDK ID 区分 USB 配置方式的官方示例](https://github.com/sipeed/MaixPy/blob/main/examples/tools/maixcam_switch_usb_mode.py)
- [device_id 实现：由系统板卡配置提供 ID](https://github.com/sipeed/MaixCDK/blob/main/components/basic/src/maix_sys.cpp)
