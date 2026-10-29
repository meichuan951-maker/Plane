# input：图像输入与相机采集

本目录负责获得图像、深度和采集时间，把数据交给上层应用。这里有两套输入接口：
供 `demo_aim_cpp` 使用的合成/RealSense 输入，以及供普通 USB 相机应用使用的 UVC 子进程采集。
目标检测与跟踪在 [`../vision/`](../vision/README.md)，本目录不判断装甲身份，也不控制飞行。

## 主要文件

- [`synthetic.cpp`](synthetic.cpp)：生成固定规则的彩色图、米制深度和注入检测框，包含检测丢失、深度缺失等离线夹具。
  它不运行真实检测器；`synthetic-rgb` 模式由应用层清除深度后复用这些样本。
- [`realsense.cpp`](realsense.cpp)：可选 D435i RGB-D 采集，把深度对齐到彩色像素网格、换算为米，
  记录时间同步状态并提供 SDK 反投影。未编译 SDK 支持时，保留明确报错的替代实现。
- [`uvc_stream.cpp`](uvc_stream.cpp)：`CameraStream` 的父进程监管，负责启动采集子进程、共享帧读取、
  控制命令传递、断流检测、有界重连和停止清理。
- [`uvc_worker.cpp`](uvc_worker.cpp)：UVC 子进程，通过 OpenCV/V4L2 打开设备、设置采集格式并发布图像与设备状态。
  它通过应用自身的 `--camera-worker` 分支运行，不是单独安装的可执行程序。
- [`uvc_controls.cpp`](uvc_controls.cpp)：提供可选的 Linux V4L2 对焦/增益控制；MaixCAM Lite 默认跳过主机控制查询，成像参数由设备端 MaixPy 设置。
- [`uvc_stream_protocol.cpp`](uvc_stream_protocol.cpp)：提供进程间采集协议使用的单调时钟函数；
  共享帧、状态和命令结构定义在内部头文件中。

## 与应用、算法的连接

```text
synthetic         → Sample（Frame + 注入检测框）
RealSense         → Frame → 应用调用传统灯条/装甲候选检测
                           ↓
                 src/app/application.cpp → vision/Pipeline

CameraStream → 当前应用的 --camera-worker → UVC 相机
      ↓                 （共享内存帧 + 控制管道）
src/app/camera、src/app/armor_auto、src/app/armor_calibrate.cpp
      ↓
传统灯条 / 装甲跟踪 / RGB 标定
```

[`../app/application.cpp`](../app/application.cpp) 为 `demo_aim_cpp` 选择合成或 RealSense 输入。
[`../app/camera/`](../app/camera/) 与 [`../app/armor_auto/`](../app/armor_auto/)
分别编排 `camera_light_test`、`armor_auto` 的 UVC 采集；
[`../app/armor_calibrate.cpp`](../app/armor_calibrate.cpp) 也复用 `CameraStream`。
普通 UVC 当前没有直接作为 `demo_aim_cpp` 的输入源。

## 与其他目录的联系

- [`../../include/demo_aim/input.hpp`](../../include/demo_aim/input.hpp)：合成、RealSense 的公共声明；
  `Frame` 和检测框等数据类型位于 [`observation.hpp`](../../include/demo_aim/observation.hpp)。
  [`../../include/demo_aim/detail/input/`](../../include/demo_aim/detail/input/) 存放 UVC 采集、硬件控制与进程协议的内部头文件。
- [`../../docs/interfaces.md`](../../docs/interfaces.md)：图像、坐标和时钟契约；合成输入使用虚拟时间，实时采集使用本机单调时钟。
- [`../../config/armor_auto.yaml`](../../config/armor_auto.yaml)：自动装甲入口的设备、格式、分辨率和帧率设置；
  [`../../scripts/armor_auto.sh`](../../scripts/armor_auto.sh) 启动装甲识别，
  [`../../tools/maixcam/run_light_test.py`](../../tools/maixcam/run_light_test.py) 按 MaixCAM Lite 的设备身份选择 UVC 节点并启动预览。
  参数由应用层解析后传给采集模块，采集实现不自行加载这些 YAML。
- [`../../tools/maixcam/`](../../tools/maixcam/)：MaixCAM Lite 设备端采集、MaixVision 预览和 UVC 推流脚本；
  `tools/vision_video_worker.cpp` 使用自己的录像读取循环，不经过这里的 UVC 采集器。
- [`../../test/input/`](../../test/input/)：用模拟采集进程和控制封装验证帧传递、超时、重连及停止等行为；
  [`../../test/core/test_aim.cpp`](../../test/core/test_aim.cpp) 同时验证合成输入与视觉流水线。

[`../../CMakeLists.txt`](../../CMakeLists.txt) 将合成/RealSense 编入 `aim_input`，
将 UVC 监管、控制和协议编入 `aim_camera_stream`，将 UVC worker 编入 `aim_uvc_worker`。
RealSense 受 `AIM_WITH_REALSENSE` 和 SDK 可用性控制；UVC 使用 Linux 接口，不依赖 RealSense 或其他厂商 SDK。
定位报文由 [`../io/`](../io/README.md) 接收，地图坐标与任务由 [`../navigation/`](../navigation/README.md) 处理，
它们不属于相机输入层。构建和运行命令见[项目 README](../../README.md)。
