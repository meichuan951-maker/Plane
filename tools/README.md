# 测量与相机诊断工具

本目录存放开发阶段的辅助工具：测量传统视觉的资源开销、测试视觉和离线电控的并行负载，以及 MaixCAM Lite 的采集、预览和 UVC 接入。
业务视觉算法来自 [src/vision/](../src/vision/README.md)，这些工具负责组织输入、采样和报告。
普通相机预览及装甲识别入口在 [src/app/](../src/app/README.md)，日常启动脚本在 [scripts/](../scripts/)。

## 文件分工

- [vision_memory_probe.cpp](vision_memory_probe.cpp)：单进程算法探针。对同一张 BGR 图片反复运行传统灯条/装甲配对，可用 `--armor-config` 启用跟踪和配置允许的姿态估计。输出算法耗时、检测情况与内核记录的生命周期 RSS 峰值，不启动相机、GUI 或飞控。
- [benchmark_vision_memory.py](benchmark_vision_memory.py)：`compare` 启动上述探针，重复测量传统视觉的内存；`profile` 启动指定命令并采样整个进程树，可用于测量真实采集链。通过 Linux `/proc` 获取 PSS/RSS/SwapPss，保存 JSON 和 `.cases/` 原始记录。
- [vision_video_worker.cpp](vision_video_worker.cpp)：流式读取录像的 C++ 视觉工作进程，使用传统灯条/装甲识别；实现 ready / 共同测量窗口 / result / release 协议，供联测父进程调度。传统录像模式禁用 PnP，避免使用与录像相机不匹配的内参。
- [benchmark_vision_control.py](benchmark_vision_control.py)：父进程调度器，分别测量控制单独、传统视觉单独和二者联合运行的三种场景；启动视觉 worker 和 `px4_go.offline_control`，统一预热和测量窗口，生成 JSON、CSV 与 `.cases/` 日志。
- [resource_sampling.py](resource_sampling.py)：供联测调度器使用的 Linux `/proc` 采样模块，统计进程树 CPU、PSS/RSS/SwapPss 和采样峰值；本身不是业务运行入口。
- [maixcam/](maixcam/README.md)：MaixCAM Lite 设备端的 MaixVision 预览、灯条/装甲候选检测、UVC 配置与推流，以及电脑端按设备身份选择 UVC 节点的启动器。设备端脚本通过 MaixVision 运行，电脑端启动器复用 `build/camera_light_test`。

`__pycache__/` 是 Python 自动生成的缓存，不保存工具源码或实验结论。

## 与其他目录的联系

- 根 [CMakeLists.txt](../CMakeLists.txt) 将两个 C++ 文件构建为 `vision_memory_probe` 和 `vision_video_worker`，链接现有 `aim_armor_auto` 等传统视觉库。通过 [scripts/build.sh](../scripts/build.sh) 配置和构建，产物放在统一的 `build/` 下；Python 工具不参加 C++ 编译。
- [config/](../config/) 提供灯条、跟踪和标定配置。静态图探针能按配置启用跟踪/PnP，录像 worker 则明确禁用 PnP。不能用静态图或负载测试代替相机标定和识别准确率验收。
- [px4_go/control.py](../px4_go/control.py) 和 [px4_go/offline_control.py](../px4_go/offline_control.py) 为联测提供内存 MAVLink 控制负载。该路径不连接真实串口、UDP 或 PX4，不把视觉结果变成飞行指令；真实视觉偏航入口另见 [px4_go/README.md](../px4_go/README.md)。
- 常规资源采样只使用 Python 标准库，控制 worker 另需 `pymavlink`。
- [runs/](../runs/) 存放实验报告和原始日志，运行产物不随 Git 提交。联测和内存工具要求新输出路径，避免覆盖已有实验。
- [docs/vision_control_benchmark.md](../docs/vision_control_benchmark.md) 说明联测协议、统计口径和已有实测；根 [README.md](../README.md) 的“传统视觉内存测量”介绍内存测量口径；相机约束见[硬件说明](../docs/hardware.md)。
- [test/build/test_resource_sampling.py](../test/build/test_resource_sampling.py)、[test/build/test_vision_control_benchmark.py](../test/build/test_vision_control_benchmark.py) 和 [test/vision/test_video_worker.py](../test/vision/test_video_worker.py) 覆盖采样、失败处理、共同窗口及录像 worker 协议；测试登记集中在 [test/CMakeLists.txt](../test/CMakeLists.txt)。

## 常用入口

以下命令均从项目根目录执行；图片和录像路径需替换为真实文件，输出文件名需尚不存在。

先配置工程，再构建 C++ 工具：

```bash
./scripts/build.sh
cmake --build build --target vision_memory_probe vision_video_worker -j2
```

重复测量静态图片上的传统视觉内存：

```bash
python3 tools/benchmark_vision_memory.py compare \
  --image /实际路径/frame.png --threads 2 --repeats 3 \
  --output runs/memory_measurement_new.json
```

测量录像传统视觉与离线控制的并行负载：

```bash
python3 tools/benchmark_vision_control.py --video /实际路径/demo.avi \
  --scenarios traditional_joint --duration 30 --warmup 6 --repeats 3 \
  --output runs/vision_control_new/report.json
```

采样真实装甲识别程序的整个进程树时，由 `profile` 启动该程序：

```bash
python3 tools/benchmark_vision_memory.py profile --duration 60 \
  --output runs/armor_live_memory_new.json -- \
  ./build/armor_auto --config config/armor_auto.yaml --no-show
```

该命令会实际打开配置中的相机；测试前应确认采集入口能正常工作。资源数据只代表本次机器和所选负载，算法进程峰值、采样峰值和整机内存需求需分别解读。
`compare` 保留命令名，每轮只测传统视觉；C++ 探针与录像 worker 默认使用传统模式，可显式传 `--mode traditional`。
联测默认运行 `control_only`、`traditional_only`、`traditional_joint` 三种场景；更多参数见工具 `--help`；MaixCAM Lite 的设备端与主机端步骤见 [相机接入说明](maixcam/README.md)。
