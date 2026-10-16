# 红蓝装甲板 YOLOv8 数据集与训练指南

本文路径和命令均以项目根目录为工作目录。构建和运行入口见 [主 README](../README.md)。

更新日期：2026-09-21。

本指南用于 `no_demo` 项目：使用 D435i 彩色图识别参考图片中的装甲板，必须区分红方和蓝方；训练在开发电脑上进行，部署平台为 Orange Pi 5 8GB、Ubuntu 22.04，运行程序为 C++。

本文是操作说明，不表示数据集已收集、模型已训练或板端已验证。参考图片位于 `test_1/截图 2026-09-21 18-26-52.png`，其中上方为红色侧灯、下方为蓝色侧灯的同类图案装甲板。

## 1. 固定类别：一个模型，两个类别

采用一个 YOLOv8 检测模型同时学习红蓝两类，不需要分别训练两个模型。类别编号从标注到训练、导出和部署始终保持一致：

```yaml
names:
  0: target_red
  1: target_blue
```

- `target_red`：参考图案对应的装甲板，侧灯为红色。
- `target_blue`：参考图案对应的装甲板，侧灯为蓝色。
- 其他图案装甲板、普通红蓝灯、白色文字，不属于这两个类别。
- 不设置 `background` 类；负样本使用空标签。

红蓝表示观测到的目标颜色，不自动表示己方或敌方。后续任务选择哪个颜色，需要另行配置。

## 2. 收集什么图片

优先用实际部署的 D435i 采集彩色图，保留完整画面及原始分辨率。普通 YOLO 检测训练只使用彩色图；深度、内参和时间戳可以另外保存，供后续测距回放验证。

参考图适合统一标注标准，不能作为实拍数据的替代品。仅复制、旋转或换背景生成大量图片，不能证明模型能识别实际比赛场景。

### 2.1 红蓝两类都需要覆盖的情况

- 距离：画面中的目标尺寸由大到小，覆盖实际使用距离。
- 角度：正面、左斜、右斜、俯视和仰视。
- 光照：室外阴影、阳光、逆光、曝光变化和反光。
- 位置：图像中央、四角、靠近边缘。
- 运动：轻微模糊、振动、短暂遮挡。
- 背景：地面、场地结构、机器人、人员及复杂纹理。
- 数量：单个目标、多个同色目标、红蓝同时出现。

让红蓝目标尽量在相同背景、相近距离和光照下都有样本，防止模型把背景当作颜色类别依据。

### 2.2 负样本与干扰样本

补充没有目标的比赛场景，以及容易误判的其他装甲图案、白色标识、红蓝灯条和反光物体。

只要画面中存在符合类别定义的目标，就必须标注，不能当作负样本。含有无法判断的模糊或颜色不明目标时，应先移入待复核目录，不要留空标签直接加入训练。

### 2.3 起步采集量

第一轮可先收集约 800～1500 张有明显差异的图像，红蓝目标实例数量尽量接近，约 10%～20% 图像为无目标负样本。这个范围是采集计划，不是模型准确率保证；一张图片可能包含多个目标。

从视频采样时，可先每隔 0.5～1 秒取一帧，再人工去重。慢速视频可以降低采样频率，快速视角变化时可以适当提高。不要用大量几乎相同的连续帧凑数量。

保留来源信息，例如拍摄日期、场景、视频编号和原始帧号。文件名可以采用：

```text
session01_video01_frame000100.jpg
session01_video01_frame000130.jpg
session02_video03_frame000200.jpg
```

来自其他比赛的视频可补充背景和干扰，但要检查目标图案、拍摄视角、颜色和压缩质量是否与自己的任务一致。

## 3. 统一标注标准

标注工具可使用 CVAT。新建检测项目，按顺序创建 `target_red` 和 `target_blue`，导出后再次检查类别编号。

操作顺序：创建项目 → 创建任务并上传图片 → 用矩形工具画框 → 选择红/蓝类别 → 复核 → 导出 `Ultralytics YOLO Detection`。

### 3.1 框选整个目标装甲板

- 使用轴对齐矩形框，包住整块装甲板的外壳及左右侧灯。
- 框尽量贴合外轮廓，避免带入大片背景。
- 不要一张框整个装甲板，另一张只框白色图案。
- 不要把红蓝两块板合并成一个框。
- 一张图中出现多个符合定义的目标时，全部标注。

类别判断必须结合目标图案和侧灯颜色。不能仅凭一处红光或蓝光就标成目标。

### 3.2 遮挡、边缘和颜色不明

首轮先标注图案和颜色都能可靠确认的样本。

- 轻微遮挡但外轮廓可以可靠判断时，按完整板轮廓画框，不凭空扩大。
- 图像边缘截断时，框不得超出图像；颜色和身份不确定的样本先排除。
- 严重模糊、灯光过曝成白色、红蓝无法判断时，移到待复核集合。
- 灯未亮但不能从当前图像可靠确定颜色时，不按前一帧颜色强行标注。

检测框中心只是目标位置代理点，不等于规定的接触区域，也不能从检测框直接推断装甲板完整姿态。

## 4. 数据划分与目录

起步按训练集 80%、验证集 10%、测试集 10% 划分。先按视频、拍摄批次或场景分组，再划分集合；同一段连续视频不要随机拆到三个集合。

训练集用于更新模型；验证集用于选模型和调参数；测试集只用于最终独立评估。红蓝两类在三个集合中都应有代表性样本。增强图和它的原图必须留在同一集合。

项目目录约定如下：

```text
no_demo/
├── datasets/
│   └── target_red_blue/
│       ├── data.yaml
│       ├── images/
│       │   ├── train/
│       │   ├── val/
│       │   └── test/
│       └── labels/
│           ├── train/
│           ├── val/
│           └── test/
├── models/
├── test_1/
└── config/aim.yaml
```

在项目根目录创建空目录：

```bash
mkdir -p datasets/target_red_blue/images/{train,val,test}
mkdir -p datasets/target_red_blue/labels/{train,val,test}
```

创建 `datasets/target_red_blue/data.yaml`：

```yaml
path: /home/user/nohuman_plane_std/no_demo/datasets/target_red_blue
train: images/train
val: images/val
test: images/test
names:
  0: target_red
  1: target_blue
```

如果在其他训练电脑上运行，修改 `path` 为那台电脑上的数据集绝对路径。

### 4.1 标签文件

图片与标签同名，例如 `images/train/scene001.jpg` 对应 `labels/train/scene001.txt`。

每行对应一个目标：

```text
class_id center_x center_y width height
```

中心和宽高分别除以图像宽、高，归一化到 0～1；类别 ID 是整数 0 或 1。假设原图大小为 W×H、框为 x1/y1/x2/y2，则：

```text
center_x = (x1 + x2) / (2 * W)
center_y = (y1 + y2) / (2 * H)
width    = (x2 - x1) / W
height   = (y2 - y1) / H
```

以下仅为格式示例，不是参考图片的真实标注：

```text
0 0.30 0.50 0.20 0.25
1 0.70 0.50 0.20 0.25
```

无目标图像保留同名空 `.txt` 文件，便于人工确认它已被审核。不能把未标注图像误当负样本。

### 4.2 训练前检查

- 每张图片均已人工审核，所有目标均已标注。
- 类别只有 0 和 1，含义与 data.yaml 一致。
- 框尺寸为正，完整框位于图像范围内。
- 没有训练/验证/测试重复图片或近乎相同的相邻视频帧。
- 每个集合包含红、蓝及无目标场景。
- 随机可视化几十张标签叠加图，确认坐标没有偏移。

## 5. 准备训练环境

训练在有可用 GPU 的开发电脑上进行，香橙派负责推理。训练工具使用 Python，不需要把 Python 实现重新放回 C++ 项目源码。

在独立训练环境中安装与显卡驱动匹配的 PyTorch，再安装 Ultralytics。以下命令用于创建环境，不能代替 CUDA/PyTorch 兼容性检查：

```bash
python3 -m venv ~/venvs/armor-yolo
source ~/venvs/armor-yolo/bin/activate
python -m pip install --upgrade pip
python -m pip install ultralytics
python -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

后续示例中 `device=0` 表示第一张 CUDA GPU；只有上述检查为 True 且设备可正常训练时使用。CPU 调试可改成 `device=cpu`，但不建议在香橙派上完成正式训练。

每次实验记录依赖和模型版本：

```bash
python -m pip freeze > training_environment.txt
```

将环境记录、data.yaml、类别定义和数据集划分一起保存。不同实验使用不同 `name`，不要覆盖旧结果。

## 6. 训练红蓝双类别模型

在 `no_demo` 项目根目录执行：

```bash
yolo detect train \
  model=yolov8n.pt \
  data=datasets/target_red_blue/data.yaml \
  imgsz=640 epochs=100 batch=8 device=0 \
  hsv_h=0 hsv_s=0 bgr=0 fliplr=0 flipud=0 \
  project=runs/train name=target_red_blue_v1
```

这是起步配置：

- 使用 YOLOv8n 预训练权重微调；首次使用可能需要下载权重。
- 总类别数由 data.yaml 推导为 2，不设置 `single_cls=True`。
- `batch=8` 根据 GPU 显存调整，显存不足时降到 4 或 2。
- `epochs=100` 是训练上限起点，不保证一定需要或足够。
- 色相、饱和度、RGB/BGR 通道增强先关闭，避免改变红蓝类别依据。
- 水平和垂直翻转先关闭，避免生成不符合真实安装状态的图案方向。
- 其他增强使用工具默认值，训练前检查生成的训练批次预览，尤其关注侧灯是否被裁掉或失去颜色。

训练完成后重点查看：

```text
runs/train/target_red_blue_v1/weights/best.pt
runs/train/target_red_blue_v1/results.csv
runs/train/target_red_blue_v1/args.yaml
```

不要只看 loss 下降。需要检查红蓝分别的查准率、召回率、混淆情况和漏检/误检图片。补数据时优先收集真实失败场景，而不是无限增加相似参考图。

## 7. 独立测试与实拍检查

选定模型后，运行未参与调参的测试集：

```bash
yolo detect val \
  model=runs/train/target_red_blue_v1/weights/best.pt \
  data=datasets/target_red_blue/data.yaml \
  split=test imgsz=640 device=0 \
  project=runs/evaluate name=target_red_blue_test_v1
```

保存测试图片的预测结果以便人工审查：

```bash
yolo detect predict \
  model=runs/train/target_red_blue_v1/weights/best.pt \
  source=datasets/target_red_blue/images/test \
  imgsz=640 conf=0.6 device=0 save=True \
  project=runs/predict name=target_red_blue_test_v1
```

重点检查红误判为蓝、蓝误判为红、相似图案误报、远距漏检以及无目标场景误报。测试集发现问题后可以开启下一轮开发，但需要保留新的独立数据作下一版最终评估，不能长期拿同一测试集调参。

尚未采集实拍数据时，不预先承诺识别率和帧率。部署验收还应测量 D435i 输入下的端到端延迟、深度有效率和长时间运行情况。

## 8. 导出当前 C++ 程序需要的 ONNX

```bash
yolo export \
  model=runs/train/target_red_blue_v1/weights/best.pt \
  format=onnx imgsz=640 batch=1 \
  dynamic=False half=False nms=False opset=12
```

当前程序要求 YOLOv8 detect 的单个 float32 原始输出，支持 `[1,4+C,N]` 或 `[1,N,4+C]`。这里 C=2，因此每个候选有 6 个特征。不能换用分类、分割、姿态模型或内置 NMS 的导出结果。

`opset=12` 是当前项目的起始兼容设置，不是保证；实际模型必须在当前 OpenCV 版本中验证加载和推理。导出后使用同一批实拍输入对比 PT 与 ONNX 的类别、框和置信度，确认差异可接受。

将生成的 `best.onnx` 复制到香橙派项目的：

```text
models/target_red_blue_v1.onnx
```

同时保留训练环境、类别表与模型哈希：

```bash
sha256sum models/target_red_blue_v1.onnx
```

当前后端是 OpenCV CPU，ONNX 不会自动调用 Orange Pi 5 的 NPU。RKNN 是后续独立的转换和后端接入工作。

## 9. 接入现有 C++ 框架

### 9.1 允许红蓝两类参与观测

当前 `config/aim.yaml` 已允许红蓝两类。使用双类别模型时保持：

```yaml
class_ids: [0, 1]
```

只允许红方时设 `[0]`；只允许蓝方时设 `[1]`。这只是候选过滤，模型总类别数仍然是 2。

### 9.2 香橙派运行命令

在项目根目录、相机已连接且模型已复制后执行：

```bash
./build_cpp/demo_aim_cpp --source realsense \
  --config config/aim.yaml \
  --weights models/target_red_blue_v1.onnx \
  --num-classes 2 --input-size 640 \
  --frames 0 --threads 2 \
  --output runs/target_red_blue_camera.jsonl
```

有可用桌面时可附加 `--show`；SSH 无图形环境时省略。

日志中的 `class_id=0` 表示红方，`class_id=1` 表示蓝方。当前 Pipeline 每帧只选择一个目标并进行同类别关联；允许两个类别不代表同时输出两个目标，也不代表自动判定敌我。

检测成功但输出 `valid=false` 时，还需检查连续确认、深度质量、RGB-D 同步及图像年龄。仅有颜色正确的框，不足以证明三维观测有效。`contact_permitted` 仍为 false，框中心不等于接触区域。

## 10. 提供训练数据时交付什么

- 原始彩色图片或视频，以及各自的拍摄来源/批次。
- 红蓝类别定义和一致的框选标准。
- 同名 YOLO 标签文件与 data.yaml。
- 已按拍摄组划分的 train、val、test 集合。
- 待复核图片单独存放，不混入已确认负样本。
- 实拍距离、光照和拍摄设备的简要记录。

第一步先完成一小批红蓝各类标注，复核标准后再扩大采集，避免整批数据返工。

## 参考资料

- [Ultralytics YOLO 检测数据格式](https://docs.ultralytics.com/datasets/detect/)
- [CVAT Ultralytics YOLO 导入导出格式](https://docs.cvat.ai/docs/dataset_management/formats/format-yolo-ultralytics/)
- [Ultralytics 训练参数](https://docs.ultralytics.com/modes/train/)
- [Ultralytics 数据增强](https://docs.ultralytics.com/guides/yolo-data-augmentation/)
- [Ultralytics 模型导出](https://docs.ultralytics.com/modes/export/)
