// 应用编排：解析参数 → 选择输入/检测器 → 调用 Pipeline → 记录、发布和显示结果。
#include "application.hpp"
#include "demo_aim/core.hpp"
#include "demo_aim/input.hpp"
#include "demo_aim/yolo.hpp"
#include "json_log.hpp"
#include "options.hpp"
#include "ros_output.hpp"
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
namespace aim::app {
namespace {
// 信号处理只置位，避免在异步信号上下文里做不安全的 IO 或资源释放。
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
} // namespace
// 统一管理设备和输出对象生命周期；业务异常最终转成无效诊断消息。
int run(int argc, char **argv) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--help") {
      help();
      return 0;
    }
// 输出对象放在 try 外，catch 中仍可尝试记录故障。
  std::unique_ptr<JsonLog> output;
  std::unique_ptr<RosOutput> ros;
  try {
    const auto a = parse(argc, argv);
// 限制 OpenCV 内部工作线程，控制推理时的 CPU 并行度。
    cv::setNumThreads(a.threads);
    const auto config = aim::Config::load(a.config);
    aim::Pipeline pipeline(config);
    output = std::make_unique<JsonLog>(a.output);
    if (!a.preview.empty()) {
      if (std::filesystem::exists(a.preview))
        throw std::runtime_error(
            "Preview directory already exists; use a fresh name");
      std::filesystem::create_directories(a.preview);
    }
// 只有指定权重才实例化检测器；合成/无权重回放直接使用提供的框。
    std::unique_ptr<aim::YoloV8> detector;
    if (!a.weights.empty())
      detector = std::make_unique<aim::YoloV8>(a.weights, a.classes,
                                               a.input_size, config);
    std::unique_ptr<aim::RealSense> camera;
    std::unique_ptr<aim::Replay> replay;
    if (a.source == "realsense")
      camera = std::make_unique<aim::RealSense>(a.serial);
    if (a.source == "replay")
      replay = std::make_unique<aim::Replay>(a.manifest);
    if (a.ros)
      ros = std::make_unique<RosOutput>();
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    int count = 0, valid = 0, detections_valid = 0;
// 顺序流水线：每次处理一帧，确保状态关联、时间检查按采集顺序进行。
    while (!interrupted && (a.frames == 0 || count < a.frames)) {
      aim::Sample sample;
// 实时帧在推理后取 now，使相机等待、对齐和推理耗时参与过期判断。
      if (camera)
        sample.frame = camera->read();
      else if (replay) {
        if (!replay->next(sample))
          break;
      } else
        sample = aim::synthetic(static_cast<std::uint64_t>(count));
      if (a.source == "synthetic-rgb") {
        sample.frame.depth_m.release();
        sample.frame.intrinsics = {};
        sample.frame.rectified = false;
        sample.frame.source = "synthetic_rgb_injected_boxes";
      }
// 重跑检测时替换注入框；回放把本次推理耗时加入虚拟处理时间。
      if (detector) {
        const double start = aim::monotonicNow();
        sample.detections = detector->detect(sample.frame.color_bgr);
        if (replay) {
          sample.process_s += aim::monotonicNow() - start;
          sample.frame.source = "replay_yolov8_onnx_cpp";
        }
      }
      if (camera)
        sample.process_s = aim::monotonicNow();
      auto solution =
          pipeline.process(sample.frame, sample.detections, sample.process_s);
      if (detector)
        solution.model_sha256 = detector->modelHash();
// 每帧记录有效和无效结果，便于从 reason 分析失效原因。
      const auto payload = solution.json().dump();
      output->write(payload);
      if (ros)
        ros->publish(payload);
      ++count;
      if (solution.valid)
        ++valid;
      if (solution.detection_valid)
        ++detections_valid;
// 预览按需生成；图片保存与窗口显示在处理之后，会影响下一帧吞吐。
      if (a.show || !a.preview.empty()) {
        const auto image = aim::annotate(sample.frame, solution);
        if (!a.preview.empty()) {
          std::ostringstream name;
          name << std::setw(6) << std::setfill('0') << sample.frame.sequence
               << ".png";
          if (!cv::imwrite(
                  (std::filesystem::path(a.preview) / name.str()).string(),
                  image))
            throw std::runtime_error("Preview write failed");
        }
        if (a.show) {
          cv::imshow("Armor observation C++", image);
          const int key = cv::waitKey(1) & 255;
          if (key == 27 || key == 'q')
            break;
        }
      }
    }
    if (a.show)
      cv::destroyAllWindows();
    std::cout << aim::Json({{"frames", count},
                            {"detections_valid", detections_valid},
                            {"valid", valid},
                            {"output", output->path()},
                            {"flight_control", false}})
                     .dump()
              << '\n';
// Ctrl+C/SIGTERM 设置中断标志，正常退出作用域后由 RAII 释放资源。
    return interrupted ? 130 : 0;
// 失败时尽力写日志并发布 invalid；发布不保证送达，消费端仍需看门狗。
  } catch (const std::exception &error) {
    aim::Solution failure;
    failure.reason = failure.position_reason = "runtime_error";
    failure.emitted_s = aim::monotonicNow();
    auto failure_json = failure.json();
    failure_json["capture_monotonic_s"] = nullptr;
    failure_json["time_domain"] = nullptr;
    failure_json["age_ms"] = nullptr;
    failure_json["error"] = error.what();
    const auto message = failure_json.dump();
    std::cerr << message << '\n';
    try {
      if (output)
        output->write(message);
      if (ros)
        ros->publish(message);
    } catch (...) {
    }
    return 1;
  }
}
} // namespace aim::app
