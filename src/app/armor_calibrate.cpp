#include "demo_aim/detail/app/armor_calibrate.hpp"
#include "demo_aim/detail/input/uvc_stream.hpp"
#include "demo_aim/camera_calibration.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
namespace aim {
namespace {
volatile std::sig_atomic_t calibration_stopped = 0;
void calibrationStop(int) { calibration_stopped = 1; }
} // namespace
int calibrateArmorCamera(const ArmorAutoConfig &c, int columns, int rows,
                         double square_mm, const std::string &output) {
  if (columns < 3 || rows < 3 || columns > 20 || rows > 20 ||
      !std::isfinite(square_mm) || square_mm < 2 || square_mm > 200)
    throw std::invalid_argument(
        "Provide actual --board-cols / --board-rows / --square-mm");
  if (output.empty() || std::filesystem::exists(output))
    throw std::invalid_argument("Calibration output must be a new file");
  StreamOptions s;
  calibration_stopped = 0;
  std::signal(SIGINT, calibrationStop);
  std::signal(SIGTERM, calibrationStop);
  s.device = c.device;
  s.format = c.format;
  s.width = c.width;
  s.height = c.height;
  s.fps = c.fps;
  s.controls = false;
  CameraStream camera(s);
  CameraControls controls;
  const std::string window =
      "UVC RGB calibration - move printed board, SPACE capture, c solve";
  cv::namedWindow(window, cv::WINDOW_NORMAL);
  cv::resizeWindow(window, c.width, c.height);
  cv::Mat raw, gray, view;
  std::vector<cv::Point2f> corners;
  std::vector<std::vector<cv::Point2f>> views;
  std::uint64_t generation = 0;
  bool found = false, solved = false;
  std::string status =
      "Move board through different positions, distances and tilts";
  while (!solved && !calibration_stopped) {
    const auto state = camera.poll(raw, controls);
    if (state == CameraStream::Result::Failed)
      throw std::runtime_error(camera.message());
    if (generation != camera.generation()) {
      if (generation) {
        views.clear();
        status = "Camera reconnected: capture set cleared";
      }
      generation = camera.generation();
    }
    if (state == CameraStream::Result::Frame) {
      if (raw.cols != c.width || raw.rows != c.height)
        throw std::runtime_error(
            "Negotiated resolution differs from calibration settings");
      cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
      found = cv::findChessboardCorners(gray, {columns, rows}, corners,
                                        cv::CALIB_CB_ADAPTIVE_THRESH |
                                            cv::CALIB_CB_NORMALIZE_IMAGE |
                                            cv::CALIB_CB_FAST_CHECK);
      if (found)
        cv::cornerSubPix(
            gray, corners, {5, 5}, {-1, -1},
            {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, .01});
    }
    if (camera.frameAge() > .25 || raw.empty()) {
      view = cv::Mat(c.height, c.width, CV_8UC3, cv::Scalar(20, 20, 20));
      found = false;
    } else {
      view = raw.clone();
      cv::drawChessboardCorners(view, {columns, rows}, corners, found);
    }
    cv::rectangle(view, {0, 0}, {view.cols, 64}, {20, 20, 20}, cv::FILLED);
    cv::putText(view, "SPACE capture | c solve (>=25 views) | q/Esc exit",
                {8, 19}, cv::FONT_HERSHEY_SIMPLEX, .43, {255, 255, 255}, 1,
                cv::LINE_AA);
    cv::putText(view,
                "Views " + std::to_string(views.size()) + " | Board " +
                    std::to_string(columns) + "x" + std::to_string(rows) +
                    (found ? " FOUND" : " NOT FOUND"),
                {8, 38}, cv::FONT_HERSHEY_SIMPLEX, .45,
                found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 180, 255), 1);
    cv::putText(view, status, {8, 57}, cv::FONT_HERSHEY_SIMPLEX, .35,
                {255, 255, 255}, 1);
    cv::imshow(window, view);
    const int key = cv::waitKey(10) & 255;
    if (key == 'q' || key == 27 ||
        cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) == 0)
      break;
    if (key == ' ' && found && camera.frameAge() <= .25) {
      bool duplicate = false;
      for (const auto &old : views) {
        double d = 0;
        for (std::size_t i = 0; i < corners.size(); ++i)
          d += cv::norm(corners[i] - old[i]);
        if (d / corners.size() < 12)
          duplicate = true;
      }
      if (duplicate)
        status = "Too similar: move/tilt board before next capture";
      else if (views.size() < 80) {
        views.push_back(corners);
        status = "Captured - move/tilt board, then SPACE";
      }
    }
    if (key == 'c') {
      if (views.size() < 25) {
        status = "Need >=25 different views; capture more";
        continue;
      }
      RgbCalibration calibration;
      try {
        calibration = calibrateRgbViews(views, {c.width, c.height},
                                        {columns, rows}, square_mm);
      } catch (const std::exception &e) {
        status = e.what();
        std::cerr << status << '\n';
        continue;
      }
      const auto &K = calibration.camera;
      const auto &D = calibration.distortion;
      const double rms = calibration.rms_px,
                   worst = calibration.validation_max_rms_px;
      YAML::Emitter y;
      y.SetDoublePrecision(17);
      y << YAML::BeginMap;
      y << YAML::Key << "camera_width" << YAML::Value << c.width << YAML::Key
        << "camera_height" << YAML::Value << c.height;
      std::vector<double> matrix(K.ptr<double>(), K.ptr<double>() + 9);
      std::vector<double> distortion(D.ptr<double>(), D.ptr<double>() + 5);
      y << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << matrix;
      y << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow
        << distortion;
      y << YAML::Key << "calibration_rms_px" << YAML::Value << rms;
      y << YAML::Key << "validation_max_rms_px" << YAML::Value << worst;
      y << YAML::Key << "board_normal_spread_deg" << YAML::Value
        << calibration.normal_spread_deg;
      y << YAML::Key << "captured_views" << YAML::Value << views.size();
      y << YAML::Key << "camera_device" << YAML::Value << c.device;
      y << YAML::Key << "board_inner_cols" << YAML::Value << columns;
      y << YAML::Key << "board_inner_rows" << YAML::Value << rows;
      y << YAML::Key << "board_square_mm" << YAML::Value << square_mm
        << YAML::EndMap;
      std::ofstream file(output);
      file << y.c_str() << '\n';
      if (!file)
        throw std::runtime_error("Could not save RGB calibration");
      std::cerr << "Saved " << output << " RMS=" << rms
                << " validation_max=" << worst << " px\n";
      solved = true;
    }
  }
  camera.stop();
  cv::destroyWindow(window);
  return solved ? 0 : 4;
}
} // namespace aim
