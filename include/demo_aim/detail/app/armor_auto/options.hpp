#pragma once

#include <string>

namespace aim::armor_auto_app {

struct Options {
  std::string config{"config/armor_auto.yaml"};
  std::string output;
  std::string image;
  std::string save_dir;
  std::string calibration_output;
  bool show{true};
  bool calibrate{false};
  int frames{0};
  int seconds{0};
  int hz{10};
  int columns{0};
  int rows{0};
  double square_mm{0};
};

void printHelp();
Options parseOptions(int argc, char **argv);

} // namespace aim::armor_auto_app
