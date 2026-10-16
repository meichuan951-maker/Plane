#pragma once
#include <opencv2/core.hpp>
#include <ostream>
#include <string>
namespace aim::depth_app {
struct Options {
  bool show{true};
  bool local_world_fixed{false};
  bool center_depth{false};
  int frames{0}, seconds{0}, hz{10};
  cv::Rect roi;
  std::string output;
  std::string snapshot;
};
Options parse(int argc, char **argv);
void help(std::ostream &out);
} // namespace aim::depth_app
