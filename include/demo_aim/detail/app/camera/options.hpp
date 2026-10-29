#pragma once
#include <string>
namespace aim::camera_app {
struct Options {
  std::string device, image, config{"config/light_bars.yaml"}, format{"MJPG"},
      save_dir;
  int width{640}, height{360}, fps{30}, frames{0}, reconnect_seconds{15};
  bool show{true}, masks{false}, controls{false};
};
void help();
void listDevices();
Options parse(int argc, char **argv);
}
