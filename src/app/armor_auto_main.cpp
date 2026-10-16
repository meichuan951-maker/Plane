#include "../input/uvc_stream.hpp"
#include "armor_auto/application.hpp"
#include <exception>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--camera-worker")
    return aim::cameraWorkerMain(argc, argv);
  try {
    return aim::armor_auto_app::run(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << "armor_auto: " << error.what() << '\n';
    return 1;
  }
}
