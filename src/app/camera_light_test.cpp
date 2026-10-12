// 独立普通相机测试入口；参数、显示和设备控制各自分层。
#include "camera/application.hpp"
#include "../input/uvc_stream.hpp"
#include <string>
int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "--camera-worker")
    return aim::cameraWorkerMain(argc, argv);
  return aim::camera_app::run(argc, argv);
}
