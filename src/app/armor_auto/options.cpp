#include "demo_aim/detail/app/armor_auto/options.hpp"
#include "demo_aim/detail/app/common/parse.hpp"
#include <iostream>
#include <stdexcept>

namespace aim::armor_auto_app {

void printHelp() {
  std::cout << "Automatic armor detection/tracking + calibrated RGB PnP (no "
               "manual ROI)\n"
               "--config config/armor_auto.yaml --no-show --frames N --seconds "
               "N --hz 10\n"
               "--output FILE.jsonl (append) --image FILE --save-dir NEW_DIR\n"
               "Color config: 1=blue, 2=red. q/Esc quit, r reset target.\n"
               "Without RGB calibration: detection only, numeric pose/distance "
               "null.\n"
               "Calibration: --calibrate --board-cols N --board-rows M "
               "--square-mm S\n"
               "             --calibration-output NEW.yaml (default config "
               "calibration path)\n"
               "XYZ uses RGB optical frame X right/Y down/Z forward, meters.\n"
               "normal_yaw/pitch: board surface; bearing_yaw/pitch: target "
               "direction, degrees.\n";
}

Options parseOptions(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string k(argv[i]);
    auto next = [&]() {
      if (++i == argc)
        throw std::invalid_argument("Missing value: " + k);
      return argv[i];
    };
    if (k == "--no-show")
      o.show = false;
    else if (k == "--calibrate")
      o.calibrate = true;
    else if (k == "--config")
      o.config = next();
    else if (k == "--output")
      o.output = next();
    else if (k == "--image")
      o.image = next();
    else if (k == "--save-dir")
      o.save_dir = next();
    else if (k == "--frames")
      o.frames = app::parseInteger(next());
    else if (k == "--seconds")
      o.seconds = app::parseInteger(next());
    else if (k == "--hz")
      o.hz = app::parseInteger(next());
    else if (k == "--board-cols")
      o.columns = app::parseInteger(next());
    else if (k == "--board-rows")
      o.rows = app::parseInteger(next());
    else if (k == "--square-mm") {
      const std::string s(next());
      std::size_t n = 0;
      o.square_mm = std::stod(s, &n);
      if (n != s.size())
        throw std::invalid_argument("Invalid square size");
    } else if (k == "--calibration-output")
      o.calibration_output = next();
    else
      throw std::invalid_argument("Unknown option: " + k);
  }
  if (o.frames < 0 || o.seconds < 0 || o.hz < 1 || o.hz > 60)
    throw std::invalid_argument("Invalid frame/time/rate limit");
  return o;
}

} // namespace aim::armor_auto_app
