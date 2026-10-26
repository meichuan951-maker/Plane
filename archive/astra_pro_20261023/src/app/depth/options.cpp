#include "demo_aim/detail/app/depth/options.hpp"
#include "demo_aim/detail/app/common/parse.hpp"
#include <stdexcept>
namespace aim::depth_app {
Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    auto next = [&]() {
      if (++i == argc)
        throw std::invalid_argument("Missing value: " + key);
      return argv[i];
    };
    if (key == "--no-show")
      o.show = false;
    else if (key == "--local-world-fixed")
      o.local_world_fixed = true;
    else if (key == "--center-depth")
      o.center_depth = true;
    else if (key == "--manual-depth")
      o.center_depth = false;
    else if (key == "--snapshot")
      o.snapshot = next();
    else if (key == "--frames")
      o.frames = app::parseInteger(next());
    else if (key == "--seconds")
      o.seconds = app::parseInteger(next());
    else if (key == "--hz")
      o.hz = app::parseInteger(next());
    else if (key == "--output")
      o.output = next();
    else if (key == "--roi") {
      const int x = app::parseInteger(next()), y = app::parseInteger(next());
      const int w = app::parseInteger(next()), h = app::parseInteger(next());
      if (x < 0 || y < 0 || w < 8 || h < 8 || w > 640 || h > 480 ||
          x > 640 - w || y > 480 - h)
        throw std::invalid_argument("ROI must be inside 640x480, at least 8x8");
      o.roi = {x, y, w, h};
      o.center_depth = false;
    } else
      throw std::invalid_argument("Unknown option: " + key);
  }
  if (o.frames < 0 || o.seconds < 0 || o.hz < 1 || o.hz > 30 ||
      (!o.show && o.roi.empty() && !o.center_depth))
    throw std::invalid_argument(
        "Use --center-depth or --roi x y w h with --no-show; hz 1..30");
  return o;
}
void help(std::ostream &out) {
  out << "Astra Pro LIVE center/manual depth ROI (approximate SDK projection)\n"
         "Drag inside armor surface in DEPTH image. No RGB alignment / "
         "tracking.\n"
         "q/Esc: quit; c: center probe; r: clear/manual ROI.\n"
         "--center-depth: measure center 32x32 depth surface, no mouse "
         "selection.\n"
         "Not associated with RGB detection; walls also produce distances.\n"
         "--manual-depth: select a depth ROI by dragging.\n"
         "--roi x y w h --no-show --frames N --seconds N --hz 10\n"
         "--output FILE.jsonl (append; default stdout; diagnostics on "
         "stderr)\n"
         "--local-world-fixed: fixed camera anchor; world X forward, "
         "Y left, Z up; origin at depth optical center.\n"
         "--snapshot FILE.png: refresh a diagnostic preview each second.\n"
         "Raw depth optical frame: X right, Y down, Z forward; "
         "meters/degrees.\n"
         "normal_yaw/pitch: surface orientation; bearing_yaw/pitch: "
         "target direction.\n";
}
} // namespace aim::depth_app
