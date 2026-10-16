#include "../src/app/armor_auto/preview.hpp"
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;
namespace app = aim::armor_auto_app;

void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}

struct TemporaryDirectory {
  fs::path path;
  TemporaryDirectory() {
    auto pattern = (fs::temp_directory_path() / "armor-snapshot-XXXXXX").string();
    const char *created = mkdtemp(pattern.data());
    if (!created)
      throw std::runtime_error("Cannot create snapshot fixture directory");
    path = created;
  }
  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(path, error);
  }
};

void snapshotRegressions() {
  TemporaryDirectory temporary;
  aim::ArmorAutoConfig config;
  config.width = 320;
  config.height = 240;
  aim::LightBarResult detection;
  aim::TrackedArmor target;
  app::FrameContext context{1, 1, 120, true, false, 10, .01};

  // A window drawn earlier may contain a waiting screen or a prior frame.
  // Saving another frame within the 33 ms draw interval must use the new raw
  // pixels and target, even though no window refresh occurred.
  const cv::Mat previous(config.height, config.width, CV_8UC3,
                         cv::Scalar(3, 7, 11));
  const cv::Mat cached_preview =
      app::drawPreview(previous, detection, target, config, context);
  cv::Mat latest(config.height, config.width, CV_8UC3,
                 cv::Scalar(21, 67, 143));
  aim::ArmorCandidate armor;
  armor.box = {120, 130, 50, 60};
  target.target = armor;
  context.frame = 2;
  context.monotonic_s += .008;
  const auto current = temporary.path / "current";
  app::saveSnapshot(current.string(), latest, detection, target, config,
                    context);
  const auto saved_raw = cv::imread((current / "raw.png").string());
  const auto saved_overlay = cv::imread((current / "overlay.png").string());
  check(!saved_raw.empty() && !saved_overlay.empty(), "Snapshot files missing");
  check(cv::norm(saved_raw, latest, cv::NORM_INF) == 0,
        "Raw snapshot differs from the final captured frame");
  check(saved_overlay.at<cv::Vec3b>(220, 220) ==
            latest.at<cv::Vec3b>(220, 220),
        "Snapshot retained pixels from the throttled preview cache");
  check(saved_overlay.at<cv::Vec3b>(130, 120) == cv::Vec3b(0, 255, 255),
        "Snapshot did not draw the final frame's selected target");
  check(cv::norm(saved_overlay, cached_preview, cv::NORM_INF) > 0,
        "Snapshot reused the earlier window preview");

  for (bool failed : {false, true}) {
    // A cached confirmed target must not survive stream expiry/failure in a
    // saved overlay; retain only raw.png as a diagnostic capture.
    context.age_s = failed ? .01 : .251;
    context.failed = failed;
    const auto expired = temporary.path / (failed ? "failed" : "stale");
    app::saveSnapshot(expired.string(), latest, detection, target, config,
                      context);
    const auto overlay = cv::imread((expired / "overlay.png").string());
    const auto raw = cv::imread((expired / "raw.png").string());
    check(cv::norm(raw, latest, cv::NORM_INF) == 0,
          "Expired capture was lost from raw diagnostic image");
    check(overlay.at<cv::Vec3b>(220, 220) == cv::Vec3b(20, 20, 20) &&
              overlay.at<cv::Vec3b>(130, 120) == cv::Vec3b(20, 20, 20),
          "Expired snapshot still displayed a live image or target");
  }

  bool rejected = false;
  try {
    app::saveSnapshot(current.string(), previous, detection, target, config,
                      context);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected &&
            cv::norm(cv::imread((current / "raw.png").string()), latest,
                     cv::NORM_INF) == 0,
        "Snapshot overwrote an existing directory");
}

} // namespace

int main() {
  try {
    snapshotRegressions();
    std::cout << "Armor snapshot frame/freshness regressions passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
