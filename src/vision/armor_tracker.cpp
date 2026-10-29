#include "demo_aim/armor_auto.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace aim {
ArmorTracker::ArmorTracker(const ArmorAutoConfig &config) : config_(config) {}
void ArmorTracker::reset() {
  previous_.reset();
  hits_ = 0;
  seen_ = clock_ = -1;
}
TrackedArmor ArmorTracker::update(const std::vector<ArmorCandidate> &candidates,
                                  cv::Size size, double time_s) {
  if (!std::isfinite(time_s) || (clock_ >= 0 && time_s <= clock_)) {
    reset();
    return {};
  }
  clock_ = time_s;
  if (seen_ >= 0 && time_s - seen_ > config_.lost_seconds) {
    previous_.reset();
    hits_ = 0;
  }
  const ArmorCandidate *best = nullptr;
  double cost = std::numeric_limits<double>::infinity();
  const int wanted = config_.target_color == 1 ? 1 : 0;
  const double scale = previous_
      ? std::max(16., std::hypot(double(previous_->box.width),
                               double(previous_->box.height)))
      : std::max(1., std::hypot(size.width, size.height));
  const float previous_area = previous_ ? previous_->box.area() : 0.0f;
  const cv::Point2f image_center(size.width * .5f, size.height * .5f);
  for (const auto &c : candidates) {
    if (c.class_id != wanted || !std::isfinite(c.center.x) ||
        !std::isfinite(c.center.y) || !std::isfinite(c.geometry_score) ||
        c.geometry_score < config_.detector.pair_min_score ||
        !std::isfinite(c.box.width) || !std::isfinite(c.box.height) ||
        c.box.width <= 0 || c.box.height <= 0)
      continue;
    double value = 0;
    if (previous_) {
      const auto &p = *previous_;
      const double distance = cv::norm(c.center - p.center);
      const double area_ratio = c.box.area() / previous_area;
      if (distance > scale * config_.association_distance_ratio ||
          area_ratio < 1 / config_.association_size_ratio ||
          area_ratio > config_.association_size_ratio)
        continue;
      value = distance / scale + .2 * std::abs(std::log(area_ratio));
    } else
      value = cv::norm(c.center - image_center) / scale -
              .05 * c.geometry_score;
    if (value < cost) {
      cost = value;
      best = &c;
    }
  }
  if (!best) {
    hits_ = 0;
    return {std::nullopt, id_, false};
  }
  if (!previous_)
    ++id_;
  previous_ = *best;
  seen_ = time_s;
  hits_ = std::min(hits_ + 1, config_.confirmation_frames);
  return {*best, id_, hits_ >= config_.confirmation_frames};
}
} // namespace aim
