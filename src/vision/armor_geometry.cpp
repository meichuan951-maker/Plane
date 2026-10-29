#include "demo_aim/light_bars.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
namespace aim {
namespace {
struct BarGeometry {
  const ColorLightBar *bar;
  cv::Point2f axis;
  double length;
};
struct Pair {
  std::size_t left, right;
  double score;
};
// 小冲突组求总几何评分最大的匹配；大组退回确定性贪心，限制指数算法的内存/耗时。
std::vector<std::size_t> selectPairs(const std::vector<Pair> &pairs,
                                     std::size_t count) {
  std::vector<std::vector<std::size_t>> edges(count);
  for (std::size_t e = 0; e < pairs.size(); ++e) {
    edges[pairs[e].left].push_back(e);
    edges[pairs[e].right].push_back(e);
  }
  std::vector<bool> seen(count, false), used(count, false);
  std::vector<int> local(count, -1);
  std::vector<std::size_t> selected;
  constexpr std::size_t exact_limit = 12;
  for (std::size_t root = 0; root < count; ++root) {
    if (seen[root] || edges[root].empty())
      continue;
    std::vector<std::size_t> group{root};
    seen[root] = true;
    for (std::size_t k = 0; k < group.size(); ++k) {
      const auto vertex = group[k];
      for (auto e : edges[vertex]) {
        const auto next =
            pairs[e].left == vertex ? pairs[e].right : pairs[e].left;
        if (!seen[next]) {
          seen[next] = true;
          group.push_back(next);
        }
      }
    }
    std::sort(group.begin(), group.end());
    for (std::size_t i = 0; i < group.size(); ++i)
      local[group[i]] = static_cast<int>(i);
    if (group.size() > exact_limit) {
      std::vector<std::size_t> ranked;
      for (auto v : group)
        for (auto e : edges[v])
          if (pairs[e].left == v)
            ranked.push_back(e);
      std::stable_sort(ranked.begin(), ranked.end(), [&](auto a, auto b) {
        return pairs[a].score > pairs[b].score;
      });
      for (auto e : ranked) {
        const auto &p = pairs[e];
        if (!used[p.left] && !used[p.right]) {
          selected.push_back(e);
          used[p.left] = used[p.right] = true;
        }
      }
      continue;
    }
    const std::uint32_t full = (1u << group.size()) - 1;
    std::vector<double> memo(full + 1, -1);
    std::vector<int> choice(full + 1, -1);
    memo[0] = 0;
    const std::function<double(std::uint32_t)> solve = [&](std::uint32_t mask) {
      if (memo[mask] >= 0)
        return memo[mask];
      const auto first = static_cast<unsigned>(__builtin_ctz(mask));
      const auto rest = mask & ~(1u << first);
      double best =
          solve(rest); // 允许不配对，不能为了增加数量使用被过滤的候选。
      for (auto e : edges[group[first]]) {
        const auto &p = pairs[e];
        const auto other = p.left == group[first] ? p.right : p.left;
        const auto bit = 1u << static_cast<unsigned>(local[other]);
        if (!(rest & bit))
          continue;
        const double value = p.score + solve(rest & ~bit);
        if (value > best + 1e-12) {
          best = value;
          choice[mask] = static_cast<int>(e);
        }
      }
      return memo[mask] = best;
    };
    solve(full);
    for (auto mask = full; mask;) {
      const auto first = static_cast<unsigned>(__builtin_ctz(mask));
      const int e = choice[mask];
      mask &= ~(1u << first);
      if (e >= 0) {
        selected.push_back(static_cast<std::size_t>(e));
        const auto &p = pairs[static_cast<std::size_t>(e)];
        const auto other = p.left == group[first] ? p.right : p.left;
        mask &= ~(1u << static_cast<unsigned>(local[other]));
      }
    }
  }
  return selected;
}
} // namespace
std::vector<ArmorCandidate>
pairLightBars(const std::vector<ColorLightBar> &bars, const LightBarConfig &c) {
  c.validate();
  std::vector<BarGeometry> geometry;
  for (const auto &bar : bars) {
    const auto finite = [](cv::Point2f p) {
      return std::isfinite(p.x) && std::isfinite(p.y);
    };
    if (bar.class_id < 0 || bar.class_id > 1 || !finite(bar.top) ||
        !finite(bar.bottom) || !finite(bar.rect.center))
      continue;
    const auto axis = bar.bottom - bar.top;
    const double length = cv::norm(axis);
    if (!std::isfinite(length) || length < 1 || axis.y <= 0)
      continue;
    geometry.push_back({&bar, axis / static_cast<float>(length), length});
  }
  std::stable_sort(
      geometry.begin(), geometry.end(), [](const auto &a, const auto &b) {
        const auto ac = a.bar->rect.center, bc = b.bar->rect.center;
        if (ac.x != bc.x)
          return ac.x < bc.x;
        if (ac.y != bc.y)
          return ac.y < bc.y;
        return a.bar->class_id < b.bar->class_id;
      });
  std::vector<Pair> pairs;
  const double expected_ratio = c.armor_width_m / c.lightbar_length_m;
  const double log_max_length_ratio = std::log(c.pair_max_length_ratio);
  for (std::size_t i = 0; i < geometry.size(); ++i) {
    const auto &l = geometry[i];
    std::multiset<float> intervening_y;
    std::size_t next_intervening = i + 1;
    // 长度比有上界，因此可以给搜索横向距离设上界；不扫整个画面。
    const double max_dx =
        c.pair_max_ratio * l.length * (1 + c.pair_max_length_ratio) / 2;
    for (std::size_t j = i + 1; j < geometry.size(); ++j) {
      const auto &r = geometry[j];
      while (next_intervening < j &&
             geometry[next_intervening].bar->rect.center.x <
                 r.bar->rect.center.x) {
        const auto &middle = *geometry[next_intervening++].bar;
        if (middle.class_id == l.bar->class_id &&
            middle.rect.center.x > l.bar->rect.center.x)
          intervening_y.insert(middle.rect.center.y);
      }
      const auto span = r.bar->rect.center - l.bar->rect.center;
      if (span.x > max_dx)
        break;
      if (l.bar->class_id != r.bar->class_id || span.x <= 0)
        continue;
      const double height = (l.length + r.length) / 2;
      const double side_ratio =
          std::max(l.length, r.length) / std::min(l.length, r.length);
      const double y_offset = std::abs(span.y) / height;
      if (side_ratio > c.pair_max_length_ratio ||
          y_offset > c.pair_max_y_offset)
        continue;
      const double separation = cv::norm(span);
      const double ratio = separation / height;
      if (ratio < c.pair_min_ratio || ratio > c.pair_max_ratio)
        continue;
      const double angle =
          std::acos(
              std::clamp(static_cast<double>(l.axis.dot(r.axis)), -1.0, 1.0)) *
          180 / CV_PI;
      if (angle > c.pair_max_angle_deg)
        continue;
      const auto horizontal = span / static_cast<float>(separation);
      const double rectangular_error =
          std::asin(std::clamp(
              std::max(std::abs(static_cast<double>(l.axis.dot(horizontal))),
                       std::abs(static_cast<double>(r.axis.dot(horizontal)))),
              0.0, 1.0)) *
          180 / CV_PI;
      if (rectangular_error > c.pair_max_rectangular_error_deg)
        continue;
      const double middle_y = (l.bar->rect.center.y + r.bar->rect.center.y) / 2;
      // 排序维护中间灯条的 y，范围查询替代原来的第三层全量扫描。
      const auto blocked = intervening_y.lower_bound(
          static_cast<float>(middle_y - height * 0.5));
      if (blocked != intervening_y.end() && *blocked < middle_y + height * 0.5)
        continue;
      const double cost =
          std::log(side_ratio) / log_max_length_ratio +
          y_offset / c.pair_max_y_offset + angle / c.pair_max_angle_deg +
          rectangular_error / c.pair_max_rectangular_error_deg +
          0.25 * std::abs(std::log(ratio / expected_ratio));
      const double score = std::exp(-cost);
      if (score >= c.pair_min_score)
        pairs.push_back({i, j, score});
    }
  }
  std::vector<ArmorCandidate> result;
  for (auto e : selectPairs(pairs, geometry.size())) {
    const auto &p = pairs[e];
    const auto &l = *geometry[p.left].bar, &r = *geometry[p.right].bar;
    ArmorCandidate armor;
    armor.class_id = l.class_id;
    armor.geometry_score = p.score;
    armor.corners = {l.top, r.top, r.bottom, l.bottom};
    armor.center = (l.rect.center + r.rect.center) * 0.5f;
    float min_x = armor.corners[0].x, max_x = min_x;
    float min_y = armor.corners[0].y, max_y = min_y;
    for (const auto &point : armor.corners) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    armor.box = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
    result.push_back(armor);
  }
  std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) {
    return a.center.x != b.center.x ? a.center.x < b.center.x
                                    : a.center.y < b.center.y;
  });
  return result;
}
} // namespace aim
