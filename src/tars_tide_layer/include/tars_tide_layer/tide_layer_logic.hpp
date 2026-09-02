#ifndef TARS_TIDE_LAYER__TIDE_LAYER_LOGIC_HPP_
#define TARS_TIDE_LAYER__TIDE_LAYER_LOGIC_HPP_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace tars_tide_layer
{

inline bool should_clear_tide_cell(bool in_prev, bool in_curr)
{
  return in_prev && !in_curr;
}

inline bool should_mark_tide_cell(bool in_curr)
{
  return in_curr;
}

inline bool polygons_equivalent(
  const std::vector<std::pair<double, double>> & a,
  const std::vector<std::pair<double, double>> & b,
  double eps = 1e-4)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::hypot(a[i].first - b[i].first, a[i].second - b[i].second) > eps) {
      return false;
    }
  }
  return true;
}

inline bool polygon_lists_equivalent(
  const std::vector<std::vector<std::pair<double, double>>> & a,
  const std::vector<std::vector<std::pair<double, double>>> & b,
  double eps = 1e-4)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (!polygons_equivalent(a[i], b[i], eps)) {
      return false;
    }
  }
  return true;
}

struct RingAabb
{
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  bool valid{false};
};

inline RingAabb ring_aabb(
  const std::vector<std::pair<double, double>> & ring)
{
  RingAabb box;
  if (ring.empty()) {
    return box;
  }
  box.min_x = box.max_x = ring.front().first;
  box.min_y = box.max_y = ring.front().second;
  box.valid = true;
  for (const auto & pt : ring) {
    box.min_x = std::min(box.min_x, pt.first);
    box.max_x = std::max(box.max_x, pt.first);
    box.min_y = std::min(box.min_y, pt.second);
    box.max_y = std::max(box.max_y, pt.second);
  }
  return box;
}

inline bool ring_is_closed(
  const std::vector<std::pair<double, double>> & ring,
  double close_m = 2.0)
{
  if (ring.size() < 3) {
    return false;
  }
  return std::hypot(
    ring.front().first - ring.back().first,
    ring.front().second - ring.back().second) <= close_m;
}

inline bool aabb_intersects_window(
  const RingAabb & box,
  double min_x, double min_y, double max_x, double max_y,
  double margin)
{
  if (!box.valid) {
    return false;
  }
  return !(box.max_x + margin < min_x ||
           box.min_x - margin > max_x ||
           box.max_y + margin < min_y ||
           box.min_y - margin > max_y);
}

inline bool clip_segment_to_aabb(
  double & x0, double & y0, double & x1, double & y1,
  double min_x, double min_y, double max_x, double max_y)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  double t0 = 0.0;
  double t1 = 1.0;
  auto clip = [&](double p, double q) {
    if (std::abs(p) < 1e-12) {
      return q >= 0.0;
    }
    const double r = q / p;
    if (p < 0.0) {
      if (r > t1) {
        return false;
      }
      if (r > t0) {
        t0 = r;
      }
    } else {
      if (r < t0) {
        return false;
      }
      if (r < t1) {
        t1 = r;
      }
    }
    return true;
  };
  if (!clip(-dx, x0 - min_x) || !clip(dx, max_x - x0) ||
    !clip(-dy, y0 - min_y) || !clip(dy, max_y - y0))
  {
    return false;
  }
  const double nx0 = x0 + t0 * dx;
  const double ny0 = y0 + t0 * dy;
  const double nx1 = x0 + t1 * dx;
  const double ny1 = y0 + t1 * dy;
  x0 = nx0;
  y0 = ny0;
  x1 = nx1;
  y1 = ny1;
  return true;
}

inline double distance_to_segment(
  double px, double py,
  double x1, double y1,
  double x2, double y2)
{
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double len_sq = dx * dx + dy * dy;
  double t = 0.0;
  if (len_sq > 0.0) {
    t = std::max(
      0.0, std::min(
        1.0,
        ((px - x1) * dx + (py - y1) * dy) / len_sq));
  }
  return std::hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

inline bool point_in_ring(
  double px, double py,
  const std::vector<std::pair<double, double>> & ring)
{
  bool inside = false;
  const size_t n = ring.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = ring[i].first;
    const double yi = ring[i].second;
    const double xj = ring[j].first;
    const double yj = ring[j].second;
    if (((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

inline void touch_world(
  double x, double y,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (min_x == nullptr || min_y == nullptr ||
    max_x == nullptr || max_y == nullptr)
  {
    return;
  }
  *min_x = std::min(x, *min_x);
  *min_y = std::min(y, *min_y);
  *max_x = std::max(x, *max_x);
  *max_y = std::max(y, *max_y);
}

// Expand bounds by on-window segments only. A km-scale ring AABB that
// merely overlaps the map must not drag every far vertex into the update.
inline bool touch_ring_in_window(
  const std::vector<std::pair<double, double>> & ring,
  double wx0, double wy0, double wx1, double wy1,
  double margin,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (ring.size() < 2) {
    return false;
  }
  const bool closed = ring_is_closed(ring);
  const size_t n = ring.size();
  const size_t last = closed ? n : n - 1;
  const double clip_x0 = wx0 - margin;
  const double clip_y0 = wy0 - margin;
  const double clip_x1 = wx1 + margin;
  const double clip_y1 = wy1 + margin;
  bool touched = false;
  for (size_t i = 0; i < last; ++i) {
    const size_t j = closed ? ((i + 1) % n) : (i + 1);
    double xa = ring[i].first;
    double ya = ring[i].second;
    double xb = ring[j].first;
    double yb = ring[j].second;
    if (!clip_segment_to_aabb(
        xa, ya, xb, yb, clip_x0, clip_y0, clip_x1, clip_y1))
    {
      continue;
    }
    touch_world(xa, ya, min_x, min_y, max_x, max_y);
    touch_world(xb, yb, min_x, min_y, max_x, max_y);
    touched = true;
  }
  return touched;
}

template<typename Fn>
inline void visit_corridor_cells(
  const std::vector<std::pair<double, double>> & ring,
  double origin_x, double origin_y, double res,
  int min_i, int min_j, int max_i, int max_j,
  double margin,
  Fn && fn)
{
  if (ring.size() < 2 || res <= 0.0 || min_i >= max_i || min_j >= max_j) {
    return;
  }
  const bool closed = ring_is_closed(ring);
  const size_t n = ring.size();
  const size_t last = closed ? n : n - 1;
  const double wx0 = origin_x + min_i * res;
  const double wy0 = origin_y + min_j * res;
  const double wx1 = origin_x + max_i * res;
  const double wy1 = origin_y + max_j * res;
  const double clip_x0 = wx0 - margin;
  const double clip_y0 = wy0 - margin;
  const double clip_x1 = wx1 + margin;
  const double clip_y1 = wy1 + margin;
  for (size_t s = 0; s < last; ++s) {
    const size_t e = closed ? ((s + 1) % n) : (s + 1);
    const double ox0 = ring[s].first;
    const double oy0 = ring[s].second;
    const double ox1 = ring[e].first;
    const double oy1 = ring[e].second;
    double cx0 = ox0;
    double cy0 = oy0;
    double cx1 = ox1;
    double cy1 = oy1;
    if (!clip_segment_to_aabb(
        cx0, cy0, cx1, cy1, clip_x0, clip_y0, clip_x1, clip_y1))
    {
      continue;
    }
    const double cdx = cx1 - cx0;
    const double cdy = cy1 - cy0;
    const double clen = std::hypot(cdx, cdy);
    const double chunk = std::max(margin, 4.0 * res);
    const int n_chunks = (clen > chunk)
      ? static_cast<int>(std::ceil(clen / chunk)) : 1;
    for (int c = 0; c < n_chunks; ++c) {
      const double t0 = static_cast<double>(c) / static_cast<double>(n_chunks);
      const double t1 = static_cast<double>(c + 1) / static_cast<double>(n_chunks);
      const double sx0 = cx0 + t0 * cdx;
      const double sy0 = cy0 + t0 * cdy;
      const double sx1 = cx0 + t1 * cdx;
      const double sy1 = cy0 + t1 * cdy;
      const double minx = std::min(sx0, sx1) - margin;
      const double maxx = std::max(sx0, sx1) + margin;
      const double miny = std::min(sy0, sy1) - margin;
      const double maxy = std::max(sy0, sy1) + margin;
      int i0 = static_cast<int>(std::floor((minx - origin_x) / res));
      int i1 = static_cast<int>(std::floor((maxx - origin_x) / res)) + 1;
      int j0 = static_cast<int>(std::floor((miny - origin_y) / res));
      int j1 = static_cast<int>(std::floor((maxy - origin_y) / res)) + 1;
      i0 = std::max(i0, min_i);
      i1 = std::min(i1, max_i);
      j0 = std::max(j0, min_j);
      j1 = std::min(j1, max_j);
      for (int j = j0; j < j1; ++j) {
        const double wy = origin_y + (j + 0.5) * res;
        for (int i = i0; i < i1; ++i) {
          const double wx = origin_x + (i + 0.5) * res;
          if (distance_to_segment(wx, wy, ox0, oy0, ox1, oy1) <= margin) {
            fn(i, j);
          }
        }
      }
    }
  }
}

inline bool should_restamp_tide(
  bool need_clear, bool origin_moved, bool size_changed, bool painted)
{
  return need_clear || origin_moved || size_changed || !painted;
}

// Interior fill for small closed rings only. A keepout whose AABB covers
// the whole 500 m map would otherwise PIP every cell.
template<typename Fn>
inline void visit_fill_cells(
  const std::vector<std::pair<double, double>> & ring,
  double origin_x, double origin_y, double res,
  int min_i, int min_j, int max_i, int max_j,
  Fn && fn)
{
  if (ring.size() < 3 || res <= 0.0 || !ring_is_closed(ring)) {
    return;
  }
  const auto box = ring_aabb(ring);
  if (!box.valid) {
    return;
  }
  int i0 = static_cast<int>(std::floor((box.min_x - origin_x) / res));
  int i1 = static_cast<int>(std::floor((box.max_x - origin_x) / res)) + 1;
  int j0 = static_cast<int>(std::floor((box.min_y - origin_y) / res));
  int j1 = static_cast<int>(std::floor((box.max_y - origin_y) / res)) + 1;
  i0 = std::max(i0, min_i);
  i1 = std::min(i1, max_i);
  j0 = std::max(j0, min_j);
  j1 = std::min(j1, max_j);
  const long cells = static_cast<long>(i1 - i0) * static_cast<long>(j1 - j0);
  if (cells <= 0 || cells > 400000L) {
    return;
  }
  for (int j = j0; j < j1; ++j) {
    const double wy = origin_y + (j + 0.5) * res;
    for (int i = i0; i < i1; ++i) {
      const double wx = origin_x + (i + 0.5) * res;
      if (point_in_ring(wx, wy, ring)) {
        fn(i, j);
      }
    }
  }
}

}  // namespace tars_tide_layer

#endif  // TARS_TIDE_LAYER__TIDE_LAYER_LOGIC_HPP_