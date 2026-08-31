#include "tars_tide_layer/tide_layer.hpp"

#include <algorithm>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tars_tide_layer::TideLayer, nav2_costmap_2d::Layer)

namespace tars_tide_layer
{

namespace
{
std::vector<std::pair<double, double>> ring_from_line_strip(
  const visualization_msgs::msg::Marker & marker)
{
  std::vector<std::pair<double, double>> ring;
  ring.reserve(marker.points.size());
  for (const auto & pt : marker.points) {
    ring.emplace_back(pt.x, pt.y);
  }
  if (ring.size() >= 2 &&
    std::hypot(
      ring.front().first - ring.back().first,
      ring.front().second - ring.back().second) < 1e-6)
  {
    ring.pop_back();
  }
  return ring;
}

std::vector<RingAabb> boxes_from_rings(
  const std::vector<std::vector<std::pair<double, double>>> & rings)
{
  std::vector<RingAabb> boxes;
  boxes.reserve(rings.size());
  for (const auto & ring : rings) {
    boxes.push_back(ring_aabb(ring));
  }
  return boxes;
}
}  // namespace

TideLayer::TideLayer() = default;

void TideLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TideLayer: node is null");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("polygon_margin_m", rclcpp::ParameterValue(4.0));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "polygon_margin_m", polygon_margin_m_);
  if (polygon_margin_m_ < 0.0) {
    polygon_margin_m_ = 0.0;
  }

  resolution_ = layered_costmap_->getCostmap()->getResolution();
  origin_x_ = layered_costmap_->getCostmap()->getOriginX();
  origin_y_ = layered_costmap_->getCostmap()->getOriginY();
  size_x_ = layered_costmap_->getCostmap()->getSizeInCellsX();
  size_y_ = layered_costmap_->getCostmap()->getSizeInCellsY();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  markers_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/tide/water_polygon_markers", qos,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      markersCallback(msg);
    });

  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "TideLayer initialized: margin=%.1fm, resolution=%.3f, multi-ring markers",
    polygon_margin_m_, resolution_);
}

void TideLayer::markersCallback(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(polygon_mutex_);
  std::vector<Ring> incoming;
  incoming.reserve(msg->markers.size());
  for (const auto & marker : msg->markers) {
    if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
      continue;
    }
    if (marker.action != visualization_msgs::msg::Marker::ADD) {
      continue;
    }
    if (marker.type != visualization_msgs::msg::Marker::LINE_STRIP) {
      continue;
    }
    Ring ring = ring_from_line_strip(marker);
    if (ring.size() >= 2) {
      incoming.push_back(std::move(ring));
    }
  }
  updateRingsCache(std::move(incoming));
}

void TideLayer::updateRingsCache(std::vector<Ring> incoming)
{
  const bool incoming_valid = !incoming.empty();
  if (!incoming_valid) {
    if (polygon_active_ && !polygons_cache_.empty()) {
      last_clear_polygons_ = polygons_cache_;
      last_clear_boxes_ = polygon_boxes_;
      need_clear_ = true;
    }
    polygons_cache_.clear();
    polygon_boxes_.clear();
    polygon_active_ = false;
    return;
  }
  if (polygon_active_ &&
    !polygon_lists_equivalent(polygons_cache_, incoming))
  {
    last_clear_polygons_ = polygons_cache_;
    last_clear_boxes_ = polygon_boxes_;
    need_clear_ = true;
  } else if (!polygon_active_) {
    last_clear_polygons_.clear();
    last_clear_boxes_.clear();
    need_clear_ = false;
  }
  polygons_cache_ = std::move(incoming);
  polygon_boxes_ = boxes_from_rings(polygons_cache_);
  polygon_active_ = true;
}

void TideLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(polygon_mutex_);

  if (!enabled_) {
    return;
  }

  auto * cmap = layered_costmap_->getCostmap();
  if (!cmap) {
    return;
  }
  const double wx0 = cmap->getOriginX();
  const double wy0 = cmap->getOriginY();
  const double wx1 = wx0 + cmap->getSizeInMetersX();
  const double wy1 = wy0 + cmap->getSizeInMetersY();

  bool touched = false;
  if (polygon_active_) {
    for (const auto & ring : polygons_cache_) {
      touched = touch_ring_in_window(
          ring, wx0, wy0, wx1, wy1, polygon_margin_m_,
          min_x, min_y, max_x, max_y) ||
        touched;
    }
  }
  if (!last_clear_polygons_.empty()) {
    for (const auto & ring : last_clear_polygons_) {
      touched = touch_ring_in_window(
          ring, wx0, wy0, wx1, wy1, polygon_margin_m_,
          min_x, min_y, max_x, max_y) ||
        touched;
    }
  }
  if (touched) {
    *min_x -= polygon_margin_m_;
    *min_y -= polygon_margin_m_;
    *max_x += polygon_margin_m_;
    *max_y += polygon_margin_m_;
  }
}

void TideLayer::stampRings(
  const std::vector<Ring> & rings,
  int min_i, int min_j, int max_i, int max_j,
  std::vector<uint8_t> & mask,
  std::vector<int> & cells) const
{
  const int width = max_i - min_i;
  if (width <= 0 || max_j <= min_j) {
    return;
  }
  auto mark = [&](int i, int j) {
    if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
      return;
    }
    const int k = (j - min_j) * width + (i - min_i);
    if (mask[static_cast<size_t>(k)] != 0) {
      return;
    }
    mask[static_cast<size_t>(k)] = 1;
    cells.push_back(k);
  };
  for (const auto & ring : rings) {
    if (ring_is_closed(ring) && ring.size() >= 3) {
      visit_fill_cells(
        ring, origin_x_, origin_y_, resolution_,
        min_i, min_j, max_i, max_j, mark);
    }
    visit_corridor_cells(
      ring, origin_x_, origin_y_, resolution_,
      min_i, min_j, max_i, max_j, polygon_margin_m_, mark);
  }
}

void TideLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(polygon_mutex_);

  if (!enabled_) {
    current_ = true;
    return;
  }

  origin_x_ = layered_costmap_->getCostmap()->getOriginX();
  origin_y_ = layered_costmap_->getCostmap()->getOriginY();
  size_x_ = layered_costmap_->getCostmap()->getSizeInCellsX();
  size_y_ = layered_costmap_->getCostmap()->getSizeInCellsY();
  resolution_ = layered_costmap_->getCostmap()->getResolution();

  const int width = max_i - min_i;
  const int height = max_j - min_j;
  if (width <= 0 || height <= 0) {
    current_ = true;
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  const size_t n_cells = static_cast<size_t>(width) * static_cast<size_t>(height);

  std::vector<uint8_t> curr_mask;
  std::vector<int> curr_cells;
  if (polygon_active_) {
    curr_mask.assign(n_cells, 0);
    curr_cells.reserve(4096);
    stampRings(
      polygons_cache_, min_i, min_j, max_i, max_j, curr_mask, curr_cells);
  }

  if (!last_clear_polygons_.empty()) {
    std::vector<uint8_t> prev_mask(n_cells, 0);
    std::vector<int> prev_cells;
    prev_cells.reserve(4096);
    stampRings(
      last_clear_polygons_, min_i, min_j, max_i, max_j, prev_mask, prev_cells);
    for (int k : prev_cells) {
      if (!curr_mask.empty() && curr_mask[static_cast<size_t>(k)] != 0) {
        continue;
      }
      const int i = min_i + (k % width);
      const int j = min_j + (k / width);
      master_array[master_grid.getIndex(i, j)] = nav2_costmap_2d::FREE_SPACE;
    }
    last_clear_polygons_.clear();
    last_clear_boxes_.clear();
    need_clear_ = false;
  }

  if (!curr_cells.empty()) {
    for (int k : curr_cells) {
      const int i = min_i + (k % width);
      const int j = min_j + (k / width);
      master_array[master_grid.getIndex(i, j)] =
        nav2_costmap_2d::LETHAL_OBSTACLE;
    }
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 10000,
    "TideLayer paint rings=%zu cells=%zu margin=%.1fm window=%dx%d",
    polygons_cache_.size(), curr_cells.size(), polygon_margin_m_,
    width, height);

  current_ = true;
}

void TideLayer::reset()
{
  std::lock_guard<std::mutex> lock(polygon_mutex_);
  polygons_cache_.clear();
  polygon_boxes_.clear();
  last_clear_polygons_.clear();
  last_clear_boxes_.clear();
  polygon_active_ = false;
  need_clear_ = false;
  current_ = false;
}

}  // namespace tars_tide_layer
