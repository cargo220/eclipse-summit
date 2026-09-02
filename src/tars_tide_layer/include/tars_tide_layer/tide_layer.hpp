#ifndef TARS_TIDE_LAYER__TIDE_LAYER_HPP_
#define TARS_TIDE_LAYER__TIDE_LAYER_HPP_

#include <cstdint>
#include <mutex>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tars_tide_layer/tide_layer_logic.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace tars_tide_layer
{

class TideLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TideLayer();
  ~TideLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

  bool isClearable() override { return true; }

private:
  using Ring = std::vector<std::pair<double, double>>;

  void markersCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg);

  void updateRingsCache(std::vector<Ring> incoming);

  void stampRings(
    const std::vector<Ring> & rings,
    int min_i, int min_j, int max_i, int max_j,
    std::vector<uint8_t> & mask,
    std::vector<int> & cells) const;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
    markers_sub_;
  std::mutex polygon_mutex_;

  std::vector<Ring> polygons_cache_;
  std::vector<RingAabb> polygon_boxes_;
  std::vector<Ring> last_clear_polygons_;
  std::vector<RingAabb> last_clear_boxes_;
  bool polygon_active_{false};
  bool need_clear_{false};
  bool painted_{false};
  double polygon_margin_m_{4.0};

  double resolution_{0.1};
  double origin_x_{0.0};
  double origin_y_{0.0};
  unsigned int size_x_{0};
  unsigned int size_y_{0};
};

}  // namespace tars_tide_layer

#endif  // TARS_TIDE_LAYER__TIDE_LAYER_HPP_
