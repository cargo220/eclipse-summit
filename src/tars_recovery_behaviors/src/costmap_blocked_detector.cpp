// TARS recovery — COSTMAP branch: high occupancy near robot.

#include "tars_recovery_behaviors/costmap_blocked_detector.hpp"

#include <cmath>

namespace tars_recovery_behaviors
{

namespace
{

// Absolute names: bt_navigator must not resolve these under its own namespace.
constexpr const char * kDefaultCostmapTopic = "/local_costmap/costmap";
constexpr const char * kRobotOdomTopic = "/odometry/filtered";

}  // namespace

BT::PortsList CostmapBlockedDetector::providedPorts()
{
  // 아래 기본값이 COSTMAP 임계. BT XML 이 덮어씀.
  return {
    BT::InputPort<std::string>(
      "costmap_topic", kDefaultCostmapTopic,
      "local costmap OccupancyGrid 토픽."),
    BT::InputPort<double>(
      "radius_m", 1.0,
      "COSTMAP 임계: 로봇 주변 검사 반경(m). 이 안만 봄."),
    BT::InputPort<int>(
      "min_occupancy", 90,
      "COSTMAP 임계: 셀 occupancy ≥ 이 값이면 막힘 (0~100, 100=lethal). "
      "-1 unknown 은 막힘으로 안 봄."),
    BT::InputPort<double>(
      "pose_max_age_s", 1.0,
      "COSTMAP 임계: 로봇 pose 가 이(초)보다 오래되면 판정 포기 → UNKNOWN."),
    BT::InputPort<double>(
      "costmap_max_age_s", 2.0,
      "COSTMAP 임계: costmap 이 이(초)보다 오래되면 판정 포기 → UNKNOWN."),
  };
}

CostmapBlockedDetector::CostmapBlockedDetector(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("costmap_topic", costmap_topic_);
  getInput("radius_m", radius_m_);
  getInput("min_occupancy", min_occupancy_);
  getInput("pose_max_age_s", pose_max_age_s_);
  getInput("costmap_max_age_s", costmap_max_age_s_);

  const rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  const rclcpp::QoS odom_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, map_qos,
    std::bind(&CostmapBlockedDetector::costmapCallback, this, std::placeholders::_1),
    sub_option);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    kRobotOdomTopic, odom_qos,
    std::bind(&CostmapBlockedDetector::odomCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "CostmapBlockedDetector ready (COSTMAP): topic=%s radius=%.2f "
    "min_occupancy=%d — high cost near robot => COSTMAP branch",
    costmap_topic_.c_str(), radius_m_, min_occupancy_);
}

void CostmapBlockedDetector::costmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_costmap_ = msg;
}

void CostmapBlockedDetector::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_pose_stamp_s_ = node_->now().seconds();
  have_robot_pose_ = true;
}

bool CostmapBlockedDetector::neighborhoodBlocked(
  const nav_msgs::msg::OccupancyGrid & grid,
  double robot_x, double robot_y) const
{
  if (grid.info.resolution <= 0.0 || grid.info.width == 0 ||
    grid.info.height == 0 || grid.data.empty())
  {
    return false;
  }

  const double res = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);

  const int robot_mx = static_cast<int>(std::floor((robot_x - origin_x) / res));
  const int robot_my = static_cast<int>(std::floor((robot_y - origin_y) / res));
  const int cell_radius = static_cast<int>(std::ceil(radius_m_ / res));

  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      const int mx = robot_mx + dx;
      const int my = robot_my + dy;
      if (mx < 0 || my < 0 || mx >= width || my >= height) {
        continue;
      }
      const double wx = origin_x + (mx + 0.5) * res;
      const double wy = origin_y + (my + 0.5) * res;
      const double dist = std::hypot(wx - robot_x, wy - robot_y);
      if (dist > radius_m_) {
        continue;
      }
      const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(width) +
        static_cast<size_t>(mx);
      if (idx >= grid.data.size()) {
        continue;
      }
      const int8_t occ = grid.data[idx];
      // 임계 min_occupancy: 이 이상이면 막힘. -1(unknown)은 항상 미만이라 제외.
      if (occ >= static_cast<int8_t>(min_occupancy_)) {
        return true;
      }
    }
  }
  return false;
}

BT::NodeStatus CostmapBlockedDetector::tick()
{
  // 구독 콜백은 여기서만 돈다 — 헤더의 callback_group_ 주석 참고.
  callback_group_executor_.spin_some();

  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  double rx = 0.0;
  double ry = 0.0;
  bool have_pose = false;
  double pose_age = 1e9;
  double map_age = 1e9;

  {
    const double now_s = node_->now().seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    grid = last_costmap_;
    have_pose = have_robot_pose_;
    rx = robot_x_;
    ry = robot_y_;
    if (have_robot_pose_) {
      pose_age = now_s - robot_pose_stamp_s_;
    }
    if (grid) {
      // Prefer message stamp when available; fall back to "have map" if zero.
      const double map_stamp =
        static_cast<double>(grid->header.stamp.sec) +
        1e-9 * static_cast<double>(grid->header.stamp.nanosec);
      if (map_stamp > 0.0) {
        map_age = now_s - map_stamp;
      } else {
        map_age = 0.0;
      }
    }
  }

  if (!grid || map_age > costmap_max_age_s_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "CostmapBlockedDetector: no fresh costmap (age=%.2f) -> FAILURE "
      "(try UNKNOWN)",
      map_age);
    return BT::NodeStatus::FAILURE;
  }

  if (!have_pose || pose_age > pose_max_age_s_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "CostmapBlockedDetector: no fresh robot pose (age=%.2f) -> FAILURE "
      "(try UNKNOWN)",
      pose_age);
    return BT::NodeStatus::FAILURE;
  }

  // Costmap frame should match odom frame (TARS: map). No TF here for v0.
  if (neighborhoodBlocked(*grid, rx, ry)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "CostmapBlockedDetector: occupancy>=%d within %.2fm of robot "
      "(%.2f, %.2f) -> SUCCESS (COSTMAP branch)",
      min_occupancy_, radius_m_, rx, ry);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "CostmapBlockedDetector: neighborhood free enough -> FAILURE "
    "(try UNKNOWN)");
  return BT::NodeStatus::FAILURE;
}

}  // namespace tars_recovery_behaviors
