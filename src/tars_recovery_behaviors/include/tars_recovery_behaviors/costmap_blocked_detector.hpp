// TARS recovery — COSTMAP / real-obstacle classifier (ConditionNode).
//
// SUCCESS when local costmap shows high occupancy near the robot
// (lethal / near-lethal evidence). FAILURE when map missing, robot pose
// missing, or neighborhood is free enough -> try UNKNOWN branch.
//
// Signals:
//   local_costmap/costmap   nav_msgs/OccupancyGrid (frame: map on TARS)
//   /odometry/filtered      robot pose in map (EKF)

#ifndef TARS_RECOVERY_BEHAVIORS__COSTMAP_BLOCKED_DETECTOR_HPP_
#define TARS_RECOVERY_BEHAVIORS__COSTMAP_BLOCKED_DETECTOR_HPP_

#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tars_recovery_behaviors
{

class CostmapBlockedDetector : public BT::ConditionNode
{
public:
  CostmapBlockedDetector(
    const std::string & name, const BT::NodeConfiguration & config);
  ~CostmapBlockedDetector() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // True if any cell within radius_m of (robot_x, robot_y) has occupancy
  // >= min_occupancy (OccupancyGrid: -1 unknown, 0 free, 100 lethal).
  bool neighborhoodBlocked(
    const nav_msgs::msg::OccupancyGrid & grid,
    double robot_x, double robot_y) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // bt_navigator 의 client 노드는 아무도 spin 하지 않는다 — 기본 콜백그룹에
  // 둔 구독은 콜백이 영원히 안 돈다. 전용 콜백그룹 + 로컬 executor 를 두고
  // tick마다 직접 spin. stall_detector.hpp 참고.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_costmap_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_pose_stamp_s_{0.0};
  bool have_robot_pose_{false};

  // --- COSTMAP 판정 임계 (BT 포트로 덮어쓸 수 있음) ---
  std::string costmap_topic_{"/local_costmap/costmap"};
  // radius_m: 로봇 주변 이 반경(m) 안 셀만 검사.
  double radius_m_{1.0};
  // min_occupancy: OccupancyGrid 값이 이 이상이면 "막힌 셀".
  //   0=자유 ~ 100=lethal. Nav2 매핑에서 99≈inscribed, 100=lethal. 기본 90.
  //   -1(unknown)은 막힘으로 치지 않음 → UNKNOWN 분기로 넘김.
  int min_occupancy_{90};
  // pose_max_age_s: /odometry/filtered 가 이 시간(초)보다 오래되면 COSTMAP 실패.
  double pose_max_age_s_{1.0};
  // costmap_max_age_s: costmap 메시지가 이보다 오래되면 COSTMAP 실패.
  double costmap_max_age_s_{2.0};
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__COSTMAP_BLOCKED_DETECTOR_HPP_
