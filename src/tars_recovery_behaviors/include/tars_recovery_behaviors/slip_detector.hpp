// TARS recovery branching — SLIP / wheel-spin classifier (ConditionNode).
//
// SLIP (헛돌기) = wheels keep turning but GNSS body barely advances.
// Distinct from STALL (wheel/motor nearly stopped under load).
//
//   SUCCESS -> slip recovery branch
//   FAILURE -> obstacle / default branch
//
// Signals:
//   /odom      wheel odometry (eclipse_test_controller)
//   /gps/fix  NavSatFix + status (GPS_node; survives EKF death)

#ifndef TARS_RECOVERY_BEHAVIORS__SLIP_DETECTOR_HPP_
#define TARS_RECOVERY_BEHAVIORS__SLIP_DETECTOR_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace tars_recovery_behaviors
{

struct GpsFixSample
{
  double stamp_s{0.0};
  double lat_deg{0.0};
  double lon_deg{0.0};
  int8_t status{0};
};

class SlipDetector : public BT::ConditionNode
{
public:
  SlipDetector(const std::string & name, const BT::NodeConfiguration & config);
  ~SlipDetector() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // Horizontal metres between two WGS84 points (haversine).
  static double haversineMeters(
    double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

  // Displacement threshold for a NavSatStatus level (conservative).
  double maxDispForStatus(int8_t status) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;

  // bt_navigator 의 client 노드는 아무도 spin 하지 않는다 — 기본 콜백그룹에
  // 둔 구독은 콜백이 영원히 안 돈다. 전용 콜백그룹 + 로컬 executor 를 두고
  // tick마다 직접 spin. stall_detector.hpp 참고.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  mutable std::mutex windows_mutex_;
  // (stamp_s, |v_linear| from wheel odom twist)
  std::deque<std::pair<double, double>> wheel_speed_window_;
  // Valid GNSS samples only (finite lat/lon, status >= min_fix_status_)
  std::deque<GpsFixSample> gps_fix_window_;

  // --- SLIP 판정 임계 (BT 포트로 덮어쓸 수 있음, 기본값은 여기) ---
  // window_s: 휠 속도·GPS 변위를 모아 보는 시간 창(초).
  double window_s_{10.0};
  // min_wheel_speed_mps: 창 안 |/odom 선속도| 평균이 이 값 이상이면 "바퀴가 돈다".
  //   컨트롤러 MOTOR_STALL_MIN_CMD_MPS(0.05)와 같은 자릿수.
  double min_wheel_speed_mps_{0.05};
  // min_fix_status: 이 등급 미만 GPS 샘플은 버림.
  //   -1=NO_FIX, 0=FIX(단독), 1=SBAS/Float, 2=GBAS/RTK Fixed.
  int min_fix_status_{0};
  // max_disp_*_m: 창 양 끝 GPS 수평 이동(m)이 이보다 작으면 "몸은 거의 안 감".
  //   양 끝 중 더 나쁜 status로 어느 임계를 쓸지 고름 (보수적).
  double max_disp_fix_m_{0.50};   // status 0 단독 fix — GPS 노이즈 커서 여유
  double max_disp_sbas_m_{0.35};   // status 1 Float/DGNSS
  double max_disp_gbas_m_{0.15};   // status 2 RTK Fixed — 가장 빡셈
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__SLIP_DETECTOR_HPP_
