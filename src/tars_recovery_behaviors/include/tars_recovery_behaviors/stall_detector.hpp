// TARS recovery branching — STALL classifier (ConditionNode).
//
// STALL = wheel/motor effectively not turning under load (controller safety
// already labeled it). Distinct from SLIP (wheels still spinning).
//
//   SUCCESS -> stall recovery branch
//   FAILURE -> try SlipDetector / obstacle branch (fail-open on no data)

#ifndef TARS_RECOVERY_BEHAVIORS__STALL_DETECTOR_HPP_
#define TARS_RECOVERY_BEHAVIORS__STALL_DETECTOR_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace tars_recovery_behaviors
{

class StallDetector : public BT::ConditionNode
{
public:
  StallDetector(const std::string & name, const BT::NodeConfiguration & config);
  ~StallDetector() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void safetyStateCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_sub_;

  // bt_navigator 의 client 노드는 아무도 spin 하지 않는다. 구독을 기본
  // 콜백그룹에 두면 콜백이 영원히 실행되지 않아, 퍼블리셔가 멀쩡한데도 창이
  // 항상 비고 판정이 fail-open 으로 새어나간다(2026-08-12 라이브에서 실제로
  // 세 탐지기가 동시에 눈이 멀었다). Nav2 자체 조건 노드와 같은 방식으로
  // 전용 콜백그룹 + 로컬 executor 를 두고 tick 마다 직접 spin 한다.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  mutable std::mutex windows_mutex_;
  std::deque<std::pair<double, std::string>> safety_window_;

  // --- STALL 판정 임계 ---
  // window_s: /motor/safety_state 를 뒤져 볼 시간 창(초).
  //   이 창 안 메시지 중 하나라도 "stall" 부분문자열이 있으면 STALL SUCCESS.
  //   물리 임계(cmd/PWM/휠속도)는 여기가 아니라 eclipse_test_controller 가
  //   WARN_STALL_* / FAULT_stall_* 문자열을 낼 때 이미 적용됨.
  double window_s_{10.0};
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__STALL_DETECTOR_HPP_
