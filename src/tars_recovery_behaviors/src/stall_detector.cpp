// TARS recovery — STALL condition (wheel/motor nearly stopped under load).

#include "tars_recovery_behaviors/stall_detector.hpp"

#include <algorithm>
#include <cctype>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tars_recovery_behaviors/costmap_blocked_detector.hpp"
#include "tars_recovery_behaviors/recovery_announce.hpp"
#include "tars_recovery_behaviors/slip_detector.hpp"

namespace tars_recovery_behaviors
{

namespace
{

constexpr const char * kSafetyStateTopic = "/motor/safety_state";

std::string toLower(const std::string & s)
{
  std::string out(s);
  std::transform(out.begin(), out.end(), out.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return out;
}

}  // namespace

BT::PortsList StallDetector::providedPorts()
{
  return {
    // 임계 window_s (기본 10s): safety_state 히스토리 길이.
    // 물리 스톨 기준(cmd≥0.05, 휠≤0.02, PWM≥500, 1.5s)은 컨트롤러 쪽 상수.
    BT::InputPort<double>(
      "window_s", 10.0,
      "STALL 임계: /motor/safety_state 를 볼 시간 창(초). "
      "창 안 \"stall\" 부분문자열 있으면 SUCCESS."),
  };
}

StallDetector::StallDetector(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("window_s", window_s_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5));
  safety_sub_ = node_->create_subscription<std_msgs::msg::String>(
    kSafetyStateTopic, reliable_qos,
    std::bind(&StallDetector::safetyStateCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "StallDetector ready (STALL only): window_s=%.1f — matches controller "
    "WARN_STALL_* / FAULT_stall_* (high PWM, cmd, near-zero wheel speed)",
    window_s_);
}

void StallDetector::safetyStateCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  const double stamp_s = node_->now().seconds();
  std::lock_guard<std::mutex> lock(windows_mutex_);
  safety_window_.emplace_back(stamp_s, msg->data);
  const double cutoff = stamp_s - window_s_;
  while (!safety_window_.empty() && safety_window_.front().first < cutoff) {
    safety_window_.pop_front();
  }
}

BT::NodeStatus StallDetector::tick()
{
  // 구독 콜백은 여기서만 돈다 — 헤더의 callback_group_ 주석 참고.
  callback_group_executor_.spin_some();

  std::deque<std::pair<double, std::string>> safety_snap;
  {
    const double now_s = node_->now().seconds();
    const double cutoff = now_s - window_s_;
    std::lock_guard<std::mutex> lock(windows_mutex_);
    while (!safety_window_.empty() && safety_window_.front().first < cutoff) {
      safety_window_.pop_front();
    }
    safety_snap = safety_window_;
  }

  // Fail-open: 창에 메시지 없으면 STALL 아님 → SLIP/COSTMAP/UNKNOWN 쪽으로.
  if (safety_snap.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "StallDetector: empty safety window -> FAILURE (try slip/costmap/unknown)");
    return BT::NodeStatus::FAILURE;
  }

  // STALL 판정 기준(복구 쪽): 문자열에 "stall" 포함 여부만 봄.
  //   예: WARN_STALL_PWM_*, FAULT_stall_pwm_high_no_motion_*
  //   온도/HW 등 FAULT_* 는 "stall" 없어서 여기 안 걸림 (의도, v1.1).
  for (const auto & sample : safety_snap) {
    const std::string state = toLower(sample.second);
    if (state.find("stall") != std::string::npos) {
      RCLCPP_INFO(
        node_->get_logger(),
        "StallDetector: safety_state '%s' -> SUCCESS (STALL branch)",
        sample.second.c_str());
      return BT::NodeStatus::SUCCESS;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "StallDetector: no stall marker in window -> FAILURE (try slip/obstacle)");
  return BT::NodeStatus::FAILURE;
}

}  // namespace tars_recovery_behaviors

// Single BT_REGISTER_NODES per .so.
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tars_recovery_behaviors::StallDetector>("StallDetector");
  factory.registerNodeType<tars_recovery_behaviors::SlipDetector>("SlipDetector");
  factory.registerNodeType<tars_recovery_behaviors::CostmapBlockedDetector>(
    "CostmapBlockedDetector");
  factory.registerNodeType<tars_recovery_behaviors::RecoveryAnnounce>("RecoveryAnnounce");
}
