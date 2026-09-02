// TARS mudflat recovery behaviors.
//
// SetHeight — publishes a target probe/actuator height to /drive/height_step_mm
// at the start of a recovery episode, then waits a fixed duration. Same
// TimedBehavior<nav2_msgs::action::Wait>. The wait
// duration is read from the set_height.wait_duration parameter and the countdown
// simply runs until it elapses; no height-arrival confirmation is performed.

#include "tars_recovery_behaviors/set_height.hpp"

#include "nav2_util/node_utils.hpp"

namespace tars_recovery_behaviors
{

void SetHeight::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // Parameters live in the behavior_server node namespace (this plugin does
  // not own a node), so names carry the "set_height." plugin prefix.
  nav2_util::declare_parameter_if_not_declared(
    node, "set_height.target_height_mm", rclcpp::ParameterValue(30.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "set_height.wait_duration", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "set_height.enable", rclcpp::ParameterValue(true));

  node->get_parameter("set_height.target_height_mm", target_height_mm_);
  node->get_parameter("set_height.wait_duration", wait_duration_);
  node->get_parameter("set_height.enable", enable_);

  // /drive/height_step_mm is consumed by the controller's height bus (the
  // actual servo write is owned by eclipse_test_controller). RELIABLE QoS
  // matches the controller's publisher/subscriber default.
  height_pub_ = node->create_publisher<std_msgs::msg::Float32>(
    "/drive/height_step_mm", 10);

  RCLCPP_INFO(
    logger_,
    "SetHeight configured: enable=%s, target_height_mm=%.2f, wait_duration=%.2f",
    enable_ ? "true" : "false", target_height_mm_, wait_duration_);
}

void SetHeight::onCleanup()
{
  auto node = node_.lock();
  height_pub_.reset();
  (void)node;
}

nav2_behaviors::Status SetHeight::onRun(
  const std::shared_ptr<const WaitAction::Goal> command)
{
  (void)command;  // duration is governed by the set_height.wait_duration param.

  // Publish the commanded target height immediately at episode start, so the
  // actuator starts moving toward it during the wait.
  if (enable_) {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(target_height_mm_);
    height_pub_->publish(msg);
  }

  // Pure time-based countdown: no height-arrival check. The wait duration
  // comes from the set_height.wait_duration parameter ("기본 Wait 카운트다운").
  wait_duration_s_ = wait_duration_;
  wait_end_ = clock_->now() + rclcpp::Duration::from_seconds(wait_duration_s_);
  feedback_ = std::make_shared<WaitAction::Feedback>();

  RCLCPP_INFO(
    logger_,
    "SetHeight episode start: target_height_mm=%.2f, wait=%.2fs",
    target_height_mm_, wait_duration_s_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status SetHeight::onCycleUpdate()
{
  // Identical countdown contract to nav2_behaviors::Wait: non-blocking,
  // publishes time_left feedback at the shared behavior_server cycle rate.
  const auto current_point = clock_->now();
  if (current_point >= wait_end_) {
    return nav2_behaviors::Status::SUCCEEDED;
  }

  feedback_->time_left = wait_end_ - current_point;
  action_server_->publish_feedback(feedback_);
  return nav2_behaviors::Status::RUNNING;
}

void SetHeight::onActionCompletion()
{
  // Intentionally empty: SetHeight has no post-episode bookkeeping.
}

}  // namespace tars_recovery_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tars_recovery_behaviors::SetHeight, nav2_core::Behavior)