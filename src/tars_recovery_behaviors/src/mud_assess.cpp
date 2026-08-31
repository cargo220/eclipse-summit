// TARS mudflat recovery behaviors.
//
// MudAssess — measurement-only recovery behavior (v1). See mud_assess.hpp
// for the design contract. Reuses nav2_msgs/action/Wait so BT trees can
// route to it with <Wait server_name="mud_assess" .../> and no custom BT
// node is needed.

#include "tars_recovery_behaviors/mud_assess.hpp"

#include <iomanip>
#include <sstream>
#include <string>

#include "nav2_util/node_utils.hpp"

namespace tars_recovery_behaviors
{

void MudAssess::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // Parameters live in the behavior_server node namespace (this plugin does
  // not own a node), so names carry the "mud_assess." plugin prefix.
  nav2_util::declare_parameter_if_not_declared(
    node, "mud_assess.probe_timeout_s", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "mud_assess.assessment_topic",
    rclcpp::ParameterValue(std::string("recovery/mud_assessment")));
  nav2_util::declare_parameter_if_not_declared(
    node, "mud_assess.enable_recording", rclcpp::ParameterValue(true));

  node->get_parameter("mud_assess.probe_timeout_s", probe_timeout_s_);
  node->get_parameter("mud_assess.assessment_topic", assessment_topic_);
  node->get_parameter("mud_assess.enable_recording", enable_recording_);

  // Publisher QoS of all three subscribed topics was verified from the live
  // sources to be default RELIABLE, KeepLast(10):
  //   /motor/safety_state  std_msgs/String
  //     (eclipse_test_controller.py create_publisher(..., 10))
  //   /present_current     eclipse_pkg_msgs/msg/PresentCurrent, data1=left,
  //     data2=right (eclipse_test_controller.py publish_present_current)
  //   /probe/angle         std_msgs/Float32 (probe_sensor.py)
  // RELIABLE KeepLast(5) is therefore compatible with every publisher.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(5));

  assessment_pub_ =
    node->create_publisher<std_msgs::msg::String>(assessment_topic_, 10);

  safety_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/motor/safety_state", qos,
    std::bind(&MudAssess::safetyStateCallback, this, std::placeholders::_1));
  current_sub_ = node->create_subscription<eclipse_pkg_msgs::msg::PresentCurrent>(
    "/present_current", qos,
    std::bind(&MudAssess::presentCurrentCallback, this, std::placeholders::_1));
  probe_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    "/probe/angle", qos,
    std::bind(&MudAssess::probeAngleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "MudAssess configured: recording=%s, topic=%s, probe_timeout_s=%.3f",
    enable_recording_ ? "true" : "false", assessment_topic_.c_str(),
    probe_timeout_s_);
}

void MudAssess::onCleanup()
{
  auto node = node_.lock();
  assessment_pub_.reset();
  safety_sub_.reset();
  current_sub_.reset();
  probe_sub_.reset();
  (void)node;
}

void MudAssess::safetyStateCallback(std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  last_safety_state_ = msg->data;
  last_safety_time_ = clock_->now();
  safety_received_ = true;
}

void MudAssess::presentCurrentCallback(
  eclipse_pkg_msgs::msg::PresentCurrent::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  // PresentCurrent field mapping verified from
  // eclipse_test_controller.py publish_present_current(cu_l, cu_r):
  // data1 = left wheel current, data2 = right wheel current.
  last_current_l_ = msg->data1;
  last_current_r_ = msg->data2;
  last_current_time_ = clock_->now();
  current_received_ = true;
}

void MudAssess::probeAngleCallback(std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  last_probe_angle_ = msg->data;
  last_probe_time_ = clock_->now();
  probe_received_ = true;
}

nav2_behaviors::Status MudAssess::onRun(
  const std::shared_ptr<const WaitAction::Goal> command)
{
  const rclcpp::Duration wait_duration(command->time);
  episode_start_time_ = clock_->now();
  wait_end_ = episode_start_time_ + wait_duration;
  wait_duration_s_ = wait_duration.seconds();
  feedback_ = std::make_shared<WaitAction::Feedback>();

  // Snapshot of the cached signals at episode start, taken under a single
  // lock so the recorded values are mutually consistent.
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const double probe_age_s = probe_received_ ?
      (episode_start_time_ - last_probe_time_).seconds() : -1.0;
    // Probe absent for longer than probe_timeout_s -> treat as not mounted
    // (the probe is an optional sensor; absence is a normal state).
    snap_probe_available_ = probe_received_ && probe_age_s <= probe_timeout_s_;
    snap_probe_angle_ = probe_received_ ? last_probe_angle_ : 0.0f;
    snap_probe_age_s_ = probe_age_s;
    snap_current_l_ = current_received_ ? last_current_l_ : 0.0f;
    snap_current_r_ = current_received_ ? last_current_r_ : 0.0f;
    // /motor/safety_state carries e.g. "OK" or "FAULT_stall_pwm_high_no_motion_...";
    // any stall condition therefore contains the "stall" substring.
    snap_stall_flag_ = last_safety_state_.find("stall") != std::string::npos;
  }
  episode_valid_ = true;

  RCLCPP_INFO(
    logger_,
    "MudAssess episode start: wait=%.3fs, probe_available=%s, stall=%s",
    wait_duration_s_,
    snap_probe_available_ ? "true" : "false",
    snap_stall_flag_ ? "true" : "false");

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status MudAssess::onCycleUpdate()
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

void MudAssess::onActionCompletion()
{
  // Called by the base class on every terminal path (succeeded, failed,
  // canceled, preempted). The episode happened either way, so record it.
  if (!enable_recording_ || !episode_valid_) {
    episode_valid_ = false;
    return;
  }
  publishEpisodeRecord();
  episode_valid_ = false;
}

void MudAssess::publishEpisodeRecord()
{
  // CSV: timestamp,wait_duration_s,probe_available,probe_angle,probe_age_s,
  //      current_l,current_r,stall_flag,mode
  // probe_age_s = -1.000 when the probe topic was never received.
  std::ostringstream csv;
  csv << std::fixed << std::setprecision(3)
      << episode_start_time_.seconds() << ','
      << wait_duration_s_ << ','
      << (snap_probe_available_ ? "true" : "false") << ','
      << snap_probe_angle_ << ','
      << snap_probe_age_s_ << ','
      << snap_current_l_ << ','
      << snap_current_r_ << ','
      << (snap_stall_flag_ ? "true" : "false") << ','
      << "v1_record_only";

  std_msgs::msg::String msg;
  msg.data = csv.str();
  assessment_pub_->publish(msg);

  RCLCPP_INFO(logger_, "MudAssess episode record: %s", msg.data.c_str());
}

}  // namespace tars_recovery_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tars_recovery_behaviors::MudAssess, nav2_core::Behavior)
