// TARS mudflat recovery behaviors.
//
// MudAssess — measurement-only recovery behavior (v1).
// Drop-in replacement for nav2's stock Wait inside the TARS recovery
// fallback tree: it waits the requested duration (identical feedback/result
// contract to nav2_behaviors/Wait) while passively recording mud-relevant
// signals (probe angle availability, wheel motor currents, stall state) as
// one CSV line on a diagnostic topic at episode end.
//
// v1 is strictly record-only:
//   - never commands velocity (only the base-class stop command on
//     termination touches cmd_vel),
//   - never blocks the shared behavior_server cycle loop (all sensor reads
//     come from subscription caches filled on the executor thread),
//   - never changes the recovery outcome (returns SUCCEEDED after the wait
//     elapses, exactly like Wait).

#ifndef TARS_RECOVERY_BEHAVIORS__MUD_ASSESS_HPP_
#define TARS_RECOVERY_BEHAVIORS__MUD_ASSESS_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "eclipse_pkg_msgs/msg/present_current.hpp"

namespace tars_recovery_behaviors
{

using WaitAction = nav2_msgs::action::Wait;

class MudAssess : public nav2_behaviors::TimedBehavior<WaitAction>
{
public:
  MudAssess() = default;
  ~MudAssess() override = default;

  // nav2_core::Behavior lifecycle hooks (Humble: all void-returning).
  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion() override;

  // TimedBehavior execution hooks.
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const WaitAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  // Subscription callbacks — cache latest value + receive time only.
  // They run on the behavior_server executor thread, while the action
  // execute loop runs on the SimpleActionServer async thread, so all cache
  // access is guarded by cache_mutex_.
  void safetyStateCallback(std_msgs::msg::String::SharedPtr msg);
  void presentCurrentCallback(eclipse_pkg_msgs::msg::PresentCurrent::SharedPtr msg);
  void probeAngleCallback(std_msgs::msg::Float32::SharedPtr msg);

  // Publishes the one-line CSV episode record on assessment_topic_.
  void publishEpisodeRecord();

  // --- Subscriptions + caches ---
  std::mutex cache_mutex_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_sub_;
  std::string last_safety_state_;
  rclcpp::Time last_safety_time_{0, 0, RCL_ROS_TIME};
  bool safety_received_{false};

  rclcpp::Subscription<eclipse_pkg_msgs::msg::PresentCurrent>::SharedPtr current_sub_;
  float last_current_l_{0.0f};
  float last_current_r_{0.0f};
  rclcpp::Time last_current_time_{0, 0, RCL_ROS_TIME};
  bool current_received_{false};

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr probe_sub_;
  float last_probe_angle_{0.0f};
  rclcpp::Time last_probe_time_{0, 0, RCL_ROS_TIME};
  bool probe_received_{false};

  // --- Episode state (captured in onRun, published in onActionCompletion) ---
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr assessment_pub_;
  rclcpp::Time episode_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wait_end_{0, 0, RCL_ROS_TIME};
  double wait_duration_s_{0.0};
  bool episode_valid_{false};
  WaitAction::Feedback::SharedPtr feedback_;

  // Snapshot of the cached signals taken at onRun (consistent single read).
  bool snap_probe_available_{false};
  float snap_probe_angle_{0.0f};
  double snap_probe_age_s_{-1.0};
  float snap_current_l_{0.0f};
  float snap_current_r_{0.0f};
  bool snap_stall_flag_{false};

  // Parameters (read within the behavior_server node namespace).
  double probe_timeout_s_{1.0};
  std::string assessment_topic_{"recovery/mud_assessment"};
  bool enable_recording_{true};
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__MUD_ASSESS_HPP_
