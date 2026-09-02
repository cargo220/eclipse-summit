// TARS mudflat recovery behaviors.
//
// SetHeight — recovery behavior that raises/lowers the probe (or any height
// actuator) by publishing a target height on /drive/height_step_mm during a
// recovery episode, then waits a fixed duration.
//
// Design (same skeleton as MudAssess):
//   - derives from nav2_behaviors::TimedBehavior<nav2_msgs::action::Wait>, so
//     BT trees can route to it the same way they route to Wait / MudAssess,
//   - onRun() publishes <Float32> target_height on /drive/height_step_mm,
//   - onCycleUpdate() only runs the base Wait countdown — it never inspects
//     whether the height was actually reached,
//   - onActionCompletion() is intentionally empty.

#ifndef TARS_RECOVERY_BEHAVIORS__SET_HEIGHT_HPP_
#define TARS_RECOVERY_BEHAVIORS__SET_HEIGHT_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace tars_recovery_behaviors
{

using WaitAction = nav2_msgs::action::Wait;

class SetHeight : public nav2_behaviors::TimedBehavior<WaitAction>
{
public:
  SetHeight() = default;
  ~SetHeight() override = default;

  // nav2_core::Behavior lifecycle hooks (Humble: all void-returning).
  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion() override;

  // TimedBehavior execution hooks.
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const WaitAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  // Publisher for the target height command, created in onConfigure so the
  // node stays healthy even before any episode runs.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_pub_;

  // Episode state (captured in onRun, used for the countdown).
  rclcpp::Time wait_end_{0, 0, RCL_ROS_TIME};
  double wait_duration_s_{0.0};
  WaitAction::Feedback::SharedPtr feedback_;

  // Parameters (read within the behavior_server node namespace,
  // "set_height." plugin prefix — MudAssess pattern).
  double target_height_mm_{30.0};
  double wait_duration_{2.0};
  bool enable_{true};
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__SET_HEIGHT_HPP_