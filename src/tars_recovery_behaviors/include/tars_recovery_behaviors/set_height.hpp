// TARS mudflat recovery behaviors.
//
// SetHeight — /drive/height_step_mm 에 목표 높이를 내고 Wait 카운트다운만 한다.
// 실제 도달 여부는 보지 않는다.

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

  // Parameters (behavior_server namespace, set_height. prefix).
  double target_height_mm_{30.0};
  double wait_duration_{2.0};
  bool enable_{true};
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__SET_HEIGHT_HPP_