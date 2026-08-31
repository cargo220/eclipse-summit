// Lightweight recovery-branch announcement for operators/logs.
// Always returns SUCCESS; place after a classifier that already passed.

#ifndef TARS_RECOVERY_BEHAVIORS__RECOVERY_ANNOUNCE_HPP_
#define TARS_RECOVERY_BEHAVIORS__RECOVERY_ANNOUNCE_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace tars_recovery_behaviors
{

class RecoveryAnnounce : public BT::SyncActionNode
{
public:
  RecoveryAnnounce(const std::string & name, const BT::NodeConfiguration & config);
  ~RecoveryAnnounce() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace tars_recovery_behaviors

#endif  // TARS_RECOVERY_BEHAVIORS__RECOVERY_ANNOUNCE_HPP_
