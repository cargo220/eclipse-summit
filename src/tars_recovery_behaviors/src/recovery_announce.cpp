#include "tars_recovery_behaviors/recovery_announce.hpp"

namespace tars_recovery_behaviors
{

BT::PortsList RecoveryAnnounce::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "branch", "UNKNOWN",
      "Recovery structure name: STALL | SLIP | OBSTACLE | ..."),
  };
}

RecoveryAnnounce::RecoveryAnnounce(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  // Transient status for Foxglove / ros2 topic echo; depth 1 is enough.
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/recovery/active_branch", 10);
}

BT::NodeStatus RecoveryAnnounce::tick()
{
  std::string branch = "UNKNOWN";
  getInput("branch", branch);

  RCLCPP_WARN(
    node_->get_logger(),
    "[RECOVERY] running structure: %s", branch.c_str());

  std_msgs::msg::String msg;
  msg.data = branch;
  status_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace tars_recovery_behaviors
