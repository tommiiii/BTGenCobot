#include "bt_nav2_plugins/pick_object.hpp"

namespace bt_nav2_plugins
{

PickObject::PickObject(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  simulated_duration_(3.0)  // 3 seconds to simulate pick action
{
  // Get ROS node from config
  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("PickObject: 'node' not found in blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "PickObject BT node initialized");
}

BT::NodeStatus PickObject::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "PickObject: Starting pick operation");

  // Get target pose from input port
  if (!getInput("target_pose", target_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "PickObject: target_pose not provided");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "PickObject: Target pose [%.2f, %.2f, %.2f]",
    target_pose_.pose.position.x,
    target_pose_.pose.position.y,
    target_pose_.pose.position.z);

  start_time_ = node_->get_clock()->now();

  // TODO: In full implementation, this would:
  // 1. Call MoveIt or joint trajectory controller to move arm
  // 2. Open gripper
  // 3. Move to pre-grasp pose
  // 4. Move to grasp pose
  // 5. Close gripper
  // 6. Lift object

  RCLCPP_INFO(node_->get_logger(), "PickObject: Simulating pick operation (3s)...");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickObject::onRunning()
{
  // Simulate pick operation taking some time
  auto elapsed = (node_->get_clock()->now() - start_time_).seconds();

  if (elapsed < simulated_duration_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,  // 1 second throttle
      "PickObject: In progress... (%.1f/%.1f s)",
      elapsed,
      simulated_duration_);
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "PickObject: Pick operation completed");
  return BT::NodeStatus::SUCCESS;
}

void PickObject::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "PickObject: Halted");
  // TODO: Stop any in-progress manipulator motions
}

}  // namespace bt_nav2_plugins
