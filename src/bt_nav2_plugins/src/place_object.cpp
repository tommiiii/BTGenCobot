#include "bt_nav2_plugins/place_object.hpp"

namespace bt_nav2_plugins
{

PlaceObject::PlaceObject(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  simulated_duration_(2.5)  // 2.5 seconds to simulate place action
{
  // Get ROS node from config
  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("PlaceObject: 'node' not found in blackboard");
  }

  RCLCPP_INFO(node_->get_logger(), "PlaceObject BT node initialized");
}

BT::NodeStatus PlaceObject::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "PlaceObject: Starting place operation");

  // Get target pose from input port
  if (!getInput("target_pose", target_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "PlaceObject: target_pose not provided");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "PlaceObject: Target pose [%.2f, %.2f, %.2f]",
    target_pose_.pose.position.x,
    target_pose_.pose.position.y,
    target_pose_.pose.position.z);

  start_time_ = node_->get_clock()->now();

  // TODO: In full implementation, this would:
  // 1. Move to pre-place pose (above target)
  // 2. Lower arm to place pose
  // 3. Open gripper to release object
  // 4. Retract arm to safe position

  RCLCPP_INFO(node_->get_logger(), "PlaceObject: Simulating place operation (2.5s)...");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlaceObject::onRunning()
{
  // Simulate place operation taking some time
  auto elapsed = (node_->get_clock()->now() - start_time_).seconds();

  if (elapsed < simulated_duration_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,  // 1 second throttle
      "PlaceObject: In progress... (%.1f/%.1f s)",
      elapsed,
      simulated_duration_);
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "PlaceObject: Place operation completed");
  return BT::NodeStatus::SUCCESS;
}

void PlaceObject::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "PlaceObject: Halted");
  // TODO: Stop any in-progress manipulator motions
}

}  // namespace bt_nav2_plugins
