#include "bt_nav2_plugins/pick_object.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace bt_nav2_plugins
{

PickObject::PickObject(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  service_call_sent_(false)
{
  // Get ROS node from config
  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("PickObject: 'node' not found in blackboard");
  }

  // Create service client
  manipulator_client_ = node_->create_client<btgencobot_interfaces::srv::ManipulatorAction>(
    "/manipulator_action");

  RCLCPP_INFO(node_->get_logger(), "PickObject BT node initialized");
}

BT::NodeStatus PickObject::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "PickObject: Starting pick operation");

  // Get target pose from input port (receives object_pose value from BT)
  if (!getInput("target_pose", target_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "PickObject: target_pose not provided");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "PickObject: Object pose [%.2f, %.2f, %.2f]",
    target_pose_.pose.position.x,
    target_pose_.pose.position.y,
    target_pose_.pose.position.z);

  service_call_sent_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickObject::onRunning()
{
  // Check if service is available
  if (!service_call_sent_) {
    if (!manipulator_client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "PickObject: Waiting for /manipulator_action service...");
      return BT::NodeStatus::RUNNING;
    }

    // Create and send service request
    auto request = std::make_shared<btgencobot_interfaces::srv::ManipulatorAction::Request>();
    request->action_type = "pick";
    request->target_pose = target_pose_;

    RCLCPP_INFO(node_->get_logger(), "PickObject: Sending pick request to service...");

    future_result_ = manipulator_client_->async_send_request(request).future.share();
    service_call_sent_ = true;

    return BT::NodeStatus::RUNNING;
  }

  // Check if service response is ready
  if (future_result_.wait_for(0s) != std::future_status::ready) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "PickObject: Waiting for pick operation to complete...");
    return BT::NodeStatus::RUNNING;
  }

  // Get service response
  auto response = future_result_.get();

  if (response->success) {
    RCLCPP_INFO(node_->get_logger(), "PickObject: Pick operation completed successfully");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PickObject: Pick operation failed: %s",
      response->error_message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

void PickObject::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "PickObject: Halted");
  service_call_sent_ = false;
}

}  // namespace bt_nav2_plugins
