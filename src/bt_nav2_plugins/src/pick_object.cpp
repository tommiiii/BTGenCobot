#include "bt_nav2_plugins/pick_object.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace bt_nav2_plugins
{

PickObject::PickObject(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  service_call_sent_(false),
  response_received_(false)
{
  // Get ROS node from config (Nav2's node - used for logging)
  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("PickObject: 'node' not found in blackboard");
  }

  // Create a separate node for service calls - we spin this ourselves
  service_node_ = std::make_shared<rclcpp::Node>("pick_object_service_node");

  // Create service client on our own node
  manipulator_client_ = service_node_->create_client<btgencobot_interfaces::srv::ManipulatorAction>(
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

  // Get object dimensions (with defaults)
  if (!getInput("object_height", object_height_)) {
    object_height_ = 0.1;  // default 10cm
  }
  if (!getInput("object_width", object_width_)) {
    object_width_ = 0.05;  // default 5cm
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "PickObject: Object pose [%.2f, %.2f, %.2f], height=%.3f, width=%.3f",
    target_pose_.pose.position.x,
    target_pose_.pose.position.y,
    target_pose_.pose.position.z,
    object_height_,
    object_width_);

  // Reset state
  service_call_sent_ = false;
  response_received_ = false;
  response_.reset();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickObject::onRunning()
{
  // Spin our service node to process callbacks
  rclcpp::spin_some(service_node_);

  // Check if service is available and send request
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
    request->object_height = static_cast<float>(object_height_);
    request->object_width = static_cast<float>(object_width_);
    request->object_depth = static_cast<float>(object_width_);  // Assume symmetric for now

    RCLCPP_INFO(node_->get_logger(), "PickObject: Sending pick request to service...");

    // Send async request with callback
    manipulator_client_->async_send_request(
      request,
      [this](rclcpp::Client<btgencobot_interfaces::srv::ManipulatorAction>::SharedFuture future) {
        try {
          response_ = future.get();
          response_received_ = true;
          RCLCPP_INFO(node_->get_logger(), "PickObject: Response received via callback");
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(), "PickObject: Service call failed: %s", e.what());
          response_received_ = true;  // Mark as received so we can handle the error
        }
      });
    service_call_sent_ = true;

    return BT::NodeStatus::RUNNING;
  }

  // Check if service response is ready
  if (!response_received_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "PickObject: Waiting for pick operation to complete...");
    return BT::NodeStatus::RUNNING;
  }

  // Check response
  if (!response_) {
    RCLCPP_ERROR(node_->get_logger(), "PickObject: Service returned null response");
    return BT::NodeStatus::FAILURE;
  }

  if (response_->success) {
    RCLCPP_INFO(node_->get_logger(), "PickObject: Pick operation completed successfully");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PickObject: Pick operation failed: %s",
      response_->error_message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

void PickObject::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "PickObject: Halted");
  service_call_sent_ = false;
  response_received_ = false;
  response_.reset();
}

}  // namespace bt_nav2_plugins
