#ifndef BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_
#define BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_

#include <string>
#include <memory>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "btgencobot_interfaces/srv/manipulator_action.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node to place an object with the manipulator
 *
 * Input Ports:
 *   target_pose - Pose where object should be placed
 *
 * This node calls the /manipulator_action service to execute a place operation.
 * The service uses ikpy for inverse kinematics and controls the arm via
 * ros2_control trajectory controllers.
 */
class PlaceObject : public BT::StatefulActionNode
{
public:
  PlaceObject(
    const std::string & name,
    const BT::NodeConfiguration & config);

  ~PlaceObject() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Pose where to place object")
    };
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // Nav2's node for logging
  rclcpp::Node::SharedPtr node_;
  
  // Separate node for service calls - we spin this ourselves
  rclcpp::Node::SharedPtr service_node_;
  
  geometry_msgs::msg::PoseStamped target_pose_;

  // Service client for manipulator action
  rclcpp::Client<btgencobot_interfaces::srv::ManipulatorAction>::SharedPtr manipulator_client_;
  
  // Callback-based response handling
  btgencobot_interfaces::srv::ManipulatorAction::Response::SharedPtr response_;
  std::atomic<bool> service_call_sent_;
  std::atomic<bool> response_received_;
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_
