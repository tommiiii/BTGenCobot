#ifndef BT_NAV2_PLUGINS__PICK_OBJECT_HPP_
#define BT_NAV2_PLUGINS__PICK_OBJECT_HPP_

#include <string>
#include <memory>
#include <future>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "btgencobot_interfaces/srv/manipulator_action.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node to pick up an object with the manipulator
 *
 * Input Ports:
 *   target_pose - Pose of object to pick up (should receive object_pose from DetectObject)
 *   object_height - Estimated height of object in meters (for collision-free approach)
 *   object_width - Estimated width of object in meters
 *
 * This node calls the /manipulator_action service to execute a pick operation.
 * The service uses ikpy for inverse kinematics and controls the arm via
 * ros2_control trajectory controllers.
 */
class PickObject : public BT::StatefulActionNode
{
public:
  PickObject(
    const std::string & name,
    const BT::NodeConfiguration & config);

  ~PickObject() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Pose of object to pick"),
      BT::InputPort<double>("object_height", 0.1, "Estimated object height in meters"),
      BT::InputPort<double>("object_width", 0.05, "Estimated object width in meters")
    };
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped target_pose_;
  double object_height_;
  double object_width_;

  // Service client for manipulator action
  rclcpp::Client<btgencobot_interfaces::srv::ManipulatorAction>::SharedPtr manipulator_client_;
  std::shared_future<btgencobot_interfaces::srv::ManipulatorAction::Response::SharedPtr> future_result_;
  bool service_call_sent_;
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__PICK_OBJECT_HPP_
