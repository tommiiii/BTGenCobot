#ifndef BT_NAV2_PLUGINS__PICK_OBJECT_HPP_
#define BT_NAV2_PLUGINS__PICK_OBJECT_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node to pick up an object with the manipulator
 *
 * Input Ports:
 *   target_pose - Pose of object to pick up
 *
 * This is a placeholder implementation that simulates pick behavior.
 * In a full implementation, this would:
 *   1. Move arm to pre-grasp pose
 *   2. Open gripper
 *   3. Approach object
 *   4. Close gripper
 *   5. Lift object
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
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Pose of object to pick")
    };
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped target_pose_;
  rclcpp::Time start_time_;
  double simulated_duration_;  // seconds
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__PICK_OBJECT_HPP_
