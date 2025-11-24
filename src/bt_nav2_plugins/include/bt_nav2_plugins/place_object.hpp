#ifndef BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_
#define BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node to place an object with the manipulator
 *
 * Input Ports:
 *   target_pose - Pose where object should be placed
 *
 * This is a placeholder implementation that simulates place behavior.
 * In a full implementation, this would:
 *   1. Move to pre-place pose above target
 *   2. Lower arm to place pose
 *   3. Open gripper
 *   4. Retract arm
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

  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped target_pose_;
  rclcpp::Time start_time_;
  double simulated_duration_;  // seconds
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_
