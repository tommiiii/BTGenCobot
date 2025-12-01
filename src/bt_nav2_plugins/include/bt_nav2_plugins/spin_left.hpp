#ifndef BT_NAV2_PLUGINS__SPIN_LEFT_HPP_
#define BT_NAV2_PLUGINS__SPIN_LEFT_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node for spinning left (counterclockwise)
 *
 * This wraps the Nav2 Spin action with a positive target_yaw
 * to ensure counterclockwise rotation.
 *
 * Input Ports:
 *   spin_dist - Rotation angle in radians (always positive, will be used as-is)
 *   time_allowance - Maximum time allowed for the spin (seconds)
 */
class SpinLeft : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
  SpinLeft(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("spin_dist", 1.57, "Spin angle in radians (positive)"),
      BT::InputPort<double>("time_allowance", 30.0, "Maximum time allowed in seconds")
    });
  }
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__SPIN_LEFT_HPP_
