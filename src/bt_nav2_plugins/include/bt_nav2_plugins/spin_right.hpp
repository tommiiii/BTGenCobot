#ifndef BT_NAV2_PLUGINS__SPIN_RIGHT_HPP_
#define BT_NAV2_PLUGINS__SPIN_RIGHT_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node for spinning right (clockwise)
 *
 * This wraps the Nav2 Spin action with a negative target_yaw
 * to ensure clockwise rotation.
 *
 * Input Ports:
 *   spin_dist - Rotation angle in radians (always positive, will be negated internally)
 *   time_allowance - Maximum time allowed for the spin (seconds)
 */
class SpinRight : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
  SpinRight(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("spin_dist", 1.57, "Spin angle in radians (positive, will be negated)"),
      BT::InputPort<double>("time_allowance", 30.0, "Maximum time allowed in seconds")
    });
  }
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__SPIN_RIGHT_HPP_
