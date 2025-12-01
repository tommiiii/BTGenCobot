#include "bt_nav2_plugins/spin_left.hpp"
#include <cmath>

namespace bt_nav2_plugins
{

SpinLeft::SpinLeft(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
}

void SpinLeft::on_tick()
{
  // Get spin distance (should be positive for left/counterclockwise)
  double spin_dist = 1.57;  // Default 90 degrees
  getInput("spin_dist", spin_dist);

  // Ensure positive value for counterclockwise rotation
  goal_.target_yaw = std::abs(spin_dist);

  // Get time allowance
  double time_allowance = 30.0;
  getInput("time_allowance", time_allowance);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace bt_nav2_plugins
