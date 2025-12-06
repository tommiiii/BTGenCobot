#include "bt_nav2_plugins/detect_object.hpp"
#include "bt_nav2_plugins/pick_object.hpp"
#include "bt_nav2_plugins/place_object.hpp"
#include "bt_nav2_plugins/spin_left.hpp"
#include "bt_nav2_plugins/spin_right.hpp"
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav2_plugins::DetectObject>("DetectObject");
  factory.registerNodeType<bt_nav2_plugins::PickObject>("PickObject");
  factory.registerNodeType<bt_nav2_plugins::PlaceObject>("PlaceObject");
  
  // SpinLeft and SpinRight use registerBuilder to set the action server name to "spin"
  BT::NodeBuilder spin_left_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bt_nav2_plugins::SpinLeft>(name, "spin", config);
    };
  factory.registerBuilder<bt_nav2_plugins::SpinLeft>("SpinLeft", spin_left_builder);
  
  BT::NodeBuilder spin_right_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<bt_nav2_plugins::SpinRight>(name, "spin", config);
    };
  factory.registerBuilder<bt_nav2_plugins::SpinRight>("SpinRight", spin_right_builder);
}
