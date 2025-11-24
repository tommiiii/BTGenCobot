#include "bt_nav2_plugins/detect_object.hpp"
#include "bt_nav2_plugins/pick_object.hpp"
#include "bt_nav2_plugins/place_object.hpp"
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_nav2_plugins::DetectObject>("DetectObject");
  factory.registerNodeType<bt_nav2_plugins::PickObject>("PickObject");
  factory.registerNodeType<bt_nav2_plugins::PlaceObject>("PlaceObject");
}
