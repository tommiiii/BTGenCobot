cmake_minimum_required(VERSION 3.8)
project(bt_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(btgencobot_interfaces REQUIRED)

# Add combined BehaviorTree plugin library
add_library(bt_nav2_plugins SHARED
  src/detect_object.cpp
  src/pick_object.cpp
  src/place_object.cpp
  src/spin_left.cpp
  src/spin_right.cpp
  src/plugin_registration.cpp
)

target_include_directories(bt_nav2_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(bt_nav2_plugins
  behaviortree_cpp
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  btgencobot_interfaces
)

target_link_libraries(bt_nav2_plugins
  ${OpenCV_LIBRARIES}
)

target_compile_definitions(bt_nav2_plugins PRIVATE BT_PLUGIN_EXPORT)

# Install
install(TARGETS
    bt_nav2_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins/
)

ament_export_include_directories(include)
ament_export_libraries(
  bt_nav2_plugins
)
ament_export_dependencies(
  behaviortree_cpp
  nav2_behavior_tree
  nav2_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  btgencobot_interfaces
)

ament_package()
