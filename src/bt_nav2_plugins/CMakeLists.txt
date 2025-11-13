cmake_minimum_required(VERSION 3.8)
project(bt_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)

# Add BehaviorTree plugin library
add_library(say_text_bt_node SHARED
  src/say_text.cpp
)

target_include_directories(say_text_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(say_text_bt_node
  behaviortree_cpp
  nav2_behavior_tree
  rclcpp
)

# Register as BehaviorTree plugin
target_compile_definitions(say_text_bt_node PRIVATE BT_PLUGIN_EXPORT)

# Install
install(TARGETS say_text_bt_node
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
ament_export_libraries(say_text_bt_node)
ament_export_dependencies(
  behaviortree_cpp
  nav2_behavior_tree
  rclcpp
)

ament_package()
