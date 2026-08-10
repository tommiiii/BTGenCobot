cmake_minimum_required(VERSION 3.8)
project(manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(btgencobot_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(play_motion2_msgs REQUIRED)

add_executable(manipulator_service src/manipulator_service.cpp)
ament_target_dependencies(manipulator_service
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  btgencobot_interfaces
  control_msgs
  play_motion2_msgs
)

install(TARGETS
  manipulator_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
