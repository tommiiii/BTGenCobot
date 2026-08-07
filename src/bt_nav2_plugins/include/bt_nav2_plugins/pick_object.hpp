#ifndef BT_NAV2_PLUGINS__PICK_OBJECT_HPP_
#define BT_NAV2_PLUGINS__PICK_OBJECT_HPP_

#include <string>
#include <memory>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "btgencobot_interfaces/srv/manipulator_action.hpp"
#include "btgencobot_interfaces/srv/detect_object.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node to pick up an object with the manipulator
 *
 * Flow:
 * 1. Tilt head down so camera can see objects near the robot
 * 2. Capture fresh camera image for close-range detection
 * 3. Call /detect_object service for accurate pose estimation
 * 4. Call /manipulator_action service to execute pick
 * 5. Tilt head back to neutral
 *
 * Falls back to using the initial detection's cached pose if head control is unavailable.
 *
 * Input Ports:
 *   object_description - Natural language description of object to pick (e.g., "red cup")
 *   object_pose - Optional live-fallback pose captured before navigation
 *   box_threshold - Detection confidence threshold (default: 0.35)
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
      BT::InputPort<std::string>("object_description", "Natural language description of object to pick"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "object_pose", "Pose from the single live graph-miss detection"),
      BT::InputPort<double>("box_threshold", 0.35, "Detection confidence threshold (0-1)")
    };
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // Callbacks for camera data
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Non-blocking head tilt: returns true if goal was accepted
  using GoalHandle = rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;
  bool sendHeadTiltGoalAsync(
    double head_2_radians,
    double duration_sec,
    std::shared_future<GoalHandle::SharedPtr> & out_future);

  // Convert detection result to 3D pose
  geometry_msgs::msg::PoseStamped computeObjectPose(
    float center_x,
    float center_y,
    float depth_value,
    const std::string & frame_id);

  // Nav2's node for logging
  rclcpp::Node::SharedPtr node_;

  // Separate node for service calls and subscriptions
  rclcpp::Node::SharedPtr service_node_;

  // TF2 for coordinate transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Service clients
  rclcpp::Client<btgencobot_interfaces::srv::DetectObject>::SharedPtr detect_client_;
  rclcpp::Client<btgencobot_interfaces::srv::ManipulatorAction>::SharedPtr manipulator_client_;

  // Head action client (tilt camera down to see objects near robot)
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr head_client_;

  // Subscriptions for camera data
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Latest camera data
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;

  // Camera calibration
  bool has_camera_info_;
  double fx_, fy_, cx_, cy_;

  // Input parameters
  std::string object_description_;
  double box_threshold_;

  // State machine for the pick operation
  enum class PickState {
    TILTING_HEAD,
    WAITING_FOR_IMAGE,
    DETECTING,
    PICKING,
    RETURNING_HEAD,
    DONE
  };
  PickState state_;

  // Head tilt state
  bool head_tilt_sent_;
  bool head_tilt_done_;
  GoalHandle::SharedPtr head_goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> head_goal_future_;
  std::shared_future<GoalHandle::WrappedResult> head_result_future_;
  static constexpr double HEAD_TILT_DOWN = -1.047;  // -60° (max down) in radians
  static constexpr double HEAD_TILT_NEUTRAL = 0.0;
  static constexpr double HEAD_TILT_DURATION = 2.0;  // seconds for trajectory
  static constexpr double HEAD_TILT_TIMEOUT = 5.0;   // max wait for result
  static constexpr double POST_TILT_SETTLE_SEC = 1.0; // wait after tilt before accepting images
  static constexpr double MAX_MANIPULATION_DISTANCE = 0.95;
  rclcpp::Time head_settle_until_;                    // don't accept images before this time

  // Detection state
  btgencobot_interfaces::srv::DetectObject::Response::SharedPtr detection_response_;
  std::atomic<bool> detection_sent_;
  std::atomic<bool> detection_received_;

  // Pick state
  btgencobot_interfaces::srv::ManipulatorAction::Response::SharedPtr pick_response_;
  std::atomic<bool> pick_sent_;
  std::atomic<bool> pick_received_;

  // Computed object pose and dimensions
  geometry_msgs::msg::PoseStamped object_pose_;
  double object_height_;
  double object_width_;

  // Timing
  rclcpp::Time operation_start_time_;
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__PICK_OBJECT_HPP_
