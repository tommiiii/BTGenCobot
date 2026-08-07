#ifndef BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_
#define BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_

#include <string>
#include <memory>
#include <atomic>
#include <future>

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
 * @brief BT node to place an object with the manipulator
 *
 * The robot should already be positioned near the place location (via prior navigation).
 * Hydra/Nav2 must bring the robot to a manipulation standoff. This node aims
 * the camera, refines the support pose, and rejects poses outside the checked
 * arm envelope.
 *
 * Input Ports:
 *   place_description - Natural language description of where to place (e.g., "table", "box")
 *   box_threshold - Detection confidence threshold (default: 0.35)
 *
 * The node:
 * 1. Captures current camera image
 * 2. Calls /detect_object service to detect the place surface/location
 * 3. Computes place pose from detection result
 * 4. Calls /manipulator_action service to execute place
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
      BT::InputPort<std::string>("place_description", "Natural language description of where to place"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "place_pose", "Support pose resolved from the Hydra object bounds"),
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
  using GoalHandle =
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>;
  bool sendHeadTiltGoalAsync(
    double head_2_radians,
    double duration_sec,
    std::shared_future<GoalHandle::SharedPtr> & out_future);

  // Convert detection result to 3D pose
  geometry_msgs::msg::PoseStamped computePlacePose(
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
  std::string place_description_;
  double box_threshold_;

  // State machine for the place operation
  enum class PlaceState {
    AIMING_CAMERA,
    WAITING_FOR_IMAGE,
    DETECTING,
    PLACING,
    DONE
  };
  PlaceState state_;
  GoalHandle::SharedPtr head_goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> head_goal_future_;
  std::shared_future<GoalHandle::WrappedResult> head_result_future_;
  rclcpp::Time camera_ready_after_;
  static constexpr double HEAD_TILT_PLACE = -0.65;
  static constexpr double HEAD_TILT_DURATION = 1.5;
  static constexpr double HEAD_TILT_TIMEOUT = 5.0;
  static constexpr double CAMERA_SETTLE_SEC = 0.6;
  static constexpr double MAX_MANIPULATION_DISTANCE = 0.70;

  // Detection state
  btgencobot_interfaces::srv::DetectObject::Response::SharedPtr detection_response_;
  std::atomic<bool> detection_sent_;
  std::atomic<bool> detection_received_;

  // Place state
  btgencobot_interfaces::srv::ManipulatorAction::Response::SharedPtr place_response_;
  std::atomic<bool> place_sent_;
  std::atomic<bool> place_received_;

  // Computed place pose
  geometry_msgs::msg::PoseStamped place_pose_;

  // Timing
  rclcpp::Time operation_start_time_;
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__PLACE_OBJECT_HPP_
