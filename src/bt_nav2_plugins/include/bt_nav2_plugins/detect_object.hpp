#ifndef BT_NAV2_PLUGINS__DETECT_OBJECT_HPP_
#define BT_NAV2_PLUGINS__DETECT_OBJECT_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "btgencobot_interfaces/srv/detect_object.hpp"

namespace bt_nav2_plugins
{

/**
 * @brief BT node to detect objects using Grounding DINO (via ROS2 service)
 *
 * This node uses Grounding DINO for fast, accurate open-vocabulary object detection.
 * It calls a ROS2 service provided by the grounding_dino_service node.
 *
 * Input Ports:
 *   object_description - Description of object to find (e.g., "red cup", "table")
 *   box_threshold - Confidence threshold for detection (default: 0.35)
 *
 * Output Ports:
 *   target_pose - Detected object pose in map frame
 *   detected - Boolean indicating if object was found
 *   confidence - Detection confidence score (0-1)
 */
class DetectObject : public BT::StatefulActionNode
{
public:
  DetectObject(
    const std::string & name,
    const BT::NodeConfiguration & config);

  ~DetectObject() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("object_description", "Description of object to detect"),
      BT::InputPort<double>("box_threshold", 0.35, "Detection confidence threshold (0-1)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Detected object pose"),
      BT::OutputPort<bool>("detected", "Whether object was detected"),
      BT::OutputPort<double>("confidence", "Detection confidence score")
    };
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  // Callbacks
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Convert pixel coordinates to 3D pose using camera intrinsics
  geometry_msgs::msg::PoseStamped pixelToPose(
    float center_x,
    float center_y,
    float depth_value,
    const std::string & frame_id);

  // Node and TF
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS2 service client
  rclcpp::Client<btgencobot_interfaces::srv::DetectObject>::SharedPtr detect_client_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Latest data
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;

  // Camera calibration (from camera_info topic)
  bool has_camera_info_;
  double fx_;  // Focal length X
  double fy_;  // Focal length Y
  double cx_;  // Principal point X
  double cy_;  // Principal point Y

  // Detection state
  std::string object_description_;
  bool service_call_sent_;
  double box_threshold_;
  rclcpp::Client<btgencobot_interfaces::srv::DetectObject>::SharedFuture future_result_;
};

}  // namespace bt_nav2_plugins

#endif  // BT_NAV2_PLUGINS__DETECT_OBJECT_HPP_
