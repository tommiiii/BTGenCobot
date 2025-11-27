#include "bt_nav2_plugins/detect_object.hpp"

#include <stdexcept>
#include <chrono>
#include <cmath>
#include "cv_bridge/cv_bridge.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace bt_nav2_plugins
{

DetectObject::DetectObject(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  has_camera_info_(false),
  service_call_sent_(false),
  box_threshold_(0.35)
{
  // Create ROS 2 node
  node_ = std::make_shared<rclcpp::Node>("detect_object_bt_node");

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(), "DetectObject BT node initialized");

  // Create service client for object detection
  detect_client_ = node_->create_client<btgencobot_interfaces::srv::DetectObject>("/detect_object");

  // Subscribe to camera topics
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera",
    10,
    std::bind(&DetectObject::imageCallback, this, std::placeholders::_1));

  depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/depth",
    10,
    std::bind(&DetectObject::depthCallback, this, std::placeholders::_1));

  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info",
    10,
    std::bind(&DetectObject::cameraInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Subscribed to /camera, /camera/depth, /camera/camera_info");
}

BT::NodeStatus DetectObject::onStart()
{
  // Get input ports
  if (!getInput<std::string>("object_description", object_description_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input 'object_description'");
    return BT::NodeStatus::FAILURE;
  }

  // Get detection threshold (with default)
  if (!getInput<double>("box_threshold", box_threshold_)) {
    box_threshold_ = 0.35;  // default
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Starting object detection for: '%s' (threshold: %.2f)",
    object_description_.c_str(),
    box_threshold_);

  service_call_sent_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectObject::onRunning()
{
  // Spin node to process callbacks
  rclcpp::spin_some(node_);

  // Check if we have image data
  if (!latest_image_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      1000,
      "Waiting for camera image...");
    return BT::NodeStatus::RUNNING;
  }

  // Warn if no camera info (but continue with defaults)
  if (!has_camera_info_) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "No camera_info received. Using default calibration values. "
      "3D pose estimation may be inaccurate!");
    // Set default values (typical for RGB cameras)
    fx_ = 554.3;
    fy_ = 554.3;
    cx_ = 320.5;
    cy_ = 240.5;
  }

  // If service call not sent yet, send it
  if (!service_call_sent_) {
    // Check if service is available
    if (!detect_client_->wait_for_service(0s)) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "Waiting for /detect_object service...");
      return BT::NodeStatus::RUNNING;
    }

    // Create service request
    auto request = std::make_shared<btgencobot_interfaces::srv::DetectObject::Request>();
    request->image = *latest_image_;
    request->object_description = object_description_;
    request->box_threshold = static_cast<float>(box_threshold_);

    RCLCPP_INFO(node_->get_logger(), "Sending detection request to service...");

    // Send async request
    future_result_ = detect_client_->async_send_request(request).future.share();
    service_call_sent_ = true;

    return BT::NodeStatus::RUNNING;
  }

  // Check if service response is ready
  if (future_result_.wait_for(0s) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }

  // Get service response
  auto response = future_result_.get();

  // Check if detection was successful
  if (!response->detected) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Object '%s' not detected: %s",
      object_description_.c_str(),
      response->error_message.c_str());

    setOutput("detected", false);
    setOutput("confidence", 0.0);
    return BT::NodeStatus::FAILURE;
  }

  // Get depth at detected point
  float depth = 1.5;  // Default depth if no depth camera (1.5m is reasonable for navigation)
  if (latest_depth_) {
    try {
      cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(
        latest_depth_,
        sensor_msgs::image_encodings::TYPE_32FC1);

      int x = static_cast<int>(response->center_x);
      int y = static_cast<int>(response->center_y);

      // Ensure coordinates are within bounds
      if (x >= 0 && x < depth_ptr->image.cols && y >= 0 && y < depth_ptr->image.rows) {
        depth = depth_ptr->image.at<float>(y, x);

        RCLCPP_INFO(
          node_->get_logger(),
          "Raw depth at pixel (%d, %d): %.2fm", x, y, depth);

        // Check for invalid depth
        if (std::isnan(depth) || depth <= 0.0) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Invalid depth at detection point, using estimated depth of 1.5m");
          depth = 1.5;
        }
        // Clamp depth to reasonable navigation range (0.5m to 5.0m)
        else if (depth < 0.5) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Depth too close (%.2fm), clamping to 0.5m", depth);
          depth = 0.5;
        }
        else if (depth > 5.0) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Depth too far (%.2fm), clamping to 5.0m", depth);
          depth = 5.0;
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to get depth: %s. Using estimated depth of 1.5m",
        e.what());
      depth = 1.5;
    }
  } else {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "No depth data available. Using estimated depth of 1.5m");
  }

  // Convert pixel coordinates to 3D pose
  // Use the frame_id from the latest image, but validate it exists in TF
  std::string camera_frame = latest_image_->header.frame_id;

  // Try to transform a test point to verify the frame exists
  geometry_msgs::msg::TransformStamped test_transform;
  bool frame_exists = false;
  try {
    test_transform = tf_buffer_->lookupTransform(
      "map",
      camera_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    frame_exists = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Camera frame '%s' not found in TF tree: %s. Trying alternatives...",
      camera_frame.c_str(),
      ex.what());
  }

  // If frame doesn't exist, try common camera frame names
  if (!frame_exists) {
    std::vector<std::string> frame_candidates = {
      "camera_rgb_optical_frame",
      "base_footprint/camera_rgb_optical_frame",
      "camera_link",
      "base_footprint/camera_link",
      "camera_rgb_frame",
      "base_footprint/camera_rgb_frame"
    };

    for (const auto & candidate : frame_candidates) {
      try {
        test_transform = tf_buffer_->lookupTransform(
          "map",
          candidate,
          tf2::TimePointZero,
          tf2::durationFromSec(0.1));
        camera_frame = candidate;
        frame_exists = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "Using camera frame: %s",
          camera_frame.c_str());
        break;
      } catch (const tf2::TransformException & ex) {
        // Try next candidate
        continue;
      }
    }

    if (!frame_exists) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not find any valid camera frame in TF tree!");
      setOutput("detected", false);
      setOutput("confidence", 0.0);
      return BT::NodeStatus::FAILURE;
    }
  }

  geometry_msgs::msg::PoseStamped target_pose = pixelToPose(
    response->center_x,
    response->center_y,
    depth,
    camera_frame);

  RCLCPP_INFO(
    node_->get_logger(),
    "Detected '%s' at pixel (%.1f, %.1f) with depth %.2fm -> pose (%.2f, %.2f, %.2f) in %s frame (confidence: %.2f)",
    response->phrase.c_str(),
    response->center_x,
    response->center_y,
    depth,
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z,
    target_pose.header.frame_id.c_str(),
    response->confidence);

  // Set output ports
  setOutput("target_pose", target_pose);
  setOutput("detected", true);
  setOutput("confidence", static_cast<double>(response->confidence));

  return BT::NodeStatus::SUCCESS;
}

void DetectObject::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "DetectObject node halted");
  service_call_sent_ = false;
}

void DetectObject::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_image_ = msg;
}

void DetectObject::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_depth_ = msg;
}

void DetectObject::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!has_camera_info_) {
    // Extract camera intrinsics from K matrix
    // K = [fx  0  cx]
    //     [ 0 fy  cy]
    //     [ 0  0   1]
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    has_camera_info_ = true;

    RCLCPP_INFO(
      node_->get_logger(),
      "Camera calibration received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
      fx_, fy_, cx_, cy_);
  }
}

geometry_msgs::msg::PoseStamped DetectObject::pixelToPose(
  float center_x,
  float center_y,
  float depth_value,
  const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;

  // Pinhole camera model: convert pixel to 3D point in camera frame
  // In optical frame: X = right, Y = down, Z = forward (depth)
  // X = (u - cx) * Z / fx
  // Y = (v - cy) * Z / fy
  // Z = depth

  // Apply approach offset in camera frame (before transformation)
  // Stop 30cm before the object by reducing depth
  double approach_offset = 0.3;  // meters
  double target_depth = depth_value;

  if (target_depth > 0.5) {  // Only apply offset if object is reasonably far
    target_depth = depth_value - approach_offset;
  }

  // Pinhole camera model: convert pixel to 3D point in camera frame
  double x = (center_x - cx_) * target_depth / fx_;
  double y = (center_y - cy_) * target_depth / fy_;

  // Create pose in camera optical frame
  // ROS optical frame convention: +X right, +Y down, +Z forward
  pose.header.frame_id = frame_id;
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;       // Right
  pose.pose.position.y = y;       // Down
  pose.pose.position.z = target_depth;   // Forward (depth with offset applied)
  pose.pose.orientation.w = 1.0;  // No rotation

  RCLCPP_INFO(
    node_->get_logger(),
    "Camera frame pose: (%.2f, %.2f, %.2f) in %s (original depth: %.2fm, target depth: %.2fm)",
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z,
    frame_id.c_str(),
    depth_value,
    target_depth);

  try {
    // First, transform the ACTUAL object position (without approach offset) to get the true object location
    geometry_msgs::msg::PoseStamped object_pose_camera;
    object_pose_camera.header.frame_id = frame_id;
    object_pose_camera.header.stamp = node_->now();
    object_pose_camera.pose.position.x = (center_x - cx_) * depth_value / fx_;
    object_pose_camera.pose.position.y = (center_y - cy_) * depth_value / fy_;
    object_pose_camera.pose.position.z = depth_value;  // Original depth, not reduced
    object_pose_camera.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped object_pose_map = tf_buffer_->transform(
      object_pose_camera,
      "map",
      tf2::durationFromSec(1.0));

    // Now transform the approach goal (with offset)
    geometry_msgs::msg::PoseStamped pose_map = tf_buffer_->transform(
      pose,
      "map",
      tf2::durationFromSec(1.0));

    // For 2D navigation, set z=0 (ground level)
    pose_map.pose.position.z = 0.0;
    object_pose_map.pose.position.z = 0.0;

    // Calculate orientation: robot at goal position should face toward the object
    double dx = object_pose_map.pose.position.x - pose_map.pose.position.x;
    double dy = object_pose_map.pose.position.y - pose_map.pose.position.y;
    double yaw = std::atan2(dy, dx);

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose_map.pose.orientation.x = q.x();
    pose_map.pose.orientation.y = q.y();
    pose_map.pose.orientation.z = q.z();
    pose_map.pose.orientation.w = q.w();

    RCLCPP_INFO(
      node_->get_logger(),
      "Navigation goal: (%.2f, %.2f, %.2f) in %s with yaw %.2f rad toward object at (%.2f, %.2f)",
      pose_map.pose.position.x,
      pose_map.pose.position.y,
      pose_map.pose.position.z,
      pose_map.header.frame_id.c_str(),
      yaw,
      object_pose_map.pose.position.x,
      object_pose_map.pose.position.y);

    return pose_map;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Could not transform to map frame: %s. Returning camera frame pose.",
      ex.what());
    return pose;
  }
}

}  // namespace bt_nav2_plugins
