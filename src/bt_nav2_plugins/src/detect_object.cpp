#include "bt_nav2_plugins/detect_object.hpp"

#include <stdexcept>
#include <chrono>
#include "cv_bridge/cv_bridge.hpp"

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
    future_result_ = detect_client_->async_send_request(request);
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
  float depth = 1.0;  // Default depth if no depth camera
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

        // Check for invalid depth
        if (std::isnan(depth) || depth <= 0.0) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Invalid depth at detection point, using estimated depth of 1.0m");
          depth = 1.0;
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to get depth: %s. Using estimated depth of 1.0m",
        e.what());
      depth = 1.0;
    }
  } else {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "No depth data available. Using estimated depth of 1.0m");
  }

  // Convert pixel coordinates to 3D pose
  geometry_msgs::msg::PoseStamped target_pose = pixelToPose(
    response->center_x,
    response->center_y,
    depth);

  RCLCPP_INFO(
    node_->get_logger(),
    "✓ Detected '%s' at (%.2f, %.2f, %.2f) in %s frame with %.2f confidence",
    response->phrase.c_str(),
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
  float depth_value)
{
  geometry_msgs::msg::PoseStamped pose;

  // Pinhole camera model: convert pixel to 3D point in camera frame
  // X = (x - cx) * Z / fx
  // Y = (y - cy) * Z / fy
  // Z = depth

  double z = depth_value;
  double x_3d = (center_x - cx_) * z / fx_;
  double y_3d = (center_y - cy_) * z / fy_;

  // Create pose in camera optical frame
  // Optical frame convention: X right, Y down, Z forward
  pose.header.frame_id = "camera_optical_frame";
  pose.header.stamp = node_->now();
  pose.pose.position.x = z;       // Z forward
  pose.pose.position.y = -x_3d;   // X right (negated)
  pose.pose.position.z = -y_3d;   // Y down (negated)
  pose.pose.orientation.w = 1.0;  // No rotation

  try {
    // Transform to map frame
    geometry_msgs::msg::PoseStamped pose_map = tf_buffer_->transform(
      pose,
      "map",
      tf2::durationFromSec(1.0));

    RCLCPP_DEBUG(
      node_->get_logger(),
      "Transformed pose from %s to %s",
      pose.header.frame_id.c_str(),
      pose_map.header.frame_id.c_str());

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
