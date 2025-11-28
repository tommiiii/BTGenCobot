#include "bt_nav2_plugins/detect_object.hpp"

#include <algorithm>
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

  // Sample depth using segmentation mask for accurate object depth
  float depth = 1.5;  // Default if no depth available

  if (latest_depth_) {
    try {
      cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(
        latest_depth_,
        sensor_msgs::image_encodings::TYPE_32FC1);

      std::vector<float> depth_samples;

      // Check if we have a segmentation mask
      if (!response->mask.empty() && response->mask_height > 0 && response->mask_width > 0) {
        // Sample depth only from masked pixels (the actual object)
        RCLCPP_INFO(
          node_->get_logger(),
          "Using segmentation mask (%dx%d) for depth sampling",
          response->mask_width, response->mask_height);

        // Ensure mask dimensions match depth image
        if (response->mask_height != depth_ptr->image.rows || 
            response->mask_width != depth_ptr->image.cols) {
          RCLCPP_WARN(
            node_->get_logger(),
            "Mask size (%dx%d) doesn't match depth image (%dx%d), falling back to bbox",
            response->mask_width, response->mask_height,
            depth_ptr->image.cols, depth_ptr->image.rows);
        } else {
          // Sample all masked pixels
          int sample_stride = 2;  // Sample every 2nd pixel for efficiency
          for (int y = 0; y < depth_ptr->image.rows; y += sample_stride) {
            for (int x = 0; x < depth_ptr->image.cols; x += sample_stride) {
              int mask_idx = y * response->mask_width + x;
              if (mask_idx < static_cast<int>(response->mask.size()) && response->mask[mask_idx] > 0) {
                float d = depth_ptr->image.at<float>(y, x);
                if (!std::isnan(d) && d > 0.1 && d < 10.0) {
                  depth_samples.push_back(d);
                }
              }
            }
          }
        }
      }

      // Fallback to bounding box if mask sampling failed
      if (depth_samples.empty() && response->bbox.size() >= 4) {
        RCLCPP_INFO(
          node_->get_logger(),
          "Mask sampling failed or unavailable, using bounding box center");

        int cx = static_cast<int>(response->center_x);
        int cy = static_cast<int>(response->center_y);
        int radius = 5;  // Sample 5-pixel radius around center

        for (int dy = -radius; dy <= radius; dy++) {
          for (int dx = -radius; dx <= radius; dx++) {
            int x = cx + dx;
            int y = cy + dy;
            if (x >= 0 && x < depth_ptr->image.cols && y >= 0 && y < depth_ptr->image.rows) {
              float d = depth_ptr->image.at<float>(y, x);
              if (!std::isnan(d) && d > 0.1 && d < 10.0) {
                depth_samples.push_back(d);
              }
            }
          }
        }
      }

      // Use median of masked pixels (more robust than minimum for mask-based sampling)
      if (!depth_samples.empty()) {
        std::sort(depth_samples.begin(), depth_samples.end());
        
        // Use median for mask-based sampling (already filtered to object only)
        depth = depth_samples[depth_samples.size() / 2];

        RCLCPP_INFO(
          node_->get_logger(),
          "Sampled %zu depth points from mask, median: %.2fm (range: %.2f - %.2fm)",
          depth_samples.size(), depth, depth_samples.front(), depth_samples.back());
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "No valid depth samples available, using default 1.5m");
      }

    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to sample depth: %s. Using default 1.5m", e.what());
    }
  } else {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "No depth data available. Using default 1.5m");
  }

  // Use camera optical frame for pose estimation
  std::string camera_frame = "camera_rgb_optical_frame";

  geometry_msgs::msg::PoseStamped target_pose = pixelToPose(
    response->center_x,
    response->center_y,
    depth,
    camera_frame);

  RCLCPP_INFO(
    node_->get_logger(),
    "Detected '%s' at pixel (%.1f, %.1f), depth %.2fm -> goal (%.2f, %.2f) in map (conf: %.2f)",
    response->phrase.c_str(),
    response->center_x,
    response->center_y,
    depth,
    target_pose.pose.position.x,
    target_pose.pose.position.y,
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
  // Pinhole camera model: pixel to 3D point in optical frame
  // Optical frame: +X=right, +Y=down, +Z=forward
  double x = (center_x - cx_) * depth_value / fx_;
  double y = (center_y - cy_) * depth_value / fy_;
  double z = depth_value;

  // Create pose in camera optical frame
  geometry_msgs::msg::PoseStamped pose_camera;
  pose_camera.header.frame_id = frame_id;
  pose_camera.header.stamp = node_->now();
  pose_camera.pose.position.x = x;
  pose_camera.pose.position.y = y;
  pose_camera.pose.position.z = z;
  pose_camera.pose.orientation.w = 1.0;

  RCLCPP_INFO(
    node_->get_logger(),
    "Object in camera frame: (%.2f, %.2f, %.2f) at depth %.2fm",
    x, y, z, depth_value);

  // Transform to map frame
  try {
    geometry_msgs::msg::PoseStamped pose_map = tf_buffer_->transform(
      pose_camera,
      "map",
      tf2::durationFromSec(1.0));

    // Project to ground plane (z=0 for 2D navigation)
    pose_map.pose.position.z = 0.0;

    // Calculate orientation: point toward object from approach position
    // Apply 0.5m approach offset along the direction to the object
    double obj_x = pose_map.pose.position.x;
    double obj_y = pose_map.pose.position.y;
    double distance = std::sqrt(obj_x * obj_x + obj_y * obj_y);
    
    if (distance > 0.5) {
      // Move goal 0.5m back from object
      double scale = (distance - 0.5) / distance;
      pose_map.pose.position.x = obj_x * scale;
      pose_map.pose.position.y = obj_y * scale;
    }

    // Face toward object
    double yaw = std::atan2(obj_y, obj_x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose_map.pose.orientation.x = q.x();
    pose_map.pose.orientation.y = q.y();
    pose_map.pose.orientation.z = q.z();
    pose_map.pose.orientation.w = q.w();

    RCLCPP_INFO(
      node_->get_logger(),
      "Navigation goal: (%.2f, %.2f) in map, facing object at (%.2f, %.2f)",
      pose_map.pose.position.x, pose_map.pose.position.y, obj_x, obj_y);

    return pose_map;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "TF transform failed: %s", ex.what());
    return pose_camera;
  }
}

}  // namespace bt_nav2_plugins
