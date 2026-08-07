#include "bt_nav2_plugins/pick_object.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include "cv_bridge/cv_bridge.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace bt_nav2_plugins
{

PickObject::PickObject(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  has_camera_info_(false),
  state_(PickState::WAITING_FOR_IMAGE),
  head_tilt_sent_(false),
  head_tilt_done_(false),
  detection_sent_(false),
  detection_received_(false),
  pick_sent_(false),
  pick_received_(false),
  object_height_(0.1),
  object_width_(0.05)
{
  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError("PickObject: 'node' not found in blackboard");
  }

  service_node_ = std::make_shared<rclcpp::Node>("pick_object_service_node");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  detect_client_ = service_node_->create_client<btgencobot_interfaces::srv::DetectObject>(
    "/detect_object");
  manipulator_client_ = service_node_->create_client<btgencobot_interfaces::srv::ManipulatorAction>(
    "/manipulator_action");

  head_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    service_node_, "/head_controller/follow_joint_trajectory");

  auto camera_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  image_sub_ = service_node_->create_subscription<sensor_msgs::msg::Image>(
    "/head_front_camera/image",
    camera_qos,
    std::bind(&PickObject::imageCallback, this, std::placeholders::_1));

  depth_sub_ = service_node_->create_subscription<sensor_msgs::msg::Image>(
    "/head_front_camera/depth_image",
    camera_qos,
    std::bind(&PickObject::depthCallback, this, std::placeholders::_1));

  camera_info_sub_ = service_node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/head_front_camera/camera_info",
    camera_qos,
    std::bind(&PickObject::cameraInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "PickObject BT node initialized");
}

bool PickObject::sendHeadTiltGoalAsync(
  double head_2_radians,
  double duration_sec,
  std::shared_future<GoalHandle::SharedPtr> & out_future)
{
  if (!head_client_->wait_for_action_server(1s)) {
    return false;
  }

  auto goal = control_msgs::action::FollowJointTrajectory::Goal();
  goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, head_2_radians};
  point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  goal.trajectory.points = {point};

  out_future = head_client_->async_send_goal(goal);
  return true;
}

BT::NodeStatus PickObject::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "PickObject: Starting pick operation");

  if (!getInput<std::string>("object_description", object_description_)) {
    RCLCPP_ERROR(node_->get_logger(), "PickObject: Missing required input 'object_description'");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<double>("box_threshold", box_threshold_)) {
    box_threshold_ = 0.35;
  }

  head_tilt_sent_ = false;
  head_tilt_done_ = false;
  head_goal_handle_.reset();
  head_goal_future_ = {};
  head_result_future_ = {};
  head_settle_until_ = node_->now();
  detection_sent_ = false;
  detection_received_ = false;
  detection_response_.reset();
  pick_sent_ = false;
  pick_received_ = false;
  pick_response_.reset();
  latest_image_.reset();
  latest_depth_.reset();
  has_camera_info_ = false;

  operation_start_time_ = node_->now();

  geometry_msgs::msg::PoseStamped fallback_pose;
  if (getInput("object_pose", fallback_pose)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "PickObject: using the live graph-miss pose [%.2f, %.2f, %.2f] "
      "after navigation; no second detection",
      fallback_pose.pose.position.x,
      fallback_pose.pose.position.y,
      fallback_pose.pose.position.z);
    object_pose_ = fallback_pose;
    object_height_ = 0.1;
    object_width_ = 0.05;
    state_ = PickState::PICKING;
  } else if (sendHeadTiltGoalAsync(HEAD_TILT_DOWN, HEAD_TILT_DURATION, head_goal_future_)) {
    RCLCPP_INFO(node_->get_logger(), "PickObject: Head tilt goal sent, waiting for acceptance...");
    state_ = PickState::TILTING_HEAD;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "PickObject: Head unavailable; doing one fresh local detection");
    state_ = PickState::WAITING_FOR_IMAGE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickObject::onRunning()
{
  rclcpp::spin_some(service_node_);

  switch (state_) {
    case PickState::TILTING_HEAD:
    {
      if (!head_tilt_sent_) {
        head_tilt_sent_ = true;
      }

      // Phase 1: Wait for goal acceptance
      if (!head_goal_handle_) {
        if (!head_goal_future_.valid() ||
            head_goal_future_.wait_for(0s) != std::future_status::ready) {
          return BT::NodeStatus::RUNNING;
        }
        head_goal_handle_ = head_goal_future_.get();
        if (!head_goal_handle_) {
          RCLCPP_WARN(node_->get_logger(), "PickObject: Head tilt goal rejected");
          head_tilt_done_ = false;
          state_ = PickState::WAITING_FOR_IMAGE;
          return BT::NodeStatus::RUNNING;
        }
        // Goal accepted — request async result notification
        RCLCPP_INFO(node_->get_logger(), "PickObject: Head tilt accepted, waiting for execution to finish...");
        head_result_future_ = head_client_->async_get_result(head_goal_handle_);
        return BT::NodeStatus::RUNNING;
      }

      // Phase 2: Wait for result (trajectory execution complete)
      if (head_result_future_.valid() &&
          head_result_future_.wait_for(0s) == std::future_status::ready) {
        auto wrapped = head_result_future_.get();
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(node_->get_logger(), "PickObject: Head tilt execution complete");
        } else {
          RCLCPP_WARN(node_->get_logger(), "PickObject: Head tilt result code %d",
            static_cast<int>(wrapped.code));
        }
        head_tilt_done_ = true;
        RCLCPP_INFO(
          node_->get_logger(),
          "PickObject: Head tilt done, waiting %.1fs for camera to settle...",
          POST_TILT_SETTLE_SEC);
        head_settle_until_ = node_->now() + rclcpp::Duration::from_seconds(POST_TILT_SETTLE_SEC);
        latest_image_.reset();
        latest_depth_.reset();
        operation_start_time_ = node_->now();
        state_ = PickState::WAITING_FOR_IMAGE;
        return BT::NodeStatus::RUNNING;
      }

      return BT::NodeStatus::RUNNING;
    }

    case PickState::WAITING_FOR_IMAGE:
    {
      // Let the camera settle after head tilt before accepting any frames
      if (head_tilt_done_ && node_->now() < head_settle_until_) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "PickObject: Camera settling (%.1fs remaining)...",
          (head_settle_until_ - node_->now()).seconds());
        latest_image_.reset();
        latest_depth_.reset();
        return BT::NodeStatus::RUNNING;
      }

      if (!latest_image_ || !latest_depth_) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "PickObject: Waiting for camera data (image: %s, depth: %s)...",
          latest_image_ ? "OK" : "waiting",
          latest_depth_ ? "OK" : "waiting");
        return BT::NodeStatus::RUNNING;
      }

      rclcpp::Time img_time(latest_image_->header.stamp);
      rclcpp::Time dep_time(latest_depth_->header.stamp);

      // Both image and depth must be captured after the head finished tilting
      if (img_time < operation_start_time_ || dep_time < operation_start_time_) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "PickObject: Discarding stale data (img: %.3f, dep: %.3f < start: %.3f)",
          img_time.seconds(), dep_time.seconds(), operation_start_time_.seconds());
        latest_image_.reset();
        latest_depth_.reset();
        return BT::NodeStatus::RUNNING;
      }

      // Verify image and depth timestamps are close to each other (within 50ms)
      double time_diff = std::abs((img_time - dep_time).seconds());
      if (time_diff > 0.05) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "PickObject: Image/depth desync (%.0fms apart), waiting for synchronized pair",
          time_diff * 1000.0);
        latest_image_.reset();
        latest_depth_.reset();
        return BT::NodeStatus::RUNNING;
      }

      if (!has_camera_info_) {
        RCLCPP_WARN_ONCE(
          node_->get_logger(),
          "PickObject: No camera_info received, using defaults");
        fx_ = 522.19;
        fy_ = 522.19;
        cx_ = 320.0;
        cy_ = 240.0;
      }

      state_ = PickState::DETECTING;
      RCLCPP_INFO(
        node_->get_logger(),
        "PickObject: Synchronized image+depth pair acquired (t=%s %.3f), starting detection...",
        latest_image_->header.frame_id.c_str(), img_time.seconds());
      return BT::NodeStatus::RUNNING;
    }

    case PickState::DETECTING:
    {
      if (!detection_sent_) {
        if (!detect_client_->wait_for_service(0s)) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "PickObject: Waiting for /detect_object service...");
          return BT::NodeStatus::RUNNING;
        }

        auto request = std::make_shared<btgencobot_interfaces::srv::DetectObject::Request>();
        request->image = *latest_image_;
        request->object_description = object_description_;
        request->box_threshold = static_cast<float>(box_threshold_);

        detect_client_->async_send_request(request,
          [this](rclcpp::Client<btgencobot_interfaces::srv::DetectObject>::SharedFuture future) {
            try { detection_response_ = future.get(); detection_received_ = true; }
            catch (const std::exception & e) {
              RCLCPP_ERROR(node_->get_logger(), "PickObject: Detection failed: %s", e.what());
              detection_received_ = true;
            }
          });
        detection_sent_ = true;
        return BT::NodeStatus::RUNNING;
      }

      if (!detection_received_) {
        return BT::NodeStatus::RUNNING;
      }

      if (!detection_response_ || !detection_response_->detected) {
        RCLCPP_ERROR(node_->get_logger(), "PickObject: Failed to detect '%s'", object_description_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      float depth = 0.3f;
      float rcx = detection_response_->center_x;
      float rcy = detection_response_->center_y;

      if (latest_depth_) {
        try {
          auto dp = cv_bridge::toCvCopy(latest_depth_, sensor_msgs::image_encodings::TYPE_32FC1);
          struct DS { float d; int x; int y; };
          std::vector<DS> samples;

          if (detection_response_->bbox.size() >= 4) {
            int x1 = std::max(0, std::min((int)detection_response_->bbox[0], dp->image.cols-1));
            int y1 = std::max(0, std::min((int)detection_response_->bbox[1], dp->image.rows-1));
            int x2 = std::max(0, std::min((int)detection_response_->bbox[2], dp->image.cols-1));
            int y2 = std::max(0, std::min((int)detection_response_->bbox[3], dp->image.rows-1));
            int mx = (x2-x1)*0.10, my = (y2-y1)*0.10;
            for (int y = y1+my; y <= y2-my; y+=2)
              for (int x = x1+mx; x <= x2-mx; x+=2) {
                float d = dp->image.at<float>(y,x);
                if (d > 10.0) {
                  d = d / 1000.0;
                }
                if (!std::isnan(d) && d > 0.1 && d < 10.0) samples.push_back({d,x,y});
              }
          }

          if (!samples.empty()) {
            std::sort(samples.begin(), samples.end(), [](const DS& a, const DS& b){ return a.d < b.d; });
            float md = samples.front().d;
            float tol = std::max(0.05f, md*0.15f);
            std::vector<DS> obj;
            for (auto& s : samples) if (s.d <= md+tol) { obj.push_back(s); }
            if (!obj.empty()) {
              // The nearest-depth cluster is reliable for range, but its pixel
              // centroid is easily skewed by occlusion and depth shadows. Keep
              // the detector's box center as the grasp ray to avoid lateral drift.
              std::sort(obj.begin(), obj.end(), [](const DS& a, const DS& b){ return a.d < b.d; });
              depth = obj[obj.size()/2].d;
            }
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(node_->get_logger(), "PickObject: Depth failed: %s", e.what());
        }
      }

      if (detection_response_->bbox.size() >= 4 && depth > 0) {
        float bw = detection_response_->bbox[2] - detection_response_->bbox[0];
        float bh = detection_response_->bbox[3] - detection_response_->bbox[1];
        object_width_ = (bw * depth) / fx_;
        object_height_ = (bh * depth) / fy_;

        // Registered depth at the detector center measures the visible front
        // surface. For compact objects, estimate the 3D center by advancing
        // half the smaller projected extent along the same camera ray. Using
        // the smaller extent avoids over-correcting elongated objects and,
        // unlike a fixed XY offset, remains valid as the camera/head moves.
        const float estimated_diameter = std::min(object_width_, object_height_);
        const float surface_depth = depth;
        depth += 0.5f * estimated_diameter;
        RCLCPP_INFO(
          node_->get_logger(),
          "PickObject: front surface %.3fm, projected extent %.3fm, center depth %.3fm",
          surface_depth, estimated_diameter, depth);
      }

      std::string cf = latest_image_->header.frame_id;
      if (cf.empty()) cf = "head_front_camera_depth_optical_frame";
      object_pose_ = computeObjectPose(rcx, rcy, depth, cf);

      RCLCPP_INFO(
        node_->get_logger(),
        "PickObject: DETECTION DIAGNOSTICS:"
        "\n  bbox: [%.0f, %.0f, %.0f, %.0f]"
        "\n  center: (%.1f, %.1f) -> refined: (%.1f, %.1f)"
        "\n  center depth: %.3fm  fx: %.2f  fy: %.2f  cx: %.2f  cy: %.2f"
        "\n  object_pose (map): (%.3f, %.3f, %.3f)"
        "\n  has_camera_info: %s",
        detection_response_->bbox.size() >= 4 ? detection_response_->bbox[0] : 0,
        detection_response_->bbox.size() >= 4 ? detection_response_->bbox[1] : 0,
        detection_response_->bbox.size() >= 4 ? detection_response_->bbox[2] : 0,
        detection_response_->bbox.size() >= 4 ? detection_response_->bbox[3] : 0,
        detection_response_->center_x, detection_response_->center_y,
        rcx, rcy, depth, fx_, fy_, cx_, cy_,
        object_pose_.pose.position.x, object_pose_.pose.position.y, object_pose_.pose.position.z,
        has_camera_info_ ? "YES" : "NO");

      // Also log the pose transformed to base_footprint for IK verification
      try {
        auto bp = tf_buffer_->transform(object_pose_, "base_footprint", tf2::durationFromSec(0.5));
        RCLCPP_INFO(
          node_->get_logger(),
          "PickObject: object_pose (base_footprint): (%.3f, %.3f, %.3f)",
          bp.pose.position.x, bp.pose.position.y, bp.pose.position.z);
      } catch (...) {}

      state_ = PickState::PICKING;
      return BT::NodeStatus::RUNNING;
    }

    case PickState::PICKING:
    {
      if (!pick_sent_) {
        try {
          auto base_pose = tf_buffer_->transform(
            object_pose_, "base_footprint", tf2::durationFromSec(0.5));
          const double planar_distance = std::hypot(
            base_pose.pose.position.x, base_pose.pose.position.y);
          if (planar_distance > MAX_MANIPULATION_DISTANCE ||
              base_pose.pose.position.z < 0.02 ||
              base_pose.pose.position.z > 1.50) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "PickObject: target is outside the manipulation envelope "
              "(distance %.2fm, height %.2fm)",
              planar_distance, base_pose.pose.position.z);
            return BT::NodeStatus::FAILURE;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "PickObject: cannot validate target reachability: %s", ex.what());
          return BT::NodeStatus::FAILURE;
        }
        if (!manipulator_client_->wait_for_service(0s)) {
          return BT::NodeStatus::RUNNING;
        }
        auto req = std::make_shared<btgencobot_interfaces::srv::ManipulatorAction::Request>();
        req->action_type = "pick";
        req->target_pose = object_pose_;
        req->object_height = static_cast<float>(object_height_);
        req->object_width = static_cast<float>(object_width_);
        req->object_depth = static_cast<float>(object_width_);
        manipulator_client_->async_send_request(req,
          [this](rclcpp::Client<btgencobot_interfaces::srv::ManipulatorAction>::SharedFuture future) {
            try { pick_response_ = future.get(); pick_received_ = true; }
            catch (const std::exception& e) {
              RCLCPP_ERROR(node_->get_logger(), "PickObject: Pick failed: %s", e.what());
              pick_received_ = true;
            }
          });
        pick_sent_ = true;
        return BT::NodeStatus::RUNNING;
      }

      if (!pick_received_) return BT::NodeStatus::RUNNING;
      if (!pick_response_) return BT::NodeStatus::FAILURE;

      if (pick_response_->success) {
        if (head_tilt_done_) {
          head_goal_handle_.reset();
          head_goal_future_ = {};
          head_result_future_ = {};
          operation_start_time_ = node_->now();
          if (sendHeadTiltGoalAsync(
                HEAD_TILT_NEUTRAL, HEAD_TILT_DURATION, head_goal_future_)) {
            state_ = PickState::RETURNING_HEAD;
            return BT::NodeStatus::RUNNING;
          }
        }
        state_ = PickState::DONE;
        return BT::NodeStatus::SUCCESS;
      }
      if (head_tilt_done_) {
        std::shared_future<GoalHandle::SharedPtr> ignored;
        sendHeadTiltGoalAsync(HEAD_TILT_NEUTRAL, HEAD_TILT_DURATION, ignored);
      }
      return BT::NodeStatus::FAILURE;
    }

    case PickState::RETURNING_HEAD:
    {
      if (!head_goal_handle_) {
        if (head_goal_future_.valid() &&
            head_goal_future_.wait_for(0s) == std::future_status::ready) {
          head_goal_handle_ = head_goal_future_.get();
          if (head_goal_handle_) {
            head_result_future_ = head_client_->async_get_result(head_goal_handle_);
          }
        }
      } else if (
        head_result_future_.valid() &&
        head_result_future_.wait_for(0s) == std::future_status::ready)
      {
        head_result_future_.get();
        state_ = PickState::DONE;
        return BT::NodeStatus::SUCCESS;
      }
      if ((node_->now() - operation_start_time_).seconds() > HEAD_TILT_TIMEOUT) {
        RCLCPP_WARN(node_->get_logger(), "PickObject: timed out restoring neutral head pose");
        state_ = PickState::DONE;
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
    }

    case PickState::DONE:
      return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void PickObject::onHalted()
{
  state_ = PickState::WAITING_FOR_IMAGE;
  head_tilt_sent_ = false; head_tilt_done_ = false;
  head_goal_handle_.reset(); head_goal_future_ = {}; head_result_future_ = {};
  head_settle_until_ = rclcpp::Time(0);
  detection_sent_ = false; detection_received_ = false; detection_response_.reset();
  pick_sent_ = false; pick_received_ = false; pick_response_.reset();
}

void PickObject::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_image_ = msg;
}

void PickObject::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_depth_ = msg;
}

void PickObject::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!has_camera_info_) {
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    has_camera_info_ = true;
  }
}

geometry_msgs::msg::PoseStamped PickObject::computeObjectPose(
  float cx, float cy, float d, const std::string & frame_id)
{
  double x = (cx - cx_) * d / fx_;
  double y = (cy - cy_) * d / fy_;

  geometry_msgs::msg::PoseStamped pc;
  pc.header.frame_id = frame_id;
  pc.header.stamp = node_->now();
  pc.pose.position.x = x; pc.pose.position.y = y; pc.pose.position.z = d;
  pc.pose.orientation.w = 1.0;

  try {
    auto pm = tf_buffer_->transform(pc, "map", tf2::durationFromSec(1.0));
    try {
      auto rt = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      double dy = pm.pose.position.y - rt.transform.translation.y;
      double dx = pm.pose.position.x - rt.transform.translation.x;
      tf2::Quaternion q; q.setRPY(0, 0, std::atan2(dy, dx));
      pm.pose.orientation = tf2::toMsg(q);
    } catch (...) {}
    return pm;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "TF failed: %s", ex.what());
    return pc;
  }
}

}  // namespace bt_nav2_plugins
