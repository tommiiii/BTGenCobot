#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <btgencobot_interfaces/srv/manipulator_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class ManipulatorService : public rclcpp::Node
{
public:
  ManipulatorService(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("manipulator_service", options)
  {
    // Create a reentrant callback group so service callbacks and action clients don't deadlock
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Setup service server
    service_ = this->create_service<btgencobot_interfaces::srv::ManipulatorAction>(
      "/manipulator_action",
      std::bind(&ManipulatorService::handle_request, this, _1, _2),
      rmw_qos_profile_services_default,
      callback_group_);
      
    // Setup Gripper Action Client
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, "/gripper_controller/follow_joint_trajectory", callback_group_);
      
    RCLCPP_INFO(this->get_logger(), "Manipulator service (MoveIt 2) ready.");
  }

  // Need to initialize MoveIt interfaces after the node is added to an executor
  void initialize_moveit(std::shared_ptr<rclcpp::Node> shared_this)
  {
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_this, "arm_torso");
        // TIAGo MoveIt config usually has higher velocity scaling
    move_group_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_arm_->setMaxAccelerationScalingFactor(1.0);
    // Increase planning time slightly
    move_group_arm_->setPlanningTime(5.0);
  }

private:
  rclcpp::Service<btgencobot_interfaces::srv::ManipulatorAction>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_action_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  
  // Gripper settings
  const double GRIPPER_OPEN = 0.044;
  const double GRIPPER_CLOSED = 0.0;

  void handle_request(
    const std::shared_ptr<btgencobot_interfaces::srv::ManipulatorAction::Request> request,
    std::shared_ptr<btgencobot_interfaces::srv::ManipulatorAction::Response> response)
  {
    std::string action = request->action_type;
    
    // Convert to lowercase
    std::transform(action.begin(), action.end(), action.begin(),
      [](unsigned char c){ return std::tolower(c); });
      
    RCLCPP_INFO(this->get_logger(), "Received request for action: %s", action.c_str());

    bool success = false;
    try {
      if (action == "pick") {
        success = execute_pick(request->target_pose);
      } else if (action == "place") {
        success = execute_place(request->target_pose);
      } else {
        response->success = false;
        response->error_message = "Unknown action type: " + action;
        return;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Action failed: %s", e.what());
      response->success = false;
      response->error_message = e.what();
      return;
    }

    response->success = success;
    if (!success) {
      response->error_message = action + " operation failed";
    }
  }

  bool execute_pick(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    if (!move_group_arm_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized");
      return false;
    }

    // 1. Open gripper
    RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    if (!move_gripper(GRIPPER_OPEN, false)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
      return false;
    }

    // Calculate poses
    const double finger_length = 0.15;
    geometry_msgs::msg::PoseStamped grasp_pose = target_pose;
    // We command arm_tool_link, which is 'finger_length' higher than the gripper tip.
    grasp_pose.pose.position.z = target_pose.pose.position.z - 0.02 + finger_length;
    // Orientation for arm_tool_link to make gripper point DOWN:
    // X_arm=UP, Z_arm=FORWARD => q=[0, 0.707, 0, 0.707]
    grasp_pose.pose.orientation.x = 0.0;
    grasp_pose.pose.orientation.y = 0.70710678;
    grasp_pose.pose.orientation.z = 0.0;
    grasp_pose.pose.orientation.w = 0.70710678;

    // Clear octomap to prevent goal state collision with the object itself
    auto clear_client = this->create_client<std_srvs::srv::Empty>("/clear_octomap");
    if (clear_client->wait_for_service(std::chrono::seconds(1))) {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      clear_client->async_send_request(req);
      rclcpp::sleep_for(std::chrono::milliseconds(500)); // wait for octomap to clear
    }

    // 2. Move directly to grasp pose using free space planning
    RCLCPP_INFO(this->get_logger(), "Planning path to grasp pose...");
    move_group_arm_->setPoseTarget(grasp_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to grasp pose");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Moving to grasp pose...");
    if (move_group_arm_->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to grasp pose");
      return false;
    }

    // 4. Close gripper
    RCLCPP_INFO(this->get_logger(), "Closing gripper...");
    if (!move_gripper(GRIPPER_CLOSED, true)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to close gripper");
      return false;
    }
    // Wait an extra second for grasp to stabilize
    rclcpp::sleep_for(1s);

    // 5. Move back up (Cartesian path)
    RCLCPP_INFO(this->get_logger(), "Lifting object...");
    geometry_msgs::msg::PoseStamped above_pose = grasp_pose;
    above_pose.pose.position.z += 0.15;
    std::vector<geometry_msgs::msg::Pose> up_waypoints;
    up_waypoints.push_back(above_pose.pose);
    
    moveit_msgs::msg::RobotTrajectory up_trajectory;
    double fraction = move_group_arm_->computeCartesianPath(up_waypoints, 0.01, 0.0, up_trajectory);
    
    if (fraction >= 0.9) {
      move_group_arm_->execute(up_trajectory);
    } else {
      RCLCPP_WARN(this->get_logger(), "Cartesian lift failed, using free space planning...");
      move_group_arm_->setPoseTarget(above_pose);
      move_group_arm_->move();
    }

    return true;
  }

  bool execute_place(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    if (!move_group_arm_) return false;

    const double finger_length = 0.15;
    geometry_msgs::msg::PoseStamped place_pose = target_pose;
    place_pose.pose.position.z += finger_length;
    place_pose.pose.orientation.x = 0.0;
    place_pose.pose.orientation.y = 0.70710678;
    place_pose.pose.orientation.z = 0.0;
    place_pose.pose.orientation.w = 0.70710678;

    // 1. Move directly to place pose using free space planning
    RCLCPP_INFO(this->get_logger(), "Planning path to place pose...");
    move_group_arm_->setPoseTarget(place_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to place pose");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Moving to place pose...");
    if (move_group_arm_->execute(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to move to place pose");
      return false;
    }

    // 2. Open gripper to place
    RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    if (!move_gripper(GRIPPER_OPEN, false)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
      return false;
    }

    // 3. Move up (Cartesian lift)
    RCLCPP_INFO(this->get_logger(), "Lifting after place...");
    geometry_msgs::msg::PoseStamped above_pose = place_pose;
    above_pose.pose.position.z += 0.15;
    
    std::vector<geometry_msgs::msg::Pose> up_waypoints;
    up_waypoints.push_back(above_pose.pose);
    
    moveit_msgs::msg::RobotTrajectory up_trajectory;
    double fraction = move_group_arm_->computeCartesianPath(up_waypoints, 0.01, 0.0, up_trajectory);
    
    if (fraction >= 0.9) {
      move_group_arm_->execute(up_trajectory);
    } else {
      move_group_arm_->setPoseTarget(above_pose);
      move_group_arm_->move();
    }

    return true;
  }

  bool move_gripper(double position, bool force_grasp)
  {
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
      return false;
    }

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    goal_msg.trajectory.points.push_back(point);

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    
    auto promise = std::make_shared<std::promise<bool>>();
    auto future = promise->get_future();

    send_goal_options.result_callback = 
      [promise](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          promise->set_value(true);
        } else {
          promise->set_value(false);
        }
      };

    auto goal_handle_future = gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
    
    // Wait for the action to complete
    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      return future.get();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Gripper action timed out");
      return false;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // MoveIt requires NodeOptions with use_intra_process_comms(false) and a MultiThreadedExecutor
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<ManipulatorService>(node_options);
  
  // We need to pass the shared_ptr to MoveGroupInterface
  node->initialize_moveit(node);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
