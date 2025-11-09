# Mobile Manipulator Description

This package contains the URDF description, Gazebo simulation setup, and Nav2 configuration for a differential drive mobile base equipped with an OpenManipulator-X robotic arm.

## Package Contents

```
mobile_manipulator_description/
├── behavior_trees/          # Example behavior tree XML files
│   ├── example_pick_and_place.xml
│   └── simple_navigation.xml
├── config/                  # Configuration files
│   ├── arm_controller.yaml
│   └── nav2_params.yaml
├── launch/                  # Launch files
│   ├── bringup.launch.py
│   ├── gazebo.launch.py
│   └── nav2.launch.py
├── urdf/                    # Robot description files
│   ├── mobile_base.urdf.xacro
│   ├── mobile_base.gazebo.xacro
│   ├── ur5e_arm.urdf.xacro (contains OpenManipulator-X)
│   ├── ur5e_arm.gazebo.xacro
│   ├── mobile_manipulator.urdf.xacro
│   └── mobile_manipulator.ros2_control.xacro
├── worlds/                  # Gazebo world files
│   └── test_world.sdf
├── CMakeLists.txt
└── package.xml
```

## Robot Description

### Mobile Base
- **Type**: Differential drive
- **Dimensions**: 0.6m x 0.5m x 0.2m (L x W x H)
- **Wheel Configuration**: 2 drive wheels + 1 caster wheel
- **Sensors**:
  - Laser scanner (360° range, 10m max distance)
  - RGB camera (640x480 resolution)

### Manipulator
- **Type**: OpenManipulator-X (ROBOTIS)
- **License**: Open Source Hardware/Software
- **DOF**: 4 + gripper
- **Joints**:
  - joint1 (base rotation, ±π rad)
  - joint2 (shoulder, ±π/2 rad)
  - joint3 (elbow, -π/2 to π/2+0.3 rad)
  - joint4 (wrist, ±(π/2+0.3) rad)
  - gripper_left_joint (prismatic)
  - gripper_right_joint (prismatic)
- **Reference**: https://github.com/ROBOTIS-GIT/open_manipulator

## Building the Package

```bash
cd /path/to/BTGenCobot
colcon build --packages-select mobile_manipulator_description
source install/setup.bash
```

## Usage

### Launch Full Simulation with Nav2

Start the complete simulation environment including Gazebo, RViz, and Nav2:

```bash
ros2 launch mobile_manipulator_description bringup.launch.py
```

### Launch Only Gazebo Simulation

Start only the Gazebo simulation without Nav2:

```bash
ros2 launch mobile_manipulator_description gazebo.launch.py
```

Options:
- `use_sim_time:=true/false` - Use simulation time (default: true)
- `use_rviz:=true/false` - Launch RViz (default: true)
- `x_pose:=<value>` - Initial x position (default: 0.0)
- `y_pose:=<value>` - Initial y position (default: 0.0)

### Launch Only Nav2

Start only the Nav2 navigation stack (requires robot to be already spawned):

```bash
ros2 launch mobile_manipulator_description nav2.launch.py
```

Options:
- `use_sim_time:=true/false` - Use simulation time (default: true)
- `autostart:=true/false` - Auto-start navigation (default: true)
- `params_file:=<path>` - Custom Nav2 parameters file

## Testing the Setup

### 1. Set Initial Pose

In RViz, use the "2D Pose Estimate" tool to set the robot's initial position on the map.

### 2. Send Navigation Goal

Use the "2D Goal Pose" tool in RViz to send a navigation goal to the robot.

### 3. Control the Arm

Control the arm joints using joint trajectory controller:

```bash
# Example: Move arm to a specific configuration
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4],
    points: [
      { positions: [0.0, -0.5, 0.8, -0.3], time_from_start: { sec: 2 } }
    ]
  }
}"
```

### 4. Control the Gripper

Control the gripper using the gripper controller:

```bash
# Example: Open the gripper
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [gripper_left_joint, gripper_right_joint],
    points: [
      { positions: [0.019, -0.019], time_from_start: { sec: 1 } }
    ]
  }
}"

# Close the gripper
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [gripper_left_joint, gripper_right_joint],
    points: [
      { positions: [-0.005, 0.005], time_from_start: { sec: 1 } }
    ]
  }
}"
```

### 5. Test Behavior Trees

The package includes example behavior trees that can be used with the bt_navigator:

- `simple_navigation.xml` - Basic waypoint navigation
- `example_pick_and_place.xml` - Combined navigation and manipulation with gripper control

To use a custom behavior tree:

```bash
ros2 param set /bt_navigator default_nav_to_pose_bt_xml /path/to/behavior_tree.xml
```

## Topics

### Published Topics
- `/scan` - Laser scan data (sensor_msgs/LaserScan)
- `/camera/image_raw` - Camera images (sensor_msgs/Image)
- `/camera/camera_info` - Camera calibration (sensor_msgs/CameraInfo)
- `/odom` - Odometry data (nav_msgs/Odometry)
- `/joint_states` - Joint states (sensor_msgs/JointState)

### Subscribed Topics
- `/cmd_vel` - Velocity commands for mobile base (geometry_msgs/Twist)

## Services

### Navigation Services
- `/bt_navigator/navigate_to_pose` - Navigate to a pose
- `/bt_navigator/navigate_through_poses` - Navigate through multiple poses

### Costmap Services
- `/local_costmap/clear_entirely_local_costmap` - Clear local costmap
- `/global_costmap/clear_entirely_global_costmap` - Clear global costmap

## Action Servers

- `/arm_controller/follow_joint_trajectory` - Control arm joints
- `/gripper_controller/follow_joint_trajectory` - Control gripper
- `/navigate_to_pose` - Navigate to goal pose
- `/follow_path` - Follow a planned path

## Configuration

### Nav2 Parameters

The Nav2 configuration is located in `config/nav2_params.yaml`. Key parameters:

- **Robot radius**: 0.3m
- **Max velocity**: 0.26 m/s (linear), 1.0 rad/s (angular)
- **Controller**: DWB local planner
- **Planner**: NavFn global planner
- **Costmap resolution**: 0.05m

### Arm Controller Parameters

The arm controller configuration is in `config/arm_controller.yaml`:

- **Update rate**: 100 Hz
- **State publish rate**: 50 Hz
- **Control interface**: Position control
- **Controllers**: arm_controller (4 joints), gripper_controller (2 joints)

## Troubleshooting

### Robot falls through ground in Gazebo
- Check that the caster wheel is properly configured
- Verify physics parameters in the world file

### Nav2 doesn't start
- Ensure all required parameters are set in `nav2_params.yaml`
- Check that the robot_description is being published
- Verify that tf transforms are available

### Arm controller not responding
- Check that ros2_control is properly loaded
- Verify controller manager is running: `ros2 control list_controllers`
- Ensure joint limits are within bounds

## Dependencies

- ROS 2 Jazzy
- Gazebo
- Nav2
- ros2_control
- xacro
- robot_state_publisher

## License

Apache 2.0
