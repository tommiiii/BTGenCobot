#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directories
    pkg_mobile_manipulator = get_package_share_directory('mobile_manipulator')
    
    # Build resource paths for meshes - need parent directories for model:// URIs
    resource_paths = []
    
    try:
        pkg_om_description = get_package_share_directory('open_manipulator_description')
        resource_paths.append(pkg_om_description)
        # Add parent directory so Gazebo can find model://open_manipulator_description/...
        resource_paths.append(os.path.dirname(pkg_om_description))
    except:
        pass
    
    try:
        pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
        resource_paths.append(pkg_turtlebot4_description)
        resource_paths.append(os.path.dirname(pkg_turtlebot4_description))
    except:
        pass
    
    try:
        pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
        resource_paths.append(pkg_irobot_create_description)
        resource_paths.append(os.path.dirname(pkg_irobot_create_description))
    except:
        pass
    
    # Build GZ_SIM_RESOURCE_PATH
    gz_resource_path = ':'.join(resource_paths) if resource_paths else ''

    # Paths
    urdf_file = os.path.join(pkg_mobile_manipulator, 'urdf', 'mobile_manipulator.urdf.xacro')
    controller_config_file = os.path.join(pkg_mobile_manipulator, 'config', 'arm_controller.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    world_file_arg = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='/workspace/worlds/indoor_world.sdf',
        description='Full path to world file to load (options: indoor_world.sdf, basic_world.sdf, or custom path)'
    )

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'
    )

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of the robot'
    )

    # Start Gazebo Harmonic with empty world (no world file - uses default)
    env_vars = {}
    if gz_resource_path:
        env_vars['GZ_SIM_RESOURCE_PATH'] = gz_resource_path
        # Also set IGN_GAZEBO_RESOURCE_PATH for backward compatibility
        env_vars['IGN_GAZEBO_RESOURCE_PATH'] = gz_resource_path
    # Add gz_ros2_control plugin path
    env_vars['GZ_SIM_SYSTEM_PLUGIN_PATH'] = '/opt/ros/jazzy/lib'
    
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file_arg],
        output='screen',
        additional_env=env_vars
    )

    # Robot State Publisher
    robot_description_content = Command(['xacro', ' ', urdf_file])
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # Spawn robot in Gazebo - with delay to ensure Gazebo is ready
    spawn_robot_cmd = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'mobile_manipulator',
                    '-topic', 'robot_description',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    # Bridge for clock
    bridge_clock_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Bridge for sensor topics
    bridge_lidar_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/mobile_manipulator/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen',
        remappings=[
            ('/model/mobile_manipulator/odometry', '/odom')
        ]
    )

    # RViz with config file
    rviz_config_file = os.path.join(pkg_mobile_manipulator, 'rviz', 'mobile_manipulator.rviz')
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawn controllers with proper timing to avoid race conditions
    # Use TimerAction to delay controller spawning until robot is fully loaded
    spawn_joint_state_broadcaster_cmd = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Spawn arm controller after joint_state_broadcaster
    spawn_arm_controller_cmd = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Spawn gripper controller after arm controller
    spawn_gripper_controller_cmd = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)

    # Add nodes to launch description
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(bridge_clock_cmd)
    ld.add_action(bridge_lidar_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(spawn_joint_state_broadcaster_cmd)
    ld.add_action(spawn_arm_controller_cmd)
    ld.add_action(spawn_gripper_controller_cmd)

    return ld
