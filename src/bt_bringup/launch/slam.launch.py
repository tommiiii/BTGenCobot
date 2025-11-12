#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('bt_bringup'),
            'config',
            'slam_toolbox_config.yaml'
        ),
        description='Full path to the SLAM Toolbox parameters file'
    )

    # SLAM Toolbox node - async mapping mode
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Lifecycle manager for SLAM
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['slam_toolbox']}
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)

    # Add nodes
    ld.add_action(slam_toolbox_node)
    ld.add_action(lifecycle_manager_slam)

    return ld
