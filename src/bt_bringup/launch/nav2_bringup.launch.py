#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directories
    pkg_bt_bringup = get_package_share_directory('bt_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_bt_bringup, 'config', 'nav2_config.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    # Nav2 bringup launch - includes all Nav2 nodes (disable docking)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_docking': 'false',  # Use lowercase boolean
            'autostart': 'true'
        }.items()
    )

    # Activate navigation nodes after they've started (delayed to ensure nodes are configured)
    activate_nav_nodes = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/behavior_server', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/velocity_smoother', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/collision_monitor', 'activate'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/waypoint_follower', 'activate'],
                output='screen'
            ),
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add launch files
    ld.add_action(nav2_bringup_launch)
    
    # Add activation commands
    ld.add_action(activate_nav_nodes)

    return ld
