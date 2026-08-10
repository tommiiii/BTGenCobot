#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Nav2 bringup launch - local copy with docking_server removed
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bt_bringup, 'launch', 'nav2_navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # The lifecycle manager's autostart is one-shot. Heavy simulation startup
    # can make it race controller/sensor discovery and leave all servers
    # inactive, so verify and retry through the manager's supported API.
    nav2_lifecycle_guard = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='bt_bringup',
                executable='nav2_lifecycle_guard.py',
                name='nav2_lifecycle_guard',
                parameters=[{
                    'initial_delay': 20.0,
                    'retry_period': 10.0,
                    'max_attempts': 12,
                }],
                output='screen',
            )
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add launch files
    ld.add_action(nav2_bringup_launch)
    ld.add_action(nav2_lifecycle_guard)

    return ld
