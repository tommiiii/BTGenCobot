#!/usr/bin/env python3
"""Normal task bringup using the saved map and persistent Hydra DSG.

The official Hydra adapter runs in the ``btgencobot-hydra`` companion
container and loads the saved DSG. This launch owns the simulation-side
localization, Nav2, BT generation/execution, perception, and visualization
processes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bt_bringup = get_package_share_directory("bt_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    environment = LaunchConfiguration("environment")
    inference_server_url = LaunchConfiguration("inference_server_url")
    bt_output_dir = LaunchConfiguration("bt_output_dir")
    vision_startup_delay = LaunchConfiguration("vision_startup_delay")
    generation_timeout = LaunchConfiguration("generation_timeout")
    use_rviz = LaunchConfiguration("use_rviz")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_bt_bringup,
                "launch",
                "robot_bt_localization.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "environment": environment,
            "inference_server_url": inference_server_url,
            "bt_output_dir": bt_output_dir,
            "vision_startup_delay": vision_startup_delay,
            "generation_timeout": generation_timeout,
            "use_rviz": use_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "environment",
                default_value="house_pick_and_place",
                description="Environment profile backed by a saved map",
            ),
            DeclareLaunchArgument(
                "inference_server_url",
                default_value="http://host.docker.internal:8080",
            ),
            DeclareLaunchArgument(
                "bt_output_dir",
                default_value="/workspace/generated_bts",
            ),
            DeclareLaunchArgument(
                "vision_startup_delay",
                default_value="70.0",
            ),
            DeclareLaunchArgument(
                "generation_timeout",
                default_value="120.0",
                description="Wall-clock timeout for local constrained generation",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            localization_launch,
        ]
    )
