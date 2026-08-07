#!/usr/bin/env python3
"""Official MIT-SPARK Hydra with the BTGenCobot semantic adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adapter_share = get_package_share_directory("hydra_semantic_navigation")
    hydra_ros_share = get_package_share_directory("hydra_ros")
    visualizer_share = get_package_share_directory("hydra_visualizer")

    use_sim_time = LaunchConfiguration("use_sim_time")
    persistence_path = LaunchConfiguration("persistence_path")
    assume_mapping_complete = LaunchConfiguration("assume_mapping_complete")

    input_config = os.path.join(adapter_share, "config", "tiago_input.yaml")
    hydra_config = os.path.join(adapter_share, "config", "tiago_hydra.yaml")
    labelspace = os.path.join(
        adapter_share,
        "config",
        "tiago_label_space.yaml",
    )
    sink_config = os.path.join(hydra_ros_share, "config", "sinks")

    hydra = Node(
        package="hydra_ros",
        executable="hydra_ros_node",
        name="hydra",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            (
                "hydra/input/camera/rgb/image_raw",
                "/hydra/input/camera/rgb/image_raw",
            ),
            (
                "hydra/input/camera/rgb/camera_info",
                "/head_front_camera/camera_info",
            ),
            (
                "hydra/input/camera/depth_registered/image_rect",
                "/hydra/input/camera/depth_registered/image_rect",
            ),
            (
                "hydra/input/camera/semantic/image_raw",
                "/hydra/input/camera/semantic/image_raw",
            ),
            (
                "/hydra/backend/dsg",
                "/hydra/backend/live_dsg",
            ),
        ],
        arguments=[
            "--config-utilities-file",
            input_config,
            "--config-utilities-file",
            hydra_config,
            "--config-utilities-file",
            labelspace,
            "--config-utilities-file",
            os.path.join(sink_config, "mesh_segmenter_sinks.yaml")
            + "@frontend/objects",
            "--config-utilities-file",
            os.path.join(sink_config, "gvd_places_sinks.yaml")
            + "@frontend/freespace_places",
            "--config-utilities-file",
            os.path.join(sink_config, "active_window_sinks.yaml")
            + "@active_window",
            "--config-utilities-yaml",
            "{robot_id: 0, odom_frame: map, robot_frame: base_footprint, "
            "map_frame: map, enable_lcd: false, "
            "log_path: /data/house_pick_and_place/live}",
        ],
    )

    adapter = Node(
        package="hydra_semantic_navigation",
        executable="scene_graph_adapter",
        name="hydra_semantic_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "persistence_path": persistence_path,
                "assume_mapping_complete": assume_mapping_complete,
                "dsg_topic": "/hydra/backend/live_dsg",
                "published_dsg_topic": "/hydra/backend/dsg",
            }
        ],
    )

    # Keep the Foxglove marker streams tied to the Hydra process lifecycle.
    # This launch is used during mapping and normal saved-graph operation, so
    # neither mode depends on a manually started visualizer session.
    visualizer = Node(
        package="hydra_visualizer",
        executable="hydra_visualizer_node",
        name="hydra_visualizer",
        output="screen",
        arguments=[
            "--config-utilities-file",
            os.path.join(
                visualizer_share,
                "config",
                "visualizer_config.yaml",
            ),
            "--config-utilities-file",
            os.path.join(
                visualizer_share,
                "config",
                "visualizer_plugins.yaml",
            ),
            "--config-utilities-file",
            os.path.join(
                visualizer_share,
                "config",
                "external_plugins.yaml",
            ),
            "--config-utilities-yaml",
            "{graph: {type: GraphFromRos, frame_id: map}}",
        ],
        remappings=[
            ("/hydra_visualizer/dsg", "/hydra/backend/dsg"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "persistence_path",
                default_value="/data/house_pick_and_place/backend_dsg.json",
            ),
            DeclareLaunchArgument(
                "assume_mapping_complete",
                default_value="false",
            ),
            hydra,
            adapter,
            visualizer,
        ]
    )
