#!/usr/bin/env python3
"""Dedicated saved-map construction runtime.

Starts simulation, SLAM Toolbox, Nav2, semantic segmentation and autonomous
frontier exploration. MIT-SPARK Hydra runs in the companion Docker service and
consumes the RGB-D/semantic streams produced here.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def resolve_environment_profile(context, profiles_file):
    environment_id = LaunchConfiguration('environment').perform(context)
    with open(profiles_file, encoding='utf-8') as stream:
        profiles = json.load(stream)

    profile = profiles.get(environment_id)
    if profile is None:
        supported = ', '.join(sorted(profiles))
        raise RuntimeError(
            f'Unknown environment "{environment_id}". Supported: {supported}'
        )

    spawn = profile['spawn']
    return [
        SetLaunchConfiguration('resolved_world', profile['world_file']),
        SetLaunchConfiguration('resolved_spawn_x', str(spawn['x'])),
        SetLaunchConfiguration('resolved_spawn_y', str(spawn['y'])),
        SetLaunchConfiguration('resolved_spawn_yaw', str(spawn['yaw'])),
        SetLaunchConfiguration(
            'resolved_slam_max_laser_range',
            str(profile['slam_max_laser_range']),
        ),
    ]


def generate_launch_description():
    pkg_tiago_gazebo = get_package_share_directory('tiago_gazebo')
    pkg_bringup = get_package_share_directory('bt_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    environment = LaunchConfiguration('environment')
    resolved_world = LaunchConfiguration('resolved_world')
    resolved_spawn_x = LaunchConfiguration('resolved_spawn_x')
    resolved_spawn_y = LaunchConfiguration('resolved_spawn_y')
    resolved_spawn_yaw = LaunchConfiguration('resolved_spawn_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    autonomous_exploration = LaunchConfiguration('autonomous_exploration')
    exploration_speed_multiplier = LaunchConfiguration(
        'exploration_speed_multiplier'
    )
    slam_params_file = LaunchConfiguration('slam_params_file')
    resolved_slam_max_laser_range = LaunchConfiguration(
        'resolved_slam_max_laser_range'
    )
    profiles_file = os.path.join(pkg_bringup, 'config', 'environments.json')

    configured_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        root_key='',
        param_rewrites={
            'max_laser_range': resolved_slam_max_laser_range,
        },
        convert_types=True,
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tiago_gazebo, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'is_public_sim': 'True',
            'world_name': resolved_world,
            'arm_type': 'tiago-arm',
            'end_effector': 'pal-gripper',
            'ft_sensor': 'schunk-ft',
            'camera_model': 'orbbec-astra',
            'laser_model': 'sick-571',
            'base_type': 'pmb2',
            'moveit': 'False',
            'tuck_arm': 'False',
            'spawn_x': resolved_spawn_x,
            'spawn_y': resolved_spawn_y,
            'spawn_yaw': resolved_spawn_yaw,
        }.items(),
    )

    environment_publisher = Node(
        package='bt_bringup',
        executable='environment_publisher.py',
        name='environment_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'environment_id': environment,
        }],
        output='screen',
    )

    slam_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': configured_slam_params,
                }.items(),
            )
        ],
    )

    nav2_launch = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ],
        condition=IfCondition(autonomous_exploration),
    )

    semantic_segmentation = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='vision_services',
                executable='semantic_segmentation',
                name='hydra_semantic_segmentation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'max_rate_hz': 2.0,
                    'device': 'auto',
                }],
                output='screen',
            )
        ],
    )

    navigation_posture = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='semantic_exploration',
                executable='navigation_posture',
                name='navigation_posture',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        ],
    )

    explorer = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('semantic_exploration'),
                        'launch',
                        'explore.launch.py',
                    )
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'speed_multiplier': exploration_speed_multiplier,
                    'require_navigation_posture': 'true',
                }.items(),
            )
        ],
        condition=IfCondition(autonomous_exploration),
    )

    rviz = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[
                    '-d',
                    os.path.join(
                        get_package_share_directory('tiago_2dnav'),
                        'config',
                        'rviz',
                        'navigation.rviz',
                    ),
                ],
                output='screen',
            )
        ],
        condition=IfCondition(use_rviz),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'certfile': '',
            'keyfile': '',
            # Hydra's raw mesh/DSG types live in the companion container.
            # Foxglove can still display its standard Marker outputs without
            # repeatedly trying to load those unavailable private schemas.
            'topic_whitelist': [
                r'^(?!/hydra/|/hydra_visualizer/(mesh|static_objects)$).*'
            ],
            'service_whitelist': [
                r'^(?!/hydra/|/hydra_visualizer/).*'
            ],
            'param_whitelist': [
                r'^(?!/hydra/|/hydra_visualizer/).*'
            ],
            'client_topic_whitelist': ['.*'],
            'use_sim_time': use_sim_time,
            'capabilities': [
                'clientPublish',
                'services',
                'parameters',
                'connectionGraph',
            ],
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'environment',
            default_value='house_pick_and_place',
            description='Environment profile (default: house_pick_and_place)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for continuous map and scan inspection',
        ),
        DeclareLaunchArgument(
            'autonomous_exploration',
            default_value='true',
            description='Use Nav2 frontier exploration instead of teleoperation',
        ),
        DeclareLaunchArgument(
            'exploration_speed_multiplier',
            default_value='1.0',
            description='Temporary Nav2 velocity multiplier during exploration',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_bringup,
                'config',
                'slam_toolbox_config.yaml',
            ),
            description='SLAM Toolbox configuration',
        ),
        OpaqueFunction(
            function=resolve_environment_profile,
            args=[profiles_file],
        ),
        environment_publisher,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        semantic_segmentation,
        navigation_posture,
        explorer,
        foxglove_bridge,
        rviz,
    ])
