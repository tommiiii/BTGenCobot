#!/usr/bin/env python3
"""
Frontend launch (SLAM-based full stack) for an environment-driven TIAGo runtime.

Starts the TIAGo simulation, SLAM Toolbox, Nav2, BT text interface,
GroundingDINO perception, manipulator control, Foxglove bridge and the
environment publisher needed by the frontend. The active environment is
selected through the `environment` launch argument and resolved against
`config/environments.json`.
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    return [
        SetLaunchConfiguration('resolved_world', profile['world_file']),
        SetLaunchConfiguration(
            'resolved_slam_max_laser_range',
            str(profile['slam_max_laser_range']),
        ),
    ]


def generate_launch_description():
    pkg_tiago_gazebo = get_package_share_directory('tiago_gazebo')
    pkg_bt_bringup = get_package_share_directory('bt_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    environment = LaunchConfiguration('environment')
    resolved_world = LaunchConfiguration('resolved_world')
    inference_server_url = LaunchConfiguration('inference_server_url')
    bt_output_dir = LaunchConfiguration('bt_output_dir')
    slam_params_file = LaunchConfiguration('slam_params_file')
    resolved_slam_max_laser_range = LaunchConfiguration(
        'resolved_slam_max_laser_range'
    )
    profiles_file = os.path.join(pkg_bt_bringup, 'config', 'environments.json')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_environment_cmd = DeclareLaunchArgument(
        'environment',
        default_value='aws_small_house',
        description='Environment profile: aws_small_house | aws_hospital | house_pick_and_place'
    )

    declare_inference_server_url_cmd = DeclareLaunchArgument(
        'inference_server_url',
        default_value='http://host.docker.internal:8080',
        description='URL of the BT generation inference server'
    )

    declare_bt_output_dir_cmd = DeclareLaunchArgument(
        'bt_output_dir',
        default_value='/workspace/generated_bts',
        description='Directory to save generated BehaviorTrees'
    )

    declare_slam_params_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            pkg_bt_bringup, 'config', 'slam_toolbox_config.yaml'
        ),
        description='SLAM Toolbox configuration'
    )

    # Launch Gazebo with TIAGo robot
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
            'moveit': 'True'
        }.items()
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

    # SLAM Toolbox starts after Gazebo sensor bridges come up.
    slam_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bt_bringup, 'launch', 'slam.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params_file,
                }.items()
            )
        ]
    )

    nav2_launch = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bt_bringup, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time
                }.items()
            )
        ]
    )

    # BT text interface subscribes to /btgen_nl_command (frontend commands).
    bt_interface_node = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='bt_text_interface',
                executable='bt_interface_node',
                name='bt_interface_node',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'inference_server_url': inference_server_url,
                    'bt_output_dir': bt_output_dir,
                    'generation_timeout': 30.0,
                    'execution_timeout': 300.0,
                }],
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    grounding_dino_service = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='vision_services',
                executable='grounding_dino_service',
                name='grounding_dino_service',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'use_mock': False,
                    'model_name': 'IDEA-Research/grounding-dino-tiny',
                    'device': 'auto',
                    'publish_debug_images': True,
                }],
                output='screen',
            )
        ]
    )

    manipulator_service = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='manipulator_control',
                executable='manipulator_service',
                name='manipulator_service',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
            )
        ]
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
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'client_topic_whitelist': ['.*'],
            'use_sim_time': use_sim_time,
            'capabilities': ['clientPublish', 'services', 'parameters', 'connectionGraph'],
        }],
        output='screen'
    )

    rviz2_node = TimerAction(
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
                        'config', 'rviz', 'navigation.rviz'
                    )
                ],
                output='screen'
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_environment_cmd)
    ld.add_action(declare_inference_server_url_cmd)
    ld.add_action(declare_bt_output_dir_cmd)
    ld.add_action(declare_slam_params_cmd)

    ld.add_action(OpaqueFunction(
        function=resolve_environment_profile,
        args=[profiles_file],
    ))

    ld.add_action(environment_publisher)
    ld.add_action(gazebo_launch)
    ld.add_action(slam_launch)
    ld.add_action(nav2_launch)
    ld.add_action(bt_interface_node)
    ld.add_action(grounding_dino_service)
    ld.add_action(manipulator_service)
    ld.add_action(foxglove_bridge)
    ld.add_action(rviz2_node)
    return ld
