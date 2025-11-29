"""Launch file for the manipulator control service."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for manipulator service."""
    
    manipulator_service_node = Node(
        package='manipulator_control',
        executable='manipulator_service',
        name='manipulator_service',
        output='screen',
        parameters=[],
    )

    return LaunchDescription([
        manipulator_service_node,
    ])
