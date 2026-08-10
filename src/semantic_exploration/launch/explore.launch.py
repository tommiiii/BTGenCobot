from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    speed_multiplier = LaunchConfiguration("speed_multiplier")
    require_navigation_posture = LaunchConfiguration(
        "require_navigation_posture"
    )
    head_sweep = LaunchConfiguration("head_sweep")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("speed_multiplier", default_value="2.0"),
            DeclareLaunchArgument(
                "require_navigation_posture", default_value="false"
            ),
            DeclareLaunchArgument("head_sweep", default_value="false"),
            Node(
                package="semantic_exploration",
                executable="frontier_explorer",
                name="frontier_explorer",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "speed_multiplier": speed_multiplier,
                        "require_navigation_posture": (
                            require_navigation_posture
                        ),
                        "head_sweep": head_sweep,
                    }
                ],
                output="screen",
            ),
        ]
    )
