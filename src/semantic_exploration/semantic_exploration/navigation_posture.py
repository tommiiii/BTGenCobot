"""Fold TIAGo into its official ``home`` posture before autonomous driving.

The simulation's stock ``tuck_arm.py`` goes through play_motion2, which waits
for MoveIt's semantic robot description. Mapping intentionally does not start
MoveIt, so that helper never moves the arm. This node sends the same official
home trajectory directly to the already-running ros2_control controllers and
keeps a transient-local readiness publisher alive for the explorer.
"""

from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [f"arm_{index}_joint" for index in range(1, 8)]
ARM_WAYPOINTS = [
    [0.20, 0.35, -0.20, 1.94, -1.57, 1.37, -1.58],
    [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, -1.58],
    [0.50, -1.34, -0.48, 1.94, -1.49, 1.37, 0.00],
]
TORSO_WAYPOINTS = [[0.25], [0.18], [0.15]]
WAYPOINT_TIMES = [2.0, 6.0, 8.0]


class NavigationPosture(Node):
    def __init__(self) -> None:
        super().__init__("navigation_posture")
        self.declare_parameter("controller_wait_sec", 45.0)
        self.declare_parameter("result_wait_sec", 45.0)
        self.declare_parameter("max_attempts", 3)

        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._ready_pub = self.create_publisher(
            Bool, "/mapping/navigation_posture_ready", ready_qos
        )
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._torso_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/torso_controller/follow_joint_trajectory",
        )
        self._ready = False
        self.create_timer(1.0, self._publish_ready)
        self._publish_ready()

    def _publish_ready(self) -> None:
        self._ready_pub.publish(Bool(data=self._ready))

    @staticmethod
    def _goal(
        joint_names: list[str],
        positions: list[list[float]],
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        for waypoint, seconds in zip(
            positions, WAYPOINT_TIMES, strict=True
        ):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = Duration(seconds=seconds).to_msg()
            goal.trajectory.points.append(point)
        goal.goal_time_tolerance = Duration(seconds=3.0).to_msg()
        return goal

    def _wait_result(
        self,
        client: ActionClient,
        goal: FollowJointTrajectory.Goal,
        label: str,
    ) -> bool:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=float(self.get_parameter("result_wait_sec").value),
        )
        if not send_future.done():
            self.get_logger().error(f"{label} posture goal timed out")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label} posture goal was rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=float(self.get_parameter("result_wait_sec").value),
        )
        if not result_future.done():
            self.get_logger().error(f"{label} posture execution timed out")
            handle.cancel_goal_async()
            return False
        result = result_future.result()
        error_code = int(result.result.error_code)
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"{label} posture failed with controller error {error_code}: "
                f"{result.result.error_string}"
            )
            return False
        return True

    def move_home(self) -> bool:
        wait_sec = float(self.get_parameter("controller_wait_sec").value)
        self.get_logger().info("Waiting for arm and torso controllers")
        if not self._arm_client.wait_for_server(timeout_sec=wait_sec):
            self.get_logger().error("Arm trajectory controller is unavailable")
            return False
        if not self._torso_client.wait_for_server(timeout_sec=wait_sec):
            self.get_logger().error("Torso trajectory controller is unavailable")
            return False

        # Raise the torso first so the arm can fold without sweeping low.
        if not self._wait_result(
            self._torso_client,
            self._goal(["torso_lift_joint"], TORSO_WAYPOINTS),
            "Torso",
        ):
            return False
        if not self._wait_result(
            self._arm_client,
            self._goal(ARM_JOINTS, ARM_WAYPOINTS),
            "Arm",
        ):
            return False

        self._ready = True
        self._publish_ready()
        self.get_logger().info(
            "Navigation posture reached; autonomous exploration may start"
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationPosture()
    attempts = int(node.get_parameter("max_attempts").value)
    succeeded = False
    for attempt in range(1, attempts + 1):
        node.get_logger().info(
            f"Moving to navigation posture (attempt {attempt}/{attempts})"
        )
        if node.move_home():
            succeeded = True
            break
        if attempt < attempts:
            time.sleep(2.0)

    if succeeded:
        rclpy.spin(node)
    else:
        node.get_logger().fatal(
            "Navigation posture failed; exploration remains interlocked"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
