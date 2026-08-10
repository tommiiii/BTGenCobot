"""Nav2 frontier explorer used while official Hydra builds its DSG."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros

from btgencobot_interfaces.msg import ExplorationStatus

from .frontiers import cell_to_world, extract_frontiers, select_coverage_goal


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("initial_delay_sec", 8.0)
        self.declare_parameter("decision_period_sec", 2.0)
        self.declare_parameter("min_cluster_cells", 6)
        self.declare_parameter("clearance_m", 0.45)
        self.declare_parameter("blacklist_radius_m", 0.75)
        self.declare_parameter("visited_radius_m", 0.65)
        self.declare_parameter("distance_weight", 12.0)
        self.declare_parameter("no_frontier_cycles_to_finish", 6)
        self.declare_parameter("minimum_known_cells", 500)
        self.declare_parameter("semantic_coverage", True)
        self.declare_parameter("semantic_coverage_radius_m", 2.0)
        self.declare_parameter("semantic_coverage_stride_m", 0.5)
        self.declare_parameter("semantic_coverage_min_cells", 80)
        self.declare_parameter("speed_multiplier", 2.0)
        self.declare_parameter("head_sweep", True)
        self.declare_parameter("head_sweep_step_sec", 2.0)
        self.declare_parameter("observation_settle_sec", 2.0)
        self.declare_parameter("navigation_timeout_sec", 90.0)
        self.declare_parameter("no_motion_timeout_sec", 15.0)
        self.declare_parameter("no_motion_translation_m", 0.05)
        self.declare_parameter("no_motion_rotation_rad", 0.12)
        self.declare_parameter("max_consecutive_no_motion", 3)
        self.declare_parameter("require_navigation_posture", False)

        self._map: OccupancyGrid | None = None
        self._active_goal = None
        self._goal_pending = False
        self._goal_started_at = 0.0
        self._goal_cancel_requested = False
        self._active_goal_pose = PoseStamped()
        self._visited: list[tuple[float, float]] = []
        self._blacklisted: list[tuple[float, float]] = []
        self._no_frontier_cycles = 0
        self._started_at = time.monotonic()
        self._completed = False
        self._canceled = False
        self._speed_profile_applied = False
        self._last_frontier_count = 0
        self._explored_ratio = 0.0
        self._navigation_posture_ready = False
        self._initial_observation_done = False
        self._observation_until = 0.0
        self._goal_kind = "frontier"
        self._last_progress_pose: tuple[float, float, float] | None = None
        self._last_progress_at = 0.0
        self._consecutive_no_motion = 0

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._status_pub = self.create_publisher(
            ExplorationStatus, "/mapping/exploration_status", latched
        )
        self._complete_pub = self.create_publisher(
            Bool, "/mapping/exploration_complete", latched
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            latched,
        )
        self.create_subscription(
            Bool,
            "/mapping/navigation_posture_ready",
            self._navigation_posture_callback,
            latched,
        )
        self.create_service(Trigger, "/mapping/cancel_exploration", self._cancel)

        self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._head_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
        )
        self._controller_params = AsyncParameterClient(self, "/controller_server")
        self._smoother_params = AsyncParameterClient(self, "/velocity_smoother")
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._complete_pub.publish(Bool(data=False))
        self.create_timer(
            float(self.get_parameter("decision_period_sec").value),
            self._tick,
        )
        self._publish_status(
            ExplorationStatus.WAITING_FOR_MAP,
            "Waiting for SLAM and Nav2",
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _navigation_posture_callback(self, msg: Bool) -> None:
        self._navigation_posture_ready = bool(msg.data)

    def _robot_pose(self) -> tuple[float, float, float]:
        transform = self._tf_buffer.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("robot_frame").value),
            Time(),
            timeout=Duration(seconds=0.25),
        )
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw,
        )

    def _robot_position(self) -> tuple[float, float]:
        x, y, _ = self._robot_pose()
        return x, y

    def _goal_has_made_progress(self) -> bool:
        """Update the live motion watchdog and report recent base progress."""
        try:
            pose = self._robot_pose()
        except Exception:
            # TF outages have their own Nav2 failure path and must not be
            # mistaken for a physically stationary base.
            self._last_progress_at = time.monotonic()
            return True
        if self._last_progress_pose is None:
            self._last_progress_pose = pose
            self._last_progress_at = time.monotonic()
            return True
        translation = math.hypot(
            pose[0] - self._last_progress_pose[0],
            pose[1] - self._last_progress_pose[1],
        )
        rotation = abs(
            math.atan2(
                math.sin(pose[2] - self._last_progress_pose[2]),
                math.cos(pose[2] - self._last_progress_pose[2]),
            )
        )
        if (
            translation
            >= float(self.get_parameter("no_motion_translation_m").value)
            or rotation
            >= float(self.get_parameter("no_motion_rotation_rad").value)
        ):
            self._last_progress_pose = pose
            self._last_progress_at = time.monotonic()
            return True
        return (
            time.monotonic() - self._last_progress_at
            < float(self.get_parameter("no_motion_timeout_sec").value)
        )

    @staticmethod
    def _near(
        point: tuple[float, float],
        collection: list[tuple[float, float]],
        radius: float,
    ) -> bool:
        return any(math.dist(point, other) <= radius for other in collection)

    def _apply_speed_profile(self, enabled: bool) -> None:
        multiplier = min(
            2.5,
            max(1.0, float(self.get_parameter("speed_multiplier").value)),
        )
        if enabled == self._speed_profile_applied:
            return
        linear = 0.4 * multiplier if enabled else 0.4
        angular = 1.5 * min(multiplier, 1.6) if enabled else 1.5
        accel = 2.5 * min(multiplier, 1.6) if enabled else 2.5
        self._controller_params.set_parameters(
            [
                Parameter("FollowPath.max_vel_x", value=linear),
                Parameter("FollowPath.max_speed_xy", value=linear),
                Parameter("FollowPath.acc_lim_x", value=accel),
                Parameter("FollowPath.decel_lim_x", value=-accel),
            ]
        )
        self._smoother_params.set_parameters(
            [
                Parameter(
                    "max_velocity",
                    type_=Parameter.Type.DOUBLE_ARRAY,
                    value=[linear, 0.0, angular],
                ),
                Parameter(
                    "min_velocity",
                    type_=Parameter.Type.DOUBLE_ARRAY,
                    value=[-linear, 0.0, -angular],
                ),
                Parameter(
                    "max_accel",
                    type_=Parameter.Type.DOUBLE_ARRAY,
                    value=[accel, 0.0, 3.2],
                ),
                Parameter(
                    "max_decel",
                    type_=Parameter.Type.DOUBLE_ARRAY,
                    value=[-accel, 0.0, -3.2],
                ),
            ]
        )
        self._speed_profile_applied = enabled
        self.get_logger().info(
            f'Exploration motion profile {"enabled" if enabled else "restored"} '
            f"(linear limit {linear:.2f} m/s)"
        )

    def _publish_status(self, state: int, detail: str) -> None:
        msg = ExplorationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.state = state
        msg.explored_ratio = float(self._explored_ratio)
        msg.reachable_frontiers = int(self._last_frontier_count)
        msg.visited_frontiers = len(self._visited)
        msg.blacklisted_frontiers = len(self._blacklisted)
        msg.current_goal = self._active_goal_pose
        msg.detail = detail
        self._status_pub.publish(msg)

    def _finish(self, detail: str) -> None:
        if self._completed:
            return
        self._completed = True
        self._apply_speed_profile(False)
        self._complete_pub.publish(Bool(data=True))
        self._publish_status(ExplorationStatus.COMPLETED, detail)
        self.get_logger().info(detail)

    def _cancel(self, _request, response):
        self._canceled = True
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()
        self._apply_speed_profile(False)
        self._publish_status(
            ExplorationStatus.CANCELED,
            "Exploration canceled by request",
        )
        response.success = True
        response.message = "exploration cancellation requested"
        return response

    def _tick(self) -> None:
        if self._completed or self._canceled:
            return
        if time.monotonic() < self._observation_until:
            remaining = self._observation_until - time.monotonic()
            self._publish_status(
                ExplorationStatus.EXPLORING,
                f"Holding a stable semantic observation viewpoint ({remaining:.0f}s)",
            )
            return
        if self._observation_until > 0.0:
            self._observation_until = 0.0
        if (
            time.monotonic() - self._started_at
            < float(self.get_parameter("initial_delay_sec").value)
        ):
            return
        if (
            bool(self.get_parameter("require_navigation_posture").value)
            and not self._navigation_posture_ready
        ):
            self._publish_status(
                ExplorationStatus.WAITING_FOR_MAP,
                "Waiting for the safe navigation posture",
            )
            return
        if self._map is None:
            self._publish_status(
                ExplorationStatus.WAITING_FOR_MAP,
                "Waiting for an occupancy grid",
            )
            return
        if self._active_goal is not None or self._goal_pending:
            elapsed = time.monotonic() - self._goal_started_at
            timeout = float(
                self.get_parameter("navigation_timeout_sec").value
            )
            if (
                self._active_goal is not None
                and not self._goal_has_made_progress()
                and not self._goal_cancel_requested
            ):
                self._consecutive_no_motion += 1
                self._goal_cancel_requested = True
                self.get_logger().error(
                    "Robot pose did not move while Nav2 was active; canceling "
                    f"goal ({self._consecutive_no_motion}/"
                    f"{int(self.get_parameter('max_consecutive_no_motion').value)})"
                )
                self._active_goal.cancel_goal_async()
                if self._consecutive_no_motion >= int(
                    self.get_parameter("max_consecutive_no_motion").value
                ):
                    self._finish(
                        "Exploration stopped safely: the simulated base remained "
                        "immobile across multiple navigation goals"
                    )
                return
            if (
                self._active_goal is not None
                and elapsed >= timeout
                and not self._goal_cancel_requested
            ):
                self.get_logger().warning(
                    f"Frontier navigation exceeded {timeout:.0f}s; canceling "
                    "and blacklisting it"
                )
                self._goal_cancel_requested = True
                self._active_goal.cancel_goal_async()
            self._publish_status(
                ExplorationStatus.EXPLORING,
                (
                    f"Navigating to {self._goal_kind} ({elapsed:.0f}s/"
                    f"{timeout:.0f}s)"
                ),
            )
            return
        if not self._nav_client.server_is_ready():
            self._publish_status(
                ExplorationStatus.WAITING_FOR_MAP,
                "Waiting for Nav2",
            )
            return

        msg = self._map
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height,
            msg.info.width,
        )
        known_cells = int(np.count_nonzero(grid >= 0))
        self._explored_ratio = known_cells / max(1, grid.size)
        clearance_cells = max(
            1,
            int(
                math.ceil(
                    float(self.get_parameter("clearance_m").value)
                    / msg.info.resolution
                )
            ),
        )
        frontiers = extract_frontiers(
            grid,
            clearance_cells=clearance_cells,
            min_cluster_cells=int(
                self.get_parameter("min_cluster_cells").value
            ),
        )

        try:
            robot = self._robot_position()
        except Exception as exc:
            self._publish_status(
                ExplorationStatus.WAITING_FOR_MAP,
                f"Waiting for map-to-robot transform: {exc}",
            )
            return

        if not self._initial_observation_done:
            self._initial_observation_done = True
            self._visited.append(robot)
            self._observation_until = (
                time.monotonic() + self._send_head_sweep()
            )
            self._publish_status(
                ExplorationStatus.EXPLORING,
                "Observing the initial viewpoint before driving",
            )
            return

        candidates = []
        for frontier in frontiers:
            point = cell_to_world(
                frontier.goal_cell,
                msg.info.resolution,
                msg.info.origin.position.x,
                msg.info.origin.position.y,
            )
            if self._near(
                point,
                self._blacklisted,
                float(self.get_parameter("blacklist_radius_m").value),
            ) or self._near(
                point,
                self._visited,
                float(self.get_parameter("visited_radius_m").value),
            ):
                continue
            distance = math.dist(robot, point)
            score = frontier.information_gain - (
                float(self.get_parameter("distance_weight").value) * distance
            )
            candidates.append((score, distance, point))

        self._last_frontier_count = len(candidates)
        if not candidates:
            coverage_target = self._semantic_coverage_target(
                grid,
                msg,
                robot,
                clearance_cells,
            )
            if coverage_target is not None:
                self._no_frontier_cycles = 0
                self._send_navigation_goal(
                    coverage_target,
                    robot,
                    "semantic-coverage",
                )
                return
            self._no_frontier_cycles += 1
            if (
                known_cells
                >= int(self.get_parameter("minimum_known_cells").value)
                and self._no_frontier_cycles
                >= int(
                    self.get_parameter("no_frontier_cycles_to_finish").value
                )
            ):
                self._finish(
                    "Exploration complete: no reachable information-rich "
                    "frontiers remain"
                )
            else:
                self._publish_status(
                    ExplorationStatus.EXPLORING,
                    "No frontier currently reachable; checking again",
                )
            return

        self._no_frontier_cycles = 0
        _, _, point = max(candidates, key=lambda candidate: candidate[0])
        self._send_navigation_goal(point, robot, "frontier")

    def _semantic_coverage_target(
        self,
        grid: np.ndarray,
        msg: OccupancyGrid,
        robot: tuple[float, float],
        clearance_cells: int,
    ) -> tuple[float, float] | None:
        if not bool(self.get_parameter("semantic_coverage").value):
            return None

        def to_cell(point: tuple[float, float]) -> tuple[int, int]:
            return (
                int((point[1] - msg.info.origin.position.y) / msg.info.resolution),
                int((point[0] - msg.info.origin.position.x) / msg.info.resolution),
            )

        goal_cell = select_coverage_goal(
            grid,
            [to_cell(point) for point in self._visited],
            to_cell(robot),
            clearance_cells=clearance_cells,
            coverage_radius_cells=max(
                1,
                int(
                    float(
                        self.get_parameter("semantic_coverage_radius_m").value
                    )
                    / msg.info.resolution
                ),
            ),
            candidate_stride_cells=max(
                1,
                int(
                    float(
                        self.get_parameter("semantic_coverage_stride_m").value
                    )
                    / msg.info.resolution
                ),
            ),
            min_uncovered_cells=int(
                self.get_parameter("semantic_coverage_min_cells").value
            ),
            excluded_cells=[to_cell(point) for point in self._blacklisted],
            exclusion_radius_cells=max(
                1,
                int(
                    float(self.get_parameter("blacklist_radius_m").value)
                    / msg.info.resolution
                ),
            ),
        )
        if goal_cell is None:
            return None
        return cell_to_world(
            goal_cell,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
        )

    def _send_navigation_goal(
        self,
        point: tuple[float, float],
        robot: tuple[float, float],
        kind: str,
    ) -> None:
        self._apply_speed_profile(True)
        yaw = math.atan2(point[1] - robot[1], point[0] - robot[0])
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter("map_frame").value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._active_goal_pose = pose

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._goal_pending = True
        self._goal_started_at = time.monotonic()
        self._goal_cancel_requested = False
        self._goal_kind = kind
        self._last_progress_pose = (robot[0], robot[1], 0.0)
        self._last_progress_at = time.monotonic()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, target=point: self._goal_response(result, target)
        )
        self._publish_status(
            ExplorationStatus.EXPLORING,
            f"Sending {kind} goal ({point[0]:.2f}, {point[1]:.2f})",
        )

    def _goal_response(self, future, target: tuple[float, float]) -> None:
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._blacklisted.append(target)
            self.get_logger().warning(f"Frontier goal call failed: {exc}")
            return
        if not goal_handle.accepted:
            self._blacklisted.append(target)
            self.get_logger().warning("Frontier goal rejected by Nav2")
            return
        if self._canceled:
            goal_handle.cancel_goal_async()
            return
        self._active_goal = goal_handle
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda done, point=target: self._goal_result(done, point)
        )

    def _goal_result(self, future, target: tuple[float, float]) -> None:
        try:
            status = int(future.result().status)
        except Exception as exc:
            status = -1
            self.get_logger().warning(f"Frontier navigation failed: {exc}")
        self._active_goal = None
        self._goal_pending = False
        self._goal_started_at = 0.0
        self._goal_cancel_requested = False
        self._last_progress_pose = None
        self._last_progress_at = 0.0
        self._active_goal_pose = PoseStamped()
        if status == 4:
            self._consecutive_no_motion = 0
            self._visited.append(target)
            self.get_logger().info(
                f"Visited {self._goal_kind} viewpoint "
                f"({target[0]:.2f}, {target[1]:.2f})"
            )
            self._observation_until = (
                time.monotonic() + self._send_head_sweep()
            )
        else:
            self._blacklisted.append(target)
            self.get_logger().warning(
                f"Blacklisted failed frontier ({target[0]:.2f}, "
                f"{target[1]:.2f}), Nav2 status={status}"
            )

    def _send_head_sweep(self) -> float:
        settle = max(
            0.0,
            float(self.get_parameter("observation_settle_sec").value),
        )
        if (
            not bool(self.get_parameter("head_sweep").value)
            or not self._head_client.server_is_ready()
        ):
            return settle
        step = max(
            1.0,
            float(self.get_parameter("head_sweep_step_sec").value),
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        for index, pan in enumerate((-1.0, 1.0, 0.0), start=1):
            point = JointTrajectoryPoint()
            point.positions = [pan, -0.35]
            duration = index * step
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int(
                (duration - int(duration)) * 1e9
            )
            goal.trajectory.points.append(point)
        self._head_client.send_goal_async(goal)
        return 3.0 * step + settle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node._apply_speed_profile(False)
        node.destroy_node()
        rclpy.shutdown()
