#!/usr/bin/env python3
"""Manipulator Control Service using ikpy for inverse kinematics.

This ROS2 service node provides pick and place operations for the OpenManipulator-X arm.
It uses ikpy for inverse kinematics and sends trajectory goals to ros2_control.

Service: /manipulator_action (btgencobot_interfaces/srv/ManipulatorAction)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from btgencobot_interfaces.srv import ManipulatorAction
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
import numpy as np
import math
import time

try:
    import ikpy.chain
    import ikpy.link
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False


class ManipulatorService(Node):
    """ROS2 service node for manipulator pick and place operations."""

    # OpenManipulator-X joint names
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
    GRIPPER_JOINTS = ['gripper_left_joint']

    # Joint limits (from URDF)
    JOINT_LIMITS = {
        'joint1': (-2.827, 2.827),      # ±162°
        'joint2': (-1.791, 1.571),      # -102° to +90°
        'joint3': (-0.942, 1.382),      # -54° to +79°
        'joint4': (-1.791, 2.042),      # -102° to +117°
    }

    # Gripper positions
    GRIPPER_OPEN = 0.019
    GRIPPER_CLOSED = -0.010

    # Predefined arm positions (joint1, joint2, joint3, joint4)
    # These are tuned for OpenManipulator-X mounted with -90° pitch on base_scan
    POSES = {
        'home': [0.0, 0.0, 0.0, 0.0],
        'ready': [0.0, -0.3, 0.5, -0.2],
        'pre_grasp': [0.0, 0.2, 0.6, 0.8],
        'grasp': [0.0, 0.5, 0.4, 1.0],
        'lift': [0.0, 0.0, 0.5, 0.5],
    }

    def __init__(self):
        super().__init__('manipulator_service')

        self.callback_group = ReentrantCallbackGroup()

        # Current joint states
        self.current_joint_positions = {}
        self.joint_state_received = False

        # TF2 for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # Action clients for arm and gripper controllers
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Use GripperCommand action (GripperActionController)
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )

        # Create service
        self.service = self.create_service(
            ManipulatorAction,
            '/manipulator_action',
            self._handle_request,
            callback_group=self.callback_group
        )

        # Initialize ikpy chain if available
        self.arm_chain = None
        if IKPY_AVAILABLE:
            self._init_ikpy_chain()
        else:
            self.get_logger().warn('ikpy not available, using predefined poses only')

        self.get_logger().info('ManipulatorService initialized')
        self.get_logger().info('Waiting for action servers...')

        # Wait for action servers
        self._wait_for_servers()

    def _init_ikpy_chain(self):
        """Initialize ikpy chain for the OpenManipulator-X arm."""
        try:
            # Define the kinematic chain manually based on URDF
            # The arm is mounted with -90° pitch, so the chain needs adjustment
            # Chain: base_scan -> link1 -> link2 -> link3 -> link4 -> link5 -> end_effector
            
            # Link lengths from URDF (in meters)
            # link1 to link2: xyz="0.012 0.0 0.017"
            # link2 to link3: xyz="0.0 0.0 0.0595"
            # link3 to link4: xyz="0.024 0 0.128"
            # link4 to link5: xyz="0.124 0.0 0.0"
            # link5 to end_effector: xyz="0.126 0.0 0.0"

            # Define links with active_links mask to handle fixed end-effector
            links = [
                ikpy.link.OriginLink(),
                ikpy.link.URDFLink(
                    name="link1",
                    origin_translation=[0, 0, 0.04],  # mount offset from base_scan
                    origin_orientation=[0, -np.pi/2, 0],  # -90° pitch mount
                    rotation=[0, 0, 1],  # joint1 rotates around Z
                ),
                ikpy.link.URDFLink(
                    name="link2",
                    origin_translation=[0.012, 0, 0.017],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # joint2 rotates around Y
                ),
                ikpy.link.URDFLink(
                    name="link3",
                    origin_translation=[0, 0, 0.0595],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # joint3 rotates around Y
                ),
                ikpy.link.URDFLink(
                    name="link4",
                    origin_translation=[0.024, 0, 0.128],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 1, 0],  # joint4 rotates around Y
                ),
                ikpy.link.URDFLink(
                    name="end_effector",
                    origin_translation=[0.124 + 0.126, 0, 0],  # link5 + end_effector offset
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 0],  # Dummy rotation axis (will be masked)
                ),
            ]
            # active_links_mask: True for active joints, False for fixed links
            # [OriginLink, link1, link2, link3, link4, end_effector]
            active_links_mask = [False, True, True, True, True, False]
            self.arm_chain = ikpy.chain.Chain(
                name='open_manipulator_x',
                links=links,
                active_links_mask=active_links_mask
            )
            self.get_logger().info('ikpy chain initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ikpy chain: {e}')
            self.arm_chain = None

    def _wait_for_servers(self, timeout_sec: float = 30.0):
        """Wait for action servers to be available.
        
        This method waits for both arm and gripper controllers to become available.
        It will log warnings if servers are not found but will not crash.
        """
        self.get_logger().info('Waiting for arm_controller action server...')
        try:
            arm_ready = self.arm_action_client.wait_for_server(timeout_sec=timeout_sec)
            if arm_ready:
                self.get_logger().info('arm_controller action server connected')
            else:
                self.get_logger().warn(
                    f'arm_controller action server not available after {timeout_sec}s. '
                    'Will retry when service is called.'
                )
        except Exception as e:
            self.get_logger().warn(f'Error waiting for arm_controller: {e}')

        self.get_logger().info('Waiting for gripper_controller action server...')
        try:
            gripper_ready = self.gripper_action_client.wait_for_server(timeout_sec=timeout_sec)
            if gripper_ready:
                self.get_logger().info('gripper_controller action server connected')
            else:
                self.get_logger().warn(
                    f'gripper_controller action server not available after {timeout_sec}s. '
                    'Will retry when service is called.'
                )
        except Exception as e:
            self.get_logger().warn(f'Error waiting for gripper_controller: {e}')

    def _joint_state_callback(self, msg: JointState):
        """Store current joint positions."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
        self.joint_state_received = True

    def _handle_request(self, request, response):
        """Handle ManipulatorAction service request."""
        action_type = request.action_type.lower()
        target_pose = request.target_pose

        self.get_logger().info(
            f'Received {action_type} request at '
            f'({target_pose.pose.position.x:.3f}, '
            f'{target_pose.pose.position.y:.3f}, '
            f'{target_pose.pose.position.z:.3f})'
        )

        try:
            if action_type == 'pick':
                success = self._execute_pick(target_pose)
            elif action_type == 'place':
                success = self._execute_place(target_pose)
            else:
                response.success = False
                response.error_message = f'Unknown action type: {action_type}'
                return response

            response.success = success
            if not success:
                response.error_message = f'{action_type} operation failed'
            return response

        except Exception as e:
            self.get_logger().error(f'{action_type} failed: {e}')
            response.success = False
            response.error_message = str(e)
            return response

    def _execute_pick(self, target_pose: PoseStamped) -> bool:
        """Execute pick operation sequence using IK for target pose."""
        self.get_logger().info('Starting pick sequence')

        # 1. Move to ready position
        if not self._move_arm_to_pose('ready', duration=2.0):
            self.get_logger().error('Failed to move to ready position')
            return False

        # 2. Open gripper
        if not self._move_gripper(self.GRIPPER_OPEN, duration=1.0):
            self.get_logger().error('Failed to open gripper')
            return False

        # 3. Compute IK for target position
        grasp_joints = self._compute_ik_for_pose(target_pose)
        if grasp_joints is None:
            self.get_logger().warn('IK failed, falling back to predefined poses')
            # Fallback to predefined poses
            if not self._move_arm_to_pose('pre_grasp', duration=2.0):
                return False
            if not self._move_arm_to_pose('grasp', duration=2.0):
                return False
        else:
            # Compute pre-grasp position (slightly above/back from grasp)
            pre_grasp_pose = PoseStamped()
            pre_grasp_pose.header = target_pose.header
            pre_grasp_pose.pose = target_pose.pose
            pre_grasp_pose.pose.position.z += 0.05  # 5cm above
            
            pre_grasp_joints = self._compute_ik_for_pose(pre_grasp_pose)
            if pre_grasp_joints:
                self.get_logger().info(f'Moving to pre-grasp IK position: {pre_grasp_joints}')
                if not self._move_arm_to_joints(pre_grasp_joints, duration=2.0):
                    self.get_logger().error('Failed to move to pre-grasp position')
                    return False
            
            # Move to grasp position
            self.get_logger().info(f'Moving to grasp IK position: {grasp_joints}')
            if not self._move_arm_to_joints(grasp_joints, duration=2.0):
                self.get_logger().error('Failed to move to grasp position')
                return False

        # 4. Close gripper
        if not self._move_gripper(self.GRIPPER_CLOSED, duration=1.5):
            self.get_logger().error('Failed to close gripper')
            return False

        # 5. Lift object
        if not self._move_arm_to_pose('lift', duration=2.0):
            self.get_logger().error('Failed to lift object')
            return False

        # 6. Return to ready position
        if not self._move_arm_to_pose('ready', duration=2.0):
            self.get_logger().error('Failed to return to ready position')
            return False

        self.get_logger().info('Pick sequence completed successfully')
        return True

    def _compute_ik_for_pose(self, target_pose: PoseStamped) -> list:
        """Transform target pose to arm frame and compute IK."""
        if self.arm_chain is None:
            self.get_logger().warn('IK chain not available')
            return None

        try:
            # Transform pose from map frame to arm base frame (link1)
            # The arm is mounted on base_scan with -90° pitch
            arm_base_frame = "link1"
            
            # Wait for transform
            try:
                transform = self.tf_buffer.lookup_transform(
                    arm_base_frame,
                    target_pose.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                pose_in_arm = do_transform_pose_stamped(target_pose, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF transform failed: {e}')
                return None

            # Extract position for IK
            x = pose_in_arm.pose.position.x
            y = pose_in_arm.pose.position.y
            z = pose_in_arm.pose.position.z

            self.get_logger().info(
                f'Target in arm frame: ({x:.3f}, {y:.3f}, {z:.3f})'
            )

            # Compute IK
            return self._compute_ik([x, y, z])

        except Exception as e:
            self.get_logger().error(f'IK computation failed: {e}')
            return None

    def _execute_place(self, target_pose: PoseStamped) -> bool:
        """Execute place operation sequence using IK for target pose."""
        self.get_logger().info('Starting place sequence')

        # 1. Move to ready position (should already be holding object)
        if not self._move_arm_to_pose('ready', duration=2.0):
            self.get_logger().error('Failed to move to ready position')
            return False

        # 2. Compute IK for place position
        place_joints = self._compute_ik_for_pose(target_pose)
        if place_joints is None:
            self.get_logger().warn('IK failed, falling back to predefined poses')
            # Fallback to predefined poses
            if not self._move_arm_to_pose('pre_grasp', duration=2.0):
                return False
            if not self._move_arm_to_pose('grasp', duration=2.0):
                return False
        else:
            # Compute pre-place position (slightly above place location)
            pre_place_pose = PoseStamped()
            pre_place_pose.header = target_pose.header
            pre_place_pose.pose = target_pose.pose
            pre_place_pose.pose.position.z += 0.05  # 5cm above
            
            pre_place_joints = self._compute_ik_for_pose(pre_place_pose)
            if pre_place_joints:
                self.get_logger().info(f'Moving to pre-place IK position: {pre_place_joints}')
                if not self._move_arm_to_joints(pre_place_joints, duration=2.0):
                    self.get_logger().error('Failed to move to pre-place position')
                    return False
            
            # Move to place position
            self.get_logger().info(f'Moving to place IK position: {place_joints}')
            if not self._move_arm_to_joints(place_joints, duration=2.0):
                self.get_logger().error('Failed to move to place position')
                return False

        # 3. Open gripper to release
        if not self._move_gripper(self.GRIPPER_OPEN, duration=1.0):
            self.get_logger().error('Failed to open gripper')
            return False

        # 4. Retract to lift position
        if not self._move_arm_to_pose('lift', duration=2.0):
            self.get_logger().error('Failed to retract')
            return False

        # 5. Return to home position
        if not self._move_arm_to_pose('home', duration=2.0):
            self.get_logger().error('Failed to return to home position')
            return False

        self.get_logger().info('Place sequence completed successfully')
        return True

    def _move_arm_to_pose(self, pose_name: str, duration: float = 2.0) -> bool:
        """Move arm to a predefined pose."""
        if pose_name not in self.POSES:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False

        joint_positions = self.POSES[pose_name]
        self.get_logger().info(f'Moving arm to {pose_name}: {joint_positions}')

        return self._send_arm_trajectory(joint_positions, duration)

    def _move_arm_to_joints(self, joint_positions: list, duration: float = 2.0) -> bool:
        """Move arm to specific joint positions."""
        # Clamp to joint limits
        clamped = []
        for i, (pos, joint_name) in enumerate(zip(joint_positions, self.ARM_JOINTS)):
            limits = self.JOINT_LIMITS[joint_name]
            clamped_pos = max(limits[0], min(limits[1], pos))
            if abs(clamped_pos - pos) > 0.01:
                self.get_logger().warn(
                    f'{joint_name}: clamped {pos:.3f} to {clamped_pos:.3f}'
                )
            clamped.append(clamped_pos)

        return self._send_arm_trajectory(clamped, duration)

    def _send_arm_trajectory(self, positions: list, duration: float) -> bool:
        """Send trajectory goal to arm controller."""
        if not self.arm_action_client.server_is_ready():
            self.get_logger().error('Arm action server not ready')
            return False

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9)
        )
        trajectory.points = [point]

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Send goal and wait for result using polling (not spin_until_future_complete)
        self.get_logger().debug(f'Sending arm trajectory: {positions}')
        future = self.arm_action_client.send_goal_async(goal)
        
        # Poll for goal acceptance
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error('Failed to send arm goal (timeout)')
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return False

        # Wait for result using polling
        # Use generous timeout since simulation may run slower than real-time
        result_future = goal_handle.get_result_async()
        timeout = duration * 5 + 10.0  # 5x duration + 10s margin for slow simulation
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error('Arm trajectory timed out')
                return False
            time.sleep(0.1)

        result = result_future.result()
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Arm trajectory failed: {result.result.error_code}')
            return False

        self.get_logger().debug('Arm trajectory completed')
        return True

    def _move_gripper(self, position: float, duration: float = 1.0) -> bool:
        """Move gripper to position using GripperCommand action."""
        # Wait for gripper action server if not ready (may take time to spawn)
        if not self.gripper_action_client.server_is_ready():
            self.get_logger().info('Waiting for gripper action server...')
            if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('Gripper action server not available after 10s')
                return False
            self.get_logger().info('Gripper action server connected')

        # Clamp gripper position
        position = max(self.GRIPPER_CLOSED, min(self.GRIPPER_OPEN, position))

        # Create GripperCommand goal
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0  # Max effort for gripping

        # Send goal and wait using polling
        action_name = 'open' if position > 0 else 'close'
        self.get_logger().info(f'Gripper {action_name}: {position}')
        future = self.gripper_action_client.send_goal_async(goal)
        
        # Poll for goal acceptance
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error('Failed to send gripper goal (timeout)')
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        # Wait for result using polling
        # Use generous timeout since simulation may run slower than real-time
        result_future = goal_handle.get_result_async()
        timeout = duration * 5 + 10.0  # 5x duration + 10s margin for slow simulation
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error('Gripper command timed out')
                return False
            time.sleep(0.1)

        result = result_future.result()
        # GripperCommand result has 'reached_goal' and 'stalled' fields
        if not result.result.reached_goal and not result.result.stalled:
            self.get_logger().warn(f'Gripper may not have reached target position')
        
        self.get_logger().info(f'Gripper {action_name} completed')
        return True

    def _compute_ik(self, target_position: list) -> list:
        """Compute inverse kinematics for target position using ikpy."""
        if self.arm_chain is None:
            self.get_logger().warn('IK not available, returning None')
            return None

        try:
            # ikpy expects [x, y, z] target
            target = np.array(target_position)

            # Compute IK
            ik_solution = self.arm_chain.inverse_kinematics(target)

            # Extract joint angles (skip first and last elements - origin and end effector)
            joint_angles = ik_solution[1:5].tolist()

            self.get_logger().info(f'IK solution: {joint_angles}')
            return joint_angles

        except Exception as e:
            self.get_logger().error(f'IK computation failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)

    node = ManipulatorService()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
