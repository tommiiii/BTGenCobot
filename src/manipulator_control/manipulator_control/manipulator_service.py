#!/usr/bin/env python3
"""Manipulator Control Service using ikpy for inverse kinematics.

This ROS 2 service node provides pick and place operations for the TIAGo robot.
It uses ikpy for inverse kinematics (Torso + 7-DOF arm) and sends trajectory goals 
to ros2_control (torso_controller, arm_controller, parallel_gripper_controller).

Service: /manipulator_action (btgencobot_interfaces/srv/ManipulatorAction)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from btgencobot_interfaces.srv import ManipulatorAction
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
import numpy as np
import math
import time
import copy
import tempfile
import os

try:
    import ikpy.chain
    import ikpy.link
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False


class ManipulatorService(Node):
    """ROS2 service node for manipulator pick and place operations."""

    TORSO_JOINT = 'torso_lift_joint'
    ARM_JOINTS = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
                  'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    # Gripper positions for parallel gripper
    GRIPPER_OPEN = 0.04   # 4 cm open per finger (approx 8cm total)
    GRIPPER_CLOSED = 0.0  # Fully closed

    def __init__(self):
        super().__init__('manipulator_service')

        self.callback_group = ReentrantCallbackGroup()

        # Current joint states
        self.current_joint_positions = {}
        
        # TF2 for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action clients
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
            
        self.torso_action_client = ActionClient(
            self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory',
            callback_group=self.callback_group)

        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/parallel_gripper_controller/gripper_cmd',
            callback_group=self.callback_group)

        # Create service
        self.service = self.create_service(
            ManipulatorAction, '/manipulator_action', self._handle_request,
            callback_group=self.callback_group)

        self.arm_chain = None

        if not IKPY_AVAILABLE:
            self.get_logger().error('ikpy not available! Install with: pip3 install ikpy>=3.3')
            return

        # Subscribe to robot_description to dynamically load URDF for ikpy
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.urdf_sub = self.create_subscription(
            String, '/robot_description', self._urdf_callback, qos_profile, 
            callback_group=self.callback_group)

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10,
            callback_group=self.callback_group)

        self.get_logger().info('ManipulatorService initialized. Waiting for /robot_description...')
        self._wait_for_servers()

    def _urdf_callback(self, msg: String):
        """Parse the URDF from the topic to initialize ikpy chain."""
        if self.arm_chain is not None:
            return  # Already initialized

        self.get_logger().info('Received /robot_description. Initializing IK chain...')
        
        try:
            # Write to temporary file for ikpy parser
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(msg.data)
                temp_urdf = f.name
                
            # Define the exact alternating sequence of links and joints from base to gripper
            base_elements = [
                "base_footprint", "base_footprint_joint", "base_link", 
                "torso_fixed_joint", "torso_fixed_link", 
                "torso_lift_joint", "torso_lift_link", 
                "arm_1_joint", "arm_1_link", 
                "arm_2_joint", "arm_2_link", 
                "arm_3_joint", "arm_3_link", 
                "arm_4_joint", "arm_4_link", 
                "arm_5_joint", "arm_5_link", 
                "arm_6_joint", "arm_6_link", 
                "arm_7_joint", "arm_7_link", 
                "arm_tool_joint", "arm_tool_link", 
                "wrist_ft_joint", "wrist_ft_link", 
                "wrist_tool_joint", "wrist_ft_tool_link", 
                "gripper_tool_joint", "gripper_tool_link"
            ]
            
            # The parsed chain will have 15 elements (joints/transforms)
            # We explicitly activate only the 8 movable joints: torso_lift and arm_1 through arm_7
            active_mask = [False, False, False, True, True, True, True, True, True, True, True, False, False, False, False]
                             
            self.arm_chain = ikpy.chain.Chain.from_urdf_file(
                temp_urdf,
                base_elements=base_elements,
                active_links_mask=active_mask
            )
            
            # Print parsed joints for debugging
            joints = [link.name for link in self.arm_chain.links]
            self.get_logger().info(f'IK Chain initialized with links: {joints}')
            
            os.remove(temp_urdf)
            self.get_logger().info('IK chain successfully created for TIAGo!')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ikpy chain from URDF: {e}')

    def _wait_for_servers(self, timeout_sec: float = 10.0):
        self.get_logger().info('Waiting for action servers (arm, torso, gripper)...')
        self.arm_action_client.wait_for_server(timeout_sec=timeout_sec)
        self.torso_action_client.wait_for_server(timeout_sec=timeout_sec)
        self.gripper_action_client.wait_for_server(timeout_sec=timeout_sec)
        self.get_logger().info('Action servers wait completed.')

    def _joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def _handle_request(self, request, response):
        if self.arm_chain is None:
            response.success = False
            response.error_message = 'IK chain not initialized (missing /robot_description)'
            return response
            
        action_type = request.action_type.lower()
        target_pose = request.target_pose
        
        object_height = request.object_height if request.object_height > 0 else 0.1
        object_width = request.object_width if request.object_width > 0 else 0.05
        
        self.get_logger().info(f'Handling {action_type} for pose ({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f})')

        try:
            if action_type == 'pick':
                success = self._execute_pick(target_pose, object_height, object_width)
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

    def _execute_pick(self, target_pose: PoseStamped, object_height: float, object_width: float) -> bool:
        # Move arm to a home/ready pose to avoid collision while navigating
        self._move_torso(0.2, 2.0)
        self._move_gripper(self.GRIPPER_OPEN, 1.0)
        
        # We calculate the grasp position
        # Target Z is the top of the object. Since gripper_tool_link is at the base of the fingers,
        # we must offset the target Z UP by the length of the fingers (approx 15cm) so the fingertips reach the object.
        finger_length = 0.15
        grasp_z = target_pose.pose.position.z - 0.02 + finger_length
        above_z = target_pose.pose.position.z + 0.10 + finger_length
        
        grasp_pose = copy.deepcopy(target_pose)
        grasp_pose.pose.position.z = grasp_z
        above_pose = copy.deepcopy(target_pose)
        above_pose.pose.position.z = above_z
        
        # Calculate IK
        grasp_joints = self._compute_ik_for_pose(grasp_pose)
        above_joints = self._compute_ik_for_pose(above_pose)
        
        if not grasp_joints:
            self.get_logger().error("Pick failed: IK could not find solution for grasp pose.")
            return False
            
        if above_joints:
            self.get_logger().info("Moving above object...")
            self._move_all_joints(above_joints, 3.0)
            
        self.get_logger().info("Descending to grasp...")
        if not self._move_all_joints(grasp_joints, 2.0):
            return False
            
        self.get_logger().info("Closing gripper...")
        self._move_gripper(self.GRIPPER_CLOSED, 1.0, force_grasp=True)
        
        if above_joints:
            self.get_logger().info("Lifting object...")
            self._move_all_joints(above_joints, 2.0)
            
        return True

    def _execute_place(self, target_pose: PoseStamped) -> bool:
        finger_length = 0.15
        place_pose = copy.deepcopy(target_pose)
        place_pose.pose.position.z += finger_length
        
        above_pose = copy.deepcopy(place_pose)
        above_pose.pose.position.z += 0.10
        
        place_joints = self._compute_ik_for_pose(place_pose)
        above_joints = self._compute_ik_for_pose(above_pose)
        
        if above_joints:
            self._move_all_joints(above_joints, 3.0)
            
        if place_joints:
            self._move_all_joints(place_joints, 2.0)
            
        self._move_gripper(self.GRIPPER_OPEN, 1.0)
        
        if above_joints:
            self._move_all_joints(above_joints, 2.0)
            
        return True

    def _compute_ik_for_pose(self, target_pose: PoseStamped) -> list:
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_footprint",
                target_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pose_in_base = do_transform_pose_stamped(target_pose, transform)
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None

        # Extract XYZ for IK
        target = [pose_in_base.pose.position.x, pose_in_base.pose.position.y, pose_in_base.pose.position.z]
        self.get_logger().info(f'IK target (base_footprint): x={target[0]:.3f} y={target[1]:.3f} z={target[2]:.3f}')
        
        # Use initial guess based on current positions to minimize motion
        # active_links_mask will map to 8 active joints
        current_angles = []
        # Torso
        current_angles.append(self.current_joint_positions.get(self.TORSO_JOINT, 0.2))
        # Arm 1-7
        for j in self.ARM_JOINTS:
            current_angles.append(self.current_joint_positions.get(j, 0.0))
            
        # Pad with 0s for fixed/origin links that ikpy adds
        # ikpy output includes the origin link and the end effector link (so 8 + 2 = 10 elements usually)
        guess = [0.0] * len(self.arm_chain.links)
        active_indices = self.arm_chain.active_links_mask
        
        # Place current joint angles into the guess array for active links
        j_idx = 0
        for i, is_active in enumerate(active_indices):
            if is_active and j_idx < len(current_angles):
                guess[i] = current_angles[j_idx]
                j_idx += 1

        ik_solution = self.arm_chain.inverse_kinematics(
            target_position=target,
            target_orientation=[0.0, 0.0, 1.0], # Z-axis points into wrist, so UP means fingers point DOWN
            orientation_mode="Z",
            initial_position=guess
        )

        # Extract active joints
        result_joints = []
        for i, is_active in enumerate(active_indices):
            if is_active:
                result_joints.append(ik_solution[i])

        if len(result_joints) != 8:
            self.get_logger().error(f'Expected 8 joints from IK, got {len(result_joints)}')
            return None

        # Forward kinematics to verify IK convergence
        fk_result = self.arm_chain.forward_kinematics(ik_solution)
        fk_pos = fk_result[:3, 3]
        error = [fk_pos[i] - target[i] for i in range(3)]
        self.get_logger().info(
            f'IK result: joints={[f"{float(j):.3f}" for j in result_joints]}'
            f'\n  FK position: ({fk_pos[0]:.3f}, {fk_pos[1]:.3f}, {fk_pos[2]:.3f})'
            f'\n  Target:      ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
            f'\n  Error:        ({error[0]:.4f}, {error[1]:.4f}, {error[2]:.4f}) = {sum(e**2 for e in error)**0.5:.4f}m'
        )

        return result_joints

    def _move_all_joints(self, joint_angles: list, duration: float) -> bool:
        """Move torso and arm simultaneously."""
        torso_angle = joint_angles[0]
        arm_angles = joint_angles[1:8]
        
        t_fut = self._send_torso_trajectory(torso_angle, duration)
        a_fut = self._send_arm_trajectory(arm_angles, duration)
        
        # Wait for both
        return a_fut and t_fut

    def _move_torso(self, position: float, duration: float) -> bool:
        return self._send_torso_trajectory(position, duration)

    def _send_torso_trajectory(self, position: float, duration: float) -> bool:
        if not self.torso_action_client.server_is_ready():
            return False
        
        # Clip torso
        position = max(0.0, min(0.35, position))

        trajectory = JointTrajectory()
        trajectory.joint_names = [self.TORSO_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        future = self.torso_action_client.send_goal_async(goal)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
        
        if not future.done() or not future.result().accepted:
            return False
            
        # Wait for actual execution to finish
        result_future = future.result().get_result_async()
        start = time.time()
        while not result_future.done() and time.time() - start < duration + 5.0:
            time.sleep(0.05)
            
        return result_future.done()

    def _send_arm_trajectory(self, positions: list, duration: float) -> bool:
        if not self.arm_action_client.server_is_ready():
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = self.ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        future = self.arm_action_client.send_goal_async(goal)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
            
        if not future.done() or not future.result().accepted:
            return False
            
        # Wait for actual execution to finish
        result_future = future.result().get_result_async()
        start = time.time()
        while not result_future.done() and time.time() - start < duration + 5.0:
            time.sleep(0.05)
            
        return result_future.done()

    def _move_gripper(self, position: float, duration: float = 1.0, force_grasp: bool = False) -> bool:
        if not self.gripper_action_client.server_is_ready():
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 20.0 if force_grasp else 10.0

        future = self.gripper_action_client.send_goal_async(goal)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            time.sleep(0.05)
            
        if not future.done() or not future.result().accepted:
            return False
            
        # Wait for actual execution to finish
        result_future = future.result().get_result_async()
        start = time.time()
        while not result_future.done() and time.time() - start < duration + 5.0:
            time.sleep(0.05)
            
        return result_future.done()

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
