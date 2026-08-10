"""ROS2 Action Server for BehaviorTree Generation and Execution"""
import json
from difflib import SequenceMatcher
import math
import re
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Optional

import requests
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.task import Future

from btgencobot_interfaces.action import GenerateAndExecuteBT
from btgencobot_interfaces.srv import (
    DiscoverSemanticObject,
    ResolveSemanticNavigation,
)
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bt_text_interface.semantic_compilation import (
    reacquisition_requested,
    supply_live_pose_to_following_pick,
)


NAV_STATUS_NAMES = {
    1: 'UNKNOWN', 2: 'ACCEPTED', 3: 'EXECUTING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'
}

def _escape_xml_attr(value) -> str:
    return (
        str(value)
        .replace('&', '&amp;')
        .replace('"', '&quot;')
        .replace('<', '&lt;')
        .replace('>', '&gt;')
    )


class BTInterfaceNode(Node):
    """ROS2 Action Server for generating BehaviorTrees from natural language and executing them via Nav2"""

    def __init__(self):
        super().__init__('bt_interface_node')
        self._declare_parameters()
        self._initialize_state()
        self._setup_interfaces()
        self._log_configuration()

    def _declare_parameters(self):
        """Declare and load ROS parameters"""
        self.declare_parameter('inference_server_url', 'http://host.docker.internal:8080')
        self.declare_parameter('bt_output_dir', '/workspace/generated_bts')
        self.declare_parameter('generation_timeout', 30.0)
        self.declare_parameter('execution_timeout', 300.0)
        self.declare_parameter('feedback_rate', 2.0)
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('nav_server_wait_timeout', 90.0)
        self.declare_parameter(
            'semantic_resolver_service',
            '/hydra/resolve_semantic_navigation',
        )
        self.declare_parameter('semantic_queue_timeout', 1800.0)
        self.declare_parameter('semantic_queue_poll_period', 2.0)
        self.declare_parameter('semantic_resolver_call_timeout', 5.0)
        self.declare_parameter(
            'semantic_discovery_service',
            '/mapping/discover_semantic_object',
        )
        self.declare_parameter('semantic_discovery_call_timeout', 30.0)
        # Kept for service-wire compatibility. Unknown objects are grounded
        # against one current RGB-D frame, never by scanning saved keyframes.
        self.declare_parameter('semantic_discovery_max_keyframes', 1)
        self.declare_parameter('semantic_discovery_search_timeout', 25.0)

        self.inference_url = self.get_parameter('inference_server_url').value
        self.bt_output_dir = Path(self.get_parameter('bt_output_dir').value)
        self.generation_timeout = self.get_parameter('generation_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.nav_action_name = self.get_parameter('nav_action').value
        self.nav_server_wait_timeout = self.get_parameter('nav_server_wait_timeout').value
        self.semantic_resolver_service = self.get_parameter(
            'semantic_resolver_service'
        ).value
        self.semantic_queue_timeout = float(
            self.get_parameter('semantic_queue_timeout').value
        )
        self.semantic_queue_poll_period = float(
            self.get_parameter('semantic_queue_poll_period').value
        )
        self.semantic_resolver_call_timeout = float(
            self.get_parameter('semantic_resolver_call_timeout').value
        )
        self.semantic_discovery_service = self.get_parameter(
            'semantic_discovery_service'
        ).value
        self.semantic_discovery_call_timeout = float(
            self.get_parameter('semantic_discovery_call_timeout').value
        )
        self.semantic_discovery_max_keyframes = int(
            self.get_parameter('semantic_discovery_max_keyframes').value
        )
        self.semantic_discovery_search_timeout = float(
            self.get_parameter('semantic_discovery_search_timeout').value
        )

        self.bt_output_dir.mkdir(parents=True, exist_ok=True)

    def _initialize_state(self):
        """Initialize state variables"""
        self.current_goal_handle = None
        self.current_nav_goal_handle = None
        self.active_client_goal_handle = None
        self.is_executing = False
        self.last_bt_xml = None
        self._known_entities = ''

    @staticmethod
    def _set_future_result_if_pending(future: Future, result) -> None:
        """Complete a ROS future safely when service and timeout race."""
        try:
            if not future.done():
                future.set_result(result)
        except RuntimeError:
            # The competing completion won after the done() check.
            pass

    async def _wait_for_rclpy_future(self, future: Future, timeout: float):
        """Await a ROS future with a wall-clock timeout.

        rclpy advances action-server coroutines itself; there is no asyncio
        event loop, so asyncio.wait_for cannot be used here.
        """
        completion = Future(executor=self.executor)

        def service_finished(done_future: Future) -> None:
            self._set_future_result_if_pending(
                completion,
                ('service', done_future),
            )

        timer = threading.Timer(
            timeout,
            lambda: self._set_future_result_if_pending(
                completion,
                ('timeout', None),
            ),
        )
        timer.daemon = True
        future.add_done_callback(service_finished)
        timer.start()
        try:
            outcome, completed_future = await completion
        finally:
            timer.cancel()

        if outcome == 'timeout':
            future.cancel()
            raise TimeoutError
        return completed_future.result()

    async def _wall_sleep(self, duration: float) -> None:
        """Yield an rclpy coroutine for a bounded amount of wall time."""
        completion = Future(executor=self.executor)
        timer = threading.Timer(
            duration,
            lambda: self._set_future_result_if_pending(completion, None),
        )
        timer.daemon = True
        timer.start()
        try:
            await completion
        finally:
            timer.cancel()

    def _setup_interfaces(self):
        """Setup ROS interfaces: publishers, subscribers, action servers/clients, services"""
        self.action_callback_group = ReentrantCallbackGroup()

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._bt_xml_publisher = self.create_publisher(String, '/generated_behavior_tree', qos_latched)
        self._bt_execution_feedback_publisher = self.create_publisher(
            String,
            '/bt_execution_feedback',
            10,
        )
        self._cmd_vel_nav_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._cmd_vel_smoothed_publisher = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        self._cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bt_republish_timer = self.create_timer(2.0, self._republish_last_bt)

        self._action_server = ActionServer(
            self, GenerateAndExecuteBT, '/generate_and_execute_bt',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, self.nav_action_name,
            callback_group=self.action_callback_group
        )

        self._self_client = ActionClient(
            self, GenerateAndExecuteBT, '/generate_and_execute_bt',
            callback_group=self.action_callback_group
        )
        self._semantic_resolver_client = self.create_client(
            ResolveSemanticNavigation,
            self.semantic_resolver_service,
            callback_group=self.action_callback_group,
        )
        self._semantic_discovery_client = self.create_client(
            DiscoverSemanticObject,
            self.semantic_discovery_service,
            callback_group=self.action_callback_group,
        )

        self._emergency_stop_srv = self.create_service(
            Trigger, '/emergency_stop_bt', self.emergency_stop_callback
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._command_subscriber = self.create_subscription(
            String, '/btgen_nl_command', self.command_topic_callback, qos_profile
        )
        self._known_entities_subscriber = self.create_subscription(
            String,
            '/hydra/known_entities',
            self._known_entities_callback,
            qos_latched,
        )

    def _log_configuration(self):
        self.get_logger().info(f'BT output directory: {self.bt_output_dir}')
        self.get_logger().info(f'Inference server URL: {self.inference_url}')
        self.get_logger().info(f'Navigation action target: {self.nav_action_name}')
        self.get_logger().info(
            f'Semantic resolver: {self.semantic_resolver_service}'
        )
        self.get_logger().info('BT Interface Node initialized')

    def _known_entities_callback(self, msg: String):
        self._known_entities = msg.data

    @staticmethod
    def _semantic_label_similarity(query: str, candidate: str) -> float:
        def normalize(value: str) -> str:
            value = value.strip().lower().replace('_', ' ').replace('-', ' ')
            value = re.sub(r'^(?:the|a|an)\s+', '', value)
            return re.sub(r'\s+', ' ', value)

        query = normalize(query)
        candidate = normalize(candidate)
        if not query or not candidate:
            return 0.0
        if query == candidate:
            return 1.0
        if query in candidate or candidate in query:
            return 0.88
        query_tokens = set(query.split())
        candidate_tokens = set(candidate.split())
        token_score = (
            len(query_tokens & candidate_tokens)
            / max(len(query_tokens), len(candidate_tokens))
        )
        return max(
            0.85 * SequenceMatcher(None, query, candidate).ratio(),
            token_score,
        )

    def _ground_generated_semantic_types(self, xml_string: str) -> str:
        """Correct room/object layer selection using the live Hydra inventory."""
        try:
            context = json.loads(self._known_entities)
        except (TypeError, json.JSONDecodeError):
            return xml_string
        if not isinstance(context, dict) or not context.get('ready'):
            return xml_string

        rooms = [str(value) for value in context.get('rooms', [])]
        objects = [str(value) for value in context.get('objects', [])]
        try:
            root = ET.fromstring(xml_string)
        except ET.ParseError:
            return xml_string

        changed = False
        for element in root.iter():
            if self._semantic_node_id(element) != 'NavigateSemantic':
                continue
            reference = element.get('entity_ref', '')
            current_type, separator, label = reference.partition(':')
            if not separator:
                label = reference
            room_score = max(
                (
                    self._semantic_label_similarity(label, candidate)
                    for candidate in rooms
                ),
                default=0.0,
            )
            object_score = max(
                (
                    self._semantic_label_similarity(label, candidate)
                    for candidate in objects
                ),
                default=0.0,
            )
            threshold = 0.72
            if room_score >= threshold and room_score > object_score + 0.04:
                grounded_type = 'room'
            elif object_score >= threshold and object_score > room_score + 0.04:
                grounded_type = 'object'
            elif room_score >= threshold and object_score < threshold:
                grounded_type = 'room'
            elif object_score >= threshold and room_score < threshold:
                grounded_type = 'object'
            else:
                grounded_type = 'object'

            grounded_reference = f'{grounded_type}:{label.strip()}'
            if (
                grounded_reference != reference
                or element.get('entity_type') != grounded_type
            ):
                self.get_logger().info(
                    f'Grounded semantic reference {reference!r} as '
                    f'{grounded_reference!r} '
                    f'(room={room_score:.2f}, object={object_score:.2f})'
                )
                element.set('entity_ref', grounded_reference)
                element.set('entity_type', grounded_type)
                changed = True

        return ET.tostring(root, encoding='unicode') if changed else xml_string

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.command}')
        if self.is_executing:
            self.get_logger().warn('Already executing — rejecting concurrent goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.current_goal_handle = goal_handle
        self.is_executing = True

        command = goal_handle.request.command
        self.get_logger().info(f'Command: {command}')

        result = GenerateAndExecuteBT.Result()
        result.success = False
        result.bt_xml_path = ''
        result.error_message = ''

        try:
            self.publish_feedback(goal_handle, 'generating', 0.1, 'Calling inference server...')
            bt_xml, error = await self.generate_bt_from_command(command)
            
            # Uscita pulita se cancellato durante la generazione
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Esecuzione interrotta per nuovo comando (preemption).")
                goal_handle.canceled()
                result.success = True
                result.error_message = 'Task preempted gracefully'
                return result

            if bt_xml is None:
                result.error_message = f'BT generation failed: {error}'
                self.get_logger().error(result.error_message)
                self.publish_feedback(goal_handle, 'failed', 1.0, result.error_message)
                goal_handle.abort()
                return result

            self.publish_feedback(
                goal_handle,
                'grounding',
                0.25,
                'Resolving semantic actions through Hydra...',
            )
            bt_xml, compile_error = await self.compile_semantic_actions(
                bt_xml,
                goal_handle,
            )
            if bt_xml is None:
                result.error_message = (
                    f'Semantic navigation resolution failed: {compile_error}'
                )
                self.get_logger().error(result.error_message)
                self.publish_feedback(
                    goal_handle,
                    'failed',
                    1.0,
                    result.error_message,
                )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result

            self.publish_feedback(goal_handle, 'validating', 0.3, 'Validating generated BT...')
            is_valid, val_error = self.validate_bt_xml(bt_xml)
            if not is_valid:
                result.error_message = f'BT validation failed: {val_error}'
                self.get_logger().error(result.error_message)
                self.publish_feedback(goal_handle, 'failed', 1.0, result.error_message)
                goal_handle.abort()
                return result

            self.publish_feedback(goal_handle, 'validating', 0.4, 'Writing BT to file...')

            bt_file_path = self.write_bt_file(bt_xml)
            result.bt_xml_path = str(bt_file_path)

            self.last_bt_xml = bt_xml
            bt_msg = String()
            bt_msg.data = self.add_uids_for_foxglove(bt_xml)
            self._bt_xml_publisher.publish(bt_msg)

            self.publish_feedback(goal_handle, 'executing', 0.5, 'Executing BehaviorTree...')
            execution_success, exec_error = await self.execute_bt(bt_file_path, goal_handle)
            
            # --- MODIFICA CHIAVE PER IL FRONTEND ---
            # Se è stato cancellato dall'utente (preemption), chiudiamo con successo per non far crashare la UI
            if goal_handle.is_cancel_requested or exec_error == 'PREEMPTED':
                self.get_logger().info('Task sostituito con successo dal nuovo comando.')
                result.success = True
                result.error_message = 'Task preempted by new command'
                return result

            if not execution_success:
                result.error_message = f'BT execution failed: {exec_error}'
                self.get_logger().error(result.error_message)
                self.publish_feedback(goal_handle, 'failed', 1.0, result.error_message)
                goal_handle.abort()
                return result

            self.get_logger().info('BT execution completed successfully')
            self.publish_feedback(goal_handle, 'completed', 1.0, 'BT execution completed')

            result.success = True
            goal_handle.succeed()

        except Exception as e:
            result.error_message = f'Unexpected error: {str(e)}'
            self.get_logger().error(result.error_message)
            self.publish_feedback(goal_handle, 'failed', 1.0, result.error_message)
            goal_handle.abort()

        finally:
            if self.current_goal_handle == goal_handle:
                self.is_executing = False
                self.current_goal_handle = None

        return result

    def _parse_semantic_navigation_command(self, command: str) -> Optional[dict]:
        try:
            envelope = json.loads(command)
        except json.JSONDecodeError:
            return None

        if not isinstance(envelope, dict):
            return None
        if envelope.get('type') != 'semantic_navigation':
            return None

        semantic = envelope.get('semantic_navigation')
        if not isinstance(semantic, dict):
            raise ValueError('semantic_navigation envelope missing semantic_navigation object')

        waypoints = semantic.get('waypoints')
        if not isinstance(waypoints, list) or len(waypoints) == 0:
            raise ValueError('semantic_navigation request must include at least one waypoint')

        for index, waypoint in enumerate(waypoints, start=1):
            if not isinstance(waypoint, dict):
                raise ValueError(f'Waypoint {index} is not an object')
            if 'x' not in waypoint or 'y' not in waypoint:
                raise ValueError(f'Waypoint {index} must include x and y')

        return {
            'command': envelope.get('command') or f"go to {semantic.get('destination_label', 'destination')}",
            'semantic_navigation': semantic,
        }

    def _semantic_waypoint_to_pose(self, waypoint: dict) -> dict:
        return {
            'x': float(waypoint['x']),
            'y': float(waypoint['y']),
            'frame_id': waypoint.get('frame_id') or waypoint.get('frameId') or 'map',
            'yaw_rad': waypoint.get('yaw_rad', waypoint.get('yawRad')),
        }

    def _pose_to_bt_string(self, pose: dict) -> str:
        yaw_rad = float(pose.get('yaw_rad') or 0.0)
        # Nav2 pose strings use quaternion x;y;z;w after position. For planar
        # room navigation only yaw is relevant.
        half_yaw = yaw_rad / 2.0
        z = math.sin(half_yaw)
        w = math.cos(half_yaw)
        return f'0;{pose["frame_id"]};{pose["x"]};{pose["y"]};0;0;0;{z};{w}'

    def _build_semantic_navigation_rewritten_input(self, semantic: dict) -> str:
        destination_label = semantic.get('destination_label') or semantic.get('destination_id') or 'destination'
        start_node_id = semantic.get('start_node_id', 'unknown')
        target_node_id = semantic.get('target_node_id', 'unknown')
        topological_path = semantic.get('topological_path') or []
        waypoints = [
            self._semantic_waypoint_to_pose(waypoint)
            for waypoint in semantic.get('waypoints', [])
        ]

        waypoint_lines = []
        for index, waypoint in enumerate(waypoints, start=1):
            pose_value = self._pose_to_bt_string(waypoint)
            waypoint_lines.append(
                f'- waypoint_{index}: goal="{pose_value}" path="{{path_{index}}}"'
            )

        return '\n'.join([
            'Structure: Sequence',
            'Actions: ComputePathToPose, FollowPath',
            f'Task: Navigate to the resolved semantic destination "{destination_label}".',
            f'Route source: {semantic.get("planner", "external-route-planner")}.',
            f'Topological route: {" -> ".join(topological_path)}.',
            f'Start node: {start_node_id}.',
            f'Target node: {target_node_id}.',
            'Use only the metric waypoints below. Do not invent rooms, goals, coordinates, object actions, recovery actions, or additional navigation targets.',
            'For each waypoint, generate a ComputePathToPose action immediately followed by a FollowPath action using the same path key.',
            *waypoint_lines,
        ])

    def _build_semantic_navigation_instruction(self) -> str:
        return (
            'Generate a BehaviorTree XML for a resolved room-navigation task. '
            'The route has already been computed by an external topological route planner. '
            'Use only ComputePathToPose and FollowPath actions. '
            'For every provided waypoint, first compute a path to the exact goal string, '
            'then follow that path. Copy each goal attribute exactly as provided, '
            'including the leading timestamp/frame fields such as 0;map;x;y;0;0;0;z;w. '
            'Do not shorten, normalize, or reinterpret goal strings. '
            'Do not add DetectObject, PickObject, PlaceObject, '
            'Spin, BackUp, Wait, conditions, extra goals, or recovery branches. '
            'Output only valid XML with BTCPP_format="4".'
        )

    def _semantic_navigation_expected_goals(self, semantic: dict) -> list[str]:
        return [
            self._pose_to_bt_string(self._semantic_waypoint_to_pose(waypoint))
            for waypoint in semantic.get('waypoints', [])
        ]

    def _get_bt_node_id(self, element: ET.Element) -> str:
        return element.tag if element.tag != 'Action' else element.get('ID', element.tag)

    def _validate_semantic_navigation_bt(self, bt_xml: str, semantic: dict) -> tuple[bool, Optional[str]]:
        try:
            root = ET.fromstring(bt_xml)
        except ET.ParseError as e:
            return False, f'Invalid XML: {e}'

        expected_goals = self._semantic_navigation_expected_goals(semantic)
        compute_nodes = [
            element
            for element in root.iter()
            if self._get_bt_node_id(element) == 'ComputePathToPose'
        ]
        follow_nodes = [
            element
            for element in root.iter()
            if self._get_bt_node_id(element) == 'FollowPath'
        ]

        if len(compute_nodes) != len(expected_goals):
            return False, (
                f'Expected {len(expected_goals)} ComputePathToPose node(s), '
                f'got {len(compute_nodes)}'
            )
        if len(follow_nodes) < len(expected_goals):
            return False, (
                f'Expected at least {len(expected_goals)} FollowPath node(s), '
                f'got {len(follow_nodes)}'
            )

        actual_goals = [node.get('goal') for node in compute_nodes]
        for index, (actual_goal, expected_goal) in enumerate(zip(actual_goals, expected_goals), start=1):
            if actual_goal != expected_goal:
                return False, (
                    f'Waypoint {index} goal mismatch. '
                    f'Expected "{expected_goal}", got "{actual_goal}"'
                )

        return True, None

    def _build_resolved_route_navigation_bt(self, semantic: dict) -> str:
        destination_label = _escape_xml_attr(
            semantic.get('destination_label') or semantic.get('destination_id') or 'Destination'
        )
        waypoints = [
            self._semantic_waypoint_to_pose(waypoint)
            for waypoint in semantic.get('waypoints', [])
        ]

        sequence_lines = [f'      <Sequence name="Navigate to {destination_label}">']
        multiple_waypoints = len(waypoints) > 1
        for index, pose in enumerate(waypoints, start=1):
            pose_value = _escape_xml_attr(self._pose_to_bt_string(pose))
            path_key = f'{{path_{index}}}'
            plan_name = (
                f'Plan path to {destination_label} waypoint {index}'
                if multiple_waypoints
                else f'Plan path to {destination_label}'
            )
            move_name = (
                f'Move to {destination_label} waypoint {index}'
                if multiple_waypoints
                else f'Move to {destination_label}'
            )
            sequence_lines.append(
                f'        <ComputePathToPose name="{plan_name}" goal="{pose_value}" path="{path_key}" planner_id="GridBased"/>'
            )
            sequence_lines.append(
                f'        <FollowPath name="{move_name}" path="{path_key}" controller_id="FollowPath"/>'
            )
        sequence_lines.append('      </Sequence>')

        xml_lines = [
            '<root BTCPP_format="4" main_tree_to_execute="MainTree">',
            '  <BehaviorTree ID="MainTree">',
            *sequence_lines,
            '  </BehaviorTree>',
            '</root>',
        ]
        return '\n'.join(xml_lines)

    async def generate_bt_from_semantic_navigation(self, payload: dict) -> tuple[Optional[str], Optional[str]]:
        semantic = payload['semantic_navigation']
        command = payload['command']
        rewritten_input = self._build_semantic_navigation_rewritten_input(semantic)
        custom_instruction = self._build_semantic_navigation_instruction()

        self.get_logger().info(
            'Generating semantic navigation BT through inference server: '
            f'{semantic.get("start_node_id")} -> {semantic.get("target_node_id")} '
            f'({len(semantic.get("waypoints", []))} waypoint(s))'
        )

        try:
            response = requests.post(
                f'{self.inference_url}/generate_bt',
                json={
                    'command': command,
                    'max_tokens': 1024,
                    'temperature': 0.0,
                    'prompt_format': 'alpaca',
                    'use_query_rewriting': False,
                    'rewritten_input': rewritten_input,
                    'custom_instruction': custom_instruction,
                },
                timeout=self.generation_timeout
            )
            if response.status_code == 200:
                data = response.json()
                bt_xml = data.get('bt_xml')
                if data.get('success', False) and bt_xml:
                    is_valid_semantic_bt, validation_error = self._validate_semantic_navigation_bt(bt_xml, semantic)
                    if not is_valid_semantic_bt:
                        self.get_logger().warn(
                            'Semantic navigation LLM output is not executable; '
                            'compiling resolved route in backend: '
                            f'{validation_error}'
                        )
                        return self._build_resolved_route_navigation_bt(semantic), None

                    self.get_logger().info('Semantic navigation BT generated by inference server')
                    return bt_xml, None

                self.get_logger().warn(
                    f'Semantic navigation LLM generation failed; '
                    f'compiling resolved route in backend: '
                    f'{data.get("error", "Unknown error")}'
                )
            else:
                self.get_logger().warn(
                    f'Semantic navigation inference server HTTP {response.status_code}, '
                    'compiling resolved route in backend'
                )
        except requests.Timeout:
            self.get_logger().warn(
                'Semantic navigation inference server timeout; '
                'compiling resolved route in backend'
            )
        except requests.ConnectionError:
            self.get_logger().warn(
                'Semantic navigation inference server unavailable; '
                'compiling resolved route in backend'
            )
        except Exception as e:
            self.get_logger().warn(
                f'Semantic navigation generation error; '
                f'compiling resolved route in backend: {e}'
            )

        return self._build_resolved_route_navigation_bt(semantic), None

    async def generate_bt_from_command(self, command: str) -> tuple[Optional[str], Optional[str]]:
        try:
            semantic_payload = self._parse_semantic_navigation_command(command)
        except ValueError as e:
            return None, str(e)

        if semantic_payload:
            return await self.generate_bt_from_semantic_navigation(semantic_payload)

        try:
            response = requests.post(
                f'{self.inference_url}/generate_bt',
                json={
                    'command': command,
                    'max_tokens': 1024,
                    'temperature': 0.0,
                    'prompt_format': 'alpaca',
                    'use_query_rewriting': True,
                    'scene_graph_context': self._known_entities or None,
                },
                timeout=self.generation_timeout
            )
            if response.status_code != 200:
                return None, f'HTTP {response.status_code}: {response.text}'
            data = response.json()
            if not data.get('success', False):
                return None, data.get('error', 'Unknown error')
            bt_xml = data.get('bt_xml')
            if bt_xml:
                bt_xml = self._ground_generated_semantic_types(bt_xml)
            return bt_xml, None
        except requests.Timeout:
            return None, 'Inference server timeout'
        except requests.ConnectionError:
            return None, 'Could not connect to inference server'
        except Exception as e:
            return None, f'Request failed: {str(e)}'

    @staticmethod
    def _semantic_node_id(element: ET.Element) -> str:
        if element.tag == 'Action':
            return element.get('ID', '')
        return element.tag

    @staticmethod
    def _pose_message_to_bt_string(pose_stamped) -> str:
        pose = pose_stamped.pose
        frame_id = pose_stamped.header.frame_id or 'map'
        return (
            f'0;{frame_id};'
            f'{pose.position.x:.9g};{pose.position.y:.9g};{pose.position.z:.9g};'
            f'{pose.orientation.x:.9g};{pose.orientation.y:.9g};'
            f'{pose.orientation.z:.9g};{pose.orientation.w:.9g}'
        )

    @staticmethod
    def _replace_child(
        parent: ET.Element,
        old_child: ET.Element,
        new_child: ET.Element,
    ) -> None:
        children = list(parent)
        index = children.index(old_child)
        parent.remove(old_child)
        parent.insert(index, new_child)

    def _compile_resolved_semantic_node(
        self,
        semantic_node: ET.Element,
        response,
        route_index: int,
    ) -> ET.Element:
        destination = response.destination_label or semantic_node.get(
            'entity_ref', 'destination'
        )
        sequence = ET.Element(
            'Sequence',
            {
                'name': f'Navigate to remembered {destination}',
                '_semantic_destination_id': response.destination_id,
                '_semantic_graph_version': str(response.graph_version),
            },
        )
        for waypoint_index, waypoint in enumerate(response.waypoints, start=1):
            path_key = f'{{semantic_path_{route_index}_{waypoint_index}}}'
            goal = self._pose_message_to_bt_string(waypoint)
            ET.SubElement(
                sequence,
                'ComputePathToPose',
                {
                    'name': f'Plan semantic route {route_index}.{waypoint_index}',
                    'goal': goal,
                    'path': path_key,
                    'planner_id': 'GridBased',
                },
            )
            follow_path_attributes = {
                'name': f'Follow semantic route {route_index}.{waypoint_index}',
                'path': path_key,
                'controller_id': 'FollowPath',
            }
            if response.entity_type == 'object':
                # Manipulation depends on the base actually facing the object;
                # the relaxed general navigation tolerance can leave it near
                # the edge of the arm workspace even when XY is acceptable.
                follow_path_attributes['goal_checker_id'] = (
                    'manipulation_goal_checker'
                )
            ET.SubElement(
                sequence,
                'FollowPath',
                follow_path_attributes,
            )
        return sequence

    async def _resolve_semantic_node(
        self,
        semantic_node: ET.Element,
        goal_handle,
        queue_deadline: float,
    ):
        entity_ref = semantic_node.get('entity_ref', '').strip()
        if not entity_ref:
            return None, 'NavigateSemantic is missing required entity_ref'

        last_queue_message = ''
        discovery_attempted = False
        discovery_resolution_deadline = 0.0
        while time.monotonic() < queue_deadline:
            if goal_handle.is_cancel_requested:
                return None, 'PREEMPTED'

            if not self._semantic_resolver_client.service_is_ready():
                queue_message = (
                    f'Waiting for the Hydra adapter before resolving {entity_ref}'
                )
            else:
                request = ResolveSemanticNavigation.Request()
                request.entity_ref = entity_ref
                ref_type = (
                    entity_ref.split(':', 1)[0].strip().lower()
                    if ':' in entity_ref
                    else ''
                )
                request.entity_type = (
                    ref_type
                    if ref_type in {'room', 'object'}
                    else semantic_node.get('entity_type', '')
                )
                request.label = semantic_node.get('label', '')
                request.preferred_id = semantic_node.get('preferred_id', '')
                request.allow_stale = (
                    semantic_node.get('allow_stale', 'false').lower() == 'true'
                )
                try:
                    response = await self._wait_for_rclpy_future(
                        self._semantic_resolver_client.call_async(request),
                        self.semantic_resolver_call_timeout,
                    )
                except TimeoutError:
                    queue_message = (
                        f'Hydra resolver timed out after '
                        f'{self.semantic_resolver_call_timeout:.0f}s for '
                        f'{entity_ref}'
                    )
                except Exception as exc:
                    queue_message = (
                        f'Hydra resolver call failed for {entity_ref}: {exc}'
                    )
                else:
                    if response.success:
                        if not response.waypoints:
                            return None, (
                                f'Hydra resolved {entity_ref} without navigation '
                                'waypoints'
                            )
                        return response, None
                    fallback_statuses = {
                        ResolveSemanticNavigation.Request.STATUS_UNKNOWN_DESTINATION,
                        ResolveSemanticNavigation.Request.STATUS_STALE_DESTINATION,
                    }
                    is_object_fallback = (
                        response.status in fallback_statuses
                        and entity_ref.lower().startswith('object:')
                    )
                    if (
                        is_object_fallback
                        and not discovery_attempted
                        and not self._semantic_discovery_client.service_is_ready()
                    ):
                        queue_message = (
                            'Waiting for the single-view detector before checking '
                            f'{entity_ref}'
                        )
                    elif is_object_fallback and not discovery_attempted:
                        discovery_attempted = True
                        discovery_request = DiscoverSemanticObject.Request()
                        discovery_request.label = entity_ref.split(':', 1)[1]
                        discovery_request.box_threshold = 0.45
                        discovery_request.max_keyframes = 1
                        discovery_request.max_duration_sec = min(
                            self.semantic_discovery_search_timeout,
                            max(0.0, self.semantic_discovery_call_timeout - 5.0),
                        )
                        discovery_request.max_observations = 1
                        self.publish_feedback(
                            goal_handle,
                            'grounding',
                            0.25,
                            f'Checking the current camera view once for {entity_ref}',
                        )
                        try:
                            discovery = await self._wait_for_rclpy_future(
                                self._semantic_discovery_client.call_async(
                                    discovery_request
                                ),
                                self.semantic_discovery_call_timeout,
                            )
                        except TimeoutError:
                            return None, (
                                f'free-text discovery timed out after '
                                f'{self.semantic_discovery_call_timeout:.0f}s '
                                f'for {entity_ref}'
                            )
                        if not discovery.success:
                            return None, (
                                discovery.error_message
                                or response.error_message
                            )
                        if discovery.detected_pose.header.frame_id:
                            semantic_node.set(
                                '_live_object_pose',
                                self._pose_message_to_bt_string(
                                    discovery.detected_pose
                                ),
                            )
                        self.get_logger().info(
                            f'Added one live-view observation for {entity_ref}'
                        )
                        # Bound the cross-container DDS handoff by observing
                        # actual resolver success, not by assuming a latency.
                        discovery_resolution_deadline = time.monotonic() + 5.0
                        await self._wall_sleep(0.1)
                        continue
                    elif (
                        is_object_fallback
                        and discovery_attempted
                        and time.monotonic() < discovery_resolution_deadline
                    ):
                        queue_message = (
                            'Waiting for Hydra to index the live observation for '
                            f'{entity_ref}'
                        )
                    elif (
                        response.status
                        != ResolveSemanticNavigation.Request.STATUS_GRAPH_NOT_READY
                    ):
                        return None, (
                            response.error_message
                            or f'could not resolve {entity_ref}'
                        )
                    else:
                        queue_message = (
                            response.error_message
                            or f'Hydra is still mapping before resolving {entity_ref}'
                        )

            if queue_message != last_queue_message:
                self.get_logger().info(
                    f'Queued semantic command: {queue_message}'
                )
                last_queue_message = queue_message
            remaining = max(0.0, queue_deadline - time.monotonic())
            self.publish_feedback(
                goal_handle,
                'queued',
                0.25,
                f'{queue_message}; waiting up to {remaining:.0f}s',
            )
            await self._wall_sleep(self.semantic_queue_poll_period)

        return None, (
            f'timed out after {self.semantic_queue_timeout:.0f}s waiting for '
            f'Hydra to resolve {entity_ref}'
        )

    async def compile_semantic_actions(
        self,
        xml_string: str,
        goal_handle,
    ) -> tuple[Optional[str], Optional[str]]:
        """Compile model-level semantic actions into executable Nav2 actions.

        ``NavigateSemantic`` deliberately is not a BehaviorTree.CPP plugin. It is
        a constrained-generation token whose entity reference is resolved against
        the official Hydra DSG immediately before execution.
        """
        try:
            root = ET.fromstring(xml_string)
        except ET.ParseError as exc:
            return None, f'invalid generated XML: {exc}'

        semantic_nodes: list[tuple[ET.Element, ET.Element]] = []
        for parent in root.iter():
            for child in list(parent):
                if self._semantic_node_id(child) == 'NavigateSemantic':
                    semantic_nodes.append((parent, child))

        if not semantic_nodes:
            return xml_string, None

        queue_deadline = time.monotonic() + self.semantic_queue_timeout
        for route_index, (parent, semantic_node) in enumerate(
            semantic_nodes,
            start=1,
        ):
            response, error = await self._resolve_semantic_node(
                semantic_node,
                goal_handle,
                queue_deadline,
            )
            if response is None:
                return None, error
            live_object_pose = semantic_node.attrib.pop(
                '_live_object_pose',
                '',
            )
            reused_live_pose = supply_live_pose_to_following_pick(
                root,
                semantic_node,
                live_object_pose,
            )
            if reused_live_pose:
                self.get_logger().info(
                    'Reusing the navigation-time object pose for PickObject '
                    'because reacquire=false'
                )
            elif live_object_pose and reacquisition_requested(semantic_node):
                self.get_logger().info(
                    'Discarding the navigation-time object pose; PickObject '
                    'will reacquire from the final manipulation standoff'
                )
            if (
                response.entity_type == 'object'
                and response.destination_pose.header.frame_id
            ):
                document_order = list(root.iter())
                semantic_index = document_order.index(semantic_node)
                for candidate in document_order[semantic_index + 1:]:
                    candidate_id = (
                        candidate.get('ID')
                        if candidate.tag == 'Action'
                        else candidate.tag
                    )
                    if candidate_id == 'NavigateSemantic':
                        break
                    if candidate_id == 'PlaceObject':
                        candidate.set(
                            'place_pose',
                            self._pose_message_to_bt_string(
                                response.destination_pose
                            ),
                        )
                        self.get_logger().info(
                            'Using Hydra support geometry for PlaceObject; '
                            'no close-range detection will run'
                        )
                        break
            compiled = self._compile_resolved_semantic_node(
                semantic_node,
                response,
                route_index,
            )
            self._replace_child(parent, semantic_node, compiled)
            self.get_logger().info(
                f'Compiled {semantic_node.get("entity_ref")} via Hydra '
                f'node {response.destination_id} into '
                f'{len(response.waypoints)} Nav2 waypoint(s)'
            )

        return ET.tostring(root, encoding='unicode'), None

    def validate_bt_xml(self, xml_string: str) -> tuple[bool, Optional[str]]:
        try:
            root = ET.fromstring(xml_string)
            if root.tag != 'root': return False, f"Root element must be 'root', got '{root.tag}'"
            if root.get('BTCPP_format') != '4': return False, "BTCPP_format must be '4'"
            if len(root.findall('BehaviorTree')) == 0: return False, 'No BehaviorTree element found'
            return True, None
        except Exception as e:
            return False, f'Validation error: {str(e)}'

    def add_uids_for_foxglove(self, xml_string: str) -> str:
        """Add unique _uid attributes to all nodes for Foxglove Polymath BT panel visualization.
        This is only used for publishing to the topic, not for the file written to disk."""
        try:
            root = ET.fromstring(xml_string)
            node_counts = {}  # Track counts per node type for uniqueness

            def add_uid_recursive(element):
                """Recursively add _uid to element and all children."""
                tag = element.tag

                if tag == 'root':
                    uid = 'root'
                elif tag == 'BehaviorTree':
                    tree_id = element.get('ID', 'Tree')
                    uid = f'BehaviorTree_{tree_id}'
                else:
                    name = element.get('name')
                    node_id = element.get('ID')  # For explicit syntax <Action ID="..."/>

                    if name:
                        base = f'{tag}_{name}'
                    elif node_id:
                        base = f'{tag}_{node_id}'
                    elif tag in ('Action', 'Condition'):
                        base = tag
                    else:
                        base = tag

                    # Add distinguishing attribute for certain nodes
                    if tag == 'DetectObject':
                        obj = element.get('object_description', '')
                        if obj:
                            base = f'{tag}_{obj}'
                    elif tag == 'PickObject':
                        obj = element.get('object_description', '')
                        if obj:
                            base = f'{tag}_{obj}'
                    elif tag == 'PlaceObject':
                        desc = element.get('place_description', '')
                        if desc:
                            base = f'{tag}_{desc}'
                    elif tag in ('SpinLeft', 'SpinRight'):
                        dist = element.get('spin_dist', '')
                        if dist:
                            base = f'{tag}_{dist}rad'
                    elif tag == 'Wait':
                        dur = element.get('wait_duration', '')
                        if dur:
                            base = f'{tag}_{dur}s'
                    elif tag == 'Repeat':
                        cycles = element.get('num_cycles', '')
                        if cycles:
                            base = f'{tag}_{cycles}x'

                    base = base.replace(' ', '_').replace('"', '').replace("'", '')
                    node_counts[base] = node_counts.get(base, 0) + 1
                    count = node_counts[base]
                    uid = f'{base}_{count}' if count > 1 else base

                element.set('_uid', uid)
                for child in element:
                    add_uid_recursive(child)

            add_uid_recursive(root)
            return ET.tostring(root, encoding='unicode', xml_declaration=True)
        except ET.ParseError:
            return xml_string

    def write_bt_file(self, xml_content: str) -> Path:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = self.bt_output_dir / f'bt_{timestamp}_{uuid.uuid4().hex[:8]}.xml'
        with open(file_path, 'w') as f: f.write(xml_content)
        file_path.chmod(0o644)
        return file_path

    def _extract_initial_nav_pose_from_bt(self, bt_file_path: Path) -> Optional[dict]:
        try:
            root = ET.parse(bt_file_path).getroot()
        except Exception as e:
            self.get_logger().warn(f'Unable to parse BT goal for action pose: {e}')
            return None

        compute_goals = [
            element.get('goal')
            for element in root.iter()
            if self._get_bt_node_id(element) == 'ComputePathToPose'
        ]
        compute_goals = [goal for goal in compute_goals if goal]
        if not compute_goals:
            return None

        initial_goal = compute_goals[0]
        parts = initial_goal.split(';')
        if len(parts) != 9:
            self.get_logger().warn(f'Unexpected BT goal format: {initial_goal}')
            return None

        try:
            return {
                'frame_id': parts[1] or 'map',
                'x': float(parts[2]),
                'y': float(parts[3]),
                'z': float(parts[4]),
                'qx': float(parts[5]),
                'qy': float(parts[6]),
                'qz': float(parts[7]),
                'qw': float(parts[8]),
            }
        except ValueError as e:
            self.get_logger().warn(f'Invalid numeric BT goal value: {e}')
            return None

    async def execute_bt(
        self,
        bt_file_path: Path,
        goal_handle,
    ) -> tuple[bool, Optional[str]]:
        try:
            if not self._nav_client.wait_for_server(timeout_sec=self.nav_server_wait_timeout):
                return False, f'Nav2 action server not available: {self.nav_action_name}'

            nav_goal = NavigateToPose.Goal()
            nav_goal.behavior_tree = str(bt_file_path.absolute())
            nav_goal.pose.header.frame_id = 'map'
            # Dynamic object tasks compute their real goal inside the BT. Keep
            # the mandatory action-level placeholder a valid planar pose.
            nav_goal.pose.pose.orientation.w = 1.0
            initial_pose = self._extract_initial_nav_pose_from_bt(bt_file_path)
            if initial_pose:
                nav_goal.pose.header.frame_id = initial_pose['frame_id']
                nav_goal.pose.pose.position.x = initial_pose['x']
                nav_goal.pose.pose.position.y = initial_pose['y']
                nav_goal.pose.pose.position.z = initial_pose['z']
                nav_goal.pose.pose.orientation.x = initial_pose['qx']
                nav_goal.pose.pose.orientation.y = initial_pose['qy']
                nav_goal.pose.pose.orientation.z = initial_pose['qz']
                nav_goal.pose.pose.orientation.w = initial_pose['qw']
                self.get_logger().info(
                    'Using initial BT waypoint as Nav2 action goal: '
                    f'{initial_pose["frame_id"]} '
                    f'({initial_pose["x"]:.2f}, {initial_pose["y"]:.2f})'
                )
            
            send_goal_future = self._nav_client.send_goal_async(nav_goal)
            start_wait = time.time()
            while not send_goal_future.done() and (time.time() - start_wait) < 10.0:
                time.sleep(0.05)

            if not send_goal_future.done():
                return False, 'Timed out waiting for Nav2 to accept the BT goal'
            self.current_nav_goal_handle = send_goal_future.result()
            if (
                self.current_nav_goal_handle is None
                or not self.current_nav_goal_handle.accepted
            ):
                return False, 'Navigation goal rejected by Nav2'

            get_result_future = self.current_nav_goal_handle.get_result_async()
            start_time = self.get_clock().now()
            last_feedback_time = start_time

            while not get_result_future.done():
                # Uscita pulita senza loggare come errore critico
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Ricevuto stop/preemption, fermo Nav2...')
                    self.current_nav_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    return False, 'PREEMPTED'

                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > self.execution_timeout:
                    self.current_nav_goal_handle.cancel_goal_async()
                    return False, f'Execution timeout after {self.execution_timeout}s'

                current_time = self.get_clock().now()
                if (current_time - last_feedback_time).nanoseconds / 1e9 >= (1.0 / self.feedback_rate):
                    progress = min(0.5 + (elapsed / self.execution_timeout) * 0.5, 0.99)
                    self.publish_feedback(goal_handle, 'executing', progress, f'Executing BT... ({int(elapsed)}s elapsed)')
                    last_feedback_time = current_time

                time.sleep(0.1)

            result = get_result_future.result()
            if result.status == 4:
                return True, None
            else:
                return False, f'Navigation failed with status: {NAV_STATUS_NAMES.get(result.status, result.status)}'

        except Exception as e:
            return False, f'Execution error: {str(e)}'

    def publish_feedback(self, goal_handle, status: str, progress: float, step: str):
        feedback = GenerateAndExecuteBT.Feedback()
        feedback.status, feedback.progress, feedback.current_step = status, progress, step
        goal_handle.publish_feedback(feedback)

        msg = String()
        msg.data = json.dumps({
            'status': status,
            'progress': progress,
            'current_step': step,
            'timestamp_ms': int(time.time() * 1000),
        })
        self._bt_execution_feedback_publisher.publish(msg)

    def command_topic_callback(self, msg: String):
        command = msg.data.strip()
        if not command: return

        if command.upper() in ["STOP", "STOP_EXECUTION"]:
            self.get_logger().warn("Ricevuto STOP dal frontend. Interrompo il task.")
            self._cancel_active_execution()
            if self.current_goal_handle:
                self.current_goal_handle.abort()
                self.current_goal_handle = None
            self._publish_zero_velocity()
            self.is_executing = False
            return

        if self.is_executing:
            self.get_logger().warn('Un task è già in esecuzione. Lo cancello per applicare la correzione...')
            self._cancel_active_execution()
            if self.current_goal_handle:
                self.current_goal_handle.abort()
                self.current_goal_handle = None
            self._publish_zero_velocity()
            self.is_executing = False
            time.sleep(0.2)

        goal_msg = GenerateAndExecuteBT.Goal(command=command)
        if self._self_client.wait_for_server(timeout_sec=2.0):
            send_goal_future = self._self_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self._goal_sent_callback)

    def _goal_sent_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.active_client_goal_handle = goal_handle
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')

    def emergency_stop_callback(self, request, response):
        if not self.is_executing:
            response.success, response.message = True, 'No active BT execution to stop'
            return response
        try:
            self._cancel_active_execution()
            if self.current_goal_handle:
                self.current_goal_handle.abort()
                self.current_goal_handle = None
            self._publish_zero_velocity()
            self.is_executing = False
            response.success, response.message = True, 'BT execution aborted successfully'
        except Exception as e:
            response.success, response.message = False, f'Emergency stop failed: {str(e)}'
        return response

    def _publish_zero_velocity(self):
        stop_msg = Twist()
        for _ in range(10):
            self._cmd_vel_nav_publisher.publish(stop_msg)
            self._cmd_vel_smoothed_publisher.publish(stop_msg)
            self._cmd_vel_publisher.publish(stop_msg)
            time.sleep(0.05)

    def _cancel_active_execution(self):
        cancel_futures = []

        if self.current_nav_goal_handle:
            cancel_futures.append(self.current_nav_goal_handle.cancel_goal_async())
            self.current_nav_goal_handle = None

        if self.active_client_goal_handle:
            cancel_futures.append(self.active_client_goal_handle.cancel_goal_async())
            self.active_client_goal_handle = None

        for future in cancel_futures:
            start_wait = time.time()
            while not future.done() and (time.time() - start_wait) < 0.5:
                time.sleep(0.05)

    def _republish_last_bt(self):
        if self.last_bt_xml:
            bt_msg = String(data=self.add_uids_for_foxglove(self.last_bt_xml))
            self._bt_xml_publisher.publish(bt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BTInterfaceNode()
    try: rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
