"""ROS2 Action Server for BehaviorTree Generation and Execution"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import requests
import xml.etree.ElementTree as ET
import uuid
import re
from datetime import datetime
from pathlib import Path
from typing import Optional

# ROS messages
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from btgencobot_interfaces.action import GenerateAndExecuteBT


def rewrite_query(query: str, inference_url: str, timeout: float = 10.0) -> str:
    """
    Rewrite query using LLM-based rewriter on inference server.
    Returns raw query if rewriting fails.
    
    Args:
        query: Raw user query
        inference_url: URL of the inference server
        timeout: Request timeout in seconds
        
    Returns:
        Rewritten query, or original query if rewriting fails
    """
    try:
        # Call inference server's rewrite endpoint
        rewrite_url = f"{inference_url}/rewrite_query"
        response = requests.post(
            rewrite_url,
            json={"query": query},
            timeout=timeout
        )
        
        if response.status_code == 200:
            result = response.json()
            if result.get("success"):
                return result["rewritten_query"]
        
        # If server call failed, return original query
        print(f"Warning: Server rewriting failed (status {response.status_code}), using original query")
        return query
        
    except Exception as e:
        print(f"Warning: Could not reach inference server for rewriting: {e}, using original query")
        return query


class BTInterfaceNode(Node):
    """
    ROS2 Action Server for generating BehaviorTrees from natural language
    and executing them via Nav2
    """

    def __init__(self):
        super().__init__('bt_interface_node')

        # Parameters
        self.declare_parameter('inference_server_url', 'http://host.docker.internal:8080')
        self.declare_parameter('bt_output_dir', '/workspace/generated_bts')
        self.declare_parameter('generation_timeout', 30.0)  # seconds
        self.declare_parameter('execution_timeout', 300.0)  # seconds (increased for manipulation tasks)
        self.declare_parameter('feedback_rate', 2.0)  # Hz - how often to publish feedback

        self.inference_url = self.get_parameter('inference_server_url').value
        self.bt_output_dir = Path(self.get_parameter('bt_output_dir').value)
        self.generation_timeout = self.get_parameter('generation_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value
        self.feedback_rate = self.get_parameter('feedback_rate').value

        # Create output directory if it doesn't exist
        self.bt_output_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'BT output directory: {self.bt_output_dir}')
        self.get_logger().info(f'Inference server URL: {self.inference_url}')

        # Callback groups for concurrent execution
        self.action_callback_group = ReentrantCallbackGroup()
        
        # Publisher for BT XML (for visualization in Foxglove/RViz)
        # Use a dedicated topic for generated BTs, separate from Nav2's active BT
        self._bt_xml_publisher = self.create_publisher(
            String,
            '/generated_behavior_tree',
            10
        )

        # Action server for BT generation and execution
        self._action_server = ActionServer(
            self,
            GenerateAndExecuteBT,
            '/generate_and_execute_bt',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )

        # Action client for Nav2 navigation
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.action_callback_group
        )

        # Action client for self-invocation (topic->action bridge)
        self._self_client = ActionClient(
            self,
            GenerateAndExecuteBT,
            '/generate_and_execute_bt',
            callback_group=self.action_callback_group
        )

        # Emergency stop service
        self._emergency_stop_srv = self.create_service(
            Trigger,
            '/emergency_stop_bt',
            self.emergency_stop_callback
        )

        # Topic subscriber for NL commands (integrates with Foxglove)
        # Use VOLATILE durability for compatibility with both VOLATILE and TRANSIENT_LOCAL publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._command_subscriber = self.create_subscription(
            String,
            '/btgen_nl_command',
            self.command_topic_callback,
            qos_profile
        )

        # State tracking
        self.current_goal_handle = None
        self.current_nav_goal_handle = None
        self.is_executing = False

        self.get_logger().info('BT Interface Node initialized')
        self.get_logger().info('Action server: /generate_and_execute_bt')
        self.get_logger().info('Topic subscriber: /btgen_nl_command')
        self.get_logger().info('Emergency stop service: /emergency_stop_bt')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info(f'Received goal request: {goal_request.command}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the action: generate BT from command and execute it
        """
        self.get_logger().info('=' * 80)
        self.get_logger().info('Executing BT generation and execution action')
        self.get_logger().info('=' * 80)

        self.current_goal_handle = goal_handle
        self.is_executing = True

        command = goal_handle.request.command
        self.get_logger().info(f'Original command: {command}')
        
        # Rewrite query for better BT generation
        command_rewritten = rewrite_query(command, self.inference_url)
        if command != command_rewritten:
            self.get_logger().info(f'Rewritten command: {command_rewritten}')
        command = command_rewritten

        # Initialize result
        result = GenerateAndExecuteBT.Result()
        result.success = False
        result.bt_xml_path = ''
        result.error_message = ''

        try:
            # Step 1: Generate BT from command
            self.publish_feedback(goal_handle, 'generating', 0.1, 'Calling inference server...')

            bt_xml, error = await self.generate_bt_from_command(command)

            if bt_xml is None:
                result.error_message = f'BT generation failed: {error}'
                self.get_logger().error(result.error_message)
                self.is_executing = False
                return result

            self.get_logger().info('✓ BT generation successful')

            # Step 2: Validate BT XML
            self.publish_feedback(goal_handle, 'validating', 0.3, 'Validating generated BT...')

            is_valid, val_error = self.validate_bt_xml(bt_xml)

            if not is_valid:
                result.error_message = f'BT validation failed: {val_error}'
                self.get_logger().error(result.error_message)
                self.is_executing = False
                return result

            self.get_logger().info('✓ BT validation successful')

            # Step 3: Write BT to file
            self.publish_feedback(goal_handle, 'validating', 0.4, 'Writing BT to file...')

            bt_file_path = self.write_bt_file(bt_xml)
            result.bt_xml_path = str(bt_file_path)

            self.get_logger().info(f'✓ BT written to: {bt_file_path}')
            
            # Publish BT XML to dedicated topic for visualization
            bt_msg = String()
            bt_msg.data = bt_xml
            self._bt_xml_publisher.publish(bt_msg)
            self.get_logger().info('✓ BT published to /generated_behavior_tree topic')

            # Step 4: Execute BT with Nav2
            self.publish_feedback(goal_handle, 'executing', 0.5, 'Executing BehaviorTree...')

            execution_success, exec_error = await self.execute_bt(bt_file_path, goal_handle)

            if not execution_success:
                result.error_message = f'BT execution failed: {exec_error}'
                self.get_logger().error(result.error_message)
                self.is_executing = False
                return result

            # Success!
            self.get_logger().info('✓ BT execution completed successfully')
            self.publish_feedback(goal_handle, 'completed', 1.0, 'BT execution completed')

            result.success = True
            goal_handle.succeed()

        except Exception as e:
            result.error_message = f'Unexpected error: {str(e)}'
            self.get_logger().error(result.error_message)
            self.get_logger().error(f'Exception: {e}', exc_info=True)
            goal_handle.abort()

        finally:
            self.is_executing = False
            self.current_goal_handle = None

        self.get_logger().info('=' * 80)
        return result

    async def generate_bt_from_command(self, command: str) -> tuple[Optional[str], Optional[str]]:
        """
        Call the inference server to generate BT XML from natural language command

        Args:
            command: Natural language command

        Returns:
            (bt_xml, error_message) - bt_xml is None if generation failed
        """
        try:
            self.get_logger().info(f'Calling inference server: {self.inference_url}/generate_bt')

            # Make HTTP POST request
            # IMPORTANT: Use Alpaca format with few-shot examples as the model was trained on this format
            response = requests.post(
                f'{self.inference_url}/generate_bt',
                json={
                    'command': command,
                    'max_tokens': 1024,
                    'temperature': 0.9,  # Higher temp for more diverse outputs (paper uses 0.9)
                    'use_few_shot': True,  # Model was trained with few-shot examples
                    'prompt_format': 'alpaca',  # Model was trained in Alpaca format
                    'use_query_rewriter': False  # DISABLED: Query rewriter adds complexity and wrong descriptions
                },
                timeout=self.generation_timeout
            )

            if response.status_code != 200:
                return None, f'HTTP {response.status_code}: {response.text}'

            data = response.json()

            if not data.get('success', False):
                return None, data.get('error', 'Unknown error')

            bt_xml = data.get('bt_xml')
            method = data.get('method_used')
            gen_time_ms = data.get('generation_time_ms')

            self.get_logger().info(f'Generation method: {method}, time: {gen_time_ms}ms')

            return bt_xml, None

        except requests.Timeout:
            return None, 'Inference server timeout'
        except requests.ConnectionError:
            return None, 'Could not connect to inference server'
        except Exception as e:
            return None, f'Request failed: {str(e)}'

    def validate_bt_xml(self, xml_string: str) -> tuple[bool, Optional[str]]:
        """
        Validate BehaviorTree XML structure

        Args:
            xml_string: BT XML content

        Returns:
            (is_valid, error_message)
        """
        try:
            root = ET.fromstring(xml_string)

            # Check root element
            if root.tag != 'root':
                return False, f"Root element must be 'root', got '{root.tag}'"

            # Check BTCPP_format
            btcpp_format = root.get('BTCPP_format')
            if btcpp_format != '4':
                return False, f"BTCPP_format must be '4', got '{btcpp_format}'"

            # Check for BehaviorTree elements
            behavior_trees = root.findall('BehaviorTree')
            if len(behavior_trees) == 0:
                return False, 'No BehaviorTree element found'

            # Validate each tree has an ID
            for bt in behavior_trees:
                tree_id = bt.get('ID')
                if not tree_id:
                    return False, 'BehaviorTree missing ID attribute'

                if len(list(bt)) == 0:
                    return False, f"BehaviorTree '{tree_id}' is empty"

            return True, None

        except ET.ParseError as e:
            return False, f'XML parse error: {str(e)}'
        except Exception as e:
            return False, f'Validation error: {str(e)}'

    def write_bt_file(self, xml_content: str) -> Path:
        """
        Write BT XML to file with UUID naming

        Args:
            xml_content: BT XML string

        Returns:
            Path to written file
        """
        # Generate filename with timestamp and UUID
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        unique_id = uuid.uuid4().hex[:8]
        filename = f'bt_{timestamp}_{unique_id}.xml'

        file_path = self.bt_output_dir / filename

        # Write file
        with open(file_path, 'w') as f:
            f.write(xml_content)

        # Ensure readable by Nav2
        file_path.chmod(0o644)

        return file_path

    async def execute_bt(self, bt_file_path: Path, goal_handle) -> tuple[bool, Optional[str]]:
        """
        Execute BT by calling Nav2's navigate_to_pose action with behavior_tree parameter

        Args:
            bt_file_path: Path to BT XML file
            goal_handle: Goal handle for feedback

        Returns:
            (success, error_message)
        """
        try:
            # Wait for Nav2 action server
            self.get_logger().info('Waiting for Nav2 action server...')
            if not self._nav_client.wait_for_server(timeout_sec=5.0):
                return False, 'Nav2 action server not available'

            # Create NavigateToPose goal
            nav_goal = NavigateToPose.Goal()

            # Set behavior_tree parameter to our generated BT file
            nav_goal.behavior_tree = str(bt_file_path.absolute())

            # Note: pose is optional when using custom BT
            # The BT itself determines navigation goals
            nav_goal.pose.header.frame_id = 'map'
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(f'Sending NavigateToPose goal with BT: {bt_file_path}')

            # Send goal
            send_goal_future = self._nav_client.send_goal_async(nav_goal)

            # Wait for goal acceptance (executor handles spinning)
            import time
            start_wait = time.time()
            while not send_goal_future.done() and (time.time() - start_wait) < 5.0:
                time.sleep(0.05)  # Brief blocking sleep is OK since we're in a callback group thread

            if not send_goal_future.done():
                return False, 'Failed to send navigation goal (timeout)'

            self.current_nav_goal_handle = send_goal_future.result()

            if not self.current_nav_goal_handle.accepted:
                return False, 'Navigation goal rejected by Nav2'

            self.get_logger().info('Navigation goal accepted, waiting for result...')

            # Wait for result (executor handles spinning)
            import time
            get_result_future = self.current_nav_goal_handle.get_result_async()

            # Monitor progress and provide feedback
            start_time = self.get_clock().now()
            last_feedback_time = start_time
            feedback_interval = 1.0 / self.feedback_rate  # Convert Hz to seconds

            while not get_result_future.done():
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal cancelled, aborting navigation...')
                    self.current_nav_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    return False, 'Cancelled by user'

                # Check timeout
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > self.execution_timeout:
                    self.get_logger().warn(f'Execution timeout ({self.execution_timeout}s)')
                    self.current_nav_goal_handle.cancel_goal_async()
                    return False, f'Execution timeout after {self.execution_timeout}s'

                # Update feedback at controlled rate (e.g., 2 Hz instead of ~10 Hz)
                current_time = self.get_clock().now()
                time_since_last_feedback = (current_time - last_feedback_time).nanoseconds / 1e9

                if time_since_last_feedback >= feedback_interval:
                    progress = min(0.5 + (elapsed / self.execution_timeout) * 0.5, 0.99)
                    self.publish_feedback(
                        goal_handle,
                        'executing',
                        progress,
                        f'Executing BT... ({int(elapsed)}s elapsed)'
                    )
                    last_feedback_time = current_time

                # Brief blocking sleep is OK since we're in a callback group thread
                time.sleep(0.1)

            # Get result
            result = get_result_future.result()

            if result.status == 4:  # SUCCEEDED
                self.get_logger().info('Navigation completed successfully')
                return True, None
            else:
                status_names = {
                    1: 'UNKNOWN', 2: 'ACCEPTED', 3: 'EXECUTING',
                    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'
                }
                status_name = status_names.get(result.status, f'UNKNOWN({result.status})')
                return False, f'Navigation failed with status: {status_name}'

        except Exception as e:
            return False, f'Execution error: {str(e)}'

    def publish_feedback(self, goal_handle, status: str, progress: float, step: str):
        """Publish feedback to action client"""
        feedback = GenerateAndExecuteBT.Feedback()
        feedback.status = status
        feedback.progress = progress
        feedback.current_step = step

        goal_handle.publish_feedback(feedback)

        self.get_logger().info(f'[{status}] {progress*100:.0f}% - {step}')

    def command_topic_callback(self, msg: String):
        """
        Callback for /btgen_nl_command topic
        
        Receives NL command from topic (e.g., Foxglove Publish panel)
        and triggers the action server internally
        """
        command = msg.data.strip()
        
        if not command:
            self.get_logger().warn('Received empty command on /btgen_nl_command')
            return
        
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Received command from topic: {command}')
        if self.is_executing:
            self.get_logger().warn('Note: A command is currently executing. This command will be queued.')
        self.get_logger().info('=' * 80)
        
        # Create a goal and send it to the action server
        goal_msg = GenerateAndExecuteBT.Goal()
        goal_msg.command = command
        
        # Send goal asynchronously via action client (self-invocation)
        if not self._self_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server not available for self-invocation!')
            return
        
        self.get_logger().info('Sending goal to action server...')
        send_goal_future = self._self_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_sent_callback)
    
    def _goal_sent_callback(self, future):
        """Callback when goal is sent to action server"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Goal accepted by action server')
            else:
                self.get_logger().error('Goal rejected by action server!')
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')

    def emergency_stop_callback(self, request, response):
        """
        Emergency stop service callback

        Aborts current BT execution
        """
        self.get_logger().warn('=' * 80)
        self.get_logger().warn('EMERGENCY STOP REQUESTED')
        self.get_logger().warn('=' * 80)

        if not self.is_executing:
            response.success = True
            response.message = 'No active BT execution to stop'
            self.get_logger().info(response.message)
            return response

        try:
            # Cancel navigation goal if active
            if self.current_nav_goal_handle is not None:
                self.current_nav_goal_handle.cancel_goal_async()
                self.get_logger().info('Navigation goal cancelled')

            # Abort action goal if active
            if self.current_goal_handle is not None:
                self.current_goal_handle.abort()
                self.get_logger().info('Action goal aborted')

            self.is_executing = False
            response.success = True
            response.message = 'BT execution aborted successfully'

        except Exception as e:
            response.success = False
            response.message = f'Emergency stop failed: {str(e)}'
            self.get_logger().error(response.message)

        self.get_logger().warn('=' * 80)
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = BTInterfaceNode()

    # Use multi-threaded executor for concurrent action handling
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
