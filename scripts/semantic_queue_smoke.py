#!/usr/bin/env python3
"""Exercise semantic-command queueing without leaving an orphaned action goal."""

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from btgencobot_interfaces.action import GenerateAndExecuteBT
from rclpy.action import ActionClient
from rclpy.node import Node


class SemanticQueueSmoke(Node):
    def __init__(self) -> None:
        super().__init__('semantic_queue_smoke')
        self.client = ActionClient(
            self,
            GenerateAndExecuteBT,
            '/generate_and_execute_bt',
        )
        self.queued = False

    def feedback(self, message) -> None:
        feedback = message.feedback
        self.get_logger().info(
            f'{feedback.status}: {feedback.current_step}'
        )
        if feedback.status == 'queued':
            self.queued = True


def spin_until(node: SemanticQueueSmoke, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main() -> int:
    rclpy.init()
    node = SemanticQueueSmoke()
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error('BT action server was not available')
            return 2

        goal = GenerateAndExecuteBT.Goal()
        goal.command = 'go to the kitchen'
        send_future = node.client.send_goal_async(
            goal,
            feedback_callback=node.feedback,
        )
        if not spin_until(node, send_future.done, 10.0):
            node.get_logger().error('Goal acceptance timed out')
            return 3

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            node.get_logger().error('BT action server rejected the goal')
            return 4

        if not spin_until(node, lambda: node.queued, 60.0):
            node.get_logger().error(
                'Command did not enter the Hydra mapping queue'
            )
            return 5

        cancel_future = goal_handle.cancel_goal_async()
        if not spin_until(node, cancel_future.done, 10.0):
            node.get_logger().error('Cancellation response timed out')
            return 6
        if not cancel_future.result().goals_canceling:
            node.get_logger().error('BT action server rejected cancellation')
            return 7

        result_future = goal_handle.get_result_async()
        if not spin_until(node, result_future.done, 10.0):
            node.get_logger().error('Canceled goal did not finish')
            return 8
        if result_future.result().status != GoalStatus.STATUS_CANCELED:
            node.get_logger().error(
                f'Unexpected final status: {result_future.result().status}'
            )
            return 9

        node.get_logger().info(
            'PASS: semantic command queued and canceled cleanly'
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
