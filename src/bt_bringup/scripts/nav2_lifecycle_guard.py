#!/usr/bin/env python3
"""Recover Nav2 when its one-shot autostart races simulation startup.

The standard lifecycle manager leaves every managed node inactive when any
node fails its initial transition. In simulation, controller and sensor
discovery can briefly lag behind process startup, so a later retry is enough.
This guard uses Nav2's public lifecycle-manager services and exits as soon as
the complete navigation stack is active.
"""

import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_srvs.srv import Trigger


class Nav2LifecycleGuard(Node):
    def __init__(self) -> None:
        super().__init__('nav2_lifecycle_guard')
        self.declare_parameter('manager_name', '/lifecycle_manager_navigation')
        self.declare_parameter('initial_delay', 20.0)
        self.declare_parameter('retry_period', 10.0)
        self.declare_parameter('max_attempts', 12)

        manager_name = str(self.get_parameter('manager_name').value).rstrip('/')
        self.initial_delay = float(self.get_parameter('initial_delay').value)
        self.retry_period = float(self.get_parameter('retry_period').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)
        self.active_client = self.create_client(
            Trigger, f'{manager_name}/is_active'
        )
        self.manage_client = self.create_client(
            ManageLifecycleNodes, f'{manager_name}/manage_nodes'
        )

    def _call(self, client, request, timeout: float):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:  # transport errors are retryable here
            self.get_logger().warning(f'Lifecycle service call failed: {exc}')
            return None

    def _is_active(self) -> bool:
        response = self._call(self.active_client, Trigger.Request(), 10.0)
        return bool(response is not None and response.success)

    def ensure_active(self) -> bool:
        if self.initial_delay > 0.0:
            self.get_logger().info(
                f'Waiting {self.initial_delay:.0f}s for initial Nav2 autostart'
            )
            time.sleep(self.initial_delay)

        while rclpy.ok() and not self.manage_client.wait_for_service(5.0):
            self.get_logger().info('Waiting for Nav2 lifecycle manager')

        for attempt in range(1, self.max_attempts + 1):
            if self._is_active():
                self.get_logger().info('Nav2 lifecycle manager is active')
                return True

            self.get_logger().warning(
                f'Nav2 is inactive; requesting startup '
                f'(attempt {attempt}/{self.max_attempts})'
            )
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            response = self._call(self.manage_client, request, 120.0)
            if response is not None and response.success and self._is_active():
                self.get_logger().info('Nav2 startup recovery succeeded')
                return True

            if attempt < self.max_attempts:
                time.sleep(self.retry_period)

        self.get_logger().error('Nav2 remained inactive after bounded retries')
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2LifecycleGuard()
    try:
        success = node.ensure_active()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
