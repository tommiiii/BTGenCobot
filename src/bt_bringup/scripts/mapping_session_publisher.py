#!/usr/bin/env python3
"""Publish whether this runtime consumes a saved map or creates a new one."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class MappingSessionPublisher(Node):
    def __init__(self):
        super().__init__('mapping_session_publisher')
        self.declare_parameter('mapping_complete', False)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            Bool, '/mapping/exploration_complete', qos
        )
        self._message = Bool(
            data=bool(self.get_parameter('mapping_complete').value)
        )
        # Transient-local durability retains this session declaration for late
        # subscribers. Publish once so the explorer's eventual completion=True
        # cannot be overwritten by a periodic startup value.
        self._publish()

    def _publish(self):
        self._publisher.publish(self._message)


def main(args=None):
    rclpy.init(args=args)
    node = MappingSessionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
