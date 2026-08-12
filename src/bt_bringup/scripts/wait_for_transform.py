#!/usr/bin/env python3
"""Launch barrier for a required TF relationship."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-frame", required=True)
    parser.add_argument("--source-frame", required=True)
    parser.add_argument("--timeout", type=float, default=300.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("transform_readiness")
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=True)
    deadline = time.monotonic() + max(1.0, args.timeout)
    node.get_logger().info(
        f"Waiting for TF {args.target_frame} -> {args.source_frame}"
    )
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if buffer.can_transform(
                args.target_frame,
                args.source_frame,
                Time(),
            ):
                node.get_logger().info(
                    f"TF {args.target_frame} -> {args.source_frame} is ready"
                )
                return 0
            time.sleep(0.1)

        node.get_logger().error(
            f"Timed out waiting for TF "
            f"{args.target_frame} -> {args.source_frame}"
        )
        return 1
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
