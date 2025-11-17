#!/usr/bin/env python3
"""
Send NavigateToPose goal with optional custom BT XML

Usage: 
    scripts/send_goal.py 2.0 1.5
    scripts/send_goal.py 2.0 1.5 --yaw 0.0
    scripts/send_goal.py 2.0 1.5 --bt robot_description/mobile_manipulator/behavior_trees/simple_navigation.xml

Examples:
    # Basic navigation
    scripts/send_goal.py 2.0 1.5
    
    # With specific orientation
    scripts/send_goal.py 2.0 1.5 --yaw 1.57
    
    # With custom BT
    scripts/send_goal.py 2.0 1.5 --bt robot_description/mobile_manipulator/behavior_trees/simple_navigation.xml
"""

import argparse
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def quaternion_from_yaw(yaw):
    """Convert yaw angle to quaternion (z-axis rotation only)"""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }


def main():
    parser = argparse.ArgumentParser(
        description='Send NavigateToPose goal with optional custom BT',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('x', type=float, help='Target x position')
    parser.add_argument('y', type=float, help='Target y position')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target yaw orientation in radians (default: 0.0)')
    parser.add_argument('--bt', type=str, default='', help='Path to custom BT XML file (optional)')
    parser.add_argument('--frame', type=str, default='map', help='Target frame (default: map)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    node = Node('nav_goal_sender')
    
    # Create action client
    action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    
    node.get_logger().info('Waiting for action server...')
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Action server /navigate_to_pose not available!')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    
    # Create goal message
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = args.frame
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    
    # Set position
    goal_msg.pose.pose.position.x = args.x
    goal_msg.pose.pose.position.y = args.y
    goal_msg.pose.pose.position.z = 0.0
    
    # Set orientation (quaternion from yaw)
    q = quaternion_from_yaw(args.yaw)
    goal_msg.pose.pose.orientation.x = q['x']
    goal_msg.pose.pose.orientation.y = q['y']
    goal_msg.pose.pose.orientation.z = q['z']
    goal_msg.pose.pose.orientation.w = q['w']
    
    # Set custom BT file path if provided
    if args.bt:
        # Nav2 expects absolute path in the behavior_tree field
        import os
        bt_path = os.path.abspath(args.bt)
        if not os.path.exists(bt_path):
            node.get_logger().error(f'BT file not found: {bt_path}')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        goal_msg.behavior_tree = bt_path
        node.get_logger().info(f'Using custom BT from: {bt_path}')
    else:
        goal_msg.behavior_tree = ''  # Use default BT from parameter
        node.get_logger().info('Using default BT from bt_navigator parameters')
    
    # Send goal
    node.get_logger().info(f'Sending goal: x={args.x}, y={args.y}, yaw={args.yaw}')
    future = action_client.send_goal_async(goal_msg)
    
    # Wait for goal acceptance
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        goal_handle = future.result()
        if goal_handle.accepted:
            node.get_logger().info('✓ Goal accepted! Robot is navigating...')
            node.get_logger().info('  Monitor progress with: ros2 topic echo /navigate_to_pose/_action/feedback')
        else:
            node.get_logger().error('✗ Goal rejected by server')
    else:
        node.get_logger().error('✗ Failed to send goal (timeout)')
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
