#!/usr/bin/env python3
"""
Verify which BT is actually being used by monitoring behavior_tree_log

Usage:
    python3 scripts/verify_bt.py "UNIQUE_NODE_NAME"
    
This script monitors the /behavior_tree_log topic and shows which BT nodes
are being executed. Use this to verify your custom BT is actually loaded.

Example:
    # Send a goal with test_unique.xml
    python3 scripts/send_goal.py 1.0 1.0 --bt robot_description/mobile_manipulator/behavior_trees/test_unique.xml
    
    # In another terminal, verify the unique node appears
    python3 scripts/verify_bt.py "UNIQUE_TEST_BT_MARKER"
"""

import sys
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog


class BTVerifier(Node):
    def __init__(self, search_term=None):
        super().__init__('bt_verifier')
        self.search_term = search_term
        self.seen_nodes = set()
        self.found_match = False
        
        self.sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.callback,
            10
        )
        
        self.get_logger().info('Monitoring /behavior_tree_log for BT node names...')
        if self.search_term:
            self.get_logger().info(f'Searching for: "{self.search_term}"')
        self.get_logger().info('Send a navigation goal to see BT nodes in action')
        self.get_logger().info('---')
    
    def callback(self, msg):
        for event in msg.event_log:
            node_name = event.node_name
            
            # Add to seen nodes
            if node_name not in self.seen_nodes:
                self.seen_nodes.add(node_name)
                
                # Print the node
                if self.search_term and self.search_term in node_name:
                    self.get_logger().info(f'✓ FOUND: {node_name} (status: {event.current_status})')
                    self.found_match = True
                else:
                    self.get_logger().info(f'  Node: {node_name} (status: {event.current_status})')


def main():
    search_term = sys.argv[1] if len(sys.argv) > 1 else None
    
    rclpy.init()
    node = BTVerifier(search_term)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if search_term:
            if node.found_match:
                node.get_logger().info(f'\n✓ SUCCESS: Found "{search_term}" in BT execution!')
                node.get_logger().info('Your custom BT is being used.')
            else:
                node.get_logger().warn(f'\n✗ NOT FOUND: "{search_term}" was not seen in BT execution.')
                node.get_logger().warn('Your custom BT may not be loaded, or navigation hasn\'t started.')
        node.get_logger().info(f'\nTotal unique BT nodes seen: {len(node.seen_nodes)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
