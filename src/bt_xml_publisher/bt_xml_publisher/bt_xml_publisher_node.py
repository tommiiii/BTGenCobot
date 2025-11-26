#!/usr/bin/env python3
"""
BT XML Publisher Node - Publishes the currently loaded BT XML for visualization

Publishes:
  /behavior_tree (std_msgs/String) - Current active BT XML for Groot2/Foxglove
  
Subscribes:
  /behavior_tree_log (nav2_msgs/BehaviorTreeLog) - Monitors active BT execution

Note: With always_reload_bt_xml=True, this publishes the default BT. 
The actual BT used per goal is passed via action and not reflected here.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.msg import BehaviorTreeLog
import os


class BTXMLPublisher(Node):
    def __init__(self):
        super().__init__('bt_xml_publisher')
        
        # Publisher for BT XML (standard topic name for Groot2/Foxglove)
        self.bt_xml_pub = self.create_publisher(String, '/behavior_tree', 10)
        
        # Subscribe to BT log to detect BT changes
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.bt_log_callback,
            10
        )
        
        # Get BT file path from bt_navigator parameters
        self.declare_parameter('bt_xml_file', '')
        self.bt_xml_file = self.get_parameter('bt_xml_file').value
        
        # If not provided, get from bt_navigator
        if not self.bt_xml_file:
            self.get_bt_file_from_navigator()
        
        # Current BT XML content
        self.current_bt_xml = ""
        self.last_bt_file_mtime = 0
        
        # Load BT from file
        self.load_bt_xml()
        
        # Publish BT XML periodically (for new subscribers)
        self.timer = self.create_timer(2.0, self.publish_bt_xml)
        
        self.get_logger().info('BT XML Publisher started')
        self.get_logger().info(f'  - Publishing BT XML on /behavior_tree')
        self.get_logger().info(f'  - Default BT file: {self.bt_xml_file}')
        self.get_logger().info(f'  - Note: With always_reload_bt_xml=True, bt_navigator reloads from')
        self.get_logger().info(f'    the file specified in each action goal (not visible in this topic)')
        
    def get_bt_file_from_navigator(self):
        """Get BT file path from bt_navigator parameters"""
        # Use default Nav2 BT file
        self.bt_xml_file = '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml'
    
    def bt_log_callback(self, msg):
        """Monitor BT execution log for debugging"""
        # This can be used to detect which BT nodes are running
        # Useful for verifying custom BTs are actually being executed
        pass
    
    def load_bt_xml(self):
        """Load BT XML from file"""
        if os.path.exists(self.bt_xml_file):
            try:
                self.last_bt_file_mtime = os.path.getmtime(self.bt_xml_file)
                with open(self.bt_xml_file, 'r') as f:
                    self.current_bt_xml = f.read()
                self.get_logger().info(f'Loaded BT XML ({len(self.current_bt_xml)} bytes)')
            except Exception as e:
                self.get_logger().error(f'Error loading BT file: {e}')
        else:
            self.get_logger().error(f'BT file not found: {self.bt_xml_file}')
            
    def publish_bt_xml(self):
        """Publish current BT XML (reload if file changed)"""
        # Check if file was modified
        if os.path.exists(self.bt_xml_file):
            mtime = os.path.getmtime(self.bt_xml_file)
            if mtime != self.last_bt_file_mtime:
                self.get_logger().info('BT file changed, reloading...')
                self.load_bt_xml()
        
        if self.current_bt_xml:
            msg = String()
            msg.data = self.current_bt_xml
            self.bt_xml_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BTXMLPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
