#!/usr/bin/env python3
"""Frontend compatibility topic relays.

The companion web frontend was built against the TurtleBot3 + OpenManipulator-X
topic contract (fork `omen` branch). This node republishes the TIAGo-native
sensor topics under the names the frontend expects, so the frontend can be used
without modification on the merged TIAGo stack:

    /head_front_camera/image  -> /camera            (sensor_msgs/Image)
    /scan_raw                 -> /scan              (sensor_msgs/LaserScan)
    /head_front_camera/camera_info -> /camera_info   (sensor_msgs/CameraInfo)

Nav2 already consumes /scan_raw directly via nav2_config.yaml; the /scan relay
exists purely for frontend visualization. /amcl_pose and /odom are already
published under those names by AMCL and the TIAGo odometry bridge respectively,
so no relay is needed for them. /generated_behavior_tree, /bt_execution_feedback,
/btgen/environment and /btgen_nl_command are produced by bt_interface_node /
environment_publisher and already match the frontend contract.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo


class FrontendRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('frontend_relay')

        # Image: /head_front_camera/image -> /camera
        self._image_pub = self.create_publisher(Image, '/camera', 10)
        self._image_sub = self.create_subscription(
            Image, '/head_front_camera/image', self._image_cb, 10
        )

        # CameraInfo: /head_front_camera/camera_info -> /camera_info
        self._cinfo_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self._cinfo_sub = self.create_subscription(
            CameraInfo, '/head_front_camera/camera_info', self._cinfo_cb, 10
        )

        # LaserScan: /scan_raw -> /scan
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self._scan_cb, 10
        )

        self.get_logger().info(
            'Frontend relay ready: /head_front_camera/image->/camera, '
            '/scan_raw->/scan, /head_front_camera/camera_info->/camera_info'
        )

    def _image_cb(self, msg: Image) -> None:
        self._image_pub.publish(msg)

    def _cinfo_cb(self, msg: CameraInfo) -> None:
        self._cinfo_pub.publish(msg)

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontendRelayNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
