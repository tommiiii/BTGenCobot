"""One-shot live RGB-D grounding used only after a Hydra graph miss."""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from btgencobot_interfaces.msg import SemanticObjectObservation
from btgencobot_interfaces.srv import DetectObject, DiscoverSemanticObject


def _rotate(
    vector: np.ndarray,
    quaternion: tuple[float, float, float, float],
) -> np.ndarray:
    x, y, z, w = quaternion
    qvec = np.array([x, y, z], dtype=np.float64)
    return vector + 2.0 * np.cross(
        qvec,
        np.cross(qvec, vector) + w * vector,
    )


class SemanticLiveFallback(Node):
    def __init__(self) -> None:
        super().__init__("semantic_live_fallback")
        self.declare_parameter("rgb_topic", "/head_front_camera/image")
        self.declare_parameter("depth_topic", "/head_front_camera/depth_image")
        self.declare_parameter(
            "camera_info_topic", "/head_front_camera/camera_info"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("minimum_valid_depth_m", 0.25)
        self.declare_parameter("maximum_valid_depth_m", 5.0)
        self.declare_parameter("maximum_rgb_depth_delta_sec", 0.12)
        self.declare_parameter("maximum_live_frame_age_sec", 5.0)
        self.declare_parameter("detection_timeout_sec", 25.0)

        self._bridge = CvBridge()
        self._lock = threading.RLock()
        self._depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._frame = None
        self._detection_in_progress = False
        # The discovery service awaits the detector client. Both callbacks must
        # be allowed to run in the same executor while that coroutine is
        # suspended, otherwise the detector response cannot complete until the
        # discovery request times out.
        self._service_callback_group = ReentrantCallbackGroup()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._detect_client = self.create_client(
            DetectObject,
            "/detect_object",
            callback_group=self._service_callback_group,
        )
        observation_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._observation_pub = self.create_publisher(
            SemanticObjectObservation,
            "/hydra/semantic_object_observations",
            observation_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            5,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            5,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self._rgb_callback,
            5,
        )
        self.create_service(
            DiscoverSemanticObject,
            "/mapping/discover_semantic_object",
            self._discover_callback,
            callback_group=self._service_callback_group,
        )
        self.get_logger().info(
            "Live semantic fallback ready (single current-frame detection only)"
        )

    @staticmethod
    def _set_result_if_pending(future: Future, result) -> None:
        try:
            if not future.done():
                future.set_result(result)
        except RuntimeError:
            pass

    async def _wait_for(self, future: Future, timeout: float):
        completion = Future(executor=self.executor)
        future.add_done_callback(
            lambda done: self._set_result_if_pending(
                completion,
                ("service", done),
            )
        )
        timer = threading.Timer(
            timeout,
            lambda: self._set_result_if_pending(
                completion,
                ("timeout", None),
            ),
        )
        timer.daemon = True
        timer.start()
        try:
            outcome, completed = await completion
        finally:
            timer.cancel()
        if outcome == "timeout":
            future.cancel()
            raise TimeoutError
        return completed.result()

    def _depth_callback(self, msg: Image) -> None:
        self._depth = msg

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _rgb_callback(self, rgb_msg: Image) -> None:
        if self._depth is None or self._camera_info is None:
            return
        depth_msg = self._depth
        rgb_stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec / 1e9
        depth_stamp = (
            depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec / 1e9
        )
        if abs(rgb_stamp - depth_stamp) > float(
            self.get_parameter("maximum_rgb_depth_delta_sec").value
        ):
            return
        camera_frame = depth_msg.header.frame_id or rgb_msg.header.frame_id
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                camera_frame,
                rclpy.time.Time.from_msg(rgb_msg.header.stamp),
                timeout=Duration(seconds=0.15),
            )
            rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            depth = self._bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="passthrough",
            )
        except Exception:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        record = {
            "frame_id": camera_frame,
            "stamp": rgb_msg.header.stamp,
            "translation": [translation.x, translation.y, translation.z],
            "rotation": [rotation.x, rotation.y, rotation.z, rotation.w],
            "camera": {
                "fx": float(self._camera_info.k[0]),
                "fy": float(self._camera_info.k[4]),
                "cx": float(self._camera_info.k[2]),
                "cy": float(self._camera_info.k[5]),
            },
            "depth_encoding": depth_msg.encoding,
        }
        with self._lock:
            self._frame = (
                np.asarray(rgb, dtype=np.uint8).copy(),
                np.asarray(depth).copy(),
                record,
                time.monotonic(),
            )

    @staticmethod
    def _image_message(rgb: np.ndarray, record: dict) -> Image:
        msg = Image()
        msg.header.frame_id = record["frame_id"]
        msg.header.stamp = record["stamp"]
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(rgb).tobytes()
        return msg

    def _map_point(
        self,
        record: dict,
        depth: np.ndarray,
        bbox: list[float],
        fallback_center_x: float,
        fallback_center_y: float,
    ) -> tuple[float, float, float] | None:
        """Estimate the closest coherent 3-D cluster inside the detection box."""
        height, width = depth.shape[:2]
        if len(bbox) >= 4:
            x1 = max(0, min(int(round(bbox[0])), width - 1))
            y1 = max(0, min(int(round(bbox[1])), height - 1))
            x2 = max(0, min(int(round(bbox[2])), width - 1))
            y2 = max(0, min(int(round(bbox[3])), height - 1))
        else:
            col = int(round(fallback_center_x))
            row = int(round(fallback_center_y))
            x1, x2 = max(0, col - 3), min(width - 1, col + 3)
            y1, y2 = max(0, row - 3), min(height - 1, row + 3)

        margin_x = max(0, int((x2 - x1) * 0.10))
        margin_y = max(0, int((y2 - y1) * 0.10))
        sample_x1, sample_x2 = x1 + margin_x, x2 - margin_x
        sample_y1, sample_y2 = y1 + margin_y, y2 - margin_y
        region = np.asarray(
            depth[
                sample_y1 : sample_y2 + 1 : 2,
                sample_x1 : sample_x2 + 1 : 2,
            ],
            dtype=np.float64,
        )
        if "16U" in record["depth_encoding"].upper():
            region /= 1000.0
        minimum = float(self.get_parameter("minimum_valid_depth_m").value)
        maximum = float(self.get_parameter("maximum_valid_depth_m").value)
        valid_rows, valid_cols = np.nonzero(
            np.isfinite(region) & (region >= minimum) & (region <= maximum)
        )
        if valid_rows.size == 0:
            return None

        values = region[valid_rows, valid_cols]
        nearest = float(np.min(values))
        tolerance = max(0.05, nearest * 0.15)
        cluster_mask = values <= nearest + tolerance
        cluster_values = values[cluster_mask]
        cluster_rows = valid_rows[cluster_mask]
        cluster_cols = valid_cols[cluster_mask]
        if cluster_values.size == 0:
            return None

        z = float(np.median(cluster_values))
        center_x = float(sample_x1 + 2.0 * np.mean(cluster_cols))
        center_y = float(sample_y1 + 2.0 * np.mean(cluster_rows))
        camera = record["camera"]
        camera_point = np.array(
            [
                (center_x - camera["cx"]) * z / camera["fx"],
                (center_y - camera["cy"]) * z / camera["fy"],
                z,
            ]
        )
        point = _rotate(
            camera_point,
            tuple(record["rotation"]),
        ) + np.asarray(record["translation"])
        return tuple(float(value) for value in point)

    def _publish(
        self,
        label: str,
        point: tuple[float, float, float],
        confidence: float,
    ) -> None:
        observation = SemanticObjectObservation()
        observation.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        observation.header.stamp = self.get_clock().now().to_msg()
        observation.label = label
        observation.pose.header = observation.header
        observation.pose.pose.position.x = point[0]
        observation.pose.pose.position.y = point[1]
        observation.pose.pose.position.z = point[2]
        observation.pose.pose.orientation.w = 1.0
        observation.confidence = confidence
        observation.source = "grounding_dino:live_fallback"
        self._observation_pub.publish(observation)

    async def _discover_callback(self, request, response):
        label = request.label.strip()
        if not label:
            response.error_message = "object label is empty"
            return response
        if not self._detect_client.service_is_ready():
            response.error_message = "GroundingDINO service is unavailable"
            return response
        with self._lock:
            if self._detection_in_progress:
                response.error_message = "a live detection is already in progress"
                return response
            if self._frame is None:
                response.error_message = "no synchronized live RGB-D frame is available"
                return response
            rgb, depth, record, captured_at = self._frame
            age = time.monotonic() - captured_at
            if age > float(
                self.get_parameter("maximum_live_frame_age_sec").value
            ):
                response.error_message = f"current RGB-D frame is stale ({age:.1f}s)"
                return response
            self._detection_in_progress = True

        timeout = float(self.get_parameter("detection_timeout_sec").value)
        if request.max_duration_sec > 0.0:
            timeout = min(timeout, float(request.max_duration_sec))
        detect_request = DetectObject.Request()
        detect_request.image = self._image_message(rgb, record)
        detect_request.object_description = label
        detect_request.box_threshold = (
            request.box_threshold if request.box_threshold > 0.0 else 0.45
        )
        try:
            detection = await self._wait_for(
                self._detect_client.call_async(detect_request),
                timeout,
            )
        except TimeoutError:
            response.error_message = (
                f'live detection for "{label}" timed out after {timeout:.0f}s'
            )
            return response
        except Exception as exc:
            response.error_message = f'live detection for "{label}" failed: {exc}'
            return response
        finally:
            with self._lock:
                self._detection_in_progress = False

        if not detection.detected:
            response.error_message = (
                detection.error_message
                or f'"{label}" was not found in the current camera view'
            )
            return response
        point = self._map_point(
            record,
            depth,
            list(detection.bbox),
            detection.center_x,
            detection.center_y,
        )
        if point is None:
            response.error_message = f'detection for "{label}" had no valid depth'
            return response
        self._publish(label, point, float(detection.confidence))
        response.success = True
        response.observations_added = 1
        response.best_confidence = float(detection.confidence)
        response.detected_pose.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        response.detected_pose.header.stamp = self.get_clock().now().to_msg()
        response.detected_pose.pose.position.x = point[0]
        response.detected_pose.pose.position.y = point[1]
        response.detected_pose.pose.position.z = point[2]
        response.detected_pose.pose.orientation.w = 1.0
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticLiveFallback()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(num_threads=3))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
