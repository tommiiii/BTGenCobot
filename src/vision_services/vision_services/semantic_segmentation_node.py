#!/usr/bin/env python3
"""Reduced-rate ADE20K segmentation for MIT-SPARK Hydra's CPU profile."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
import torch
import torch.nn.functional as functional
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor

from .semantic_processing import load_label_grouping


class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__("hydra_semantic_segmentation")
        self.declare_parameter(
            "model_name",
            "nvidia/segformer-b2-finetuned-ade-512-512",
        )
        self.declare_parameter("input_topic", "/head_front_camera/image")
        self.declare_parameter(
            "depth_input_topic",
            "/head_front_camera/depth_image",
        )
        self.declare_parameter(
            "rgb_output_topic",
            "/hydra/input/camera/rgb/image_raw",
        )
        self.declare_parameter(
            "depth_output_topic",
            "/hydra/input/camera/depth_registered/image_rect",
        )
        self.declare_parameter(
            "output_topic",
            "/hydra/input/camera/semantic/image_raw",
        )
        self.declare_parameter("max_rate_hz", 1.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.02)
        self.declare_parameter("device", "auto")
        self.declare_parameter("require_mapping_posture", False)
        self.declare_parameter(
            "remap_config",
            str(
                Path(get_package_share_directory("vision_services"))
                / "config"
                / "ade20k_mp3d_remap.yaml"
            ),
        )

        self._model_name = str(self.get_parameter("model_name").value)
        device = str(self.get_parameter("device").value)
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self._device = torch.device(device)
        self._period = 1.0 / max(
            0.05, float(self.get_parameter("max_rate_hz").value)
        )
        self._last_start = 0.0
        self._state_lock = threading.Lock()
        self._pending_pair: tuple[Image, Image] | None = None
        self._worker_active = False
        self._worker: threading.Thread | None = None
        self._shutting_down = False
        self._frames = 0
        self._dropped = 0
        self._last_latency_ms = 0.0
        self._last_sync_skew_ms = 0.0
        self._last_error = ""
        self._mapping_posture_ready = not bool(
            self.get_parameter("require_mapping_posture").value
        )

        remap_config = str(self.get_parameter("remap_config").value).strip()
        if not remap_config:
            raise ValueError("remap_config is required for Hydra closed-set semantics")
        self._label_grouping = load_label_grouping(Path(remap_config))
        self.get_logger().info(
            f"Loading {self._model_name} for {self._label_grouping.group_count} "
            f"Hydra semantic groups on {self._device}"
        )
        self._processor = SegformerImageProcessor.from_pretrained(self._model_name)
        self._model = SegformerForSemanticSegmentation.from_pretrained(
            self._model_name
        ).to(self._device)
        self._model.eval()

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self._rgb_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("rgb_output_topic").value),
            qos,
        )
        self._depth_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("depth_output_topic").value),
            qos,
        )
        self._semantic_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            qos,
        )
        self._health_publisher = self.create_publisher(
            String, "/hydra/semantic_segmentation/health", 10
        )
        if not self._mapping_posture_ready:
            ready_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._posture_subscription = self.create_subscription(
                Bool,
                "/mapping/navigation_posture_ready",
                self._posture_callback,
                ready_qos,
            )
        self._rgb_subscriber = Subscriber(
            self,
            Image,
            str(self.get_parameter("input_topic").value),
            qos_profile=qos,
        )
        self._depth_subscriber = Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_input_topic").value),
            qos_profile=qos,
        )
        self._synchronizer = ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        self._synchronizer.registerCallback(self._image_callback)
        self.create_timer(0.05, self._try_start_worker)
        self.create_timer(2.0, self._publish_health)
        self.get_logger().info("Hydra semantic segmentation is ready")

    @staticmethod
    def _to_rgb(message: Image) -> np.ndarray:
        channels = 4 if message.encoding.lower() in {"rgba8", "bgra8"} else 3
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, channels
        )
        encoding = message.encoding.lower()
        if encoding in {"bgr8", "bgra8"}:
            image = image[..., [2, 1, 0]]
        else:
            image = image[..., :3]
        return np.ascontiguousarray(image)

    def _image_callback(self, message: Image, depth_message: Image) -> None:
        with self._state_lock:
            if self._shutting_down:
                return
            if not self._mapping_posture_ready:
                self._dropped += 1
                return
            if self._pending_pair is not None:
                self._dropped += 1
            # Keep only the newest synchronized capture while inference runs.
            # This prevents a slow CPU model from walking through an old ROS
            # queue and publishing geometry after TF history has moved on.
            self._pending_pair = (message, depth_message)
        self._try_start_worker()

    def _posture_callback(self, message: Bool) -> None:
        with self._state_lock:
            self._mapping_posture_ready = bool(message.data)
            if not self._mapping_posture_ready:
                self._pending_pair = None

    def _try_start_worker(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            if (
                self._shutting_down
                or self._worker_active
                or self._pending_pair is None
                or now - self._last_start < self._period
            ):
                return
            pair = self._pending_pair
            self._pending_pair = None
            self._worker_active = True
            self._last_start = now
        self._worker = threading.Thread(
            target=self._process_pair,
            args=pair,
            name="hydra-semantic-worker",
            daemon=True,
        )
        self._worker.start()

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def _process_pair(self, message: Image, depth_message: Image) -> None:
        started = time.monotonic()
        try:
            self._process_images(message, depth_message)
            self._frames += 1
            self._last_latency_ms = (time.monotonic() - started) * 1000.0
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"Semantic segmentation failed: {exc}")
        finally:
            with self._state_lock:
                self._worker_active = False

    def _process_images(self, message: Image, depth_message: Image) -> None:
        if message.height != depth_message.height or message.width != depth_message.width:
            raise ValueError(
                "registered RGB and depth dimensions differ: "
                f"{message.width}x{message.height} vs "
                f"{depth_message.width}x{depth_message.height}"
            )
        self._last_sync_skew_ms = abs(
            self._stamp_ns(message) - self._stamp_ns(depth_message)
        ) / 1.0e6
        configured_slop_ms = (
            float(self.get_parameter("sync_slop_sec").value) * 1000.0 + 0.1
        )
        if self._last_sync_skew_ms > configured_slop_ms:
            raise ValueError(
                f"RGB/depth skew {self._last_sync_skew_ms:.1f} ms exceeds sync slop"
            )

        rgb = self._to_rgb(message)
        inputs = self._processor(
            images=PILImage.fromarray(rgb),
            return_tensors="pt",
        )
        inputs = {name: value.to(self._device) for name, value in inputs.items()}
        with torch.inference_mode():
            logits = self._model(**inputs).logits
            logits = functional.interpolate(
                logits,
                size=(message.height, message.width),
                mode="bilinear",
                align_corners=False,
            )[0]
            source_labels = logits.argmax(dim=0).to("cpu").numpy()

        # Match MIT-SPARK semantic_inference: choose one ADE20K class first,
        # then recolor its ID into the compact Hydra label space. Grouping
        # probabilities before argmax biases labels with more source classes.
        semantic_np = self._label_grouping.source_to_group[source_labels].astype(
            np.int16,
            copy=False,
        )

        output = Image()
        output.header = message.header
        output.height = message.height
        output.width = message.width
        # Match semantic_inference's documented closed-set ROS output exactly.
        output.encoding = "16SC1"
        output.is_bigendian = False
        output.step = message.width * np.dtype(np.int16).itemsize
        output.data = semantic_np.tobytes()

        # Publish one capture-time-aligned triplet after inference completes.
        # The tight synchronizer bounds the remaining RGB/depth pose error.
        aligned_depth = copy.deepcopy(depth_message)
        aligned_depth.header.stamp = message.header.stamp
        self._rgb_publisher.publish(message)
        self._depth_publisher.publish(aligned_depth)
        self._semantic_publisher.publish(output)

    def destroy_node(self):
        with self._state_lock:
            self._shutting_down = True
            self._pending_pair = None
        return super().destroy_node()

    def _publish_health(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "model": self._model_name,
                "device": str(self._device),
                "mapping_posture_ready": self._mapping_posture_ready,
                "processed_frames": self._frames,
                "dropped_frames": self._dropped,
                "last_latency_ms": round(self._last_latency_ms, 1),
                "rgb_depth_skew_ms": round(self._last_sync_skew_ms, 2),
                "error": self._last_error,
            }
        )
        self._health_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
