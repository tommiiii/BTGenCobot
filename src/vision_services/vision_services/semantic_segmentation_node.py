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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
import torch.nn.functional as functional
from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
import yaml


class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__("hydra_semantic_segmentation")
        self.declare_parameter(
            "model_name",
            "nvidia/segformer-b0-finetuned-ade-512-512",
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
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("sync_slop_sec", 0.08)
        self.declare_parameter("device", "auto")
        self.declare_parameter(
            "remap_config",
            str(
                Path(get_package_share_directory("vision_services"))
                / "config"
                / "ade20k_mit_remap.yaml"
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
        self._processing = threading.Lock()
        self._frames = 0
        self._dropped = 0
        self._last_latency_ms = 0.0
        self._last_error = ""

        self._label_remap = self._load_remap(
            Path(self.get_parameter("remap_config").value)
        )
        self.get_logger().info(
            f"Loading {self._model_name} for Hydra semantics on {self._device}"
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
        self.create_timer(2.0, self._publish_health)
        self.get_logger().info("Hydra semantic segmentation is ready")

    @staticmethod
    def _load_remap(path: Path) -> np.ndarray:
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
        result = np.zeros(150, dtype=np.uint8)
        assigned = np.zeros(150, dtype=bool)
        for output_label, source_labels in enumerate(config["groups"]):
            for source_label in source_labels:
                if 0 <= int(source_label) < len(result) and not assigned[source_label]:
                    result[source_label] = output_label
                    assigned[source_label] = True
        return result

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
        now = time.monotonic()
        if now - self._last_start < self._period or not self._processing.acquire(False):
            self._dropped += 1
            return
        self._last_start = now
        try:
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
                )
                ade_labels = logits.argmax(dim=1)[0].to("cpu").numpy()
            semantic = self._label_remap[ade_labels]
            output = Image()
            output.header = message.header
            output.height = message.height
            output.width = message.width
            output.encoding = "mono8"
            output.is_bigendian = False
            output.step = message.width
            output.data = semantic.tobytes()

            # Publish one timestamp-aligned RGB/depth/semantic triplet only
            # after inference completes. Hydra must never pair this delayed
            # semantic mask with a newer camera/depth frame.
            aligned_depth = copy.deepcopy(depth_message)
            # Registered depth may retain a distinct optical-frame alias; only
            # the timestamp must be unified for the synchronized packet.
            aligned_depth.header.stamp = message.header.stamp
            self._rgb_publisher.publish(message)
            self._depth_publisher.publish(aligned_depth)
            self._semantic_publisher.publish(output)
            self._frames += 1
            self._last_latency_ms = (time.monotonic() - now) * 1000.0
            self._last_error = ""
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"Semantic segmentation failed: {exc}")
        finally:
            self._processing.release()

    def _publish_health(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "model": self._model_name,
                "device": str(self._device),
                "processed_frames": self._frames,
                "dropped_frames": self._dropped,
                "last_latency_ms": round(self._last_latency_ms, 1),
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
