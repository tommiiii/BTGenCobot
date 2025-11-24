#!/usr/bin/env python3
"""
Grounding DINO Object Detection Service Node

This ROS2 service node provides object detection using Grounding DINO.
It runs in the ROS2 Docker container and is called by the C++ DetectObject BT node.

Service: /detect_object (btgencobot_interfaces/srv/DetectObject)
"""

import rclpy
from rclpy.node import Node
from btgencobot_interfaces.srv import DetectObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Try to import Grounding DINO - fallback to mock if not available
try:
    import torch
    import torchvision.transforms as T
    from groundingdino.util.inference import load_model, predict
    GROUNDING_DINO_AVAILABLE = True
except ImportError as e:
    GROUNDING_DINO_AVAILABLE = False
    import_error = str(e)


class GroundingDINOService(Node):
    """ROS2 service node for Grounding DINO object detection"""

    def __init__(self):
        super().__init__('grounding_dino_service')

        # Declare parameters
        self.declare_parameter('use_mock', True)  # Start in mock mode by default
        self.declare_parameter('model_config', 'groundingdino/config/GroundingDINO_SwinT_OGC.py')
        self.declare_parameter('model_weights', '/ros2_ws/model_weights/grounding_dino/groundingdino_swint_ogc.pth')
        self.declare_parameter('device', 'auto')  # 'auto', 'cuda', or 'cpu'

        # Get parameters
        self.use_mock = self.get_parameter('use_mock').value
        self.model_config = self.get_parameter('model_config').value
        self.model_weights = self.get_parameter('model_weights').value
        device_param = self.get_parameter('device').value

        # Setup device
        if device_param == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu' if GROUNDING_DINO_AVAILABLE else 'cpu'
        else:
            self.device = device_param

        self.get_logger().info(f'Grounding DINO Service Node starting...')
        self.get_logger().info(f'  Use mock: {self.use_mock}')
        self.get_logger().info(f'  Device: {self.device}')

        # Initialize model
        self.model = None
        self.bridge = CvBridge()

        if not self.use_mock:
            if not GROUNDING_DINO_AVAILABLE:
                self.get_logger().error(f'Grounding DINO not available: {import_error}')
                self.get_logger().warning('Falling back to MOCK mode')
                self.use_mock = True
            else:
                self._load_model()

        if self.use_mock:
            self.get_logger().warning('Running in MOCK MODE - will return fake detections')

        # Create service
        self.srv = self.create_service(
            DetectObject,
            '/detect_object',
            self.detect_object_callback)

        self.get_logger().info('✓ Grounding DINO service ready at /detect_object')

    def _load_model(self):
        """Load Grounding DINO model"""
        try:
            self.get_logger().info(f'Loading Grounding DINO model...')
            self.get_logger().info(f'  Config: {self.model_config}')
            self.get_logger().info(f'  Weights: {self.model_weights}')

            self.model = load_model(
                self.model_config,
                self.model_weights,
                device=self.device
            )

            self.get_logger().info('✓ Grounding DINO model loaded successfully!')

        except FileNotFoundError as e:
            self.get_logger().error(f'Model files not found: {e}')
            self.get_logger().error(f'Please download weights to: {self.model_weights}')
            self.get_logger().warning('Falling back to MOCK mode')
            self.use_mock = True

        except Exception as e:
            self.get_logger().error(f'Failed to load Grounding DINO: {e}')
            self.get_logger().warning('Falling back to MOCK mode')
            self.use_mock = True

    def detect_object_callback(self, request, response):
        """Handle detection service requests"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')

            self.get_logger().info(
                f'Detection request: "{request.object_description}" '
                f'(image: {cv_image.shape[1]}x{cv_image.shape[0]})'
            )

            # Run detection
            if self.use_mock:
                result = self._mock_detect(cv_image, request.object_description)
            else:
                result = self._detect(cv_image, request.object_description, request.box_threshold)

            # Fill response
            response.detected = result['detected']
            response.confidence = result['confidence']
            response.center_x = result['center_x']
            response.center_y = result['center_y']
            response.bbox = result['bbox']
            response.phrase = result['phrase']
            response.error_message = result.get('error', '')

            if result['detected']:
                self.get_logger().info(
                    f'✓ Detected "{result["phrase"]}" at '
                    f'({result["center_x"]:.1f}, {result["center_y"]:.1f}) '
                    f'with {result["confidence"]:.2f} confidence'
                )
            else:
                self.get_logger().warn(f'✗ Object not detected: {response.error_message}')

            return response

        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
            response.detected = False
            response.confidence = 0.0
            response.center_x = -1.0
            response.center_y = -1.0
            response.bbox = []
            response.phrase = ''
            response.error_message = str(e)
            return response

    def _detect(self, image, text_prompt, box_threshold):
        """Run Grounding DINO detection"""
        try:
            # Convert numpy array to PIL Image for Grounding DINO
            from PIL import Image as PILImage
            pil_image = PILImage.fromarray(image)

            # Transform for Grounding DINO
            transform = T.Compose([
                T.ToTensor(),
            ])
            image_tensor = transform(pil_image)

            # Run detection
            boxes, logits, phrases = predict(
                model=self.model,
                image=image_tensor,
                caption=text_prompt,
                box_threshold=box_threshold,
                text_threshold=0.25,
                device=self.device
            )

            if len(boxes) == 0:
                return {
                    'detected': False,
                    'confidence': 0.0,
                    'center_x': -1.0,
                    'center_y': -1.0,
                    'bbox': [],
                    'phrase': '',
                    'error': f'No objects found matching "{text_prompt}"'
                }

            # Get highest confidence detection
            best_idx = logits.argmax()
            box = boxes[best_idx].cpu().numpy()  # [x_center, y_center, width, height] normalized
            confidence = float(logits[best_idx].item())
            phrase = phrases[best_idx]

            # Convert from normalized center format to pixel corner format
            h, w = image.shape[:2]

            # box format: [x_center, y_center, width, height] in [0, 1]
            x_center, y_center, box_width, box_height = box

            # Convert to pixel coordinates
            x_center_px = x_center * w
            y_center_px = y_center * h
            width_px = box_width * w
            height_px = box_height * h

            # Convert to corner format [x1, y1, x2, y2]
            x1 = x_center_px - width_px / 2
            y1 = y_center_px - height_px / 2
            x2 = x_center_px + width_px / 2
            y2 = y_center_px + height_px / 2

            return {
                'detected': True,
                'confidence': confidence,
                'center_x': float(x_center_px),
                'center_y': float(y_center_px),
                'bbox': [float(x1), float(y1), float(x2), float(y2)],
                'phrase': phrase,
                'error': ''
            }

        except Exception as e:
            self.get_logger().error(f'Grounding DINO inference failed: {e}')
            return {
                'detected': False,
                'confidence': 0.0,
                'center_x': -1.0,
                'center_y': -1.0,
                'bbox': [],
                'phrase': '',
                'error': str(e)
            }

    def _mock_detect(self, image, text_prompt):
        """Mock detection for testing (returns center of image)"""
        h, w = image.shape[:2]

        # Return fake detection at center
        center_x = w / 2.0
        center_y = h / 2.0
        bbox_width = w * 0.2
        bbox_height = h * 0.2

        x1 = center_x - bbox_width / 2
        y1 = center_y - bbox_height / 2
        x2 = center_x + bbox_width / 2
        y2 = center_y + bbox_height / 2

        self.get_logger().info(
            f'MOCK: Detected "{text_prompt}" at center ({center_x:.1f}, {center_y:.1f})'
        )

        return {
            'detected': True,
            'confidence': 0.95,
            'center_x': float(center_x),
            'center_y': float(center_y),
            'bbox': [float(x1), float(y1), float(x2), float(y2)],
            'phrase': text_prompt,
            'error': ''
        }


def main(args=None):
    rclpy.init(args=args)
    node = GroundingDINOService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
