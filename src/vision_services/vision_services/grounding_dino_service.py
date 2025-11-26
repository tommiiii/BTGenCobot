#!/usr/bin/env python3
"""Grounding DINO Object Detection Service Node

This ROS2 service node provides object detection using Grounding DINO.
Service: /detect_object (btgencobot_interfaces/srv/DetectObject)"""

import rclpy
from rclpy.node import Node
from btgencobot_interfaces.srv import DetectObject
from cv_bridge import CvBridge

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
        self._declare_parameters()
        self._setup_device()
        self._initialize_model()
        self._create_service()

    def _declare_parameters(self):
        """Declare and load ROS parameters"""
        self.declare_parameter('use_mock', True)
        self.declare_parameter('model_config', 'groundingdino/config/GroundingDINO_SwinT_OGC.py')
        self.declare_parameter('model_weights', '/ros2_ws/model_weights/grounding_dino/groundingdino_swint_ogc.pth')
        self.declare_parameter('device', 'auto')

        self.use_mock = self.get_parameter('use_mock').value
        self.model_config = self.get_parameter('model_config').value
        self.model_weights = self.get_parameter('model_weights').value
        self.device_param = self.get_parameter('device').value

    def _setup_device(self):
        """Setup compute device (CUDA or CPU)"""
        if self.device_param == 'auto' and GROUNDING_DINO_AVAILABLE:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = self.device_param

        self.get_logger().info(f'Grounding DINO Service Node starting...')
        self.get_logger().info(f'Use mock: {self.use_mock}')
        self.get_logger().info(f'Device: {self.device}')

    def _initialize_model(self):
        """Initialize Grounding DINO model or fallback to mock mode"""
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

    def _load_model(self):
        """Load Grounding DINO model"""
        try:
            self.get_logger().info('Loading Grounding DINO model...')
            self.get_logger().info(f'Config: {self.model_config}')
            self.get_logger().info(f'Weights: {self.model_weights}')

            self.model = load_model(self.model_config, self.model_weights, device=self.device)
            self.get_logger().info('Grounding DINO model loaded successfully')

        except FileNotFoundError as e:
            self.get_logger().error(f'Model files not found: {e}')
            self.get_logger().error(f'Please download weights to: {self.model_weights}')
            self.get_logger().warning('Falling back to MOCK mode')
            self.use_mock = True

        except Exception as e:
            self.get_logger().error(f'Failed to load Grounding DINO: {e}')
            self.get_logger().warning('Falling back to MOCK mode')
            self.use_mock = True

    def _create_service(self):
        """Create ROS service"""
        self.srv = self.create_service(DetectObject, '/detect_object', self.detect_object_callback)
        self.get_logger().info('Grounding DINO service ready at /detect_object')

    def detect_object_callback(self, request, response):
        """Handle detection service requests"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')

            self.get_logger().info(
                f'Detection request: "{request.object_description}" '
                f'(image: {cv_image.shape[1]}x{cv_image.shape[0]})'
            )

            if self.use_mock:
                result = self._mock_detect(cv_image, request.object_description)
            else:
                result = self._detect(cv_image, request.object_description, request.box_threshold)

            response.detected = result['detected']
            response.confidence = result['confidence']
            response.center_x = result['center_x']
            response.center_y = result['center_y']
            response.bbox = result['bbox']
            response.phrase = result['phrase']
            response.error_message = result.get('error', '')

            if result['detected']:
                self.get_logger().info(
                    f'Detected "{result["phrase"]}" at '
                    f'({result["center_x"]:.1f}, {result["center_y"]:.1f}) '
                    f'with {result["confidence"]:.2f} confidence'
                )
            else:
                self.get_logger().warn(f'Object not detected: {response.error_message}')

            return response

        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
            return self._create_error_response(response, str(e))

    def _create_error_response(self, response, error_message):
        """Create an error response"""
        response.detected = False
        response.confidence = 0.0
        response.center_x = -1.0
        response.center_y = -1.0
        response.bbox = []
        response.phrase = ''
        response.error_message = error_message
        return response

    def _detect(self, image, text_prompt, box_threshold):
        """Run Grounding DINO detection"""
        try:
            from PIL import Image as PILImage
            pil_image = PILImage.fromarray(image)

            transform = T.Compose([T.ToTensor()])
            image_tensor = transform(pil_image)

            boxes, logits, phrases = predict(
                model=self.model,
                image=image_tensor,
                caption=text_prompt,
                box_threshold=box_threshold,
                text_threshold=0.25,
                device=self.device
            )

            if len(boxes) == 0:
                return self._create_detection_result(
                    detected=False,
                    error=f'No objects found matching "{text_prompt}"'
                )

            best_idx = logits.argmax()
            box = boxes[best_idx].cpu().numpy()
            confidence = float(logits[best_idx].item())
            phrase = phrases[best_idx]

            h, w = image.shape[:2]
            x_center, y_center, box_width, box_height = box

            x_center_px = x_center * w
            y_center_px = y_center * h
            width_px = box_width * w
            height_px = box_height * h

            x1 = x_center_px - width_px / 2
            y1 = y_center_px - height_px / 2
            x2 = x_center_px + width_px / 2
            y2 = y_center_px + height_px / 2

            return self._create_detection_result(
                detected=True,
                confidence=confidence,
                center_x=x_center_px,
                center_y=y_center_px,
                bbox=[x1, y1, x2, y2],
                phrase=phrase
            )

        except Exception as e:
            self.get_logger().error(f'Grounding DINO inference failed: {e}')
            return self._create_detection_result(detected=False, error=str(e))

    def _mock_detect(self, image, text_prompt):
        """Mock detection for testing (returns center of image)"""
        h, w = image.shape[:2]

        center_x = w / 2.0
        center_y = h / 2.0
        bbox_width = w * 0.2
        bbox_height = h * 0.2

        x1 = center_x - bbox_width / 2
        y1 = center_y - bbox_height / 2
        x2 = center_x + bbox_width / 2
        y2 = center_y + bbox_height / 2

        self.get_logger().info(f'MOCK: Detected "{text_prompt}" at center ({center_x:.1f}, {center_y:.1f})')

        return self._create_detection_result(
            detected=True,
            confidence=0.95,
            center_x=center_x,
            center_y=center_y,
            bbox=[x1, y1, x2, y2],
            phrase=text_prompt
        )

    def _create_detection_result(self, detected=False, confidence=0.0, center_x=-1.0,
                                  center_y=-1.0, bbox=None, phrase='', error=''):
        """Create a standardized detection result dictionary"""
        return {
            'detected': detected,
            'confidence': confidence,
            'center_x': float(center_x),
            'center_y': float(center_y),
            'bbox': [float(v) for v in bbox] if bbox else [],
            'phrase': phrase,
            'error': error
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
