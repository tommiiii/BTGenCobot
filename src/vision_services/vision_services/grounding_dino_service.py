#!/usr/bin/env python3
"""GroundingDINO Object Detection Service Node

This ROS2 service node provides object detection using:
- GroundingDINO-Tiny for text-prompted open-vocabulary object detection
- Bounding box sampling for depth/pose estimation

Service: /detect_object (btgencobot_interfaces/srv/DetectObject)
"""

import rclpy
from rclpy.node import Node
from btgencobot_interfaces.srv import DetectObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

try:
    import torch
    from transformers import AutoProcessor, GroundingDinoForObjectDetection
    from PIL import Image as PILImage
    DEPENDENCIES_AVAILABLE = True
    import_error = None
except ImportError as e:
    DEPENDENCIES_AVAILABLE = False
    import_error = str(e)


class GroundingDINOService(Node):
    """ROS2 service node for GroundingDINO object detection with bounding box pose estimation"""

    def __init__(self):
        super().__init__('grounding_dino_service')
        self._declare_parameters()
        self._setup_device()
        self._initialize_models()
        self._create_service()

    def _declare_parameters(self):
        """Declare and load ROS parameters"""
        self.declare_parameter('use_mock', False)
        self.declare_parameter('model_name', 'IDEA-Research/grounding-dino-tiny')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('publish_debug_images', True)

        self.use_mock = self.get_parameter('use_mock').value
        self.model_name = self.get_parameter('model_name').value
        self.device_param = self.get_parameter('device').value
        self.publish_debug_images = self.get_parameter('publish_debug_images').value

    def _setup_device(self):
        """Setup compute device (CUDA or CPU)"""
        if self.device_param == 'auto':
            self.device = 'cuda' if (DEPENDENCIES_AVAILABLE and torch.cuda.is_available()) else 'cpu'
        else:
            self.device = self.device_param

        self.get_logger().info('GroundingDINO Service Node starting...')
        self.get_logger().info(f'Use mock: {self.use_mock}')
        self.get_logger().info(f'Device: {self.device}')

    def _initialize_models(self):
        """Initialize GroundingDINO model or fallback to mock mode"""
        self.model = None
        self.processor = None
        self.bridge = CvBridge()

        if not self.use_mock:
            if not DEPENDENCIES_AVAILABLE:
                self.get_logger().error(f'Dependencies not available: {import_error}')
                self.get_logger().warning('Falling back to MOCK mode')
                self.use_mock = True
            else:
                self._load_models()

        if self.use_mock:
            self.get_logger().warning('Running in MOCK MODE - will return fake detections')

    def _load_models(self):
        """Load GroundingDINO model (natively integrated in transformers, no trust_remote_code needed)"""
        try:
            self.get_logger().info('Loading GroundingDINO-Tiny model...')
            self.get_logger().info(f'Model: {self.model_name}')

            torch_dtype = torch.float16 if self.device == 'cuda' else torch.float32
            self.processor = AutoProcessor.from_pretrained(self.model_name)
            self.model = GroundingDinoForObjectDetection.from_pretrained(
                self.model_name,
                dtype=torch_dtype,
            ).to(self.device)
            self.model.eval()

            self.get_logger().info('GroundingDINO model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to load GroundingDINO model: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.get_logger().warning('Falling back to MOCK mode')
            self.use_mock = True

    def _create_service(self):
        """Create ROS2 service and debug image publisher"""
        self.service = self.create_service(
            DetectObject,
            '/detect_object',
            self.detect_callback
        )
        self.debug_image_pub = self.create_publisher(Image, '/grounding_dino/debug_image', 10)
        self.get_logger().info('Service /detect_object ready')

    def detect_callback(self, request, response):
        """Handle detection service request"""
        try:
            start_time = time.time()

            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')
            self.get_logger().info(
                f'Detection request: "{request.object_description}" '
                f'(image: {cv_image.shape[1]}x{cv_image.shape[0]})'
            )

            result = self._process_detection(cv_image, request)

            response.detected = result['detected']
            response.confidence = result['confidence']
            response.center_x = result['center_x']
            response.center_y = result['center_y']
            response.bbox = result['bbox']
            response.phrase = result['phrase']
            response.error_message = result.get('error', '')

            if 'mask' in result and result['mask'] is not None:
                response.mask = result['mask'].flatten().tolist()
                response.mask_height = result['mask'].shape[0]
                response.mask_width = result['mask'].shape[1]
            else:
                response.mask = []
                response.mask_height = 0
                response.mask_width = 0

            elapsed = (time.time() - start_time) * 1000

            if result['detected']:
                self.get_logger().info(
                    f'Detected "{result["phrase"]}" at '
                    f'({result["center_x"]:.1f}, {result["center_y"]:.1f}) '
                    f'with {result["confidence"]:.2f} confidence '
                    f'({elapsed:.1f}ms)'
                )
            else:
                self.get_logger().warn(f'Object not detected: {response.error_message} ({elapsed:.1f}ms)')

            if self.publish_debug_images:
                all_detections = result.get('all_detections', [])
                self._publish_debug_image(cv_image, result, all_detections)

            return response

        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return self._create_error_response(response, str(e))

    def _process_detection(self, image, request):
        """Process detection with GroundingDINO"""
        if self.use_mock:
            return self._mock_detect(image, request.object_description)
        else:
            return self._detect_object(
                image,
                request.object_description,
                box_threshold=request.box_threshold
            )

    def _detect_object(self, image, text_prompt, box_threshold=0.3):
        """Run GroundingDINO open-vocabulary detection to find the object matching text description"""
        try:
            h, w = image.shape[:2]

            pil_image = PILImage.fromarray(image)

            clean_prompt = text_prompt.replace('_', ' ')
            if not clean_prompt.endswith('.'):
                clean_prompt = clean_prompt + '.'

            self.get_logger().info(f'GroundingDINO detecting: "{clean_prompt}" with threshold {box_threshold}')

            inputs = self.processor(
                images=pil_image,
                text=clean_prompt,
                return_tensors="pt"
            ).to(self.device)

            with torch.no_grad():
                outputs = self.model(**inputs)

            results = self.processor.image_processor.post_process_object_detection(
                outputs,
                threshold=box_threshold,
                target_sizes=[(h, w)]
            )

            result = results[0]

            if len(result['boxes']) == 0:
                return self._create_detection_result(
                    detected=False,
                    error=f'No object found matching "{text_prompt}" at threshold {box_threshold}'
                )

            scores = result['scores'].tolist()
            boxes = result['boxes'].tolist()

            best_idx = int(np.argmax(scores))
            best_score = scores[best_idx]
            best_box = boxes[best_idx]

            x1, y1, x2, y2 = best_box
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            all_detections = []
            for i in range(len(boxes)):
                box = boxes[i]
                score = scores[i]
                all_detections.append({
                    'bbox': [float(box[0]), float(box[1]), float(box[2]), float(box[3])],
                    'center_x': float((box[0] + box[2]) / 2),
                    'center_y': float((box[1] + box[3]) / 2),
                    'confidence': float(score),
                    'phrase': text_prompt
                })

            return self._create_detection_result(
                detected=True,
                confidence=float(best_score),
                center_x=float(cx),
                center_y=float(cy),
                bbox=[float(x1), float(y1), float(x2), float(y2)],
                phrase=text_prompt,
                mask=None,
                all_detections=all_detections
            )

        except Exception as e:
            self.get_logger().error(f'GroundingDINO inference failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return self._create_detection_result(detected=False, error=str(e))

    def _mock_detect(self, image, text_prompt):
        """Mock detection for testing"""
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
                                  center_y=-1.0, bbox=None, phrase='', error='', mask=None,
                                  all_detections=None):
        """Create a standardized detection result dictionary"""
        return {
            'detected': detected,
            'confidence': confidence,
            'center_x': float(center_x),
            'center_y': float(center_y),
            'bbox': [float(v) for v in bbox] if bbox else [],
            'phrase': phrase,
            'error': error,
            'mask': mask,
            'all_detections': all_detections if all_detections else []
        }

    def _create_error_response(self, response, error_message):
        """Create an error response"""
        response.detected = False
        response.confidence = 0.0
        response.center_x = -1.0
        response.center_y = -1.0
        response.bbox = []
        response.phrase = ''
        response.error_message = error_message
        response.mask = []
        response.mask_height = 0
        response.mask_width = 0
        return response

    def _publish_debug_image(self, image, result, all_detections):
        """Publish debug visualization with bounding boxes"""
        try:
            debug_img = cv2.cvtColor(image.copy(), cv2.COLOR_RGB2BGR)

            for i, det in enumerate(all_detections[:5]):
                color = (0, 255, 0) if i == 0 else (0, 165, 255)

                x1, y1, x2, y2 = [int(v) for v in det['bbox']]
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)

                cx, cy = int(det['center_x']), int(det['center_y'])
                cv2.circle(debug_img, (cx, cy), 6, color, -1)
                cv2.circle(debug_img, (cx, cy), 6, (255, 255, 255), 2)

                label = f"#{i+1}: {det['phrase']} ({det['confidence']:.2f})"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(debug_img,
                            (x1, y1 - label_size[1] - 15),
                            (x1 + label_size[0] + 10, y1),
                            color, -1)
                cv2.putText(debug_img, label, (x1 + 5, y1 - 7),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            debug_img_rgb = cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img_rgb, encoding='rgb8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_rgb_optical_frame'
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


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