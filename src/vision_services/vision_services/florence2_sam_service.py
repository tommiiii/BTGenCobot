#!/usr/bin/env python3
"""Florence-2 + SAM Object Detection and Segmentation Service Node

This ROS2 service node provides object detection and segmentation using:
- Florence-2 for text-prompted object detection (VLM-based, understands attributes)
- SAM for fast, accurate segmentation masks

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
    from transformers import AutoProcessor, AutoModelForCausalLM
    from segment_anything import sam_model_registry, SamPredictor
    from PIL import Image as PILImage
    DEPENDENCIES_AVAILABLE = True
    import_error = None
except ImportError as e:
    DEPENDENCIES_AVAILABLE = False
    import_error = str(e)


class Florence2SAMService(Node):
    """ROS2 service node for Florence-2 + SAM object detection and segmentation"""

    def __init__(self):
        super().__init__('florence2_sam_service')
        self._declare_parameters()
        self._setup_device()
        self._initialize_models()
        self._create_service()

    def _declare_parameters(self):
        """Declare and load ROS parameters"""
        self.declare_parameter('use_mock', False)
        self.declare_parameter('florence2_model', 'microsoft/Florence-2-base')
        self.declare_parameter('sam_weights', '/workspace/models/sam/sam_vit_b_01ec64.pth')
        self.declare_parameter('sam_model_type', 'vit_b')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('publish_debug_images', True)

        self.use_mock = self.get_parameter('use_mock').value
        self.florence2_model_name = self.get_parameter('florence2_model').value
        self.sam_weights = self.get_parameter('sam_weights').value
        self.sam_model_type = self.get_parameter('sam_model_type').value
        self.device_param = self.get_parameter('device').value
        self.publish_debug_images = self.get_parameter('publish_debug_images').value

    def _setup_device(self):
        """Setup compute device (CUDA or CPU)"""
        if self.device_param == 'auto':
            self.device = 'cuda' if (DEPENDENCIES_AVAILABLE and torch.cuda.is_available()) else 'cpu'
        else:
            self.device = self.device_param

        self.get_logger().info('Florence-2 + SAM Service Node starting...')
        self.get_logger().info(f'Use mock: {self.use_mock}')
        self.get_logger().info(f'Device: {self.device}')

    def _initialize_models(self):
        """Initialize Florence-2 and SAM models or fallback to mock mode"""
        self.florence2_model = None
        self.florence2_processor = None
        self.sam_predictor = None
        self.bridge = CvBridge()

        if not self.use_mock:
            if not DEPENDENCIES_AVAILABLE:
                self.get_logger().error(f'Dependencies not available: {import_error}')
                self.get_logger().warning('Falling back to MOCK mode')
                self.use_mock = True
            else:
                self._load_models()

        if self.use_mock:
            self.get_logger().warning('Running in MOCK MODE - will return fake detections and masks')

    def _load_models(self):
        """Load Florence-2 and SAM models"""
        try:
            # Load Florence-2
            self.get_logger().info('Loading Florence-2 model...')
            self.get_logger().info(f'Model: {self.florence2_model_name}')

            torch_dtype = torch.float16 if self.device == 'cuda' else torch.float32
            self.florence2_model = AutoModelForCausalLM.from_pretrained(
                self.florence2_model_name,
                torch_dtype=torch_dtype,
                trust_remote_code=True,
                attn_implementation='eager'  # Workaround for transformers compatibility
            ).to(self.device)
            self.florence2_processor = AutoProcessor.from_pretrained(
                self.florence2_model_name,
                trust_remote_code=True
            )
            self.get_logger().info('Florence-2 model loaded successfully')

            # Load SAM
            self.get_logger().info('Loading SAM model...')
            self.get_logger().info(f'Model type: {self.sam_model_type}')
            self.get_logger().info(f'Weights: {self.sam_weights}')
            sam = sam_model_registry[self.sam_model_type](checkpoint=self.sam_weights)
            sam.to(device=self.device)
            self.sam_predictor = SamPredictor(sam)
            self.get_logger().info('SAM model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to load models: {e}')
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
        self.debug_image_pub = self.create_publisher(Image, '/florence2_sam/debug_image', 10)
        self.get_logger().info('Service /detect_object ready')

    def detect_callback(self, request, response):
        """Handle detection service request"""
        try:
            start_time = time.time()
            
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')
            self.get_logger().info(
                f'Detection request: "{request.object_description}" '
                f'(image: {cv_image.shape[1]}x{cv_image.shape[0]})'
            )

            # Process detection
            result = self._process_detection(cv_image, request)

            # Fill response
            response.detected = result['detected']
            response.confidence = result['confidence']
            response.center_x = result['center_x']
            response.center_y = result['center_y']
            response.bbox = result['bbox']
            response.phrase = result['phrase']
            response.error_message = result.get('error', '')

            # Add mask to response
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

            # Publish debug visualization
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
        """Process detection with Florence-2 + SAM"""
        if self.use_mock:
            return self._mock_detect(image, request.object_description)
        else:
            return self._detect_and_segment(image, request.object_description, request.box_threshold)

    def _detect_and_segment(self, image, text_prompt, box_threshold):
        """Run Florence-2 detection + SAM segmentation"""
        try:
            # Convert to PIL Image
            pil_image = PILImage.fromarray(image)
            
            # Use Florence-2's Caption to Phrase Grounding task
            # This takes a text description and finds it in the image with bounding boxes
            task_prompt = '<CAPTION_TO_PHRASE_GROUNDING>'
            prompt = task_prompt + text_prompt
            
            self.get_logger().info(f'Using Florence-2 prompt: "{prompt}"')
            
            # Prepare inputs
            inputs = self.florence2_processor(
                text=prompt,
                images=pil_image,
                return_tensors="pt"
            ).to(self.device, self.florence2_model.dtype)

            # Generate detections (use greedy decoding, no beam search for stability)
            with torch.no_grad():
                generated_ids = self.florence2_model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=1024,
                    num_beams=1,  # Explicitly disable beam search
                    use_cache=False,  # Disable KV cache to avoid past_key_values bug
                    do_sample=False
                )

            # Decode results
            generated_text = self.florence2_processor.batch_decode(
                generated_ids,
                skip_special_tokens=False
            )[0]

            # Parse Florence-2 output
            parsed_answer = self.florence2_processor.post_process_generation(
                generated_text,
                task=task_prompt,
                image_size=(pil_image.width, pil_image.height)
            )

            self.get_logger().info(f'Florence-2 raw output: {parsed_answer}')

            # Extract detections
            if task_prompt not in parsed_answer:
                return self._create_detection_result(
                    detected=False,
                    error=f'No objects found matching "{text_prompt}"'
                )

            detection_data = parsed_answer[task_prompt]
            bboxes = detection_data.get('bboxes', [])
            labels = detection_data.get('labels', [])

            if len(bboxes) == 0:
                return self._create_detection_result(
                    detected=False,
                    error=f'No objects found matching "{text_prompt}"'
                )

            # Store all detections for debug visualization
            all_detections = []
            for i, (bbox, label) in enumerate(zip(bboxes, labels)):
                # Only verify first detection for speed
                if i >= 3:  # Check up to 3 detections
                    continue
                
                x1, y1, x2, y2 = bbox
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                # Florence-2 doesn't provide confidence scores, use 1.0 for all
                # Verify detection with caption
                caption = self._verify_region_caption(pil_image, bbox)
                self.get_logger().info(f"Region caption: {caption}")
                
                # Check if caption matches query
                caption_lower = caption.lower()
                query_words = set(text_prompt.lower().split())
                caption_words = set(caption_lower.split())
                matches = query_words & caption_words
                match_score = len(matches) / len(query_words) if query_words else 0
                
                self.get_logger().info(f"Match score: {match_score:.2f}")
                
                # Skip if no match
                if match_score < 0.5:
                    self.get_logger().info(f"Skipping: score too low")
                    continue
                
                all_detections.append({
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'center_x': float(cx),
                    'center_y': float(cy),
                    'confidence': 1.0,
                    'phrase': label if label else text_prompt
                })

            self.get_logger().info(f'Found {len(all_detections)} detections')
            for i, det in enumerate(all_detections):
                self.get_logger().info(f"  {i}: '{det['phrase']}' at ({det['center_x']:.1f}, {det['center_y']:.1f})")

            # Select the first detection (Florence-2 should return the most relevant)
            if len(all_detections) == 0:
                return self._create_detection_result(
                    detected=False,
                    error=f"No objects found matching \"{text_prompt}\""
                )
            
            selected_detection = all_detections[0]
            x1, y1, x2, y2 = selected_detection['bbox']
            
            # Run SAM segmentation
            self.sam_predictor.set_image(image)
            input_box = np.array([x1, y1, x2, y2])
            masks, iou_predictions, _ = self.sam_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_box[None, :],
                multimask_output=False
            )

            mask = masks[0]
            mask_iou = float(iou_predictions[0])
            self.get_logger().debug(f'Segmentation mask IoU: {mask_iou:.3f}')

            return self._create_detection_result(
                detected=True,
                confidence=selected_detection['confidence'],
                center_x=selected_detection['center_x'],
                center_y=selected_detection['center_y'],
                bbox=selected_detection['bbox'],
                phrase=selected_detection['phrase'],
                mask=mask.astype(np.uint8),
                all_detections=all_detections
            )

        except Exception as e:
            self.get_logger().error(f'Florence-2 + SAM inference failed: {e}')
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

        # Create mock elliptical mask
        mask = np.zeros((h, w), dtype=np.uint8)
        y_indices, x_indices = np.ogrid[:h, :w]
        ellipse_mask = (((x_indices - center_x) / (bbox_width / 2)) ** 2 +
                       ((y_indices - center_y) / (bbox_height / 2)) ** 2) <= 1
        mask[ellipse_mask] = 1

        self.get_logger().info(f'MOCK: Detected "{text_prompt}" at center ({center_x:.1f}, {center_y:.1f})')

        return self._create_detection_result(
            detected=True,
            confidence=0.95,
            center_x=center_x,
            center_y=center_y,
            bbox=[x1, y1, x2, y2],
            phrase=text_prompt,
            mask=mask
        )


    def _verify_region_caption(self, pil_image, bbox):
        """Get caption for a region to verify its contents"""
        try:
            x1, y1, x2, y2 = map(int, bbox)
            pad = 10
            x1 = max(0, x1 - pad)
            y1 = max(0, y1 - pad)
            x2 = min(pil_image.width, x2 + pad)
            y2 = min(pil_image.height, y2 + pad)
            
            cropped = pil_image.crop((x1, y1, x2, y2))
            
            # Use faster CAPTION task
            task = "<CAPTION>"
            inputs = self.florence2_processor(
                text=task,
                images=cropped,
                return_tensors="pt"
            ).to(self.device, self.florence2_model.dtype)
            
            with torch.no_grad():
                generated_ids = self.florence2_model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=512,  # Shorter for speed
                    num_beams=1,
                    use_cache=False,
                    do_sample=False
                )
            
            generated_text = self.florence2_processor.batch_decode(
                generated_ids,
                skip_special_tokens=False
            )[0]
            
            parsed = self.florence2_processor.post_process_generation(
                generated_text,
                task=task,
                image_size=(cropped.width, cropped.height)
            )
            
            if task in parsed:
                return parsed[task]
            return ""
        except Exception as e:
            self.get_logger().warn(f"Region captioning failed: {e}")
            return ""

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
        """Publish debug visualization with top 3 detections"""
        try:
            debug_img = cv2.cvtColor(image.copy(), cv2.COLOR_RGB2BGR)
            
            # Draw mask overlay (only for selected detection)
            if result.get('mask') is not None:
                mask = result['mask']
                overlay = debug_img.copy()
                overlay[mask > 0] = [0, 255, 0]
                debug_img = cv2.addWeighted(debug_img, 0.7, overlay, 0.3, 0)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(debug_img, contours, -1, (0, 255, 0), 2)
            
            # Draw top 3 detections
            top_3 = all_detections[:3]
            colors = [(0, 0, 255), (0, 165, 255), (0, 255, 255)]  # Red, Orange, Yellow
            
            for i, det in enumerate(top_3):
                color = colors[i]
                x1, y1, x2, y2 = [int(v) for v in det['bbox']]
                thickness = 3 if i == 0 else 2
                
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, thickness)
                
                cx, cy = int(det['center_x']), int(det['center_y'])
                cv2.circle(debug_img, (cx, cy), 6, color, -1)
                cv2.circle(debug_img, (cx, cy), 6, (255, 255, 255), 2)
                
                label = f"#{i+1}: {det['phrase']}"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(debug_img, 
                            (x1, y1 - label_size[1] - 15), 
                            (x1 + label_size[0] + 10, y1), 
                            color, -1)
                cv2.putText(debug_img, label, (x1 + 5, y1 - 7), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish
            debug_img_rgb = cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img_rgb, encoding='rgb8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = 'camera_rgb_optical_frame'
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Florence2SAMService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
