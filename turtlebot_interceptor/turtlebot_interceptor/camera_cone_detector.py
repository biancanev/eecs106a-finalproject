#!/usr/bin/env python3
"""
Camera-based Yellow Cone Detector
Uses YOLO (lab8 pattern) or color-based detection to find yellow cones.
Based on lab8 perception stack with mask segmentation and depth estimation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformException
import os
from ament_index_python.packages import get_package_share_directory

# Try to import YOLO (optional - falls back to color-based if not available)
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class CameraConeDetector(Node):
    """Detect yellow cones using camera feed"""
    
    def __init__(self):
        super().__init__('camera_cone_detector')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters (will be updated from camera_info)
        self.camera_info = None
        self.camera_intrinsics = None  # (fx, fy, cx, cy) - lab8 pattern
        self.fx = 525.0  # Default focal length (will be updated)
        self.fy = 525.0
        self.cx = 320.0  # Default principal point
        self.cy = 240.0
        self.image_width = 640
        self.image_height = 480
        
        # YOLO model (optional - lab8 pattern)
        self.model = None
        self.use_yolo = False
        if YOLO_AVAILABLE:
            # Try to load YOLO model
            try:
                package_share_dir = get_package_share_directory('turtlebot_interceptor')
                model_path = os.path.join(package_share_dir, 'utilities', 'segment_bounding_box_cones.pt')
                if os.path.exists(model_path):
                    self.model = YOLO(model_path)
                    self.use_yolo = True
                    self.get_logger().info(f'✅ Loaded YOLO model: {model_path}')
                else:
                    self.get_logger().warn(f'⚠️ YOLO model not found at {model_path}, using color-based detection')
            except Exception as e:
                self.get_logger().warn(f'⚠️ Failed to load YOLO model: {e}, using color-based detection')
        
        # Cone area in meters^2 (lab8 pattern)
        # Yellow cones (obstacles) - original size
        self.CONE_AREA = 0.0208227849  # m^2 (for yellow cones)
        
        # Blue target cone is 4-5x bigger than yellow cones
        # CRITICAL: Target at 2.5-3m appears as 1.4m = depth is ~2x too small
        # Need larger multiplier: if 5x gives 1.4m, need ~7-8x to get 2.5-3m
        # Using 7.0x for more accurate depth (was 5.0x, still too small)
        self.TARGET_CONE_AREA = 0.0208227849 * 7.0  # ~0.146 m^2 (for blue target cone)
        
        # Transform from camera to base_link (lab8 pattern)
        # G = [[0, 0, 1, 0.115],
        #      [-1, 0, 0, 0],
        #      [0, -1, 0, 0],
        #      [0, 0, 0, 1]]
        self.camera_to_base_transform = np.array([
            [0, 0, 1, 0.115],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
        
        # Yellow color range in HSV (cones for obstacles)
        # Expanded range for better detection in various lighting
        self.lower_yellow = np.array([15, 80, 80])   # Lower H, lower S/V for dim lighting
        self.upper_yellow = np.array([35, 255, 255])   # Higher H for orange-yellow
        # Blue color range in HSV (target cone)
        self.lower_blue = np.array([100, 120, 60])
        self.upper_blue = np.array([130, 255, 255])
        
        # Minimum cone size (in pixels) to filter noise
        self.min_cone_area = 150  # Increased from 100 for better noise filtering
        
        # Maximum detection range (meters) - only detect close cones
        self.max_range = 2.0  # 2 meters
        
        # Cone physical dimensions
        self.cone_diameter = 0.15  # 15 cm diameter
        self.cone_radius = self.cone_diameter / 2.0  # 7.5 cm radius
        self.cone_height = 0.3  # 30 cm height (typical traffic cone)
        
        # Robot pose (for transforming to map frame)
        self.robot_pose = None
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Topic names (configurable via parameters)
        # Try /image_raw first (common), fallback to /camera/image_raw
        self.declare_parameter('image_topic', '/image_raw')  # Changed default - camera might publish here
        self.declare_parameter('camera_info_topic', '/camera_info')  # Changed default
        self.declare_parameter('pose_topic', '/amcl_pose')
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'📷 Subscribing to:')
        self.get_logger().info(f'   Image: {image_topic}')
        self.get_logger().info(f'   Camera Info: {camera_info_topic}')
        self.get_logger().info(f'   Pose: {pose_topic}')
        
        # Subscriptions
        # Match camera QoS exactly: BEST_EFFORT (from ros2 topic info)
        compatible_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Camera uses BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Camera image
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            compatible_qos
        )
        
        # Camera info (for intrinsic parameters)
        # Use same QoS as image
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            compatible_qos
        )
        self.get_logger().info(f'📷 Subscribed to camera_info: {camera_info_topic} with BEST_EFFORT QoS')
        
        # Robot pose (for coordinate transforms)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self.pose_callback,
            10
        )
        
        # Publishers
        # Detected cone positions (in map frame) - YELLOW CIRCLES
        self.cones_pub = self.create_publisher(
            MarkerArray,
            '/camera_cones',
            10
        )
        
        # Also publish to local_map frame for local obstacle awareness
        self.cones_local_pub = self.create_publisher(
            MarkerArray,
            '/camera_cones_local',
            10
        )
        
        # Debug image with detections overlaid
        self.debug_image_pub = self.create_publisher(
            Image,
            '/camera_cones_debug',
            10
        )
        
        # Individual cone positions (map frame)
        self.cone_points_pub = self.create_publisher(PointStamped, '/camera_cone_positions', 10)
        # Target (blue cone) positions
        self.target_points_pub = self.create_publisher(PointStamped, '/camera_target_positions', 10)
        # Separate marker publisher for targets to avoid clobbering yellow markers
        self.target_marker_pub = self.create_publisher(MarkerArray, '/camera_target_markers', 10)
        
        # Store confidence for each detection
        self.cone_confidences = {}  # Map cone_id -> confidence
        # Target smoothing/debounce
        self.target_history = []
        self.target_required_streak = 5  # require 5 consecutive frames for a stable target
        self.target_alpha = 0.5  # smoothing
        self.target_valid = False
        
        # Yellow cone temporal smoothing to prevent flickering
        self.cone_tracks = {}  # Map: cone_id -> (smoothed_pos, last_update_time, detection_count)
        self.cone_smoothing_alpha = 0.3  # Lower = more smoothing (less flicker)
        self.cone_min_detections = 1  # Minimum detections before publishing (was 2, lowered to show immediately)
        self.cone_timeout = 3.0  # Remove cones not seen for 1 second
        self.target_pose = None
        
        self.get_logger().info('📷 Camera Cone Detector initialized')
        self.get_logger().info(f'   Yellow range: HSV {self.lower_yellow} - {self.upper_yellow}')
        self.get_logger().info(f'   Blue range: HSV {self.lower_blue} - {self.upper_blue}')
        self.get_logger().info(f'   Min cone area: {self.min_cone_area} pixels')
        self.get_logger().info(f'   Max range: {self.max_range}m')
        self.get_logger().info(f'   Cone diameter: {self.cone_diameter*100:.1f}cm, height: {self.cone_height*100:.1f}cm')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsic parameters (lab8 pattern)"""
        # Log immediately when callback is called
        if not hasattr(self, '_camera_info_callback_called'):
            self.get_logger().info('🔔 camera_info_callback CALLED!')
            self._camera_info_callback_called = True
        
        try:
            self.camera_info = msg
            
            # Extract intrinsics from camera matrix K
            # K matrix is 3x3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            K = list(msg.k)
            if len(K) != 9:
                self.get_logger().error(f'Invalid K matrix size: {len(K)}, expected 9')
                return
            
            self.fx = float(K[0])  # Focal length x
            self.fy = float(K[4])  # Focal length y
            self.cx = float(K[2])  # Principal point x
            self.cy = float(K[5])  # Principal point y
            self.image_width = msg.width
            self.image_height = msg.height
            
            # Store as tuple (lab8 pattern)
            self.camera_intrinsics = (self.fx, self.fy, self.cx, self.cy)
            
            if not hasattr(self, '_camera_info_logged'):
                self.get_logger().info(
                    f'✅ Camera intrinsics received: {self.image_width}x{self.image_height}, '
                    f'fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}'
                )
                self._camera_info_logged = True
            else:
                # Log periodically to confirm it's still being called
                if not hasattr(self, '_camera_info_count'):
                    self._camera_info_count = 0
                self._camera_info_count += 1
                if self._camera_info_count % 100 == 0:
                    self.get_logger().debug(f'📷 Camera info received {self._camera_info_count} times')
        except Exception as e:
            self.get_logger().error(f'❌ Error in camera_info_callback: {e}', exc_info=True)
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store robot pose"""
        self.robot_pose = msg.pose.pose
    
    def image_callback(self, msg: Image):
        """Process camera image to detect cones"""
        # Log immediately when callback is called
        if not hasattr(self, '_image_callback_called'):
            self.get_logger().info('🔔 image_callback CALLED!')
            self._image_callback_called = True
        
        # ALWAYS convert and publish debug image first (even without intrinsics)
        try:
            # Convert ROS image to OpenCV - use 'bgr8' to ensure correct color space
            # This ensures blue stays blue, not orange
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return
        
        # Log first image received
        if not hasattr(self, '_first_image_logged'):
            self.get_logger().info(f'✅ Received first image: {cv_image.shape[1]}x{cv_image.shape[0]}')
            self._first_image_logged = True
        
        # Initialize detection count
        if not hasattr(self, '_detection_count'):
            self._detection_count = 0
        self._detection_count += 1
        
        # If no intrinsics, just publish raw image with status
        if self.camera_intrinsics is None:
            if not hasattr(self, '_no_intrinsics_logged'):
                self.get_logger().warn('⚠️ Waiting for camera_info... (camera_intrinsics is None)')
                self._no_intrinsics_logged = True
            # Publish debug image anyway (raw image with status)
            self.publish_debug_image(cv_image, [])
            return
        
        # Detect cones using heuristic-based color detection
        cones_yellow = self.detect_yellow_cones(cv_image)
        cones_blue = self.detect_blue_cones(cv_image)
        
        # Log detection results periodically
        if self._detection_count % 30 == 0:  # Log every 30 frames (~1 second at 30fps)
            self.get_logger().info(
                f'📷 Processed {self._detection_count} frames, yellow={len(cones_yellow)}, blue={len(cones_blue)}'
            )
        
        # ALWAYS publish debug image (even if no cones) so we can see what camera sees
        self.publish_debug_image(cv_image, cones_yellow + cones_blue)
        
        # Process detections and convert to world coordinates
        if len(cones_yellow) > 0:
            processed_cones = self.process_cone_detections(cones_yellow, cv_image, is_target=False)
            if len(processed_cones) > 0:
                self.publish_cones(processed_cones, msg.header)
                self.publish_cones_local(processed_cones, msg.header)
        if len(cones_blue) > 0:
            processed_targets = self.process_cone_detections(cones_blue, cv_image, is_target=True)
            if len(processed_targets) > 0:
                self.update_and_publish_target(processed_targets, msg.header)
    
    def process_cone_detections(self, cones, cv_image, is_target=False):
        """
        Process detected cones and convert to world coordinates (lab8 pattern).
        
        Args:
            cones: List of detected cones
            cv_image: OpenCV image
            is_target: True if blue target cone (uses larger area), False for yellow cones
        
        Returns list of (world_x, world_y, depth, confidence, pixel_pos) tuples.
        """
        processed = []
        
        if self.camera_intrinsics is None:
            return processed
        
        for cx, cy, w, h, area, mask_pixels in cones:
            # Use mask pixel count for depth estimation (lab8 pattern)
            # Pass is_target flag to use correct cone area (larger for blue target)
            point_3d, depth = self.pixel_to_3d_from_mask(mask_pixels, cy, cx, is_target=is_target)  # u=y, v=x
            
            if point_3d is None:
                continue
            
            # Convert to base_link frame (lab8 pattern)
            goal_point = (self.camera_to_base_transform @ np.array([point_3d[0], point_3d[1], point_3d[2], 1]).reshape(4, 1)).flatten()
            
            # Transform to map frame
            if self.robot_pose is not None:
                world_pos = self.base_link_to_map_frame(goal_point[:3])
                if world_pos is not None:
                    # Compute confidence based on detection quality
                    confidence = self.compute_detection_confidence(area, mask_pixels, depth)
                    processed.append((world_pos[0], world_pos[1], depth, confidence, (cx, cy)))
                    
                    # Enhanced logging for target cones to debug position errors
                    if is_target:
                        robot_pos = np.array([self.robot_pose.position.x, self.robot_pose.position.y])
                        relative_pos = world_pos[:2] - robot_pos
                        q = self.robot_pose.orientation
                        robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                        self.get_logger().info(
                            f'🎯 TARGET: camera_3d=({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f}), '
                            f'base_link=({goal_point[0]:.3f}, {goal_point[1]:.3f}), '
                            f'robot=({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, yaw={math.degrees(robot_yaw):.1f}°), '
                            f'world=({world_pos[0]:.3f}, {world_pos[1]:.3f}), '
                            f'relative=({relative_pos[0]:.3f}, {relative_pos[1]:.3f}), '
                            f'depth={depth:.3f}m, pixels={mask_pixels}, area={self.TARGET_CONE_AREA:.6f}m²'
                        )
                    else:
                        self.get_logger().debug(
                            f'📷 Cone: depth={depth:.3f}m, world=({world_pos[0]:.3f}, {world_pos[1]:.3f})'
                        )
        
        return processed
    
    def compute_detection_confidence(self, area, mask_pixels, depth):
        """
        Compute confidence based on detection quality heuristics.
        Higher confidence for larger, more solid detections at reasonable distances.
        """
        # Factor 1: Size (larger detections are more reliable)
        size_factor = min(1.0, area / (self.min_cone_area * 3))
        
        # Factor 2: Pixel density (more pixels = more reliable)
        pixel_factor = min(1.0, mask_pixels / 1000)
        
        # Factor 3: Distance (closer = more reliable for camera)
        distance_factor = self.compute_camera_confidence(depth)
        
        # Combined confidence
        confidence = (size_factor * 0.3 + pixel_factor * 0.3 + distance_factor * 0.4)
        
        return np.clip(confidence, 0.0, 1.0)
    
    def detect_cones_yolo(self, cv_image, header):
        """
        Detect cones using YOLO model (lab8 pattern).
        Returns list of (world_x, world_y, depth, confidence, pixel_pos) tuples.
        """
        cones = []
        
        if self.model is None or self.camera_intrinsics is None:
            return cones
        
        results = self.model(cv_image, verbose=False)
        img_height, img_width = cv_image.shape[:2]
        
        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                
                for i, mask in enumerate(masks):
                    # Get number of pixels in mask (lab8 pattern)
                    pixel_count = np.sum(mask)
                    
                    if pixel_count == 0:
                        continue
                    
                    # Calculate depth from pixel count (lab8 pattern)
                    # depth = sqrt((fx * fy * CONE_AREA) / pixel_count)
                    fx, fy, cx, cy = self.camera_intrinsics
                    depth = np.sqrt((fx * fy * self.CONE_AREA) / pixel_count)
                    
                    # Get u, v of cone center in image coordinates (lab8 pattern)
                    u_indices, v_indices = np.nonzero(mask)
                    u = float(np.mean(u_indices))  # Row (y in image)
                    v = float(np.mean(v_indices))  # Column (x in image)
                    
                    # Find X, Y, Z of cone in camera frame (lab8 pattern)
                    X = (v - cx) * depth / fx  # Note: v is x-coordinate in image
                    Y = (u - cy) * depth / fy  # Note: u is y-coordinate in image
                    Z = depth
                    
                    # Convert to base_link frame (lab8 pattern)
                    goal_point = (self.camera_to_base_transform @ np.array([X, Y, Z, 1]).reshape(4, 1)).flatten()
                    
                    # Transform to map frame if robot pose available
                    if self.robot_pose is not None:
                        world_pos = self.base_link_to_map_frame(goal_point[:3])
                        if world_pos is not None:
                            cones.append((world_pos[0], world_pos[1], depth, 0.9, (v, u)))  # High confidence for YOLO
                    
                    self.get_logger().info(
                        f'📷 YOLO Cone {i+1}: depth={depth:.3f}m, '
                        f'base_link=({goal_point[0]:.3f}, {goal_point[1]:.3f}), '
                        f'pixel_count={pixel_count}'
                    )
            else:
                self.get_logger().debug('No cones spotted by YOLO')
        
        return cones
    
    def base_link_to_map_frame(self, point_base):
        """Transform point from base_link to map frame"""
        if self.robot_pose is None:
            return None
        
        try:
            # Get robot pose
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            
            # Get robot orientation
            q = self.robot_pose.orientation
            import math
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            
            # Transform to map frame
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            point_map_x = robot_x + cos_yaw * point_base[0] - sin_yaw * point_base[1]
            point_map_y = robot_y + sin_yaw * point_base[0] + cos_yaw * point_base[1]
            
            return np.array([point_map_x, point_map_y, 0.0])
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return None
    
    def detect_yellow_cones(self, cv_image):
        """
        Detect yellow cones using dual-layer convolution grid + triangular shape detection.
        Uses primitive knowledge: all cones are yellow and triangular.
        Returns list of (center_x, center_y, width, height, area, mask_pixels) in image coordinates.
        """
        # Convert BGR to HSV (better for color detection)
        try:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            self.get_logger().warn(f'Failed to convert to HSV: {e}, image shape: {cv_image.shape}')
            return []
        
        # DEBUG: Sample HSV values from center of image (where cone might be)
        if not hasattr(self, '_hsv_sampled'):
            center_y, center_x = hsv.shape[0] // 2, hsv.shape[1] // 2
            sample_hsv = hsv[center_y-50:center_y+50, center_x-50:center_x+50]
            if sample_hsv.size > 0:
                avg_h = np.mean(sample_hsv[:, :, 0])
                avg_s = np.mean(sample_hsv[:, :, 1])
                avg_v = np.mean(sample_hsv[:, :, 2])
                self.get_logger().info(f'🔍 Sample HSV (center): H={avg_h:.1f}, S={avg_s:.1f}, V={avg_v:.1f}')
                self.get_logger().info(f'🔍 Using range: H=[{self.lower_yellow[0]}-{self.upper_yellow[0]}], S=[{self.lower_yellow[1]}-{self.upper_yellow[1]}], V=[{self.lower_yellow[2]}-{self.upper_yellow[2]}]')
                self._hsv_sampled = True
        
        # LAYER 1: Detect ONLY yellow (NOT blue, NOT green)
        # Yellow range (hue 20-30 in OpenCV HSV) - pure yellow only
        mask1 = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # LAYER 2: Convolution-based refinement
        # Use small convolution kernel to smooth and enhance regions
        kernel_smooth = np.ones((3, 3), np.uint8) / 9.0
        mask1_float = mask1.astype(np.float32)
        mask2 = cv2.filter2D(mask1_float, -1, kernel_smooth)
        mask2 = (mask2 > 100).astype(np.uint8) * 255  # Threshold after convolution
        
        # Combine both layers (OR operation - more permissive, either layer can detect)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # REDUCED morphological operations - they might be removing the yellow
        # Use smaller kernel and less aggressive operations
        kernel = np.ones((3, 3), np.uint8)  # Smaller kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove noise (less aggressive)
        # Skip CLOSE for now - it might be removing valid yellow regions
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # DISABLED
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Debug: Log yellow pixel count
        yellow_pixels = np.sum(mask > 0)
        yellow_percent = 100 * yellow_pixels / mask.size if mask.size > 0 else 0
        if not hasattr(self, '_yellow_pixel_logged') or self._detection_count % 30 == 0:
            self.get_logger().info(f'🔍 Yellow mask: {yellow_pixels} pixels ({yellow_percent:.1f}% of image)')
            if not hasattr(self, '_yellow_pixel_logged'):
                self._yellow_pixel_logged = True
        
        cones = []
        debug_info = {'total_contours': len(contours), 'filtered': {}}
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Heuristic 1: Minimum area filter
            if area < self.min_cone_area:
                debug_info['filtered']['area'] = debug_info['filtered'].get('area', 0) + 1
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w / 2
            center_y = y + h / 2
            
            # Heuristic 2: Aspect ratio (cones are roughly vertical/tall)
            # Cones are taller than they are wide
            aspect_ratio = h / w if w > 0 else 0
            if aspect_ratio < 0.8 or aspect_ratio > 5.0:  # Reasonable range for cones
                debug_info['filtered']['aspect'] = debug_info['filtered'].get('aspect', 0) + 1
                continue
            
            # Heuristic 3: TRIANGULAR SHAPE DETECTION (cones are triangular)
            # Approximate contour to polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Check if shape is roughly triangular (3-8 vertices for cone - allows for some noise)
            num_vertices = len(approx)
            if num_vertices < 3 or num_vertices > 12:  # Too few or too many vertices = not cone-like
                debug_info['filtered']['vertices'] = debug_info['filtered'].get('vertices', 0) + 1
                continue
            
            # Check triangularity: top point should be narrow, base should be wide
            # Create ROI mask for this contour
            mask_roi = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(mask_roi, [contour], -1, 255, -1)
            
            # Sample width at top and bottom thirds of bounding box
            # For triangular cones: top is narrow, bottom is wide
            top_row = max(0, int(y + h * 0.2))  # 20% from top of bbox
            bottom_row = min(mask.shape[0] - 1, int(y + h * 0.8))  # 80% from top of bbox
            
            # Count yellow pixels in horizontal slices within the contour
            if top_row < mask.shape[0] and bottom_row < mask.shape[0] and x+w <= mask.shape[1]:
                top_slice = mask_roi[top_row, x:x+w]
                bottom_slice = mask_roi[bottom_row, x:x+w]
                
                top_width = np.sum(top_slice > 0)
                bottom_width = np.sum(bottom_slice > 0)
                
                # Heuristic: Bottom should be wider than top (cone shape)
                # Cones are wider at the base than at the top
                if bottom_width > 0 and top_width / bottom_width > 0.85:  # Top should be at least 15% narrower
                    debug_info['filtered']['triangle'] = debug_info['filtered'].get('triangle', 0) + 1
                    continue
            
            # Heuristic 4: Solidity (cones are relatively solid shapes)
            # Cones should be fairly solid (not too hollow or fragmented)
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < 0.5:  # At least 50% solid (allows for some noise/occlusion)
                debug_info['filtered']['solidity'] = debug_info['filtered'].get('solidity', 0) + 1
                continue
            
            # Heuristic 5: Sample pixels within contour to validate yellow color
            # mask_roi already created above for triangular check
            # Count yellow pixels in the region
            yellow_pixels = np.sum(mask_roi > 0)
            total_pixels = area
            yellow_ratio = yellow_pixels / total_pixels if total_pixels > 0 else 0
            
            # Heuristic: At least 30% of the region should be yellow (VERY permissive)
            if yellow_ratio < 0.3:
                debug_info['filtered']['yellow_ratio'] = debug_info['filtered'].get('yellow_ratio', 0) + 1
                continue
            
            # Heuristic 6: Check if region is in lower half of image (cones are on ground)
            # Cones should be in the lower portion of the image (they sit on the ground)
            img_height = cv_image.shape[0]
            if center_y < img_height * 0.2:  # Too high in image (unlikely to be a cone on ground)
                debug_info['filtered']['position'] = debug_info['filtered'].get('position', 0) + 1
                continue
            
            # Get actual mask pixels for depth estimation (lab8 pattern)
            mask_pixels = yellow_pixels
            
            cones.append((center_x, center_y, w, h, area, mask_pixels))
        
        # Log debug info periodically
        if self._detection_count % 30 == 0:
            if len(contours) == 0:
                self.get_logger().warn(f'⚠️ NO CONTOURS FOUND! Yellow mask has {yellow_pixels} pixels ({yellow_percent:.1f}%)')
            else:
                self.get_logger().info(
                    f'🔍 Detection: {debug_info["total_contours"]} contours found, '
                    f'{len(cones)} passed, filtered: {debug_info["filtered"]}'
                )
                if len(cones) == 0 and len(contours) > 0:
                    self.get_logger().warn(f'⚠️ ALL {len(contours)} CONTOURS FILTERED OUT! Check filters above.')
        
        return cones

    def detect_blue_cones(self, cv_image):
        """Detect blue cones (targets) using similar heuristics to yellow."""
        cones = []
        try:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            self.get_logger().warn(f'Failed to convert to HSV (blue): {e}')
            return cones

        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_cone_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w / 2
            center_y = y + h / 2
            aspect_ratio = h / w if w > 0 else 0
            if aspect_ratio < 0.8 or aspect_ratio > 5.0:
                continue
            mask_pixels = int(np.sum(mask[y:y+h, x:x+w] > 0))
            if mask_pixels < self.min_cone_area:
                continue
            cones.append((center_x, center_y, w, h, area, mask_pixels))
        return cones
    
    def pixel_to_3d_from_mask(self, mask_pixels, u, v, is_target=False):
        """
        Convert pixel coordinates to 3D using mask pixel count (lab8 pattern).
        Uses heuristic: depth = sqrt((fx * fy * CONE_AREA) / pixel_count)
        
        Args:
            mask_pixels: Number of pixels in the mask
            u, v: Pixel coordinates (u=y, v=x in image)
            is_target: True for blue target cone (uses TARGET_CONE_AREA), False for yellow (uses CONE_AREA)
        
        Returns:
            (x, y, z) in camera frame, estimated_depth
        """
        if self.camera_intrinsics is None or mask_pixels == 0:
            return None, None
        
        fx, fy, cx, cy = self.camera_intrinsics
        
        # Depth estimation from pixel count (lab8 pattern)
        # Use larger area for blue target cone (4.5x bigger), smaller for yellow cones
        cone_area = self.TARGET_CONE_AREA if is_target else self.CONE_AREA
        depth = np.sqrt((fx * fy * cone_area) / mask_pixels)
        depth = np.clip(depth, 0.2, self.max_range)
        
        # Log which area is being used (first time only)
        if not hasattr(self, '_cone_area_logged'):
            self.get_logger().info(
                f'📐 Depth calculation: Yellow cones use area={self.CONE_AREA:.6f} m², '
                f'Blue target uses area={self.TARGET_CONE_AREA:.6f} m² (4.5x larger)'
            )
            self._cone_area_logged = True
        
        # Convert pixel to 3D (lab8 pattern: v=x, u=y)
        X = (v - cx) * depth / fx  # v is x-coordinate in image
        Y = (u - cy) * depth / fy  # u is y-coordinate in image
        Z = depth
        
        return np.array([X, Y, Z]), depth
    
    def pixel_to_3d(self, u, v, pixel_height=None):
        """
        Convert pixel coordinates to 3D point in camera frame (fallback method).
        Uses height-based estimation when mask pixels not available.
        
        Args:
            u, v: Pixel coordinates (v=x, u=y in image)
            pixel_height: Height of cone in pixels (for depth estimation)
        
        Returns:
            (x, y, z) in camera frame, estimated_depth
        """
        # Depth estimation from cone size (fallback method)
        real_cone_height = 0.3  # meters
        
        if pixel_height is not None and pixel_height > 0:
            # Depth = (focal_length * real_height) / pixel_height
            estimated_depth = (self.fy * real_cone_height) / pixel_height
            estimated_depth = np.clip(estimated_depth, 0.2, self.max_range)
        else:
            estimated_depth = 1.0
        
        # Convert pixel to 3D (lab8 pattern: v=x, u=y)
        x = (v - self.cx) * estimated_depth / self.fx
        y = (u - self.cy) * estimated_depth / self.fy
        z = estimated_depth
        
        return np.array([x, y, z]), estimated_depth
    
    def compute_camera_confidence(self, distance):
        """
        Compute camera confidence based on distance.
        Camera is more accurate when close, LIDAR when far.
        
        Confidence function:
        - At 0.5m: camera confidence = 0.9 (very high)
        - At 1.0m: camera confidence = 0.7 (high)
        - At 1.5m: camera confidence = 0.5 (medium)
        - At 2.0m: camera confidence = 0.3 (low)
        - Beyond 2.0m: camera confidence = 0.1 (very low)
        
        Returns: confidence in [0, 1]
        """
        if distance < 0.5:
            return 0.9  # Very close - camera is very accurate
        elif distance < 1.0:
            # Linear interpolation: 0.5m -> 0.9, 1.0m -> 0.7
            return 0.9 - 0.2 * (distance - 0.5) / 0.5
        elif distance < 1.5:
            # Linear interpolation: 1.0m -> 0.7, 1.5m -> 0.5
            return 0.7 - 0.2 * (distance - 1.0) / 0.5
        elif distance < 2.0:
            # Linear interpolation: 1.5m -> 0.5, 2.0m -> 0.3
            return 0.5 - 0.2 * (distance - 1.5) / 0.5
        else:
            return 0.1  # Far - camera not reliable
    
    def camera_to_map_frame(self, point_camera):
        """
        Transform point from camera frame to map frame.
        Requires robot pose and camera-to-base_link transform.
        """
        if self.robot_pose is None:
            return None
        
        try:
            # Get transform from camera to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',  # Assuming camera is mounted on camera_link
                rclpy.time.Time()
            )
            
            # Transform point to base_link
            # (simplified - full transform would use quaternions)
            # For now, assume camera is forward-facing on base_link
            point_base = point_camera  # Simplified
            
            # Transform to map frame using robot pose
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            robot_z = self.robot_pose.position.z
            
            # Get robot orientation
            q = self.robot_pose.orientation
            # Convert quaternion to euler (simplified)
            # For 2D, we only care about yaw
            import math
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            
            # Rotate point by robot orientation
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            # Transform to map frame
            point_map_x = robot_x + cos_yaw * point_base[0] - sin_yaw * point_base[1]
            point_map_y = robot_y + sin_yaw * point_base[0] + cos_yaw * point_base[1]
            
            return np.array([point_map_x, point_map_y, 0.0])
            
        except TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None
    
    def publish_cones(self, cones, header):
        """Publish detected cones with temporal smoothing to prevent flickering"""
        if self.robot_pose is None:
            return
        
        robot_pos = np.array([self.robot_pose.position.x, self.robot_pose.position.y])
        current_time = self.get_clock().now()
        current_time_sec = current_time.nanoseconds / 1e9
        
        # Step 1: Update cone tracks with new detections (temporal smoothing)
        detected_positions = {}
        for cone_data in cones:
            if len(cone_data) != 5:
                continue
            
            world_x, world_y, depth, confidence, pixel_pos = cone_data
            point_map = np.array([world_x, world_y])
            
            # Compute distance from robot
            distance = np.linalg.norm(point_map[:2] - robot_pos)
            if distance > self.max_range:
                continue
            
            # Find closest existing track (within 0.2m)
            closest_track_id = None
            min_distance = 0.2  # 20cm matching radius
            
            for track_id, (track_pos, last_time, count) in self.cone_tracks.items():
                dist = np.linalg.norm(point_map[:2] - track_pos)
                if dist < min_distance:
                    min_distance = dist
                    closest_track_id = track_id
            
            if closest_track_id is not None:
                # Update existing track with temporal smoothing
                old_pos, old_time, old_count = self.cone_tracks[closest_track_id]
                smoothed_pos = (1 - self.cone_smoothing_alpha) * old_pos + self.cone_smoothing_alpha * point_map[:2]
                self.cone_tracks[closest_track_id] = (smoothed_pos, current_time_sec, old_count + 1)
                detected_positions[closest_track_id] = (smoothed_pos, confidence, old_count + 1)
            else:
                # New cone - create track
                track_id = f"cone_{len(self.cone_tracks)}"
                self.cone_tracks[track_id] = (point_map[:2].copy(), current_time_sec, 1)
                detected_positions[track_id] = (point_map[:2], confidence, 1)
        
        # Step 2: Remove stale tracks (not seen recently)
        stale_tracks = []
        for track_id, (track_pos, last_time, count) in self.cone_tracks.items():
            if current_time_sec - last_time > self.cone_timeout:
                stale_tracks.append(track_id)
        for track_id in stale_tracks:
            del self.cone_tracks[track_id]
        
        # Step 3: Publish only stable cones (seen multiple times)
        marker_array = MarkerArray()
        published_cones = []
        
        # Log detection status
        if len(detected_positions) > 0:
            self.get_logger().debug(
                f'📷 Processing {len(detected_positions)} cone tracks, min_detections={self.cone_min_detections}'
            )
        
        for track_id, (smoothed_pos, confidence, detection_count) in detected_positions.items():
            # Publish immediately (min_detections=1 now)
            if detection_count < self.cone_min_detections:
                self.get_logger().debug(
                    f'⏳ Skipping track {track_id}: {detection_count} < {self.cone_min_detections} detections'
                )
                continue
            
            # Create BRIGHT YELLOW CYLINDER marker - CLEAN and STABLE
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "camera_cones"
            marker.id = len(marker_array.markers)  # Sequential IDs (0, 1, 2, ...)
            # CRITICAL: Use CYLINDER (3) not SPHERE (2) for yellow cones
            marker.type = Marker.CYLINDER  # CYLINDER = 3, SPHERE = 2
            marker.action = Marker.ADD  # ADD = 0
            
            marker.pose.position.x = float(smoothed_pos[0])
            marker.pose.position.y = float(smoothed_pos[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            
            # Cone dimensions (cylinder: x/y = diameter, z = height)
            marker.scale.x = self.cone_diameter  # 0.15m diameter
            marker.scale.y = self.cone_diameter  # 0.15m diameter
            marker.scale.z = self.cone_height    # 0.3m height
            
            # BRIGHT YELLOW color - CRITICAL: r=1.0, g=1.0, b=0.0 (NOT purple!)
            marker.color.r = 1.0  # Full red
            marker.color.g = 1.0  # Full green (yellow = red + green)
            marker.color.b = 0.0  # ZERO blue (any blue makes it purple/white)
            marker.color.a = 1.0  # Fully opaque
            
            # Set lifetime to prevent stale markers
            marker.lifetime.sec = 3  # 3 second lifetime
            
            marker_array.markers.append(marker)
            published_cones.append((smoothed_pos, confidence))
            
            # Publish as PointStamped (only stable cones)
            point_msg = PointStamped()
            point_msg.header.frame_id = "map"
            point_msg.header.stamp = marker.header.stamp
            point_msg.point.x = float(smoothed_pos[0])
            point_msg.point.y = float(smoothed_pos[1])
            point_msg.point.z = 0.0
            self.cone_points_pub.publish(point_msg)
        
        # Step 4: Publish markers with proper cleanup
        # CRITICAL: Always publish DELETEALL first to clear old markers
        delete_markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "camera_cones"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL  # DELETEALL = 3
        delete_markers.markers.append(delete_marker)
        self.cones_pub.publish(delete_markers)
        
        # Publish new markers - they will replace old ones
        # If no markers, the DELETEALL above will clear everything
        if len(marker_array.markers) > 0:
            self.cones_pub.publish(marker_array)
            if not hasattr(self, '_cone_pub_count'):
                self._cone_pub_count = 0
            self._cone_pub_count += 1
            # Log every publish to debug visibility issues
            m = marker_array.markers[0]
            self.get_logger().info(
                f'📷 Published {len(marker_array.markers)} YELLOW CYLINDERS: '
                f'type={m.type} (CYLINDER=3), color=({m.color.r:.1f}, {m.color.g:.1f}, {m.color.b:.1f}), '
                f'pos=({m.pose.position.x:.2f}, {m.pose.position.y:.2f}), '
                f'tracks={len(self.cone_tracks)}, min_detections={self.cone_min_detections}'
            )
        else:
            # Log when no markers to publish
            if len(detected_positions) == 0:
                self.get_logger().debug('📷 No cone detections to publish')
            else:
                self.get_logger().warn(
                    f'⚠️ {len(detected_positions)} tracks but 0 markers published! '
                    f'All tracks have < {self.cone_min_detections} detections'
                )
                    self.get_logger().info(
                        f'📷 No cones to publish (tracks: {len(self.cone_tracks)})'
                    )
    
    def publish_cones_local(self, cones, header):
        """Publish detected cones in local_map frame (relative to robot)"""
        marker_array = MarkerArray()
        
        if self.robot_pose is None:
            return
        
        robot_pos = np.array([self.robot_pose.position.x, self.robot_pose.position.y])
        
        # Process cones (format: world_x, world_y, depth, confidence, pixel_pos)
        for i, cone_data in enumerate(cones):
            if len(cone_data) == 5:
                world_x, world_y, depth, confidence, pixel_pos = cone_data
                point_map = np.array([world_x, world_y])
            else:
                continue
            
            # Compute distance from robot
            distance = np.linalg.norm(point_map[:2] - robot_pos)
            
            # Check range
            if distance > self.max_range:
                continue
            
            # Convert to local_map frame (relative to robot)
            point_local = point_map[:2] - robot_pos
            
            # Create BRIGHT YELLOW marker for local map
            marker = Marker()
            marker.header.frame_id = "base_link"  # Local frame relative to robot
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "camera_cones_local"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(point_local[0])
            marker.pose.position.y = float(point_local[1])
            marker.pose.position.z = 0.0
            
            marker.pose.orientation.w = 1.0
            
            # Use actual cone dimensions (15 cm diameter)
            marker.scale.x = self.cone_diameter  # 15 cm diameter
            marker.scale.y = self.cone_diameter  # 15 cm diameter
            marker.scale.z = self.cone_height  # 30 cm height
            
            # BRIGHT YELLOW
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            
            marker_array.markers.append(marker)
        
        # Always publish (even if empty) to clear old markers
        if len(marker_array.markers) > 0:
            self.cones_local_pub.publish(marker_array)
        else:
            # Publish DELETEALL to clear stale markers
            delete_markers = MarkerArray()
            delete_marker = Marker()
            delete_marker.header.frame_id = "base_link"
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "camera_cones_local"
            delete_marker.id = 0
            delete_marker.action = Marker.DELETEALL
            delete_markers.markers.append(delete_marker)
            self.cones_local_pub.publish(delete_markers)

    def publish_targets(self, targets, header):
        """Publish blue cones (targets) as PointStamped + markers."""
        # Deprecated in favor of update_and_publish_target
        return

    def update_and_publish_target(self, targets, header):
        """Debounce and smooth blue target detections, publish stable PointStamped + markers."""
        if len(targets) == 0:
            return
        # pick closest detection
        closest = min(targets, key=lambda t: math.hypot(t[0], t[1]))
        det = np.array([closest[0], closest[1]])
        self.target_history.append(det)
        if len(self.target_history) > self.target_required_streak:
            self.target_history.pop(0)

        if len(self.target_history) < self.target_required_streak:
            return  # wait for enough consecutive frames

        if self.target_pose is None:
            smoothed = det
        else:
            smoothed = self.target_alpha * det + (1 - self.target_alpha) * self.target_pose
        self.target_pose = smoothed
        self.target_valid = True

        # Publish stable point
        pt_msg = PointStamped()
        pt_msg.header = header
        pt_msg.point.x = float(smoothed[0])
        pt_msg.point.y = float(smoothed[1])
        pt_msg.point.z = 0.0
        self.target_points_pub.publish(pt_msg)

        # Publish marker for target (blue sphere - different namespace from yellow cones)
        marker_array = MarkerArray()
        # First delete old target markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "camera_targets"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Then add new target marker (blue sphere)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_targets"  # Different namespace from yellow cones
        marker.id = 0
        marker.type = Marker.SPHERE  # SPHERE for target (blue)
        marker.action = Marker.ADD
        marker.pose.position.x = float(smoothed[0])
        marker.pose.position.y = float(smoothed[1])
        marker.pose.position.z = 0.1
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 0.0  # Blue color
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.lifetime.sec = 1
        marker_array.markers.append(marker)
        self.target_marker_pub.publish(marker_array)
    
    def publish_debug_image(self, cv_image, cones):
        """Publish debug image - EXACT COPY of original with only bounding boxes/text overlays"""
        # DIRECT COPY - no modifications to image data, no color conversions
        # This ensures blue stays blue, colors are preserved exactly
        debug_image = np.copy(cv_image)  # Use np.copy to ensure no reference issues
        
        # Store processed cone positions for overlay
        processed_positions = {}
        if len(cones) > 0 and self.robot_pose is not None and self.camera_intrinsics is not None:
            # Process cones to get world positions for display
            processed = self.process_cone_detections(cones, cv_image)
            for cone_data in processed:
                if len(cone_data) == 5:
                    world_x, world_y, depth, confidence, pixel_pos = cone_data
                    processed_positions[pixel_pos] = (world_x, world_y, depth, confidence)
        
        # Handle format: (cx, cy, w, h, area, mask_pixels)
        for cone_data in cones:
            if len(cone_data) == 6:
                cx, cy, w, h, area, mask_pixels = cone_data
            elif len(cone_data) == 5:
                cx, cy, w, h, area = cone_data
                mask_pixels = area  # Fallback
            else:
                continue
            
            # Draw bounding box
            x = int(cx - w/2)
            y = int(cy - h/2)
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            
            # Draw center
            cv2.circle(debug_image, (int(cx), int(cy)), 5, (0, 255, 255), -1)
            
            # Draw info with world position if available
            pixel_pos = (int(cx), int(cy))
            if pixel_pos in processed_positions:
                world_x, world_y, depth, confidence = processed_positions[pixel_pos]
                # Show detailed position info with diameter
                info_text = f'Map: ({world_x:.2f}, {world_y:.2f}) | {depth:.2f}m | conf:{confidence:.2f} | D:{self.cone_diameter*100:.0f}cm'
                # Also show base_link position if available
                if self.robot_pose is not None:
                    robot_x = self.robot_pose.position.x
                    robot_y = self.robot_pose.position.y
                    rel_x = world_x - robot_x
                    rel_y = world_y - robot_y
                    dist = np.sqrt(rel_x**2 + rel_y**2)
                    info_text2 = f'Rel: ({rel_x:.2f}, {rel_y:.2f}) | Dist: {dist:.2f}m'
                    cv2.putText(debug_image, info_text2, (x, y + h + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
            else:
                info_text = f'{area:.0f}px, {mask_pixels:.0f}mask (no pose) | D:{self.cone_diameter*100:.0f}cm'
            cv2.putText(debug_image, info_text, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Add status text
        status_text = f'Cones: {len(cones)} | Intrinsics: {"OK" if self.camera_intrinsics else "WAIT"} | Pose: {"OK" if self.robot_pose else "WAIT"}'
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Publish EXACT copy - use same encoding as input
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
