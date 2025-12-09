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
        self.CONE_AREA = 0.0208227849  # m^2
        
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
        
        # Yellow color range in HSV (for yellow cones)
        # VERY WIDE range to catch any yellow
        self.lower_yellow = np.array([10, 50, 50])   # Very permissive lower bound
        self.upper_yellow = np.array([40, 255, 255])   # Very permissive upper bound
        
        # Minimum cone size (in pixels) to filter noise
        self.min_cone_area = 500  # pixels
        
        # Maximum detection range (meters) - only detect close cones
        self.max_range = 2.0  # 2 meters
        
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
        # Detected cone positions (in map frame)
        self.cones_pub = self.create_publisher(
            MarkerArray,
            '/camera_cones',
            10
        )
        
        # Debug image with detections overlaid
        self.debug_image_pub = self.create_publisher(
            Image,
            '/camera_cones_debug',
            10
        )
        
        # Individual cone positions with confidence (for MPC integration)
        # Create custom message type for cone with confidence
        from geometry_msgs.msg import Point
        from std_msgs.msg import Float32
        
        # For now, use PointStamped and add confidence in a separate topic
        # Or create a custom message (better approach)
        self.cone_points_pub = self.create_publisher(
            PointStamped,
            '/camera_cone_positions',
            10
        )
        
        # Store confidence for each detection
        self.cone_confidences = {}  # Map cone_id -> confidence
        
        self.get_logger().info('📷 Camera Cone Detector initialized')
        self.get_logger().info(f'   Yellow range: HSV {self.lower_yellow} - {self.upper_yellow}')
        self.get_logger().info(f'   Min cone area: {self.min_cone_area} pixels')
        self.get_logger().info(f'   Max range: {self.max_range}m')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsic parameters (lab8 pattern)"""
        # Log immediately when callback is called
        if not hasattr(self, '_camera_info_callback_called'):
            self.get_logger().info('🔔 camera_info_callback CALLED!')
            self._camera_info_callback_called = True
        
        try:
            self.camera_info = msg
            
            # Extract intrinsics from camera matrix K (lab8 pattern)
            # K matrix is 3x3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            K = msg.k
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
        """Process camera image to detect yellow cones (lab8 pattern)"""
        # Log immediately when callback is called
        if not hasattr(self, '_image_callback_called'):
            self.get_logger().info('🔔 image_callback CALLED!')
            self._image_callback_called = True
        
        # ALWAYS convert and publish debug image first (even without intrinsics)
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
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
        cones = self.detect_yellow_cones(cv_image)
        
        # Log detection results periodically
        if self._detection_count % 30 == 0:  # Log every 30 frames (~1 second at 30fps)
            self.get_logger().info(f'📷 Processed {self._detection_count} frames, found {len(cones)} cone candidates')
        
        # ALWAYS publish debug image (even if no cones) so we can see what camera sees
        self.publish_debug_image(cv_image, cones)
        
        # Process detections and convert to world coordinates
        if len(cones) > 0:
            processed_cones = self.process_cone_detections(cones, cv_image)
            if len(processed_cones) > 0:
                self.publish_cones(processed_cones, msg.header)
            elif self.robot_pose is None:
                if not hasattr(self, '_no_pose_logged'):
                    self.get_logger().warn('⚠️ Cones detected but robot_pose is None (waiting for /amcl_pose)')
                    self._no_pose_logged = True
    
    def process_cone_detections(self, cones, cv_image):
        """
        Process detected cones and convert to world coordinates (lab8 pattern).
        Returns list of (world_x, world_y, depth, confidence, pixel_pos) tuples.
        """
        processed = []
        
        if self.camera_intrinsics is None:
            return processed
        
        for cx, cy, w, h, area, mask_pixels in cones:
            # Use mask pixel count for depth estimation (lab8 pattern)
            point_3d, depth = self.pixel_to_3d_from_mask(mask_pixels, cy, cx)  # u=y, v=x
            
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
                    
                    self.get_logger().info(
                        f'📷 Cone detected: depth={depth:.3f}m, '
                        f'base_link=({goal_point[0]:.3f}, {goal_point[1]:.3f}), '
                        f'world=({world_pos[0]:.3f}, {world_pos[1]:.3f}), '
                        f'pixels={mask_pixels}, confidence={confidence:.2f}'
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
        
        # LAYER 1: Initial yellow color mask
        mask1 = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # LAYER 2: Convolution-based refinement
        # Use small convolution kernel to smooth and enhance yellow regions
        kernel_smooth = np.ones((3, 3), np.uint8) / 9.0
        mask1_float = mask1.astype(np.float32)
        mask2 = cv2.filter2D(mask1_float, -1, kernel_smooth)
        mask2 = (mask2 > 100).astype(np.uint8) * 255  # Threshold after convolution
        
        # Combine both layers (OR operation - more permissive, either layer can detect)
        # Changed from AND to OR to catch more yellow regions
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill gaps
        
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
            # DISABLED - accept any aspect ratio for now
            aspect_ratio = h / w if w > 0 else 0
            # if aspect_ratio < 0.5 or aspect_ratio > 10.0:  # DISABLED - too permissive
            #     debug_info['filtered']['aspect'] = debug_info['filtered'].get('aspect', 0) + 1
            #     continue
            
            # Heuristic 3: TRIANGULAR SHAPE DETECTION (cones are triangular)
            # Approximate contour to polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Check if shape is roughly triangular (3-5 vertices for cone)
            # DISABLED - accept any shape
            num_vertices = len(approx)
            # if num_vertices < 3 or num_vertices > 20:  # DISABLED
            #     debug_info['filtered']['vertices'] = debug_info['filtered'].get('vertices', 0) + 1
            #     continue
            
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
                # DISABLED - accept any shape
                # if bottom_width > 0 and top_width / bottom_width > 0.99:  # DISABLED
                #     debug_info['filtered']['triangle'] = debug_info['filtered'].get('triangle', 0) + 1
                #     continue
            
            # Heuristic 4: Solidity (cones are relatively solid shapes)
            # DISABLED - accept any solidity
            # hull = cv2.convexHull(contour)
            # hull_area = cv2.contourArea(hull)
            # solidity = area / hull_area if hull_area > 0 else 0
            # if solidity < 0.1:  # DISABLED
            #     debug_info['filtered']['solidity'] = debug_info['filtered'].get('solidity', 0) + 1
            #     continue
            
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
            # DISABLED - accept anywhere in image
            # img_height = cv_image.shape[0]
            # if center_y < img_height * 0.05:  # DISABLED
            #     debug_info['filtered']['position'] = debug_info['filtered'].get('position', 0) + 1
            #     continue
            
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
    
    def pixel_to_3d_from_mask(self, mask_pixels, u, v):
        """
        Convert pixel coordinates to 3D using mask pixel count (lab8 pattern).
        Uses heuristic: depth = sqrt((fx * fy * CONE_AREA) / pixel_count)
        
        Args:
            mask_pixels: Number of pixels in the mask (yellow region)
            u, v: Pixel coordinates (u=y, v=x in image)
        
        Returns:
            (x, y, z) in camera frame, estimated_depth
        """
        if self.camera_intrinsics is None or mask_pixels == 0:
            return None, None
        
        fx, fy, cx, cy = self.camera_intrinsics
        
        # Depth estimation from pixel count (lab8 pattern)
        # depth = sqrt((fx * fy * CONE_AREA) / pixel_count)
        depth = np.sqrt((fx * fy * self.CONE_AREA) / mask_pixels)
        depth = np.clip(depth, 0.2, self.max_range)
        
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
        """Publish detected cones with confidence scores"""
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
                # Fallback: old format
                continue
            
            # Compute distance from robot
            distance = np.linalg.norm(point_map[:2] - robot_pos)
            
            # Check range
            if distance > self.max_range:
                continue
            
            # Use provided confidence (already computed)
            
            # Create marker with confidence-based alpha
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "camera_cones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(point_map[0])
            marker.pose.position.y = float(point_map[1])
            marker.pose.position.z = 0.0
            
            marker.pose.orientation.w = 1.0
            
            # Estimate cone size from pixel size
            estimated_radius = 0.1  # Default 10cm
            marker.scale.x = estimated_radius * 2
            marker.scale.y = estimated_radius * 2
            marker.scale.z = 0.3  # Cone height
            
            # Color based on confidence (green = high, yellow = medium, red = low)
            marker.color.r = 1.0 - confidence
            marker.color.g = confidence
            marker.color.b = 0.0
            marker.color.a = 0.5 + 0.5 * confidence  # More opaque = higher confidence
            
            marker_array.markers.append(marker)
            
            # Publish as PointStamped with confidence in frame_id (temporary solution)
            # Better: create custom message type
            point_msg = PointStamped()
            point_msg.header.frame_id = f"map_confidence_{confidence:.2f}"  # Encode confidence
            point_msg.header.stamp = marker.header.stamp
            point_msg.point.x = float(point_map[0])
            point_msg.point.y = float(point_map[1])
            point_msg.point.z = float(confidence)  # Store confidence in z coordinate
            self.cone_points_pub.publish(point_msg)
            
            # Store confidence for this detection
            cone_id = f"{point_map[0]:.3f}_{point_map[1]:.3f}"
            self.cone_confidences[cone_id] = confidence
        
        if len(marker_array.markers) > 0:
            self.cones_pub.publish(marker_array)
            avg_confidence = np.mean([self.compute_camera_confidence(
                np.linalg.norm(np.array([m.pose.position.x, m.pose.position.y]) - robot_pos)
            ) for m in marker_array.markers])
            self.get_logger().info(
                f'📷 Detected {len(marker_array.markers)} cones, avg confidence: {avg_confidence:.2f}'
            )
    
    def publish_debug_image(self, cv_image, cones):
        """Publish debug image with detections overlaid and yellow mask visualization"""
        debug_image = cv_image.copy()
        
        # Show yellow mask overlay (for debugging) - use green channel for better visibility
        try:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            # Overlay mask in green channel with transparency (yellow areas get green tint)
            mask_overlay = yellow_mask.astype(np.float32) / 255.0 * 0.3  # 30% opacity
            debug_image = debug_image.astype(np.float32)
            debug_image[:, :, 1] = np.minimum(255, debug_image[:, :, 1] + mask_overlay * 100)  # Add green tint
            debug_image = debug_image.astype(np.uint8)
        except:
            pass  # If mask fails, just show original image
        
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
            
            # Draw info
            info_text = f'{area:.0f}px, {mask_pixels:.0f}mask'
            cv2.putText(debug_image, info_text, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Add status text
        status_text = f'Cones: {len(cones)} | Intrinsics: {"OK" if self.camera_intrinsics else "WAIT"} | Pose: {"OK" if self.robot_pose else "WAIT"}'
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
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

