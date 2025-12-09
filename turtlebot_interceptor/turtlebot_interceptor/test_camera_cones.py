#!/usr/bin/env python3
"""
Simple test script to visualize camera cone detection and world pose tracking.
Shows how well the camera detects cones and how their positions evolve as robot moves.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import tf2_ros
from tf2_ros import TransformException
import math


class CameraConeTest(Node):
    """Test camera cone detection and tracking"""
    
    def __init__(self):
        super().__init__('camera_cone_test')
        
        self.bridge = CvBridge()
        
        # Camera intrinsics
        self.camera_info = None
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Yellow color range
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        self.min_cone_area = 500
        
        # Robot pose
        self.robot_pose = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Track detected cones over time
        self.detection_history = deque(maxlen=100)  # Last 100 detections
        self.cone_tracks = {}  # Map: cone_id -> list of (x, y, time) tuples
        
        # Subscriptions
        from rclpy.qos import qos_profile_sensor_data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Visualization
        self.fig = None
        self.ax1 = None  # Camera view
        self.ax2 = None  # World map
        plt.ion()  # Interactive mode
        
        self.get_logger().info('📷 Camera Cone Test initialized')
        self.get_logger().info('   Viewing camera feed and world positions')
        self.get_logger().info('   Close plot window to stop')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics"""
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        
        if not hasattr(self, '_info_logged'):
            self.get_logger().info(
                f'📷 Camera: {msg.width}x{msg.height}, '
                f'fx={self.fx:.1f}, fy={self.fy:.1f}'
            )
            self._info_logged = True
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store robot pose"""
        self.robot_pose = msg.pose.pose
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.robot_theta = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
    
    def detect_yellow_cones(self, cv_image):
        """Detect yellow cones in image"""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cones = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_cone_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w / 2
            center_y = y + h / 2
            
            # Filter by aspect ratio
            aspect_ratio = h / w if w > 0 else 0
            if aspect_ratio < 1.0 or aspect_ratio > 3.0:
                continue
            
            cones.append((center_x, center_y, w, h, area))
        
        return cones
    
    def pixel_to_3d(self, u, v, pixel_height):
        """Convert pixel to 3D using camera intrinsics"""
        real_cone_height = 0.3  # meters
        
        if pixel_height > 0:
            depth = (self.fy * real_cone_height) / pixel_height
            depth = np.clip(depth, 0.2, 5.0)
        else:
            depth = 1.0
        
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth
        
        return np.array([x, y, z]), depth
    
    def camera_to_map_frame(self, point_camera):
        """Transform camera point to map frame"""
        if self.robot_pose is None:
            return None
        
        try:
            # Try to get camera-to-base_link transform
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                rclpy.time.Time()
            )
            
            # For now, assume camera is forward on base_link
            point_base = point_camera
            
            # Transform to map using robot pose
            cos_theta = math.cos(self.robot_theta)
            sin_theta = math.sin(self.robot_theta)
            
            point_map_x = self.robot_x + cos_theta * point_base[0] - sin_theta * point_base[1]
            point_map_y = self.robot_y + sin_theta * point_base[0] + cos_theta * point_base[1]
            
            return np.array([point_map_x, point_map_y, 0.0])
            
        except TransformException:
            # Fallback: assume camera is forward on base_link
            cos_theta = math.cos(self.robot_theta)
            sin_theta = math.sin(self.robot_theta)
            
            point_map_x = self.robot_x + cos_theta * point_camera[0] - sin_theta * point_camera[1]
            point_map_y = self.robot_y + sin_theta * point_camera[0] + cos_theta * point_camera[1]
            
            return np.array([point_map_x, point_map_y, 0.0])
    
    def image_callback(self, msg: Image):
        """Process camera image and visualize"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return
        
        # Detect cones
        cones = self.detect_yellow_cones(cv_image)
        
        # Process detections
        current_time = self.get_clock().now().nanoseconds / 1e9
        world_positions = []
        
        for cx, cy, w, h, area in cones:
            # Convert to 3D
            point_3d, depth = self.pixel_to_3d(cx, cy, h)
            
            # Transform to map frame
            point_map = self.camera_to_map_frame(point_3d)
            
            if point_map is not None:
                world_positions.append((point_map[0], point_map[1], depth, cx, cy, w, h))
                
                # Store in history
                self.detection_history.append({
                    'time': current_time,
                    'world_pos': (point_map[0], point_map[1]),
                    'depth': depth,
                    'pixel_pos': (cx, cy),
                    'size': (w, h),
                    'robot_pos': (self.robot_x, self.robot_y)
                })
        
        # Visualize
        self.visualize(cv_image, cones, world_positions, current_time)
    
    def visualize(self, cv_image, cones, world_positions, current_time):
        """Visualize camera view and world map"""
        if self.fig is None:
            self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(16, 6))
            self.fig.suptitle('Camera Cone Detection Test', fontsize=16)
        
        # Clear axes
        self.ax1.clear()
        self.ax2.clear()
        
        # LEFT: Camera view with detections
        self.ax1.imshow(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        self.ax1.set_title(f'Camera View (Detected: {len(cones)} cones)')
        self.ax1.axis('off')
        
        # Draw detections
        for cx, cy, w, h, area in cones:
            x = int(cx - w/2)
            y = int(cy - h/2)
            rect = plt.Rectangle((x, y), w, h, fill=False, edgecolor='yellow', linewidth=2)
            self.ax1.add_patch(rect)
            self.ax1.plot(cx, cy, 'yo', markersize=10)
            self.ax1.text(x, y - 10, f'{area:.0f}px', color='yellow', fontsize=8)
        
        # RIGHT: World map with robot and cones
        self.ax2.set_title('World Map (Robot + Detected Cones)')
        self.ax2.set_xlabel('X (m)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_aspect('equal')
        
        # Draw robot
        if self.robot_pose is not None:
            self.ax2.plot(self.robot_x, self.robot_y, 'bo', markersize=15, label='Robot')
            
            # Draw robot heading
            dx = 0.2 * math.cos(self.robot_theta)
            dy = 0.2 * math.sin(self.robot_theta)
            self.ax2.arrow(self.robot_x, self.robot_y, dx, dy,
                          head_width=0.05, head_length=0.05, fc='blue', ec='blue')
        
        # Draw current detections
        for wx, wy, depth, cx, cy, w, h in world_positions:
            self.ax2.plot(wx, wy, 'ro', markersize=12, label='Cone (current)')
            self.ax2.text(wx + 0.05, wy + 0.05, f'{depth:.2f}m', fontsize=8)
        
        # Draw detection history (trajectory)
        if len(self.detection_history) > 1:
            # Group by approximate position (track cones)
            for i, det in enumerate(self.detection_history):
                wx, wy = det['world_pos']
                age = current_time - det['time']
                
                # Color by age (recent = bright, old = faded)
                alpha = max(0.1, 1.0 - age / 5.0)  # Fade over 5 seconds
                color = (1.0, 0.5, 0.0, alpha)  # Orange, fading
                
                self.ax2.plot(wx, wy, 'o', color=color, markersize=6, alpha=alpha)
        
        # Draw robot trajectory
        if len(self.detection_history) > 1:
            robot_traj_x = [d['robot_pos'][0] for d in self.detection_history]
            robot_traj_y = [d['robot_pos'][1] for d in self.detection_history]
            self.ax2.plot(robot_traj_x, robot_traj_y, 'b-', alpha=0.3, linewidth=1, label='Robot path')
        
        self.ax2.legend(loc='upper right')
        
        # Update plot
        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)
        
        # Log detection
        if len(world_positions) > 0:
            cone_str = ', '.join([f'({wx:.2f}, {wy:.2f})' for wx, wy, _, _, _, _, _ in world_positions])
            self.get_logger().info(
                f'📷 Detected {len(world_positions)} cones: '
                f'Robot=({self.robot_x:.2f}, {self.robot_y:.2f}), '
                f'Cones=[{cone_str}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraConeTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()

