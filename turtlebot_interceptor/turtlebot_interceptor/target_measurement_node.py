#!/usr/bin/env python3
"""
Target Measurement Node - Artificial Target Provider with Terminal Homing

This node provides artificial target position measurements at a configurable rate
until the target is detected in LIDAR, at which point it switches to terminal
homing mode using LIDAR-based target detection.

Operation Modes:
1. **Artificial Mode**: Publishes target position from external source (GPS, camera, etc.)
   at a configurable rate. Used when target is far away or not visible in LIDAR.
   
2. **Terminal Homing Mode**: When LIDAR detects target within detection range,
   switches to LIDAR-based target detection for precise final approach.

Topics:
- Subscribes: /scan (LIDAR), /amcl_pose (robot pose), /target_estimate (optional external source)
- Publishes: /target_pose_measurement (target position for UKF)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import math
import transforms3d.euler as euler


class TargetMeasurementNode(Node):
    """Node that provides target measurements with terminal homing capability"""
    
    def __init__(self):
        super().__init__('target_measurement_node')
        
        # Declare parameters
        self.declare_parameter('artificial_rate', 10.0)  # Hz - rate for artificial measurements
        self.declare_parameter('lidar_detection_range', 2.0)  # m - switch to terminal homing when target within this range
        self.declare_parameter('target_radius', 0.15)  # m - target physical radius
        self.declare_parameter('use_external_source', True)  # Use external target estimate if available
        self.declare_parameter('external_source_topic', '/target_estimate')  # External target source
        self.declare_parameter('use_sim_time', False)
        
        # Get parameters
        self.artificial_rate = self.get_parameter('artificial_rate').get_parameter_value().double_value
        self.lidar_detection_range = self.get_parameter('lidar_detection_range').get_parameter_value().double_value
        self.target_radius = self.get_parameter('target_radius').get_parameter_value().double_value
        self.use_external_source = self.get_parameter('use_external_source').get_parameter_value().bool_value
        self.external_source_topic = self.get_parameter('external_source_topic').get_parameter_value().string_value
        
        # State
        self.robot_pose = None
        self.current_scan = None
        self.external_target_estimate = None
        self.terminal_homing_active = False
        self.last_lidar_detection = None
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Optional: subscribe to external target source (e.g., camera, GPS)
        if self.use_external_source:
            self.external_target_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.external_source_topic,
                self.external_target_callback,
                10
            )
        
        # Publisher
        self.target_meas_pub = self.create_publisher(
            PoseStamped,
            '/target_pose_measurement',
            10
        )
        
        # Timer for artificial measurements
        self.artificial_timer = self.create_timer(1.0 / self.artificial_rate, self.artificial_measurement_callback)
        
        # Timer for terminal homing (runs at higher rate when active)
        self.terminal_homing_timer = self.create_timer(0.1, self.terminal_homing_callback)
        
        self.get_logger().info(
            f'Target measurement node initialized - '
            f'artificial rate: {self.artificial_rate}Hz, '
            f'terminal homing range: {self.lidar_detection_range}m'
        )
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store robot pose for LIDAR-based target detection"""
        self.robot_pose = msg.pose.pose
    
    def scan_callback(self, msg: LaserScan):
        """Store LIDAR scan and check for target detection"""
        self.current_scan = msg
        
        # Check if target is detected in LIDAR
        if self.robot_pose is not None:
            target_detection = self.detect_target_in_lidar(msg)
            if target_detection is not None:
                self.last_lidar_detection = target_detection
                # Switch to terminal homing if target is within range
                dist = target_detection['distance']
                if dist <= self.lidar_detection_range and not self.terminal_homing_active:
                    self.terminal_homing_active = True
                    self.get_logger().info(
                        f'TERMINAL HOMING ACTIVATED - Target detected at {dist:.2f}m via LIDAR'
                    )
                elif dist > self.lidar_detection_range * 1.5 and self.terminal_homing_active:
                    # Switch back to artificial if target moves far away
                    self.terminal_homing_active = False
                    self.get_logger().info('Switching back to artificial measurements - target out of LIDAR range')
    
    def external_target_callback(self, msg: PoseWithCovarianceStamped):
        """Store external target estimate (from camera, GPS, etc.)"""
        self.external_target_estimate = msg
    
    def detect_target_in_lidar(self, scan: LaserScan):
        """
        Detect target in LIDAR scan.
        Returns dict with 'position' (x, y), 'distance', 'angle' if detected, None otherwise.
        """
        if self.robot_pose is None:
            return None
        
        # Extract robot pose
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        q = self.robot_pose.orientation
        roll, pitch, robot_theta = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Look for target-like object in LIDAR scan
        # Target appears as a circular object of known radius
        min_range = scan.range_min if scan.range_min > 0 else 0.25
        max_range = scan.range_max if scan.range_max > 0 else 3.5
        
        # Cluster nearby points that could be target
        candidate_points = []
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < min_range or range_val >= max_range:
                continue
            
            # Calculate point position in world frame
            angle = scan.angle_min + i * scan.angle_increment
            beam_angle = robot_theta + angle
            point_x = robot_x + range_val * math.cos(beam_angle)
            point_y = robot_y + range_val * math.sin(beam_angle)
            
            # Check if this point is at expected target radius distance
            # (target is a circular object, so we look for points at its surface)
            if min_range < range_val < self.lidar_detection_range:
                candidate_points.append({
                    'x': point_x,
                    'y': point_y,
                    'range': range_val,
                    'angle': beam_angle,
                    'scan_index': i
                })
        
        if len(candidate_points) < 3:  # Need at least 3 points to form a cluster
            return None
        
        # Cluster nearby points (target should appear as a cluster)
        clusters = []
        for point in candidate_points:
            assigned = False
            for cluster in clusters:
                # Check if point is near cluster center
                cluster_center = cluster['center']
                dist = math.sqrt((point['x'] - cluster_center[0])**2 + 
                               (point['y'] - cluster_center[1])**2)
                if dist < self.target_radius * 2:  # Within 2x target radius
                    cluster['points'].append(point)
                    # Update cluster center
                    cluster['center'] = (
                        np.mean([p['x'] for p in cluster['points']]),
                        np.mean([p['y'] for p in cluster['points']])
                    )
                    assigned = True
                    break
            if not assigned:
                clusters.append({
                    'points': [point],
                    'center': (point['x'], point['y'])
                })
        
        # Find best cluster (closest to expected target size)
        best_cluster = None
        best_score = float('inf')
        
        for cluster in clusters:
            if len(cluster['points']) < 3:  # Too few points
                continue
            
            center = cluster['center']
            dist_to_robot = math.sqrt((center[0] - robot_x)**2 + (center[1] - robot_y)**2)
            
            # Score: prefer clusters at reasonable distance with good point count
            score = abs(dist_to_robot - self.target_radius) + (10.0 / len(cluster['points']))
            
            if score < best_score:
                best_score = score
                best_cluster = cluster
        
        if best_cluster is None:
            return None
        
        # Return target detection
        center = best_cluster['center']
        dist = math.sqrt((center[0] - robot_x)**2 + (center[1] - robot_y)**2)
        angle = math.atan2(center[1] - robot_y, center[0] - robot_x)
        
        return {
            'position': center,
            'distance': dist,
            'angle': angle
        }
    
    def artificial_measurement_callback(self):
        """Publish artificial target measurement (when not in terminal homing)"""
        if self.terminal_homing_active:
            return  # Terminal homing handles measurements
        
        # Use external source if available, otherwise use last known position
        if self.external_target_estimate is not None:
            self.publish_measurement(
                self.external_target_estimate.pose.pose.position.x,
                self.external_target_estimate.pose.pose.position.y,
                source='external'
            )
        elif self.last_lidar_detection is not None:
            # Use last LIDAR detection if available
            pos = self.last_lidar_detection['position']
            self.publish_measurement(pos[0], pos[1], source='lidar_fallback')
        else:
            # No source available - log warning (only once)
            if not hasattr(self, '_warned_no_source'):
                self.get_logger().warn('No target source available - cannot publish measurements')
                self._warned_no_source = True
    
    def terminal_homing_callback(self):
        """Handle terminal homing mode - use LIDAR for precise target detection"""
        if not self.terminal_homing_active:
            return
        
        if self.current_scan is None or self.robot_pose is None:
            return
        
        # Detect target in current LIDAR scan
        detection = self.detect_target_in_lidar(self.current_scan)
        
        if detection is not None:
            pos = detection['position']
            self.publish_measurement(pos[0], pos[1], source='lidar_terminal_homing')
            self.last_lidar_detection = detection
        elif self.last_lidar_detection is not None:
            # Use last detection if current scan doesn't detect (temporary loss)
            pos = self.last_lidar_detection['position']
            self.publish_measurement(pos[0], pos[1], source='lidar_last_known')
        else:
            # Lost target - switch back to artificial
            self.terminal_homing_active = False
            self.get_logger().warn('Lost target in terminal homing - switching to artificial mode')
    
    def publish_measurement(self, x: float, y: float, source: str = 'unknown'):
        """Publish target position measurement"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.target_meas_pub.publish(msg)
        
        # Log occasionally (not every message to avoid spam)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 50 == 0:  # Log every 50th message
            self.get_logger().debug(f'Published target measurement ({x:.2f}, {y:.2f}) from {source}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetMeasurementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

