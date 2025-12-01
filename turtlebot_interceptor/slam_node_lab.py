#!/usr/bin/env python3
"""
SLAM Node - Log-Odds Occupancy Grid Mapping
Hardware-ready ROS2 node for building map from LIDAR scans
Based on log-odds occupancy grid mapping algorithm
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import transforms3d.euler as euler
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import tf2_ros
import geometry_msgs.msg

class SLAMNode(Node):
    """SLAM node using log-odds occupancy grid mapping"""
    
    def __init__(self):
        super().__init__('slam_node')
        
        # Map parameters - updated to match turtlebot3_cartographer
        self.map_width = 108  # cells
        self.map_height = 149  # cells
        self.resolution = 0.05  # 5cm per cell
        self.origin_x = -2.5
        self.origin_y = -3.75
        
        # Initialize map as unknown (-1)
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Log-odds parameters
        self.log_odds_free = -0.6
        self.log_odds_occupied = 0.5
        self.log_odds_min = -3.0
        self.log_odds_max = 3.0
        
        # Convert occupancy to log-odds lookup
        self.log_odds_map = np.zeros(256, dtype=np.float32)
        for i in range(256):
            if i == -1 or i == 255:  # Unknown
                self.log_odds_map[i] = 0.0
            elif i == 0:  # Free
                self.log_odds_map[i] = -1.5
            elif i >= 100:  # Occupied
                self.log_odds_map[i] = 1.5
            else:
                self.log_odds_map[i] = 0.0
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Use BEST_EFFORT reliability
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            1
        )

        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # State
        self.current_pose = None
        self.initial_pose = None  # To store the initial pose
        
        # Timer for map publishing
        self.map_timer = self.create_timer(0.5, self.publish_map)
        
        self.get_logger().info('SLAM node initialized - log-odds occupancy grid mapping')
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store current robot pose"""
        self.current_pose = msg.pose.pose
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
        
        # Publish the transform from map to odom
        self.publish_map_to_odom_transform()
    
    def scan_callback(self, msg: LaserScan):
        """Process LIDAR scan and update map using log-odds"""
        if self.current_pose is None:
            return
        
        # Extract pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        q = self.current_pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Process scan
        self.update_map_from_scan(x, y, yaw, msg)
    
    def update_map_from_scan(self, x, y, theta, scan):
        """Update map using LIDAR scan with log-odds occupancy grid mapping"""
        min_range = scan.range_min if scan.range_min > 0 else 0.25
        max_range = scan.range_max if scan.range_max > 0 else 3.5
        
        # Process each beam
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            # Calculate beam angle
            angle = scan.angle_min + i * scan.angle_increment
            beam_angle = theta + angle
            
            # Clamp range
            range_val = max(min_range, min(range_val, max_range))
            
            # Raycast to update cells along beam
            step_size = self.resolution * 0.5
            max_steps = int(max_range / step_size)
            
            hit_obstacle = False
            for step in range(max_steps):
                dist = step * step_size
                if dist >= range_val:
                    hit_obstacle = True
                    break
                
                # Calculate cell position
                cell_x = x + dist * math.cos(beam_angle)
                cell_y = y + dist * math.sin(beam_angle)
                
                # Convert to grid coordinates
                gx, gy = self.world_to_grid(cell_x, cell_y)
                
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    # Update log-odds for free space
                    self.update_cell_log_odds(gx, gy, self.log_odds_free)
            
            # Update obstacle cell at end of beam
            if hit_obstacle and range_val < max_range:
                obstacle_x = x + range_val * math.cos(beam_angle)
                obstacle_y = y + range_val * math.sin(beam_angle)
                gx, gy = self.world_to_grid(obstacle_x, obstacle_y)
                
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    # Update log-odds for occupied space
                    self.update_cell_log_odds(gx, gy, self.log_odds_occupied)
    
    def update_cell_log_odds(self, gx, gy, log_odds_update):
        """Update log-odds for a single cell"""
        # Get current occupancy value
        current_val = self.map_data[gy, gx]
        
        # Convert to log-odds
        if current_val == -1:  # Unknown
            current_log_odds = 0.0
        elif current_val == 0:  # Free
            current_log_odds = -1.5
        elif current_val >= 100:  # Occupied
            current_log_odds = 1.5
        else:
            current_log_odds = 0.0
        
        # Update log-odds
        new_log_odds = current_log_odds + log_odds_update
        new_log_odds = np.clip(new_log_odds, self.log_odds_min, self.log_odds_max)
        
        # Convert back to occupancy value
        if new_log_odds < -0.2:
            self.map_data[gy, gx] = 0  # Free
        elif new_log_odds > 0.2:
            self.map_data[gy, gx] = 100  # Occupied
        else:
            self.map_data[gy, gx] = -1  # Unknown
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def publish_map(self):
        """Publish occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.info.map_load_time = map_msg.header.stamp

        # Flatten map data (row-major order)
        map_msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(map_msg)

    def publish_map_to_odom_transform(self):
        """Publish the transform from map to odom"""
        if self.current_pose is None or self.initial_pose is None:
            return
        
        # Create the transform message
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'map'
        
        # Robot's position in the map frame (offset by initial pose)
        t.transform.translation.x = self.current_pose.position.x - self.initial_pose.position.x
        t.transform.translation.y = self.current_pose.position.y - self.initial_pose.position.y
        t.transform.translation.z = 0.0
        
        # Convert quaternion to rotation (identity because map is aligned with the initial pose)
        t.transform.rotation = self.current_pose.orientation
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
