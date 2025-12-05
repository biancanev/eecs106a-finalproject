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


class SLAMNode(Node):
    """SLAM node using log-odds occupancy grid mapping"""
    
    def __init__(self):
        super().__init__('slam_node')
        
        # Declare parameters (lab4 pattern)
        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', -2.5)
        self.declare_parameter('origin_y', -2.5)
        self.declare_parameter('log_odds_free', -0.6)
        self.declare_parameter('log_odds_occupied', 0.8)
        self.declare_parameter('log_odds_min', -3.0)
        self.declare_parameter('log_odds_max', 3.0)
        self.declare_parameter('occupancy_threshold', 0.25)
        
        # Get parameters
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        self.origin_y = self.get_parameter('origin_y').get_parameter_value().double_value
        
        # Initialize map as unknown (-1) - use internal log-odds representation
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Internal log-odds map for more accurate updates (lab6 pattern)
        self.log_odds_data = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Log-odds parameters (from parameters)
        self.log_odds_free = self.get_parameter('log_odds_free').get_parameter_value().double_value
        self.log_odds_occupied = self.get_parameter('log_odds_occupied').get_parameter_value().double_value
        self.log_odds_min = self.get_parameter('log_odds_min').get_parameter_value().double_value
        self.log_odds_max = self.get_parameter('log_odds_max').get_parameter_value().double_value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').get_parameter_value().double_value
        
        # Subscriptions
        # Use BEST_EFFORT QoS to match hardware LIDAR publishers
        from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
        scan_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
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
            10
        )
        
        # State
        self.current_pose = None
        
        # Timer for map publishing
        self.map_timer = self.create_timer(0.5, self.publish_map)
        
        self.get_logger().info('SLAM node initialized - log-odds occupancy grid mapping')
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store current robot pose"""
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg: LaserScan):
        """Process LIDAR scan and update map using log-odds"""
        if self.current_pose is None:
            # Log warning periodically (not every scan)
            if not hasattr(self, '_pose_warn_count'):
                self._pose_warn_count = 0
            self._pose_warn_count += 1
            if self._pose_warn_count % 50 == 0:  # Every ~5 seconds at 10Hz
                self.get_logger().warn('SLAM: Waiting for pose (current_pose is None) - check /amcl_pose topic')
            return
        
        # Extract pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        q = self.current_pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Process scan
        self.update_map_from_scan(x, y, yaw, msg)
    
    def update_map_from_scan(self, x, y, theta, scan):
        """
        Update map using LIDAR scan with log-odds occupancy grid mapping.
        Based on lab6 mapping implementation patterns:
        - Direct log-odds updates (no conversion overhead)
        - Small step size for accurate free space marking
        - Neighborhood expansion for obstacles with distance weighting
        - Intermediate occupancy values (50) for uncertain cells
        """
        min_range = scan.range_min if scan.range_min > 0 else 0.25
        max_range = scan.range_max if scan.range_max > 0 else 3.5
        
        # Helper functions for log-odds conversion (lab6 pattern)
        def occupancy_to_log_odds(val):
            """Convert occupancy value to log-odds"""
            if val == -1:  # Unknown
                return 0.0
            elif val == 0:  # Free
                return -1.5
            elif val == 100:  # Occupied
                return 1.5
            else:  # Intermediate value (0-100)
                return (val - 50) / 33.0  # Scale to [-1.5, 1.5]
        
        def log_odds_to_occupancy(lo):
            """Convert log-odds to occupancy value"""
            if lo < -self.occupancy_threshold:  # Free space
                return 0
            elif lo > self.occupancy_threshold:  # Occupied
                return 100
            else:  # Unknown/uncertain
                if lo > 0.1:  # Some evidence for occupied
                    return 50  # Uncertain but likely occupied
                return -1  # Unknown
        
        # Process each beam
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            # Calculate beam angle
            angle = scan.angle_min + i * scan.angle_increment
            beam_angle = theta + angle
            
            # Clamp range
            range_val = max(min_range, min(range_val, max_range))
            
            # Skip if range is at max (no obstacle detected) or too close (self-detection)
            if range_val >= max_range - 0.1:
                continue
            if range_val <= min_range + 0.05:  # Too close - likely self-detection
                continue
            
            # Raycast to update cells along beam (lab6 pattern - smaller step size)
            step_size = self.resolution * 0.05  # Very small steps for accuracy
            max_steps = int(range_val / step_size)
            
            # Update free space along beam
            for step in range(max_steps):
                dist = step * step_size
                if dist >= range_val - 0.05:  # Stop just before obstacle
                    break
                
                # Calculate cell position
                cell_x = x + dist * math.cos(beam_angle)
                cell_y = y + dist * math.sin(beam_angle)
                
                # Convert to grid coordinates
                gx, gy = self.world_to_grid(cell_x, cell_y)
                
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    # Update log-odds for free space (direct update to log_odds_data)
                    current_lo = self.log_odds_data[gy, gx]
                    new_lo = current_lo + self.log_odds_free
                    new_lo = np.clip(new_lo, self.log_odds_min, self.log_odds_max)
                    self.log_odds_data[gy, gx] = new_lo
                    # Update occupancy value
                    self.map_data[gy, gx] = log_odds_to_occupancy(new_lo)
            
            # Update obstacle cell at end of beam (lab6 pattern - neighborhood expansion)
            obstacle_x = x + range_val * math.cos(beam_angle)
            obstacle_y = y + range_val * math.sin(beam_angle)
            gx, gy = self.world_to_grid(obstacle_x, obstacle_y)
            
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                # Update obstacle cell and neighbors (5x5 neighborhood with distance weighting)
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        ngx, ngy = gx + dx, gy + dy
                        if 0 <= ngx < self.map_width and 0 <= ngy < self.map_height:
                            # Distance weighting (lab6 pattern)
                            dist_from_center = math.sqrt(dx*dx + dy*dy)
                            if dist_from_center <= 1.0:
                                # Stronger evidence for immediate neighbors
                                weight = 0.5
                            else:
                                # Weaker evidence for outer ring
                                weight = 0.2
                            
                            # Update log-odds
                            current_lo = self.log_odds_data[ngy, ngx]
                            new_lo = current_lo + self.log_odds_occupied * weight
                            new_lo = np.clip(new_lo, self.log_odds_min, self.log_odds_max)
                            self.log_odds_data[ngy, ngx] = new_lo
                            # Update occupancy value
                            self.map_data[ngy, ngx] = log_odds_to_occupancy(new_lo)
    
    def update_cell_log_odds(self, gx, gy, log_odds_update):
        """Update log-odds for a single cell (legacy method - now handled directly in update_map_from_scan)"""
        # This method is kept for compatibility but log-odds are now updated directly
        # in update_map_from_scan for better performance
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            current_lo = self.log_odds_data[gy, gx]
            new_lo = current_lo + log_odds_update
            new_lo = np.clip(new_lo, self.log_odds_min, self.log_odds_max)
            self.log_odds_data[gy, gx] = new_lo
            
            # Convert back to occupancy
            if new_lo < -self.occupancy_threshold:
                self.map_data[gy, gx] = 0  # Free
            elif new_lo > self.occupancy_threshold:
                self.map_data[gy, gx] = 100  # Occupied
            else:
                if new_lo > 0.1:
                    self.map_data[gy, gx] = 50  # Uncertain but likely occupied
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
        
        # Flatten map data (row-major order)
        map_msg.data = self.map_data.flatten().tolist()
        
        # Count occupied cells for debugging
        occupied_count = sum(1 for x in map_msg.data if x > 0)
        unknown_count = sum(1 for x in map_msg.data if x == -1)
        free_count = sum(1 for x in map_msg.data if x == 0)
        
        self.map_pub.publish(map_msg)
        
        # Log map status periodically
        if hasattr(self, '_map_pub_count'):
            self._map_pub_count += 1
        else:
            self._map_pub_count = 0
        
        if self._map_pub_count % 10 == 0:  # Every 5 seconds (0.5s * 10)
            self.get_logger().info(
                f'Map published: {occupied_count} occupied, {free_count} free, '
                f'{unknown_count} unknown, pose={self.current_pose is not None}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

