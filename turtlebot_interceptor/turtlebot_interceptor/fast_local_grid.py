#!/usr/bin/env python3
"""
Fast Local Occupancy Grid - For Dynamic Navigation

Updates at LIDAR rate (5-10 Hz) with simple ray-casting
Maintains a small rolling window (5m x 5m) around the robot
Perfect for maze-like environments where you need instant awareness

This runs ALONGSIDE Cartographer:
- Cartographer: Global map, slow (1-2 Hz), accurate
- This: Local grid, fast (5-10 Hz), immediate
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import transforms3d.euler as euler


class FastLocalGrid(Node):
    """High-speed local occupancy grid for dynamic navigation"""
    
    def __init__(self):
        super().__init__('fast_local_grid')
        
        # Local grid parameters
        self.grid_size = 5.0  # 5m x 5m around robot
        self.resolution = 0.05  # 5cm cells (same as Cartographer)
        self.width = int(self.grid_size / self.resolution)  # 100 cells
        self.height = int(self.grid_size / self.resolution)  # 100 cells
        
        # Log-odds parameters (aggressive for fast updates)
        self.log_odds_occupied = 2.0  # Strong occupied evidence
        self.log_odds_free = -0.4  # Weak free evidence (don't clear too fast)
        self.log_odds_min = -5.0
        self.log_odds_max = 10.0
        
        # Grid storage (log-odds)
        self.grid = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # QoS for LIDAR (BEST_EFFORT for hardware compatibility)
        lidar_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            lidar_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # CRITICAL: Subscribe to Cartographer for global prior
        # This ensures our fast local grid is consistent with global map
        self.global_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.global_map_callback,
            10
        )
        
        self.global_map = None  # Cartographer's global map (prior)
        
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/local_map',  # Different from Cartographer's /map
            10
        )
        
        # Timer for publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_map)
        
        self.get_logger().info(
            f'Fast Local Grid: {self.width}x{self.height} cells ({self.grid_size}m x {self.grid_size}m), '
            f'resolution={self.resolution}m\n'
            f'  Mode: Bayesian fusion (Cartographer prior + LIDAR updates)'
        )
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update robot pose and reinitialize grid from global map"""
        old_x, old_y = self.robot_x, self.robot_y
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # If robot moved significantly, reinitialize grid from global map (feed forward the prior)
        moved = np.sqrt((self.robot_x - old_x)**2 + (self.robot_y - old_y)**2)
        if moved > 0.2:  # Robot moved 20cm - refresh prior
            self.initialize_from_global_map()
    
    def global_map_callback(self, msg: OccupancyGrid):
        """Store Cartographer's global map as prior"""
        self.global_map = msg
        # Initialize local grid from global map
        self.initialize_from_global_map()
    
    def initialize_from_global_map(self):
        """Initialize local grid from Cartographer's global map (Bayesian prior)"""
        if self.global_map is None:
            return
        
        # Reset local grid
        self.grid = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Copy relevant portion from global map
        g_width = self.global_map.info.width
        g_height = self.global_map.info.height
        g_resolution = self.global_map.info.resolution
        g_origin_x = self.global_map.info.origin.position.x
        g_origin_y = self.global_map.info.origin.position.y
        
        # Local grid bounds
        local_min_x = self.robot_x - self.grid_size / 2
        local_min_y = self.robot_y - self.grid_size / 2
        
        # For each cell in local grid, find corresponding cell in global map
        for lx in range(self.width):
            for ly in range(self.height):
                # World coordinates of this local cell
                world_x = local_min_x + lx * self.resolution + self.resolution / 2
                world_y = local_min_y + ly * self.resolution + self.resolution / 2
                
                # Find in global map
                gx = int((world_x - g_origin_x) / g_resolution)
                gy = int((world_y - g_origin_y) / g_resolution)
                
                if 0 <= gx < g_width and 0 <= gy < g_height:
                    # Get occupancy from global map
                    g_idx = gy * g_width + gx
                    occupancy_prob = self.global_map.data[g_idx]
                    
                    if occupancy_prob >= 0:  # Known cell
                        # Convert probability [0, 100] to log-odds
                        prob = occupancy_prob / 100.0
                        if prob > 0.01 and prob < 0.99:
                            log_odds = np.log(prob / (1.0 - prob))
                            self.grid[lx, ly] = np.clip(log_odds, self.log_odds_min, self.log_odds_max)
    
    def scan_callback(self, msg: LaserScan):
        """Process LIDAR scan and update grid"""
        # Ray-cast each LIDAR beam
        angle = msg.angle_min
        for r in msg.ranges:
            # Skip invalid readings
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                angle += msg.angle_increment
                continue
            
            # Ray endpoint in world frame
            world_angle = self.robot_theta + angle
            end_x = self.robot_x + r * np.cos(world_angle)
            end_y = self.robot_y + r * np.sin(world_angle)
            
            # Ray-cast: mark free cells along ray, occupied at endpoint
            self.raycast(self.robot_x, self.robot_y, end_x, end_y, r < msg.range_max * 0.95)
            
            angle += msg.angle_increment
    
    def raycast(self, x0, y0, x1, y1, hit_obstacle):
        """Bresenham-like ray-casting for occupancy grid"""
        # Convert world coordinates to grid (centered on robot)
        def world_to_grid(wx, wy):
            # Grid center is at robot position
            gx = int((wx - (self.robot_x - self.grid_size/2)) / self.resolution)
            gy = int((wy - (self.robot_y - self.grid_size/2)) / self.resolution)
            return gx, gy
        
        gx0, gy0 = world_to_grid(x0, y0)
        gx1, gy1 = world_to_grid(x1, y1)
        
        # Bresenham line algorithm
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        
        x, y = gx0, gy0
        
        while True:
            # Mark current cell
            if 0 <= x < self.width and 0 <= y < self.height:
                if x == gx1 and y == gy1:
                    # Endpoint - mark as occupied if hit obstacle
                    if hit_obstacle:
                        self.grid[x, y] += self.log_odds_occupied
                        self.grid[x, y] = np.clip(self.grid[x, y], self.log_odds_min, self.log_odds_max)
                    break
                else:
                    # Free space along ray
                    self.grid[x, y] += self.log_odds_free
                    self.grid[x, y] = np.clip(self.grid[x, y], self.log_odds_min, self.log_odds_max)
            
            if x == gx1 and y == gy1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def publish_map(self):
        """Publish occupancy grid"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Map metadata
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Origin is bottom-left corner of grid
        msg.info.origin.position.x = self.robot_x - self.grid_size / 2
        msg.info.origin.position.y = self.robot_y - self.grid_size / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert log-odds to occupancy probability [0, 100]
        occupancy = np.zeros((self.width, self.height), dtype=np.int8)
        for i in range(self.width):
            for j in range(self.height):
                log_odds = self.grid[i, j]
                # Convert to probability: p = 1 / (1 + exp(-log_odds))
                prob = 1.0 / (1.0 + np.exp(-log_odds))
                
                if prob > 0.65:  # Occupied
                    occupancy[i, j] = 100
                elif prob < 0.35:  # Free
                    occupancy[i, j] = 0
                else:  # Unknown
                    occupancy[i, j] = -1
        
        # Flatten and publish
        msg.data = occupancy.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FastLocalGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

