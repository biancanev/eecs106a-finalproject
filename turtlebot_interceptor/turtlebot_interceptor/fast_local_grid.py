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
        
        # CRITICAL: Store obstacles in WORLD FRAME, not grid frame
        # As robot moves, we reproject obstacles into robot-centric grid
        # This prevents obstacles from disappearing when robot moves
        self.world_obstacles = {}  # {(world_x, world_y): log_odds_value}
        self.obstacle_decay_rate = 0.95  # Decay old obstacles slowly
        
        # Grid storage (log-odds) - ROBOT-CENTRIC, regenerated each update
        self.grid = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # CRITICAL: LIDAR frame offset (if LIDAR is rotated relative to base_link)
        # Common issues: LIDAR mounted backwards (180°) or sideways (90°/-90°)
        # Adjust this if obstacles appear rotated
        self.lidar_angle_offset = np.pi  # 180 degrees - LIDAR is backwards!
        
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
            f'  Mode: Bayesian fusion (Cartographer prior + LIDAR updates)\n'
            f'  LIDAR angle offset: {np.degrees(self.lidar_angle_offset):.1f}° '
            f'(adjust if obstacles appear rotated)'
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
        """Initialize WORLD obstacles from Cartographer's global map (Bayesian prior)"""
        if self.global_map is None:
            return
        
        # Copy obstacles from global map to world obstacles
        g_width = self.global_map.info.width
        g_height = self.global_map.info.height
        g_resolution = self.global_map.info.resolution
        g_origin_x = self.global_map.info.origin.position.x
        g_origin_y = self.global_map.info.origin.position.y
        
        # Only import obstacles near robot (within 10m radius)
        for gx in range(g_width):
            for gy in range(g_height):
                # World coordinates
                world_x = gx * g_resolution + g_origin_x + g_resolution / 2
                world_y = gy * g_resolution + g_origin_y + g_resolution / 2
                
                # Check if near robot
                dist_to_robot = np.sqrt((world_x - self.robot_x)**2 + (world_y - self.robot_y)**2)
                if dist_to_robot > 10.0:  # Too far
                    continue
                
                # Get occupancy
                g_idx = gy * g_width + gx
                occupancy_prob = self.global_map.data[g_idx]
                
                if occupancy_prob > 65:  # Occupied in global map
                    # Convert to log-odds and store in world frame
                    prob = occupancy_prob / 100.0
                    log_odds = np.log(prob / (1.0 - prob))
                    
                    # Discretize to our resolution
                    key = (round(world_x / self.resolution) * self.resolution,
                          round(world_y / self.resolution) * self.resolution)
                    
                    # Only add if not already present (don't override LIDAR data)
                    if key not in self.world_obstacles:
                        self.world_obstacles[key] = np.clip(log_odds, self.log_odds_min, self.log_odds_max)
        
        # Regenerate grid from world obstacles
        self.regenerate_grid_from_world()
    
    def scan_callback(self, msg: LaserScan):
        """Process LIDAR scan and update WORLD obstacles"""
        # Ray-cast each LIDAR beam
        angle = msg.angle_min
        
        # Decay all existing obstacles slightly
        for key in list(self.world_obstacles.keys()):
            self.world_obstacles[key] *= self.obstacle_decay_rate
            # Remove very weak obstacles
            if abs(self.world_obstacles[key]) < 0.5:
                del self.world_obstacles[key]
        
        for r in msg.ranges:
            # Skip invalid readings
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                angle += msg.angle_increment
                continue
            
            # Ray endpoint in WORLD frame
            # CRITICAL: Add LIDAR frame offset to correct for mounting orientation
            world_angle = self.robot_theta + angle + self.lidar_angle_offset
            end_x = self.robot_x + r * np.cos(world_angle)
            end_y = self.robot_y + r * np.sin(world_angle)
            
            # Update world obstacles (stores in world coordinates)
            self.update_world_obstacles(self.robot_x, self.robot_y, end_x, end_y, 
                                       r < msg.range_max * 0.95)
            
            angle += msg.angle_increment
        
        # Regenerate robot-centric grid from world obstacles
        self.regenerate_grid_from_world()
    
    def update_world_obstacles(self, x0, y0, x1, y1, hit_obstacle):
        """Update obstacles in WORLD coordinates (not grid coordinates)"""
        # Discretize to world grid (not robot-centric)
        # Use resolution for discretization
        def discretize(wx, wy):
            return (round(wx / self.resolution) * self.resolution,
                   round(wy / self.resolution) * self.resolution)
        
        # Mark endpoint as occupied or free
        end_key = discretize(x1, y1)
        
        if hit_obstacle:
            # Strong occupied evidence
            if end_key in self.world_obstacles:
                self.world_obstacles[end_key] += self.log_odds_occupied
            else:
                self.world_obstacles[end_key] = self.log_odds_occupied
            # Clip
            self.world_obstacles[end_key] = np.clip(self.world_obstacles[end_key],
                                                     self.log_odds_min, self.log_odds_max)
        
        # Mark free space along ray (sparse - every 10cm)
        dist = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        num_samples = int(dist / 0.1)  # Sample every 10cm
        
        for i in range(1, num_samples):  # Skip start and end
            t = i / num_samples
            wx = x0 + t * (x1 - x0)
            wy = y0 + t * (y1 - y0)
            free_key = discretize(wx, wy)
            
            if free_key in self.world_obstacles:
                self.world_obstacles[free_key] += self.log_odds_free
                # Remove if becomes very free
                if self.world_obstacles[free_key] < -3.0:
                    del self.world_obstacles[free_key]
    
    def regenerate_grid_from_world(self):
        """Regenerate robot-centric grid from world obstacles"""
        # Clear grid
        self.grid = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Grid bounds in world frame
        min_x = self.robot_x - self.grid_size / 2
        min_y = self.robot_y - self.grid_size / 2
        max_x = self.robot_x + self.grid_size / 2
        max_y = self.robot_y + self.grid_size / 2
        
        # Project world obstacles into current robot-centric grid
        for (world_x, world_y), log_odds in self.world_obstacles.items():
            # Check if obstacle is in current grid window
            if min_x <= world_x <= max_x and min_y <= world_y <= max_y:
                # Convert to grid coordinates
                gx = int((world_x - min_x) / self.resolution)
                gy = int((world_y - min_y) / self.resolution)
                
                if 0 <= gx < self.width and 0 <= gy < self.height:
                    self.grid[gx, gy] = log_odds
    
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

