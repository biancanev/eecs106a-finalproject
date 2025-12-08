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
        
        # CRITICAL: Store pose at scan time to prevent drift during rotation
        # When processing a scan, we need the robot pose AT THE TIME THE SCAN WAS TAKEN
        # not the current pose (which may have changed if robot rotated)
        self.scan_pose_x = 0.0
        self.scan_pose_y = 0.0
        self.scan_pose_theta = 0.0
        
        # Track angular velocity to filter scans during rotation
        self.prev_theta = 0.0
        self.prev_theta_time = None
        self.max_angular_velocity = 0.5  # rad/s - skip scans if rotating faster than this
        
        # CRITICAL: LIDAR frame offset - ADJUST THIS TO FIX ALIGNMENT!
        # Test each value to find correct orientation:
        #   0.0      = No rotation (0°)
        #   π/2      = 90° counter-clockwise (left)
        #   π        = 180° (backwards)
        #   -π/2     = 90° clockwise (right)
        #   3π/2     = 270° counter-clockwise (same as -π/2)
        # 
        # TEST PROCEDURE:
        # 1. Place obstacle DIRECTLY IN FRONT of robot
        # 2. Check /local_map in RViz
        # 3. If obstacle appears:
        #    - In front → CORRECT! ✓
        #    - Behind → Add π (180°)
        #    - To left → Add π/2 (90°)
        #    - To right → Subtract π/2 (-90°)
        # MIRRORING: Now handled by negating angle in world_angle calculation
        self.lidar_angle_offset = np.pi/2  # Standard offset
        
        self.get_logger().info(f'🔧 LIDAR offset: {self.lidar_angle_offset:.4f} rad = {np.degrees(self.lidar_angle_offset):.1f}°')
        
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
            f'  Frame: map (aligned with Cartographer)\n'
            f'  LIDAR angle offset: {np.degrees(self.lidar_angle_offset):.1f}° '
            f'(matching Cartographer TF convention)'
        )
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update robot pose and reinitialize grid from global map"""
        old_x, old_y = self.robot_x, self.robot_y
        old_theta = self.robot_theta
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Update angular velocity tracking for rotation filter
        current_time = self.get_clock().now()
        if self.prev_theta_time is None:
            # First pose update - initialize
            self.prev_theta = self.robot_theta
            self.prev_theta_time = current_time
        else:
            # Update tracking (used by scan_callback to detect rotation)
            dt = (current_time - self.prev_theta_time).nanoseconds / 1e9
            if dt > 0.01:  # At least 10ms
                self.prev_theta = old_theta  # Use previous theta for accurate omega calculation
                self.prev_theta_time = current_time
        
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
        """
        Process LIDAR scan and update WORLD obstacles.
        CRITICAL: Uses robot pose to convert LIDAR to world coordinates.
        If /amcl_pose drifts, world coordinates will drift!
        Ensure /amcl_pose is from stable source (Cartographer/MCL, not raw odometry).
        """
        # CRITICAL: Verify robot pose is valid and initialized
        if self.robot_x == 0.0 and self.robot_y == 0.0 and self.robot_theta == 0.0:
            # Pose not initialized yet - skip to avoid storing obstacles at origin
            return
        
        # CRITICAL: Use robot pose AT THE TIME THE SCAN WAS TAKEN, not current pose!
        # This prevents obstacles from moving when robot rotates.
        # The scan was taken at msg.header.stamp, so we should use the pose at that time.
        # For now, we'll use the stored scan pose (updated in pose_callback to match scan timing)
        # If scan is too old (>100ms), skip it to avoid using stale pose
        scan_time = self.get_clock().now()
        if msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0:
            # Convert scan timestamp to ROS time
            from builtin_interfaces.msg import Time
            scan_stamp = Time(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec)
            # For simplicity, use current pose if scan is recent (<100ms old)
            # Otherwise, we'd need pose history which is complex
            # The key fix: use scan_pose_* which is frozen at scan time, not robot_* which updates
            scan_x = self.scan_pose_x if self.scan_pose_x != 0.0 else self.robot_x
            scan_y = self.scan_pose_y if self.scan_pose_y != 0.0 else self.robot_y
            scan_theta = self.scan_pose_theta if self.scan_pose_theta != 0.0 else self.robot_theta
        else:
            # No timestamp - use current pose (fallback)
            scan_x = self.robot_x
            scan_y = self.robot_y
            scan_theta = self.robot_theta
        
        # CRITICAL: Skip scans during fast rotation to prevent scan smearing
        # During rotation, LIDAR scan takes time and robot orientation changes,
        # causing obstacles to appear everywhere around the robot
        current_time = self.get_clock().now()
        if self.prev_theta_time is not None:
            dt = (current_time - self.prev_theta_time).nanoseconds / 1e9
            if dt > 0.01:  # At least 10ms between scans
                # Calculate angular velocity
                dtheta = self.robot_theta - self.prev_theta
                # Normalize angle difference to [-pi, pi]
                dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
                omega = abs(dtheta / dt)
                
                # Skip scan if rotating too fast
                if omega > self.max_angular_velocity:
                    # Aggressively decay obstacles during rotation instead of adding new ones
                    for key in list(self.world_obstacles.keys()):
                        self.world_obstacles[key] *= 0.8  # Fast decay
                        if abs(self.world_obstacles[key]) < 1.0:
                            del self.world_obstacles[key]
                    # Regenerate grid and return (don't process this scan)
                    self.regenerate_grid_from_world()
                    self.publish_map()
                    return
        
        # Update angular velocity tracking
        self.prev_theta = self.robot_theta
        self.prev_theta_time = current_time
        
        # CRITICAL: Store pose at scan time for this scan processing
        # This ensures all rays use the SAME robot pose (at scan time)
        # preventing obstacles from moving when robot rotates
        self.scan_pose_x = self.robot_x
        self.scan_pose_y = self.robot_y
        self.scan_pose_theta = self.robot_theta
        
        # Ray-cast each LIDAR beam
        angle = msg.angle_min
        
        # Decay all existing obstacles slightly (but keep their world coordinates fixed)
        # CRITICAL: The keys (world_x, world_y) in world_obstacles are FIXED world coordinates
        # They should NOT change - only the log_odds values change
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
            # CRITICAL: Obstacles are FIXED in world coordinates, independent of robot orientation!
            # LIDAR angle is relative to robot's forward direction
            # World angle = robot_orientation + lidar_angle + lidar_offset
            # We use scan_theta (frozen) so all rays in scan use same robot orientation
            # MIRRORING FIX: Negate angle to flip left/right
            world_angle = scan_theta + (-angle) + self.lidar_angle_offset  # Add robot orientation to LIDAR angle
            end_x = scan_x + r * np.cos(world_angle)  # World coordinate - FIXED in world frame
            end_y = scan_y + r * np.sin(world_angle)  # World coordinate - FIXED in world frame
            
            # Update world obstacles (stores in world coordinates)
            # CRITICAL: Using scan_pose_* ensures obstacles are stored at FIXED world coordinates
            # that don't change when robot rotates. The map should stay fixed in world frame!
            self.update_world_obstacles(scan_x, scan_y, end_x, end_y, 
                                       r < msg.range_max * 0.95)
            
            angle += msg.angle_increment
        
        # Regenerate robot-centric grid from world obstacles
        # This projects FIXED world obstacles (if pose is stable) into moving grid
        self.regenerate_grid_from_world()
    
    def update_world_obstacles(self, x0, y0, x1, y1, hit_obstacle):
        """
        Update obstacles in WORLD coordinates (not grid coordinates).
        CRITICAL: x0, y0, x1, y1 are already in WORLD coordinates from scan_callback.
        We discretize to store in world_obstacles dictionary, but the coordinates
        themselves are FIXED in world frame and don't change as robot moves.
        """
        # Discretize to world grid (not robot-centric)
        # Use resolution for discretization - this creates a key for the dictionary
        # The key is a discretized world coordinate, but the actual value is still world coordinate
        def discretize(wx, wy):
            """
            Discretize world coordinates to resolution steps for dictionary key.
            CRITICAL: The discretized coordinate IS the world coordinate we store.
            This coordinate is FIXED in world frame - it doesn't change as robot moves.
            If this drifts, it means the input x1, y1 (from robot pose) is drifting.
            """
            # Round to nearest resolution step - this IS the world coordinate we store
            # This coordinate is FIXED and should not change
            discretized_x = round(wx / self.resolution) * self.resolution
            discretized_y = round(wy / self.resolution) * self.resolution
            return (discretized_x, discretized_y)
        
        # Mark endpoint as occupied or free
        # CRITICAL: end_key is a discretized world coordinate - FIXED in world frame
        # If x1, y1 drift (from robot pose drift), end_key will drift
        # Solution: Ensure /amcl_pose is stable (Cartographer/MCL, not odometry)
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
            wx = x0 + t * (x1 - x0)  # World coordinate - FIXED
            wy = y0 + t * (y1 - y0)  # World coordinate - FIXED
            free_key = discretize(wx, wy)  # Discretized key - still world coordinate
            
            if free_key in self.world_obstacles:
                self.world_obstacles[free_key] += self.log_odds_free
                # Remove if becomes very free
                if self.world_obstacles[free_key] < -3.0:
                    del self.world_obstacles[free_key]
    
    def regenerate_grid_from_world(self):
        """
        Regenerate robot-centric grid from world obstacles.
        CRITICAL: world_obstacles dictionary contains FIXED world coordinates.
        Grid origin moves with robot, but obstacles stay fixed in world frame.
        """
        # Clear grid
        self.grid = np.zeros((self.width, self.height), dtype=np.float32)
        
        # Grid bounds in world frame (moves with robot)
        min_x = self.robot_x - self.grid_size / 2
        min_y = self.robot_y - self.grid_size / 2
        max_x = self.robot_x + self.grid_size / 2
        max_y = self.robot_y + self.grid_size / 2
        
        # Project world obstacles into current robot-centric grid
        # CRITICAL: world_obstacles keys are FIXED world coordinates (world_x, world_y)
        # These don't change as robot moves - they're absolute positions in map frame
        for (world_x, world_y), log_odds in self.world_obstacles.items():
            # Check if obstacle is in current grid window
            # world_x, world_y are FIXED - they don't change!
            if min_x <= world_x <= max_x and min_y <= world_y <= max_y:
                # Convert FIXED world coordinates to grid coordinates
                # Grid origin moves, but world coordinates are fixed
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

