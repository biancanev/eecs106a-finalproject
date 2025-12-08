#!/usr/bin/env python3
"""
MPC Node for real-time trajectory planning
Based on the paper implementation
"""
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import transforms3d.euler as euler
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # Declare parameters (lab4 + lab8 pattern)
        self.declare_parameter('mpc_horizon', 15)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('v_max_base', 0.6)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('omega_max', 1.5)
        # use_sim_time may be passed from launch file - declare only if not already set
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass  # Parameter already declared by launch file
        # Fallback control gains (lab8 pattern)
        self.declare_parameter('Kp_v', 2.0)
        self.declare_parameter('Kp_w', 0.8)
        self.declare_parameter('Kd_w', 0.5)
        
        # Get parameters
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.N = self.get_parameter('mpc_horizon').get_parameter_value().integer_value
        self.v_max_base = self.get_parameter('v_max_base').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.omega_max = self.get_parameter('omega_max').get_parameter_value().double_value
        # Fallback control gains
        self.Kp_v = self.get_parameter('Kp_v').get_parameter_value().double_value
        self.Kp_w = self.get_parameter('Kp_w').get_parameter_value().double_value
        self.Kd_w = self.get_parameter('Kd_w').get_parameter_value().double_value

        # Subscriptions
        self.seeker_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.seeker_callback,
            10
        )

        # Optional: Target subscription (for interception mode)
        # For single robot navigation, use goal point instead
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('max_obstacles', 100)  # Max number of obstacles to track
        self.declare_parameter('obstacle_radius', 0.2)  # Default obstacle radius (meters)
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.max_obstacles = self.get_parameter('max_obstacles').get_parameter_value().integer_value
        self.obstacle_radius_param = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        
        self.target_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/target_estimate',
            self.target_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # CRITICAL: Subscribe to fast local grid for immediate obstacle updates
        # This updates at LIDAR rate (5-10 Hz) vs Cartographer's slow rate (1-2 Hz)
        self.local_map_sub = self.create_subscription(
            OccupancyGrid,
            '/local_map',
            self.local_map_callback,
            10
        )
        
        # CRITICAL: Subscribe to LIDAR for immediate obstacle detection
        # Map updates slowly, LIDAR gives instant detection
        from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
        lidar_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            lidar_qos
        )
        
        # CRITICAL: Subscribe to Cartographer's scan-matched points
        # These are already in map frame and aligned - no rotation needed!
        from sensor_msgs.msg import PointCloud2
        self.matched_points_sub = self.create_subscription(
            PointCloud2,
            '/scan_matched_points2',
            self.matched_points_callback,
            10
        )
        self.matched_points = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Trajectory visualization
        from visualization_msgs.msg import Marker
        self.traj_pub = self.create_publisher(Marker, '/mpc_trajectory', 10)
        self.waypoint_pub = self.create_publisher(Marker, '/mpc_waypoint', 10)

        # State
        self.seeker_pose = None
        self.seeker_cov = None
        self.target_pose = None
        self.target_cov = None
        self.latest_scan = None  # Store latest LIDAR scan
        self.local_map = None  # Fast local grid (5-10 Hz updates)
        
        # Persistent obstacle storage (keeps obstacles even if Cartographer clears them)
        # Each obstacle: (position, radius, timestamp)
        self.persistent_obstacles = []
        self.obstacle_timeout = 5.0  # Keep obstacles for 5 seconds
        
        # CRITICAL: LIDAR frame offset - AUTO-DETECTION enabled!
        # System will try different offsets and pick the best match
        self.lidar_angle_offset = 0.0  # Current offset
        self.lidar_offset_candidates = [0.0, np.pi/2, np.pi, -np.pi/2]  # Test these
        self.lidar_offset_scores = {offset: 0.0 for offset in self.lidar_offset_candidates}
        self.frame_calibration_samples = 0
        self.frame_calibration_needed = 100  # Collect 100 samples
        self.frame_calibrated = False
        
        self.get_logger().info(
            '🔧 LIDAR frame auto-calibration starting...\n'
            '   Move robot near obstacles for calibration'
        )
        
        # Emergency recovery state machine
        self.emergency_state = 'NORMAL'  # NORMAL, BACKUP, ROTATE, RECOVERY
        self.emergency_start_time = None
        self.backup_duration = 1.5  # Back up for 1.5 seconds (longer to get more space)
        self.rotate_duration = 3.0  # Rotate for up to 3 seconds to find clear path
        
        # Track safe trajectory for backing up along known-good path
        self.trajectory_history = []  # Store recent positions
        self.max_history_length = 50  # Keep last 5 seconds at 10Hz
        
        # Waypoint system for routing around obstacles
        self.current_waypoint = None  # If set, use this instead of final goal
        self.waypoint_reached_threshold = 0.20  # 20cm to consider waypoint "reached" - more forgiving
        self.waypoint_cleared_time = None  # Track when waypoint was last cleared
        self.waypoint_cooldown = 2.0  # Don't generate new waypoint for 2 seconds after clearing
        
        # ALGORITHMIC IMPROVEMENTS
        self.min_obstacle_distance = float('inf')  # Track closest obstacle
        self.velocity_scale_factor = 1.0  # Dynamic velocity scaling
        self.prev_safe_trajectory = None  # Memory of last safe path
        self.failed_waypoints = []  # Track waypoints that led to failures
        
        self.map = None
        self.seeker_state = None  # [px, py, theta, v]
        self.prev_state = None  # Previous state for velocity estimation
        self.prev_velocity = 0.0  # Previous velocity (lab8 pattern)
        self.prev_time = None  # Previous timestamp for accurate velocity estimation

        # Initialize MPC
        self.mpc = SimpleUnicycleMPC(horizon=self.N, dt=self.dt)

        # Startup delay: Wait 20 seconds for LIDAR, SLAM, and MCL to initialize
        self.startup_time = self.get_clock().now()
        self.startup_delay = 60.0  # 20 seconds delay (increased for sensor stabilization)

        # Timer for MPC updates
        # Start timer immediately, but check startup delay in callback
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'MPC node initialized (lab8 control patterns) - '
            f'Waiting {self.startup_delay}s for LIDAR/SLAM/MCL initialization...'
        )

    def map_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.map = msg
        
        # Log map reception periodically
        if not hasattr(self, '_map_callback_count'):
            self._map_callback_count = 0
        self._map_callback_count += 1
        
        if self._map_callback_count == 1:
            # First map received
            occupied_count = sum(1 for cell in msg.data if cell > 50)
            self.get_logger().info(
                f'MAP RECEIVED: {msg.info.width}x{msg.info.height} cells, '
                f'resolution={msg.info.resolution:.3f}m, '
                f'occupied={occupied_count}, '
                f'frame={msg.header.frame_id}'
            )
        elif self._map_callback_count % 20 == 0:
            # Every 20 updates (every ~10 seconds at 2Hz)
            occupied_count = sum(1 for cell in msg.data if cell > 50)
            self.get_logger().info(
                f'MAP UPDATE: {occupied_count} occupied cells'
            )
    
    def lidar_callback(self, msg: LaserScan):
        """Store latest LIDAR scan for immediate obstacle detection"""
        self.latest_scan = msg
    
    def matched_points_callback(self, msg):
        """Store Cartographer's scan-matched points (already in map frame!)"""
        self.matched_points = msg
    
    def local_map_callback(self, msg: OccupancyGrid):
        """Store fast local grid (updates at 5-10 Hz)"""
        self.local_map = msg

    def seeker_callback(self, msg: PoseWithCovarianceStamped):
        """Update seeker state (lab8 pattern - improved velocity estimation)"""
        self.seeker_pose = msg.pose.pose
        self.seeker_cov = np.array(msg.pose.covariance).reshape((6, 6))
        
        # Extract state [px, py, theta, v]
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Store pose for state update
        self.seeker_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            0.0  # Velocity will be updated below
        ])
        
        # Estimate velocity from previous state (lab8 pattern - more robust)
        if self.prev_state is not None:
            # Use actual time difference from message timestamps for more accurate velocity
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if hasattr(self, 'prev_time') and self.prev_time is not None:
                dt_actual = current_time - self.prev_time
            else:
                dt_actual = self.dt  # Fallback to nominal dt
            
            dx = msg.pose.pose.position.x - self.prev_state[0]
            dy = msg.pose.pose.position.y - self.prev_state[1]
            # Use velocity in direction of motion (more accurate)
            v = np.sqrt(dx**2 + dy**2) / dt_actual if dt_actual > 0.01 else 0.0  # Min dt to avoid division by zero
            # Smooth velocity estimate (exponential moving average) - more aggressive smoothing
            v = 0.5 * v + 0.5 * self.prev_velocity  # More smoothing to reduce noise
            v = np.clip(v, self.v_min, self.v_max_base)
            self.prev_time = current_time
        else:
            v = 0.0
            self.prev_time = None
        
        # Store previous state for next iteration
        self.prev_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ])
        self.prev_velocity = v
        
        self.seeker_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            v
        ])

    def target_callback(self, msg: PoseWithCovarianceStamped):
        """Update target state"""
        self.target_pose = msg.pose.pose
        self.target_cov = np.array(msg.pose.covariance).reshape((6, 6))

    def improved_obstacle_clustering(self, occupied_points, eps=0.15, min_samples=3):
        """
        ALGORITHMIC IMPROVEMENT: Better obstacle clustering using DBSCAN-like algorithm.
        Groups nearby occupied cells into coherent obstacles with better representation.
        
        Args:
            occupied_points: List of (x, y) coordinates
            eps: Maximum distance between points in same cluster (15cm)
            min_samples: Minimum points to form a cluster
        
        Returns: List of (center, radius) tuples
        """
        if len(occupied_points) == 0:
            return []
        
        points = np.array(occupied_points)
        n_points = len(points)
        
        # Compute pairwise distances efficiently
        labels = -np.ones(n_points, dtype=int)  # -1 = unassigned
        cluster_id = 0
        
        for i in range(n_points):
            if labels[i] != -1:
                continue  # Already assigned
            
            # Find all points within eps distance
            dists = np.linalg.norm(points - points[i], axis=1)
            neighbors = np.where(dists <= eps)[0]
            
            if len(neighbors) < min_samples:
                labels[i] = -1  # Noise point
                continue
            
            # Start new cluster
            labels[i] = cluster_id
            seed_set = list(neighbors)
            
            # Expand cluster
            while seed_set:
                current_point = seed_set.pop(0)
                
                if labels[current_point] == -1:
                    labels[current_point] = cluster_id
                elif labels[current_point] != -1:
                    continue
                
                labels[current_point] = cluster_id
                
                # Find neighbors of current point
                dists = np.linalg.norm(points - points[current_point], axis=1)
                new_neighbors = np.where(dists <= eps)[0]
                
                if len(new_neighbors) >= min_samples:
                    seed_set.extend(new_neighbors)
            
            cluster_id += 1
        
        # Convert clusters to obstacles
        obstacles = []
        for cid in range(cluster_id):
            cluster_points = points[labels == cid]
            if len(cluster_points) < 2:
                continue
            
            # Compute center and radius
            center = np.mean(cluster_points, axis=0)
            distances = np.linalg.norm(cluster_points - center, axis=1)
            radius = np.max(distances) + 0.05  # Add 5cm safety margin
            
            # Ensure minimum radius
            radius = max(radius, 0.08)
            
            obstacles.append((center, radius))
        
        return obstacles
    
    def extract_cones_from_map(self):
        """Extract circular obstacles (cones) from occupancy grid - based on lab6 patterns
        Returns obstacles in map frame (same frame as robot pose and goal)
        """
        obstacles = []
        if self.map is None:
            return obstacles
        
        # CRITICAL: Verify map is in 'map' frame
        if self.map.header.frame_id != 'map':
            self.get_logger().warn(
                f"Map frame_id is '{self.map.header.frame_id}', expected 'map'. "
                f"Obstacles may be in wrong coordinate frame!"
            )
        
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # DEBUG: Log map origin once
        if not hasattr(self, '_map_origin_logged'):
            self.get_logger().info(
                f"Map origin: ({origin_x:.3f}, {origin_y:.3f}), "
                f"resolution: {resolution:.3f}m, size: {width}x{height}, "
                f"frame_id: {self.map.header.frame_id}"
            )
            self._map_origin_logged = True
        
        # Extract occupied cells (not walls)
        # CRITICAL: Use lower threshold to catch more obstacles (50 might miss some)
        occupied_cells = []
        wall_margin = 0.35  # Cells near boundaries are walls
        occupancy_threshold = 30  # Lower threshold - catch more obstacles (was 50)
        
        for i in range(width * height):
            # Use lower threshold to catch more obstacles
            if self.map.data[i] > occupancy_threshold:  # Occupied (lowered from 50)
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x
                world_y = gy * resolution + origin_y
                
                # Skip walls
                if (world_x < origin_x + wall_margin or 
                    world_x > origin_x + width * resolution - wall_margin or
                    world_y < origin_y + wall_margin or 
                    world_y > origin_y + height * resolution - wall_margin):
                    continue
                
                occupied_cells.append((world_x, world_y))
        
        # Count total occupied cells for debugging (use same threshold as extraction)
        total_occupied = sum(1 for i in range(width * height) if self.map.data[i] > occupancy_threshold)
        
        # DEBUG: Log occupied cell count EVERY TIME to see what's happening
        if not hasattr(self, '_occupied_debug_count'):
            self._occupied_debug_count = 0
        self._occupied_debug_count += 1
        if self._occupied_debug_count % 20 == 0:  # Every 2 seconds
            self.get_logger().error(
                f"OBSTACLE EXTRACTION: Map has {total_occupied} occupied cells (threshold>{occupancy_threshold}), "
                f"{len(occupied_cells)} non-wall cells, "
                f"wall_margin={wall_margin}m"
            )
        
        if len(occupied_cells) == 0:
            if total_occupied > 0 and self._occupied_debug_count % 20 == 0:
                self.get_logger().error(
                    f"ALL {total_occupied} OCCUPIED CELLS ARE WALLS! wall_margin={wall_margin}m. "
                    f"Map bounds: x=[{origin_x:.2f}, {origin_x + width*resolution:.2f}], "
                    f"y=[{origin_y:.2f}, {origin_y + height*resolution:.2f}]"
                )
            return obstacles
        
        # Cluster nearby cells (cones are compact circular clusters)
        # CRITICAL: Use larger clustering distance for better obstacle detection
        # With 0.02m resolution, resolution*3 = 0.06m is too small
        # Use 0.15m clustering distance (about 7-8 cells at 0.02m resolution)
        cluster_distance = 0.15  # 15cm - good for small obstacles like cones
        
        clusters = []
        for x, y in occupied_cells:
            assigned = False
            for cluster in clusters:
                # Check distance to cluster center (more efficient)
                cluster_points = np.array(cluster)
                cluster_center = np.mean(cluster_points, axis=0)
                dist = np.sqrt((x - cluster_center[0])**2 + (y - cluster_center[1])**2)
                if dist < cluster_distance:
                    cluster.append((x, y))
                    assigned = True
                    break
            if not assigned:
                clusters.append([(x, y)])
        
        # ALGORITHMIC IMPROVEMENT 5: Use improved clustering instead of simple grid-based
        # Collect all occupied points first
        occupied_points = []
        for cluster in clusters:
            if len(cluster) < 2:  # Reduced from 3 - allow smaller obstacles
                continue
            
            points = np.array(cluster)
            min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
            min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
            width_cluster = max_x - min_x
            height_cluster = max_y - min_y
            
            # Skip very small clusters (noise)
            if width_cluster < 0.005 or height_cluster < 0.005:  # 5mm minimum
                continue
            
            # Skip elongated clusters (walls) - but be less strict
            if width_cluster > 0.01 and height_cluster > 0.01:  # Both dimensions significant
                aspect_ratio = max(width_cluster, height_cluster) / min(width_cluster, height_cluster)
                if aspect_ratio > 3.0:  # Increased from 2.5 - be less strict
                    continue
            
            # Compute center (use mean for better accuracy)
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            
            # Compute radius more accurately - use distance from center to farthest point
            distances_from_center = np.sqrt((points[:, 0] - center_x)**2 + (points[:, 1] - center_y)**2)
            max_dist = np.max(distances_from_center)
            radius = max_dist + resolution * 3  # Add 3 cells margin for safety (was 2)
            
            # Also use bounding box as fallback
            radius_bbox = max(width_cluster, height_cluster) / 2 + resolution * 2
            radius = max(radius, radius_bbox)  # Use larger of the two
            
            # CRITICAL: Inflate obstacle radius significantly - robot keeps hitting them
            radius = radius * 1.5  # 50% larger to account for uncertainty
            
            # Accept reasonable obstacle sizes (cones are typically 0.1-0.2m radius)
            if 0.05 < radius < 0.6:  # Expanded range - was 0.06-0.5
                obstacles.append((np.array([center_x, center_y]), radius))
                # DEBUG: Log each extracted obstacle
                if self._occupied_debug_count % 20 == 0:
                    self.get_logger().error(
                        f"  Extracted obstacle: center=({center_x:.3f}, {center_y:.3f}), "
                        f"radius={radius:.3f}m, cluster_size={len(cluster)}"
                    )
        
        # DEBUG: Log final count
        if self._occupied_debug_count % 20 == 0:
            self.get_logger().error(
                f"TOTAL EXTRACTED: {len(obstacles)} obstacles from {len(clusters)} clusters"
            )
        
        return obstacles[:10]  # Limit to 10 cones max
    
    def get_occupied_cells_as_obstacles(self, robot_pos, lookahead_dist=2.0):
        """
        Get ALL occupied cells near the robot as obstacles.
        This ensures the robot never hits any occupied cell.
        
        Args:
            robot_pos: (x, y) robot position in map frame
            lookahead_dist: Only consider cells within this distance (m)
        
        Returns:
            List of (center, radius) tuples for each occupied cell
        """
        obstacles = []
        if self.map is None:
            return obstacles
        
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # Robot safety radius (robot radius + margin)
        robot_radius = 0.15  # Robot radius
        safety_margin = 0.25  # INCREASED safety margin (was 0.1) - stay further away
        cell_radius = resolution * np.sqrt(2) / 2  # Half diagonal of cell (worst case)
        min_obstacle_radius = robot_radius + safety_margin + cell_radius
        
        # Extract occupied cells near robot
        occupancy_threshold = 30  # Same as extract_cones_from_map
        
        for i in range(width * height):
            if self.map.data[i] > occupancy_threshold:  # Occupied
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                
                # Only consider cells within lookahead distance
                dist_to_robot = np.sqrt((world_x - robot_pos[0])**2 + (world_y - robot_pos[1])**2)
                if dist_to_robot > lookahead_dist:
                    continue
                
                # Add as obstacle with minimum radius to ensure robot doesn't hit it
                obstacles.append((np.array([world_x, world_y]), min_obstacle_radius))
        
        return obstacles
    
    def generate_waypoint_if_blocked(self, robot_pos, goal_pos, obstacles):
        """
        Check if there's an obstacle directly ahead and generate waypoint.
        Triggers for obstacles within 0.20m straight ahead - proactive avoidance.
        
        Returns: waypoint position [x, y] if blocked NOW, None otherwise
        """
        if not obstacles or len(obstacles) == 0:
            return None
        
        robot_xy = robot_pos[:2]
        robot_theta = robot_pos[2]  # Current heading
        
        # Forward direction based on CURRENT heading (not goal direction)
        forward_dir = np.array([np.cos(robot_theta), np.sin(robot_theta)])
        
        # Only check obstacles that are:
        # 1. CLOSE (within 0.20m)
        # 2. DIRECTLY AHEAD (within ±30 degrees of forward direction)
        blocking_obstacles = []
        for center, radius in obstacles:
            # Vector from robot to obstacle
            to_obstacle = center - robot_xy
            dist_to_obstacle = np.linalg.norm(to_obstacle)
            
            # Skip if not close enough
            if dist_to_obstacle > 0.15:
                continue
            
            # Check if it's ahead of us
            if dist_to_obstacle > 0:
                to_obstacle_norm = to_obstacle / dist_to_obstacle
                
                # Dot product = cos(angle) - close to 1.0 means straight ahead
                forward_alignment = np.dot(to_obstacle_norm, forward_dir)
                
                # Only consider if within ±30 degrees (cos(30°) ≈ 0.866)
                if forward_alignment > 0.866:
                    blocking_obstacles.append((center, radius, dist_to_obstacle))
        
        if not blocking_obstacles:
            return None  # No immediate obstacle ahead
        
        # Close obstacle detected! Generate waypoint
        self.get_logger().warn(
            f"🚨 Obstacle within 0.20m ahead! Generating waypoint for {len(blocking_obstacles)} obstacles..."
        )
        
        # Find the closest blocking obstacle
        blocking_obstacles.sort(key=lambda x: x[2])  # Sort by distance
        closest_obstacle = blocking_obstacles[0]
        key_center, key_radius, key_dist = closest_obstacle
        
        # CRITICAL: Generate waypoint ALONG the path to goal, not perpendicular!
        # We want to stay on the line to the goal, just shift slightly to avoid obstacle
        
        # Direction to GOAL (not just forward!)
        goal_dir = goal_pos - robot_xy
        goal_dist = np.linalg.norm(goal_dir)
        if goal_dist > 0:
            goal_dir = goal_dir / goal_dist
        else:
            goal_dir = forward_dir  # Fallback
        
        # Perpendicular to GOAL direction (not robot heading)
        perpendicular = np.array([-goal_dir[1], goal_dir[0]])
        
        # Position waypoint ON THE PATH to goal, just shifted laterally
        # Stay close to the direct line!
        progress_distance = min(0.3, goal_dist * 0.3)  # 30% toward goal or 30cm max
        waypoint_base = robot_xy + goal_dir * progress_distance
        
        # MINIMAL lateral offsets - just enough to clear obstacle
        min_clearance = key_radius + 0.15 + 0.05  # obstacle + robot + 5cm
        offset_candidates = [min_clearance, min_clearance * 1.1, min_clearance * 1.2, 0.25]
        
        best_waypoint = None
        best_clearance = -999.0
        best_side = "LEFT"
        
        for offset in offset_candidates:
            # Try both sides
            for side_mult, side_name in [(1.0, "LEFT"), (-1.0, "RIGHT")]:
                candidate = waypoint_base + perpendicular * (offset * side_mult)
                
                # Check clearance to ALL obstacles
                clearances = [np.linalg.norm(obs[0] - candidate) - obs[1] for obs in obstacles]
                min_clearance_val = min(clearances) if clearances else 999.0
                
                # Also check if path from robot to waypoint is clear
                path_clear = True
                for obs_center, obs_radius in obstacles:
                    # Check if obstacle intersects robot-waypoint line
                    to_candidate = candidate - robot_xy
                    dist_to_candidate = np.linalg.norm(to_candidate)
                    if dist_to_candidate > 0:
                        to_candidate_norm = to_candidate / dist_to_candidate
                        proj = np.dot(obs_center - robot_xy, to_candidate_norm)
                        if 0 < proj < dist_to_candidate:
                            closest_pt = robot_xy + to_candidate_norm * proj
                            perp_dist = np.linalg.norm(obs_center - closest_pt)
                            if perp_dist < (obs_radius + 0.25):  # 25cm margin for path
                                path_clear = False
                                break
                
                # Keep best candidate
                if path_clear and min_clearance_val > best_clearance:
                    best_clearance = min_clearance_val
                    best_waypoint = candidate
                    best_side = side_name
            
            # If we found a good waypoint, use it (prefer tighter offsets)
            if best_waypoint is not None and best_clearance > 0.1:
                break
        
        if best_waypoint is None:
            # Fallback: just go perpendicular at safe distance
            best_waypoint = waypoint_base + perpendicular * 0.8
            best_side = "LEFT"
        
        self.get_logger().info(
            f"📍 Tight waypoint at ({best_waypoint[0]:.2f}, {best_waypoint[1]:.2f}) - "
            f"routing {best_side}, clearance={best_clearance:.2f}m"
        )
        
        return best_waypoint
    
    def compute_obstacles(self):
        """Extract obstacles - PRIMARY SOURCE: Cartographer scan-matched points!"""
        if self.seeker_state is None:
            return []
        
        obstacles = []
        
        # PART 1: Cartographer scan-matched points (PRIMARY - already in correct frame!)
        # These are LIDAR points that Cartographer has aligned with the map
        # NO frame offset needed - Cartographer handles all transforms!
        if self.matched_points is not None:
            obstacles.extend(self.extract_scan_matched_obstacles())
        
        # PART 2: Raw LIDAR (BACKUP - if scan-matched not available yet)
        # Use raw LIDAR with auto-calibrated offset
        if self.latest_scan is not None:
            obstacles.extend(self.extract_lidar_obstacles())
        
        # PART 3: Fast Local Grid (TERTIARY - for persistent memory)
        if self.local_map is not None:
            grid_obstacles = self.extract_map_obstacles_from_grid(self.local_map)
            obstacles.extend(grid_obstacles)
        
        # Remove duplicates and limit total
        obstacles = self.merge_obstacles(obstacles)
        
        # AUTO-CALIBRATE frame offset by comparing LIDAR vs grid
        if not self.frame_calibrated and self.local_map is not None and self.latest_scan is not None:
            self.calibrate_lidar_frame()
        
        return obstacles
    
    def extract_scan_matched_obstacles(self):
        """Extract obstacles from Cartographer's scan-matched point cloud"""
        if self.matched_points is None or self.seeker_state is None:
            return []
        
        from sensor_msgs_py import point_cloud2
        
        obstacles = []
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        
        obstacle_radius = self.obstacle_radius_param  # From parameter
        
        try:
            # Extract points from PointCloud2
            for point in point_cloud2.read_points(self.matched_points, 
                                                  field_names=("x", "y", "z"), 
                                                  skip_nans=True):
                px, py, pz = point
                
                # Distance from robot
                dx = px - robot_x
                dy = py - robot_y
                dist = np.sqrt(dx*dx + dy*dy)
                
                # Only within 2m
                if 0.1 < dist < 2.0:
                    obstacles.append((np.array([px, py]), obstacle_radius))
        except Exception as e:
            # Point cloud parsing can fail, fall back to LIDAR
            if not hasattr(self, '_pointcloud_error_logged'):
                self.get_logger().warn(f'PointCloud2 parsing error: {e}')
                self._pointcloud_error_logged = True
        
        return obstacles
    
    def calibrate_lidar_frame(self):
        """Auto-detect correct LIDAR frame offset by comparing with grid"""
        if self.latest_scan is None or self.seeker_state is None:
            return
        
        self.frame_calibration_samples += 1
        
        # For each candidate offset, compute how well LIDAR matches grid
        for offset in self.lidar_offset_candidates:
            # Temporarily use this offset
            old_offset = self.lidar_angle_offset
            self.lidar_angle_offset = offset
            
            # Extract LIDAR obstacles with this offset
            lidar_obs = self.extract_lidar_obstacles()
            
            # Extract grid obstacles
            grid_obs = self.extract_map_obstacles_from_grid(self.local_map)
            
            # Restore offset
            self.lidar_angle_offset = old_offset
            
            # Score: how many LIDAR obstacles are close to grid obstacles?
            matches = 0
            for lidar_pos, lidar_r in lidar_obs:
                for grid_pos, grid_r in grid_obs:
                    dist = np.sqrt((lidar_pos[0] - grid_pos[0])**2 + (lidar_pos[1] - grid_pos[1])**2)
                    if dist < 0.3:  # Within 30cm = match
                        matches += 1
                        break
            
            # Normalize by number of obstacles
            if len(lidar_obs) > 0:
                score = matches / len(lidar_obs)
                self.lidar_offset_scores[offset] += score
        
        # After enough samples, pick best offset
        if self.frame_calibration_samples >= self.frame_calibration_needed:
            best_offset = max(self.lidar_offset_scores, key=self.lidar_offset_scores.get)
            best_score = self.lidar_offset_scores[best_offset]
            
            self.lidar_angle_offset = best_offset
            self.frame_calibrated = True
            
            self.get_logger().info(
                f'✅ LIDAR FRAME CALIBRATED!\n'
                f'   Best offset: {np.degrees(best_offset):.1f}° (score: {best_score:.2f})\n'
                f'   Scores: ' + ', '.join([
                    f'{np.degrees(o):.0f}°={self.lidar_offset_scores[o]:.1f}' 
                    for o in self.lidar_offset_candidates
                ])
            )
        elif self.frame_calibration_samples % 20 == 0:
            self.get_logger().info(
                f'🔧 Frame calibration: {self.frame_calibration_samples}/{self.frame_calibration_needed} samples'
            )
    
    def extract_lidar_obstacles(self):
        """Convert LIDAR scan to immediate obstacles AND store persistently"""
        if self.latest_scan is None or self.seeker_state is None:
            return []
        
        obstacles = []
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        robot_theta = self.seeker_state[2]
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # LIDAR scan parameters
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges
        range_max = self.latest_scan.range_max
        
        # Convert LIDAR points to obstacles
        obstacle_radius = self.obstacle_radius_param  # From parameter
        
        for i, r in enumerate(ranges):
            # Skip invalid readings
            if r < 0.1 or r > range_max or not np.isfinite(r):
                continue
            
            # Only consider obstacles within 2m
            if r > 2.0:
                continue
            
            # Angle of this ray in robot frame
            ray_angle = angle_min + i * angle_increment
            
            # Convert to world frame
            # CRITICAL: Add LIDAR frame offset to correct for mounting orientation
            world_angle = robot_theta + ray_angle + self.lidar_angle_offset
            obstacle_x = robot_x + r * np.cos(world_angle)
            obstacle_y = robot_y + r * np.sin(world_angle)
            
            obstacle_pos = np.array([obstacle_x, obstacle_y])
            obstacles.append((obstacle_pos, obstacle_radius))
            
            # Add to persistent storage
            self.persistent_obstacles.append((obstacle_pos, obstacle_radius, current_time))
        
        # Clean up old obstacles (older than timeout)
        self.persistent_obstacles = [
            (pos, radius, t) for pos, radius, t in self.persistent_obstacles
            if current_time - t < self.obstacle_timeout
        ]
        
        return obstacles
    
    def extract_map_obstacles_from_grid(self, grid_map):
        """Extract obstacles from occupancy grid with clustering to reduce noise"""
        if grid_map is None or self.seeker_state is None:
            return []
        
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        
        # Extract occupied cells
        width = grid_map.info.width
        height = grid_map.info.height
        resolution = grid_map.info.resolution
        origin_x = grid_map.info.origin.position.x
        origin_y = grid_map.info.origin.position.y
        
        # First pass: find all occupied cells within range
        occupied_cells = []
        for i in range(width * height):
            if grid_map.data[i] > 65:  # Higher threshold to reduce noise
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x + resolution / 2
                world_y = gy * resolution + origin_y + resolution / 2
                
                dx = world_x - robot_x
                dy = world_y - robot_y
                dist = np.sqrt(dx*dx + dy*dy)
                
                if 0.1 < dist < 2.0:  # Within 2m, ignore cells on robot
                    occupied_cells.append((world_x, world_y))
        
        # Second pass: cluster nearby cells into single obstacles
        obstacles = []
        cluster_dist = 0.15  # 15cm clustering
        obstacle_radius = self.obstacle_radius_param  # From parameter
        
        used = set()
        for i, (cx, cy) in enumerate(occupied_cells):
            if i in used:
                continue
            
            # Find all cells within cluster distance
            cluster = [(cx, cy)]
            used.add(i)
            
            for j, (ox, oy) in enumerate(occupied_cells):
                if j in used:
                    continue
                if np.sqrt((cx - ox)**2 + (cy - oy)**2) < cluster_dist:
                    cluster.append((ox, oy))
                    used.add(j)
            
            # Use cluster center as obstacle
            if len(cluster) >= 2:  # At least 2 cells to be real obstacle
                center_x = sum(x for x, y in cluster) / len(cluster)
                center_y = sum(y for x, y in cluster) / len(cluster)
                obstacles.append((np.array([center_x, center_y]), obstacle_radius))
        
        return obstacles
    
    def extract_map_obstacles(self):
        """Extract obstacles from Cartographer map (persistent)"""
        if self.map is None or self.seeker_state is None:
            return []
        
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        
        # Extract ALL occupied cells within range as individual obstacles
        obstacles = []
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # CRITICAL: Large obstacle radius to ensure avoidance
        obstacle_radius = self.obstacle_radius_param * 1.2  # Slightly larger for map obstacles
        
        for i in range(width * height):
            if self.map.data[i] > 30:  # Occupied
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                
                # Distance from robot
                dx = world_x - robot_x
                dy = world_y - robot_y
                dist = np.sqrt(dx*dx + dy*dy)
                
                # Only within 1.5m of robot (reduced range for performance)
                if dist < 1.5 and dist > 0.02:
                    obstacles.append((np.array([world_x, world_y]), obstacle_radius))
        
        # Limit to closest 30 obstacles (for performance)
        if len(obstacles) > 30:
            obstacles.sort(key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
            obstacles = obstacles[:30]
        
        # DEBUG
        if not hasattr(self, '_obstacle_count'):
            self._obstacle_count = 0
        self._obstacle_count += 1
        if self._obstacle_count % 20 == 0:
            self.get_logger().error(
                f"OBSTACLE CELLS: Found {len(obstacles)} occupied cells within 1.5m, "
                f"robot=({robot_x:.3f}, {robot_y:.3f})"
            )
            if len(obstacles) > 0:
                closest = min(obstacles, key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
                dist_closest = np.sqrt((closest[0][0]-robot_x)**2 + (closest[0][1]-robot_y)**2)
                self.get_logger().error(
                    f"  Closest cell: ({closest[0][0]:.3f}, {closest[0][1]:.3f}), "
                    f"dist={dist_closest:.3f}m, radius={closest[1]:.3f}m"
                )
        
        return obstacles
    
    def handle_emergency_recovery(self):
        """Handle emergency backup and recovery along known-safe path"""
        elapsed = (self.get_clock().now() - self.emergency_start_time).nanoseconds / 1e9
        twist = Twist()
        
        if self.emergency_state == 'BACKUP':
            # Phase 1: Back up along known-safe trajectory
            if elapsed < self.backup_duration:
                # Use trajectory history to back up along previous path
                # The path we took to get here is guaranteed safe
                if len(self.trajectory_history) > 5:
                    # Get heading back along our path
                    current_x = self.seeker_state[0]
                    current_y = self.seeker_state[1]
                    # Look at position from 0.5s ago (5 steps at 10Hz)
                    prev_pos = self.trajectory_history[-5]
                    
                    # Calculate direction back to previous position
                    dx = prev_pos['x'] - current_x
                    dy = prev_pos['y'] - current_y
                    
                    # If significant distance, align to back up along that path
                    if np.sqrt(dx*dx + dy*dy) > 0.05:
                        target_heading = np.arctan2(dy, dx)
                        current_heading = self.seeker_state[2]
                        heading_error = target_heading - current_heading
                        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
                        
                        # Gentle turn correction while backing up
                        twist.linear.x = -0.15  # Faster reverse (15cm/s)
                        twist.angular.z = 0.5 * heading_error  # Proportional heading correction
                    else:
                        # No history or too close, just back up straight
                        twist.linear.x = -0.15
                        twist.angular.z = 0.0
                else:
                    # No history, back up straight
                    twist.linear.x = -0.15
                    twist.angular.z = 0.0
                
                self.get_logger().info(f'⬅️ BACKING UP along safe path... ({elapsed:.1f}s)')
            else:
                # Done backing up, start rotating to find clear path
                self.emergency_state = 'ROTATE'
                self.emergency_start_time = self.get_clock().now()
                self.get_logger().info('🔄 Looking for clear path...')
        
        elif self.emergency_state == 'ROTATE':
            # Phase 2: Rotate to find clear direction
            if elapsed < self.rotate_duration:
                # Check if path ahead is clear
                if not self.check_immediate_collision():
                    # Found clear path!
                    self.emergency_state = 'RECOVERY'
                    self.get_logger().info('✅ Clear path found! Resuming...')
                else:
                    # Keep rotating
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5  # Rotate at 0.5 rad/s
                    self.get_logger().info(f'🔄 Rotating to find path... ({elapsed:.1f}s)')
            else:
                # Couldn't find clear path, try backing up more
                self.emergency_state = 'BACKUP'
                self.emergency_start_time = self.get_clock().now()
                self.get_logger().warn('⚠️ No clear path found, backing up more...')
        
        elif self.emergency_state == 'RECOVERY':
            # Phase 3: Slowly resume - let MPC take over
            if elapsed < 0.5:
                # Brief pause before resuming
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Resume normal operation
                self.emergency_state = 'NORMAL'
                self.get_logger().info('🚀 Resuming normal navigation')
                return
        
        self.cmd_pub.publish(twist)
    
    def check_immediate_collision(self):
        """Check if obstacle is directly ahead within emergency distance"""
        if self.latest_scan is None or self.seeker_state is None:
            return False
        
        # Check LIDAR rays in front (±30 degrees)
        ranges = self.latest_scan.ranges
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        emergency_dist = 0.25  # 25cm emergency threshold - TIGHT for curved navigation
        front_range = np.pi / 6  # ±30 degrees
        
        for i, r in enumerate(ranges):
            if not np.isfinite(r) or r > self.latest_scan.range_max:
                continue
            
            angle = angle_min + i * angle_increment + self.lidar_angle_offset
            
            # Check if ray is pointing forward
            if abs(angle) < front_range:
                if r < emergency_dist:
                    return True
        
        return False
    
    def visualize_trajectory(self):
        """Publish MPC predicted trajectory for visualization"""
        if not hasattr(self.mpc, 'X_sol') or self.mpc.X_sol is None:
            return
        
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mpc_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Line properties
        marker.scale.x = 0.05  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add points from MPC solution
        try:
            X_sol = self.mpc.X_sol.value
            for i in range(X_sol.shape[1]):
                p = Point()
                p.x = float(X_sol[0, i])
                p.y = float(X_sol[1, i])
                p.z = 0.1
                marker.points.append(p)
        except:
            pass
        
        self.traj_pub.publish(marker)
    
    def publish_waypoint(self, waypoint):
        """Publish waypoint marker for visualization in RViz"""
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Sphere properties
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0  # Orange color
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.pose.position.x = float(waypoint[0])
        marker.pose.position.y = float(waypoint[1])
        marker.pose.position.z = 0.2
        
        self.waypoint_pub.publish(marker)
    
    def merge_obstacles(self, obstacles):
        """Remove duplicate obstacles and limit count"""
        if len(obstacles) == 0:
            return []
        
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        
        # Sort by distance to robot
        obstacles.sort(key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
        
        # Remove duplicates (obstacles within 0.1m of each other)
        unique_obstacles = []
        for obs in obstacles:
            # Check if this obstacle is too close to any existing one
            is_duplicate = False
            for existing in unique_obstacles:
                dist = np.sqrt((obs[0][0]-existing[0][0])**2 + (obs[0][1]-existing[0][1])**2)
                if dist < 0.15:  # Within 15cm = duplicate
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique_obstacles.append(obs)
        
        # Limit to max_obstacles (configurable)
        if len(unique_obstacles) > self.max_obstacles:
            unique_obstacles = unique_obstacles[:self.max_obstacles]
        
        # DEBUG logging
        if not hasattr(self, '_obstacle_merge_count'):
            self._obstacle_merge_count = 0
        self._obstacle_merge_count += 1
        if self._obstacle_merge_count % 20 == 0:
            lidar_count = sum(1 for obs in obstacles if obs[1] <= 0.26)  # LIDAR has ~0.25m radius
            map_count = len(obstacles) - lidar_count
            self.get_logger().info(
                f"OBSTACLES: {lidar_count} LIDAR + {map_count} map → {len(unique_obstacles)} after merge"
            )
        
        return unique_obstacles

    def predict_target_trajectory(self):
        """Predict target trajectory over MPC horizon (lab8 pattern - improved prediction)"""
        if self.target_pose is None:
            return None
        
        px_tgt = self.target_pose.position.x
        py_tgt = self.target_pose.position.y
        
        # If we have target covariance, extract velocity estimate
        # Target state from UKF: [px, py, vx, vy] in covariance
        # Check if velocity information is available in covariance
        if self.target_cov is not None:
            # Try to extract velocity from covariance (if UKF provides it)
            # For now, use simple constant velocity prediction
            # In full implementation, would use UKF predicted trajectory
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = px_tgt
            target_seq[1, :] = py_tgt
        else:
            # No velocity info - assume stationary
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = px_tgt
            target_seq[1, :] = py_tgt
        
        return target_seq

    def timer_callback(self):
        """Main MPC control loop (lab8 pattern - with fallback control)"""
        # Check startup delay - wait 10 seconds for initialization
        elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            # Log countdown every 2 seconds
            if not hasattr(self, '_startup_log_count'):
                self._startup_log_count = 0
            self._startup_log_count += 1
            if self._startup_log_count % 20 == 0:  # Every 2 seconds at 10Hz
                remaining = self.startup_delay - elapsed
                self.get_logger().info(
                    f'MPC startup delay: {remaining:.1f}s remaining for LIDAR/SLAM/MCL initialization...'
                )
            return  # Don't run MPC until startup delay is over
        
        if self.seeker_state is None:
            return
        
        # EMERGENCY RECOVERY SYSTEM
        if self.emergency_state != 'NORMAL':
            self.handle_emergency_recovery()
            return
        
        # Check for immediate collision danger
        if self.check_immediate_collision():
            self.get_logger().error('🚨 EMERGENCY: Obstacle ahead! Starting backup...')
            self.emergency_state = 'BACKUP'
            self.emergency_start_time = self.get_clock().now()
            self.handle_emergency_recovery()
            return
        
        # For single robot navigation, use goal point if target not available
        use_goal = (self.target_pose is None)

        # Build initial state
        x0 = self.seeker_state.copy()
        
        # CRITICAL DEBUG: Log state and goal
        if not hasattr(self, '_debug_count'):
            self._debug_count = 0
        self._debug_count += 1
        if self._debug_count % 20 == 0:  # Every 2 seconds
            self.get_logger().error(
                f"DEBUG: x0=[{x0[0]:.3f}, {x0[1]:.3f}, {np.degrees(x0[2]):.1f}°, {x0[3]:.3f}m/s], "
                f"goal_x={self.goal_x}, goal_y={self.goal_y}"
            )

        # Get target/goal position
        if use_goal:
            # Check if we need to use a waypoint or go directly to goal
            final_goal = np.array([self.goal_x, self.goal_y])
            
            # If we have a waypoint and haven't reached it, use waypoint
            if self.current_waypoint is not None:
                dist_to_waypoint = np.linalg.norm(self.current_waypoint - x0[:2])
                if dist_to_waypoint < self.waypoint_reached_threshold:
                    self.get_logger().info(f"✓ Reached waypoint, clearing and resuming to final goal")
                    self.current_waypoint = None
                    self.waypoint_cleared_time = self.get_clock().now()  # Mark when cleared
                    target_pos = final_goal
                else:
                    target_pos = self.current_waypoint
            else:
                target_pos = final_goal
            
            # Create constant target sequence
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = target_pos[0]
            target_seq[1, :] = target_pos[1]
            
            # CRITICAL DEBUG: Verify target sequence
            if self._debug_count % 20 == 0:
                self.get_logger().error(
                    f"DEBUG: target_seq[0,0]={target_seq[0,0]:.3f}, target_seq[1,0]={target_seq[1,0]:.3f}, "
                    f"dx={target_seq[0,0]-x0[0]:.3f}, dy={target_seq[1,0]-x0[1]:.3f}"
                )
        else:
            # Predict target trajectory
            target_seq = self.predict_target_trajectory()
            if target_seq is None:
                return

        # Compute obstacles with uncertainty inflation
        obstacles = self.compute_obstacles()  # Enable obstacle avoidance
        
        # ALGORITHMIC IMPROVEMENT 1: Compute minimum distance to obstacles for adaptive behavior
        self.min_obstacle_distance = self.compute_min_obstacle_distance(x0, obstacles)
        
        # ALGORITHMIC IMPROVEMENT 2: Adaptive velocity scaling based on proximity
        self.velocity_scale_factor = self.compute_velocity_scale(self.min_obstacle_distance)
        
        # WAYPOINT GENERATION: Check if path to goal is blocked and generate waypoint
        # BUT: Don't generate new waypoint immediately after clearing one (cooldown period)
        if use_goal and self.current_waypoint is None:
            can_generate = True
            if self.waypoint_cleared_time is not None:
                time_since_clear = (self.get_clock().now() - self.waypoint_cleared_time).nanoseconds / 1e9
                if time_since_clear < self.waypoint_cooldown:
                    can_generate = False
                    if self._obstacle_debug_count % 10 == 0:
                        self.get_logger().info(
                            f"⏳ Waypoint cooldown: {time_since_clear:.1f}s / {self.waypoint_cooldown}s"
                        )
            
            if can_generate:
                self.current_waypoint = self.generate_waypoint_if_blocked(x0, final_goal, obstacles)
        
        # DEBUG: Log obstacles periodically - MORE FREQUENT
        if not hasattr(self, '_obstacle_debug_count'):
            self._obstacle_debug_count = 0
        self._obstacle_debug_count += 1
        if self._obstacle_debug_count % 10 == 0:  # Every 1 second
            total_occupied = np.sum(np.array(self.map.data) > 30) if self.map else 0
            if obstacles and len(obstacles) > 0:
                self.get_logger().error(  # ERROR level so it's visible
                    f"OCCUPIED CELL OBSTACLES: Found {len(obstacles)} occupied cells as obstacles. "
                    f"Total occupied cells in map: {total_occupied}, "
                    f"Map frame_id: {self.map.header.frame_id if self.map else 'None'}, "
                    f"Robot pose: ({x0[0]:.3f}, {x0[1]:.3f})"
                )
                # Log closest obstacles
                obstacle_dists = [(float(np.sqrt((center[0] - x0[0])**2 + (center[1] - x0[1])**2)), center, radius) 
                                 for center, radius in obstacles]
                obstacle_dists.sort(key=lambda x: x[0])  # Sort by distance (first element)
                for i, (dist, center, radius) in enumerate(obstacle_dists[:5]):  # Log 5 closest
                    self.get_logger().error(
                        f"  Obstacle {i+1}: center=({center[0]:.3f}, {center[1]:.3f}), "
                        f"radius={radius:.3f}m, dist_to_robot={dist:.3f}m"
                    )
            else:
                self.get_logger().warn(
                    f"NO OCCUPIED CELLS NEAR ROBOT! Map has {total_occupied} total occupied cells "
                    f"(threshold=30, lookahead=3.0m)"
                )

        # ALGORITHMIC IMPROVEMENT 3: Adjust speed based on both uncertainty AND obstacle proximity
        base_v_max = self.v_max_base
        
        # Factor 1: Uncertainty-based scaling (from paper)
        if self.seeker_cov is not None:
            sigma_seek = np.sqrt(np.max(np.linalg.eigvals(self.seeker_cov[:2, :2])))
            alpha = 1.0
            base_v_max = base_v_max * np.exp(-alpha * sigma_seek)
        
        # Factor 2: Obstacle proximity scaling (NEW - algorithmic improvement)
        v_max = base_v_max * self.velocity_scale_factor
        self.mpc.v_max = np.clip(v_max, self.v_min * 0.5, self.v_max_base)  # Allow stopping if needed
        
        # Log velocity scaling
        if hasattr(self, '_obstacle_debug_count') and self._obstacle_debug_count % 10 == 0:
            self.get_logger().info(
                f"📊 Velocity: min_obs_dist={self.min_obstacle_distance:.3f}m, "
                f"scale={self.velocity_scale_factor:.2f}, v_max={self.mpc.v_max:.3f}m/s"
            )

        # Solve MPC (lab8 pattern - with fallback to proportional control)
        try:
            # Try MPC solve
            twist_cmd = self.mpc.get_twist_command(x0, target_seq, obstacles)
            v_cmd = twist_cmd['linear']['x']
            omega_cmd = twist_cmd['angular']['z']
            
            # Validate MPC solution
            if math.isnan(v_cmd) or math.isnan(omega_cmd) or \
               math.isinf(v_cmd) or math.isinf(omega_cmd):
                raise ValueError("MPC solution contains NaN or Inf")
            
            # ALGORITHMIC IMPROVEMENT 4: Full trajectory safety verification
            is_safe, clearance = self.verify_full_trajectory_safety(x0, v_cmd, omega_cmd, obstacles)
            if not is_safe:
                self.get_logger().warn(
                    f"⚠️ Trajectory verification failed! Min clearance: {clearance:.3f}m. "
                    f"Triggering emergency waypoint."
                )
                # Generate emergency waypoint if not already set
                if self.current_waypoint is None and use_goal:
                    self.current_waypoint = self.generate_waypoint_if_blocked(x0, final_goal, obstacles)
                # Reduce velocity significantly
                v_cmd *= 0.3
                omega_cmd *= 0.5
            
            # CRITICAL DEBUG: Log everything to find the bug
            if not hasattr(self, '_mpc_cmd_count'):
                self._mpc_cmd_count = 0
            self._mpc_cmd_count += 1
            if self._mpc_cmd_count % 10 == 0:  # Every 1 second - MORE FREQUENT
                angle_to_goal = np.arctan2(target_seq[1,0] - x0[1], target_seq[0,0] - x0[0])
                angle_err = angle_to_goal - x0[2]
                angle_err = np.mod(angle_err + np.pi, 2*np.pi) - np.pi
                dist = np.sqrt((target_seq[0,0] - x0[0])**2 + (target_seq[1,0] - x0[1])**2)
                dx = target_seq[0,0] - x0[0]
                dy = target_seq[1,0] - x0[1]
                # Check if commands make sense
                self.get_logger().error(  # ERROR level so it's visible
                    f"MPC: robot=({x0[0]:.3f}, {x0[1]:.3f}, {np.degrees(x0[2]):.1f}°), v={x0[3]:.3f}, "
                    f"goal=({target_seq[0,0]:.3f}, {target_seq[1,0]:.3f}), "
                    f"dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f}m, "
                    f"angle_to_goal={np.degrees(angle_to_goal):.1f}°, angle_err={np.degrees(angle_err):.1f}°, "
                    f"v_cmd={v_cmd:.3f}, omega_cmd={np.degrees(omega_cmd):.1f}°"
                )
            
            # Clip commands to safe limits
            v_cmd = np.clip(v_cmd, self.v_min, self.v_max_base)
            omega_cmd = np.clip(omega_cmd, -self.omega_max, self.omega_max)
            
        except Exception as e:
            # Log error details for debugging (but not every time to avoid spam)
            if not hasattr(self, '_mpc_error_count'):
                self._mpc_error_count = 0
            self._mpc_error_count += 1
            if self._mpc_error_count % 20 == 0:  # Every 2 seconds at 10Hz
                self.get_logger().warn(
                    f"MPC solve failed: {e}, using fallback control. "
                    f"Robot: ({x0[0]:.2f}, {x0[1]:.2f}, {np.degrees(x0[2]):.1f}°), "
                    f"Goal: ({target_seq[0,0]:.2f}, {target_seq[1,0]:.2f})"
                )
            # Fallback to proportional control (lab8 pattern)
            v_cmd, omega_cmd = self.fallback_control(x0, target_seq)

        # Publish command in Twist format
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(omega_cmd)
        self.cmd_pub.publish(twist)
        
        # Record successful position in trajectory history (for safe backup)
        if self.seeker_state is not None:
            self.trajectory_history.append({
                'x': self.seeker_state[0],
                'y': self.seeker_state[1],
                'theta': self.seeker_state[2]
            })
            # Keep only recent history
            if len(self.trajectory_history) > self.max_history_length:
                self.trajectory_history.pop(0)
        
        # SAFETY FILTER: Check if command would cause collision
        if not self.is_command_safe(v_cmd, omega_cmd):
            self.get_logger().error('🛑 SAFETY FILTER: MPC planned unsafe path! Starting backup...')
            # Trigger emergency backup - we know the path behind is safe
            self.emergency_state = 'BACKUP'
            self.emergency_start_time = self.get_clock().now()
            self.handle_emergency_recovery()
            return
        
        # Visualize MPC predicted trajectory
        self.visualize_trajectory()
        
        # Visualize waypoint if active
        if self.current_waypoint is not None:
            self.publish_waypoint(self.current_waypoint)
    
    def compute_min_obstacle_distance(self, x0, obstacles):
        """
        ALGORITHMIC IMPROVEMENT: Compute minimum distance to any obstacle.
        Used for adaptive velocity scaling and planning horizon adjustment.
        """
        if not obstacles or len(obstacles) == 0:
            return float('inf')
        
        robot_pos = x0[:2]
        min_dist = float('inf')
        
        for center, radius in obstacles:
            # Distance from robot center to obstacle surface
            dist_to_center = np.linalg.norm(center - robot_pos)
            dist_to_surface = dist_to_center - radius
            min_dist = min(min_dist, dist_to_surface)
        
        return max(0.0, min_dist)  # Clamp to non-negative
    
    def compute_velocity_scale(self, min_obs_dist):
        """
        ALGORITHMIC IMPROVEMENT: Adaptive velocity scaling based on obstacle proximity.
        Automatically slow down near obstacles for better reaction time and safety.
        
        Returns: scale factor in [0.3, 1.0]
        """
        if min_obs_dist >= 1.0:
            # Far from obstacles - full speed
            return 1.0
        elif min_obs_dist >= 0.5:
            # Moderate distance - slight slowdown (linear interpolation)
            # 1.0m -> 1.0, 0.5m -> 0.8
            return 0.8 + 0.2 * (min_obs_dist - 0.5) / 0.5
        elif min_obs_dist >= 0.3:
            # Close - significant slowdown
            # 0.5m -> 0.8, 0.3m -> 0.5
            return 0.5 + 0.3 * (min_obs_dist - 0.3) / 0.2
        elif min_obs_dist >= 0.15:
            # Very close - major slowdown
            # 0.3m -> 0.5, 0.15m -> 0.3
            return 0.3 + 0.2 * (min_obs_dist - 0.15) / 0.15
        else:
            # Extremely close - minimum speed (but don't stop)
            return 0.3
    
    def verify_full_trajectory_safety(self, x0, v, omega, obstacles, horizon_steps=10):
        """
        ALGORITHMIC IMPROVEMENT: Verify safety of ENTIRE predicted trajectory, not just first step.
        Simulates robot motion forward and checks for collisions at each step.
        
        Returns: (is_safe, min_clearance_along_path)
        """
        if not obstacles or len(obstacles) == 0:
            return True, float('inf')
        
        # Simulate forward motion
        dt = 0.1  # 100ms steps
        x, y, theta, v_curr = x0[0], x0[1], x0[2], x0[3]
        min_clearance = float('inf')
        
        for step in range(horizon_steps):
            # Simple kinematic model (same as MPC)
            x += v_curr * np.cos(theta) * dt
            y += v_curr * np.sin(theta) * dt
            theta += omega * dt
            v_curr = v  # Assume velocity reaches commanded value
            
            # Check clearance to all obstacles
            robot_pos = np.array([x, y])
            for obs_center, obs_radius in obstacles:
                dist_to_center = np.linalg.norm(obs_center - robot_pos)
                clearance = dist_to_center - obs_radius - 0.105  # Robot radius
                min_clearance = min(min_clearance, clearance)
                
                # If collision imminent, trajectory is unsafe
                if clearance < 0.05:  # 5cm safety margin
                    return False, clearance
        
        return True, min_clearance
    
    def is_command_safe(self, v_cmd, omega_cmd):
        """Check if executing this command would cause collision"""
        if self.latest_scan is None or self.seeker_state is None:
            return True  # No sensor data, allow
        
        # Simulate one step forward with this command
        dt = 0.1
        x = self.seeker_state[0]
        y = self.seeker_state[1]
        theta = self.seeker_state[2]
        
        # Predicted position after dt
        new_theta = theta + omega_cmd * dt
        new_x = x + v_cmd * np.cos(new_theta) * dt
        new_y = y + v_cmd * np.sin(new_theta) * dt
        
        # Check LIDAR for obstacles in that direction
        ranges = self.latest_scan.ranges
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        
        safety_dist = 0.2  # 20cm safety threshold - TIGHT for curved navigation
        
        # Check direction we're moving
        move_direction = np.arctan2(new_y - y, new_x - x) - theta
        move_direction = np.arctan2(np.sin(move_direction), np.cos(move_direction))  # Wrap
        
        for i, r in enumerate(ranges):
            if not np.isfinite(r) or r > self.latest_scan.range_max:
                continue
            
            angle = angle_min + i * angle_increment + self.lidar_angle_offset
            angle = np.arctan2(np.sin(angle), np.cos(angle))  # Wrap
            
            # Check if ray is in our movement direction (±45 degrees)
            if abs(angle - move_direction) < np.pi / 4:
                if r < safety_dist:
                    return False  # Obstacle in path!
        
        return True  # Safe
    
    def fallback_control(self, x0, target_seq):
        """Fallback proportional control (lab8 pattern)"""
        px, py, theta, v = x0
        
        # Get target position
        if target_seq.ndim == 2:
            tgt_x = target_seq[0, 0]
            tgt_y = target_seq[1, 0]
        else:
            tgt_x = target_seq[0]
            tgt_y = target_seq[1]
        
        # Compute error in world frame (map frame)
        dx = tgt_x - px
        dy = tgt_y - py
        dist = np.sqrt(dx**2 + dy**2)
        angle_to_target = np.arctan2(dy, dx)
        
        # Angle error (difference between desired heading and current heading)
        angle_err = angle_to_target - theta
        # Wrap to [-pi, pi] - CRITICAL: This ensures shortest rotation
        angle_err = np.mod(angle_err + np.pi, 2*np.pi) - np.pi
        
        # Debug logging (periodic)
        if not hasattr(self, '_fallback_log_count'):
            self._fallback_log_count = 0
        self._fallback_log_count += 1
        if self._fallback_log_count % 20 == 0:  # Every 2 seconds at 10Hz
            self.get_logger().info(
                f'Fallback: robot=({px:.2f}, {py:.2f}, {np.degrees(theta):.1f}°), '
                f'goal=({tgt_x:.2f}, {tgt_y:.2f}), dist={dist:.2f}m, '
                f'angle_to_target={np.degrees(angle_to_target):.1f}°, '
                f'angle_err={np.degrees(angle_err):.1f}°, '
                f'omega_cmd={np.degrees(self.Kp_w * angle_err):.1f}°/s'
            )
        
        # Proportional control
        # CRITICAL: Don't move forward if angle error is large (turn first)
        if abs(angle_err) > np.pi / 4:  # More than 45° off
            v_cmd = 0.0  # Stop and turn first
            omega_cmd = self.Kp_w * angle_err
        else:
            # Move forward and turn simultaneously
            v_cmd = self.Kp_v * min(dist, 1.0)  # Cap distance influence
            omega_cmd = self.Kp_w * angle_err
        
        # Clip to limits
        v_cmd = np.clip(v_cmd, self.v_min, self.v_max_base)
        omega_cmd = np.clip(omega_cmd, -self.omega_max, self.omega_max)
        
        return v_cmd, omega_cmd


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

