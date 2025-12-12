#!/usr/bin/env python3
"""
MPC Node for real-time trajectory planning
Based on the paper implementation
"""
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import transforms3d.euler as euler
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # Declare parameters (lab4 + lab8 pattern)
        self.declare_parameter('mpc_horizon', 15)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('v_max_base', 0.6)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('omega_max', 1.5)
        self.declare_parameter('startup_warmup', 65.0)
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
        self.startup_warmup = self.get_parameter('startup_warmup').get_parameter_value().double_value
        # Fallback control gains
        self.Kp_v = self.get_parameter('Kp_v').get_parameter_value().double_value
        self.Kp_w = self.get_parameter('Kp_w').get_parameter_value().double_value
        self.Kd_w = self.get_parameter('Kd_w').get_parameter_value().double_value

        # Subscriptions
        # Use EKF odometry if available for more consistent pose/heading
        self.seeker_sub = self.create_subscription(
            Odometry,
            '/odom_ekf',
            self.odom_ekf_callback,
            10
        )

        # Optional: Target subscription (for interception mode)
        # For single robot navigation, use goal point instead
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('max_obstacles', 100)  # Max number of obstacles to track
        self.declare_parameter('obstacle_radius', 0.08)  # Default obstacle radius (meters) - INCREASED for safety
        
        # CRITICAL FIX: Handle both int and float types for goal parameters
        # When user passes goal_x:=0 or goal_y:=0, ROS2 interprets as INTEGER, not DOUBLE
        goal_x_param = self.get_parameter('goal_x').get_parameter_value()
        goal_y_param = self.get_parameter('goal_y').get_parameter_value()
        
        # Try double first, fall back to integer
        try:
            self.goal_x = goal_x_param.double_value
        except:
            self.goal_x = float(goal_x_param.integer_value)
        
        try:
            self.goal_y = goal_y_param.double_value
        except:
            self.goal_y = float(goal_y_param.integer_value)
        
        self.max_obstacles = self.get_parameter('max_obstacles').get_parameter_value().integer_value
        self.obstacle_radius_param = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        
        # CRITICAL: Log goal to verify it's correct
        self.get_logger().info(f'🎯 GOAL SET: x={self.goal_x}, y={self.goal_y}')
        
        self.target_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/target_estimate',
            self.target_callback,
            10
        )
        # Slower, covariance-inflated broadcast coming from UKF to keep the seeker cautious
        self.target_sub_slow = self.create_subscription(
            PoseWithCovarianceStamped,
            '/target_estimate_slow',
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
        
        # Camera-based cone detection with confidence
        self.camera_cones_sub = self.create_subscription(
            PointStamped,
            '/camera_cone_positions',
            self.camera_cone_callback,
            10
        )
        self.camera_cones = []  # List of (position, confidence, timestamp) tuples
        self.camera_cone_timeout = 3.0  # Keep camera detections longer so close cones persist

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Trajectory visualization
        from visualization_msgs.msg import Marker, MarkerArray
        self.traj_pub = self.create_publisher(Marker, '/mpc_trajectory', 10)
        self.traj_array_pub = self.create_publisher(MarkerArray, '/mpc_trajectory_full', 10)
        self.waypoint_pub = self.create_publisher(Marker, '/mpc_waypoint', 10)
        self.obs_pub = self.create_publisher(Marker, '/obstacles', 10)
        # State
        self.seeker_pose = None
        self.seeker_cov = None
        self.target_pose = None
        self.target_cov = None
        self.target_pose_map_frame = None  # Target pose transformed to map frame
        self.latest_scan = None  # Store latest LIDAR scan
        self.local_map = None  # Fast local grid (5-10 Hz updates)
        
        # TF2 for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
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
        self.waypoint_cooldown = 50.0  # Don't generate new waypoint for 5 seconds after clearing (longer!)
        
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
        self.target_velocity = np.zeros(2)
        self.target_stamp = None
        self.max_target_jump = 1.5  # meters; reject obviously bad jumps

        # Initialize MPC
        self.mpc = SimpleUnicycleMPC(horizon=self.N, dt=self.dt)

        self.startup_time = self.get_clock().now()

        # Timer for MPC updates
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'🚀 MPC node initialized - {self.startup_warmup:.0f}s warmup before commanding (sensors + target EKF)'
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
        """Fallback seeker state from /amcl_pose (PoseWithCovarianceStamped)"""
        self._update_seeker_from_pose(msg.pose.pose, msg.pose.covariance, msg.header.stamp)

    def odom_ekf_callback(self, msg: Odometry):
        """Primary seeker state from /odom_ekf (Odometry)"""
        self._update_seeker_from_pose(msg.pose.pose, msg.pose.covariance, msg.header.stamp)

    def _update_seeker_from_pose(self, pose, covariance, stamp):
        self.seeker_pose = pose
        try:
            self.seeker_cov = np.array(covariance).reshape((6, 6))
        except Exception:
            self.seeker_cov = None

        q = pose.orientation
        _, _, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])

        if self.prev_state is not None:
            current_time = stamp.sec + stamp.nanosec * 1e-9
            dt_actual = current_time - self.prev_time if hasattr(self, 'prev_time') and self.prev_time is not None else self.dt
            dx = pose.position.x - self.prev_state[0]
            dy = pose.position.y - self.prev_state[1]
            v = np.sqrt(dx**2 + dy**2) / dt_actual if dt_actual > 0.01 else 0.0
            v = 0.5 * v + 0.5 * self.prev_velocity
            v = np.clip(v, self.v_min, self.v_max_base)
            self.prev_time = current_time
        else:
            v = 0.0
            self.prev_time = None

        self.prev_state = np.array([pose.position.x, pose.position.y, yaw])
        self.prev_velocity = v

        self.seeker_state = np.array([pose.position.x, pose.position.y, yaw, v])

    def target_callback(self, msg: PoseWithCovarianceStamped):
        """Update target state with proper frame transformation"""
        self.target_pose = msg.pose.pose
        self.target_cov = np.array(msg.pose.covariance).reshape((6, 6))
        
        # CRITICAL: Transform target pose to map frame if needed
        target_frame = msg.header.frame_id
        if target_frame == 'map':
            # Already in map frame - use directly
            self.target_pose_map_frame = self.target_pose
        else:
            # Need to transform to map frame
            try:
                # Create PoseStamped for transformation
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = self.target_pose
                
                # Lookup transform from target frame to map frame
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    target_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Transform pose to map frame
                transformed_pose = do_transform_pose(pose_stamped.pose, transform)
                self.target_pose_map_frame = transformed_pose
                
                if not hasattr(self, '_target_transform_count'):
                    self._target_transform_count = 0
                self._target_transform_count += 1
                if self._target_transform_count % 50 == 0:
                    self.get_logger().info(
                        f'✅ Transformed target from {target_frame} to map frame: '
                        f'({transformed_pose.position.x:.2f}, {transformed_pose.position.y:.2f})'
                    )
                    
            except TransformException as e:
                # Transform failed - log warning and use original (might be wrong frame)
                if not hasattr(self, '_target_transform_error_count'):
                    self._target_transform_error_count = 0
                self._target_transform_error_count += 1
                if self._target_transform_error_count % 50 == 0:
                    self.get_logger().warn(
                        f'⚠️ Failed to transform target from {target_frame} to map: {e}. '
                        f'Using original pose (may be in wrong frame!)'
                    )
                # Fallback: assume it's already in map frame (might be wrong!)
                self.target_pose_map_frame = self.target_pose

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
            self.get_logger().info(
                f"OBSTACLE EXTRACTION: Map has {total_occupied} occupied cells (threshold>{occupancy_threshold}), "
                f"{len(occupied_cells)} non-wall cells, "
                f"wall_margin={wall_margin}m"
            )
        
        if len(occupied_cells) == 0:
            if total_occupied > 0 and self._occupied_debug_count % 20 == 0:
                self.get_logger().warn(
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
                # Normalize direction vector FROM robot TO obstacle center
                current_x = self.seeker_state[0]
                current_y = self.seeker_state[1]
                dx = center_x - current_x
                dy = center_y - current_y
                dist = np.sqrt(dx**2 + dy**2)
                
                if dist > 0.01:  # Avoid division by zero
                    ux = dx / dist
                    uy = dy / dist
                    
                    # Push obstacle center AWAY from robot by radius (more conservative)
                    # This accounts for the fact that detected point might be on surface
                    corrected_x = center_x + ux * radius
                    corrected_y = center_y + uy * radius
                    obstacles.append((np.array([corrected_x, corrected_y]), radius))
                else:
                    # Too close, use center as-is
                    obstacles.append((np.array([center_x, center_y]), radius))
                # DEBUG: Log each extracted obstacle
                if self._occupied_debug_count % 20 == 0:
                    self.get_logger().info(
                        f"  Extracted obstacle: center=({center_x:.3f}, {center_y:.3f}), "
                        f"radius={radius:.3f}m, cluster_size={len(cluster)}"
                    )
        
        # DEBUG: Log final count
        if self._occupied_debug_count % 20 == 0:
            self.get_logger().info(
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
        # 1. CLOSE (within 0.25m)
        # 2. DIRECTLY AHEAD (within ±30 degrees of forward direction)
        blocking_obstacles = []
        for center, radius in obstacles:
            # Vector from robot to obstacle
            to_obstacle = center - robot_xy
            dist_to_obstacle = np.linalg.norm(to_obstacle)
            
            # Skip if not close enough
            if dist_to_obstacle > 0.25:
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
        # Stay VERY close to the direct line!
        progress_distance = min(0.25, goal_dist * 0.25)  # 25% toward goal or 25cm max
        waypoint_base = robot_xy + goal_dir * progress_distance
        
        # ULTRA-MINIMAL lateral offsets - absolute minimum to clear obstacle
        min_clearance = key_radius + 0.15 + 0.03  # obstacle + robot + 3cm (very tight!)
        offset_candidates = [min_clearance, min_clearance * 1.05, min_clearance * 1.1, 0.18]
        
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
    
    def camera_cone_callback(self, msg: PointStamped):
        """Store camera-detected cone positions with confidence"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        cone_pos = np.array([msg.point.x, msg.point.y])
        
        # Extract confidence from z coordinate (temporary solution)
        # Better: use custom message type
        confidence = float(msg.point.z) if msg.point.z > 0 else 0.5
        
        self.camera_cones.append((cone_pos, confidence, current_time))
        
        # Clean up old detections
        self.camera_cones = [
            (pos, conf, t) for pos, conf, t in self.camera_cones
            if current_time - t < self.camera_cone_timeout
        ]
    
    def compute_obstacles(self):
        """
        Extract obstacles with CONFIDENCE-BASED FUSION.
        Dynamic confidence switching: LIDAR (far) → Camera (close)
        """
        if self.seeker_state is None:
            return []
        
        obstacles = []
        current_time = self.get_clock().now().nanoseconds / 1e9
        robot_pos = self.seeker_state[:2]
        
        # STEP 1: Collect obstacles - Voxel Grid IS processed LIDAR!
        # PRIORITY: Voxel Grid (processed LIDAR) > Raw LIDAR > Camera (cones only)
        all_obstacles = {}  # Map: (x, y) -> (position, radius, confidence, source)
        
        # Source 1: Voxel Grid (processed LIDAR from fast_local_grid) - HIGHEST PRIORITY
        # This IS LIDAR data, just processed/aggregated - trust it completely!
        if self.local_map is not None:
            grid_obstacles = self.extract_map_obstacles_from_grid(self.local_map)
            for obs_pos, obs_radius in grid_obstacles:
                # Voxel grid = processed LIDAR - highest confidence
                grid_conf = 0.95  # Very high confidence - processed LIDAR is reliable
                key = (round(obs_pos[0], 2), round(obs_pos[1], 2))
                # Always use grid (it's processed LIDAR, most reliable)
                all_obstacles[key] = (obs_pos, obs_radius, grid_conf, 'grid')
        
        # Source 2: Raw LIDAR - HIGH PRIORITY for immediate obstacles
        # Raw LIDAR catches things voxel grid might miss (very recent)
        if self.latest_scan is not None:
            lidar_obstacles = self.extract_lidar_obstacles()
            for obs_pos, obs_radius in lidar_obstacles:
                lidar_conf = 0.90  # High confidence - raw LIDAR is trusted
                key = (round(obs_pos[0], 2), round(obs_pos[1], 2))
                # Use raw LIDAR if grid doesn't have it (grid is processed, might lag slightly)
                if key not in all_obstacles:
                    all_obstacles[key] = (obs_pos, obs_radius, lidar_conf, 'lidar')
                # If grid has it, keep grid (processed is better)
        
        # Source 3: Scan-matched points (Cartographer processed) - HIGH PRIORITY
        if self.matched_points is not None:
            scan_obstacles = self.extract_scan_matched_obstacles()
            for obs_pos, obs_radius in scan_obstacles:
                scan_conf = 0.88  # High confidence - Cartographer processed
                key = (round(obs_pos[0], 2), round(obs_pos[1], 2))
                # Use scan-matched if grid/LIDAR don't have it
                if key not in all_obstacles:
                    all_obstacles[key] = (obs_pos, obs_radius, scan_conf, 'scan')
        
        # Source 4: Camera - ONLY for cones, fuse with LIDAR for pose refinement
        # Camera detects cones, but LIDAR provides accurate pose - fuse them!
        for cone_pos, camera_conf, timestamp in self.camera_cones:
            if current_time - timestamp < self.camera_cone_timeout:
                dist = np.linalg.norm(cone_pos - robot_pos)
                key = (round(cone_pos[0], 2), round(cone_pos[1], 2))
                
                # Inflate cones more when they are close; camera tends to drop them late
                cone_radius = max(self.obstacle_radius_param, 0.35)
                if dist < 0.6:
                    cone_radius += 0.15  # extra margin when close

                # If LIDAR/grid already detected this cone, fuse camera for better pose
                if key in all_obstacles:
                    existing_pos, existing_radius, existing_conf, existing_source = all_obstacles[key]
                    # Camera refines cone position (camera sees cone, LIDAR sees obstacle)
                    if dist < 1.0:  # Close range: camera pose is more accurate
                        # Fuse: 60% LIDAR (accurate distance) + 40% camera (accurate angle)
                        fused_pos = 0.6 * existing_pos + 0.4 * cone_pos
                        fused_conf = min(0.98, existing_conf + 0.08)  # Boost confidence
                        fused_radius = max(existing_radius, cone_radius)
                        all_obstacles[key] = (fused_pos, fused_radius, fused_conf, f'{existing_source}+camera')
                    else:
                        # Far range: just boost confidence
                        fused_conf = min(0.95, existing_conf + 0.05)
                        fused_radius = max(existing_radius, cone_radius)
                        all_obstacles[key] = (existing_pos, fused_radius, fused_conf, existing_source)
                else:
                    # No LIDAR detection - camera-only (less trusted, but still valid)
                    camera_conf_final = 0.80 if dist < 1.0 else 0.70
                    all_obstacles[key] = (cone_pos, cone_radius, camera_conf_final, 'camera')
        
        # STEP 2: Fuse obstacles - Voxel Grid (processed LIDAR) is trusted
        # CRITICAL: Keep ALL obstacles - everything is an obstacle, we just distinguish cones
        fused_obstacles = {}
        for key, (pos, radius, conf, source) in all_obstacles.items():
            # Trust LIDAR sources (grid, lidar, scan) - they're all LIDAR!
            # Camera-only needs higher threshold (0.65+)
            if conf > 0.65 or 'grid' in source or 'lidar' in source or 'scan' in source:
                # Priority: grid (processed LIDAR) > lidar (raw) > scan (Cartographer) > camera
                if key not in fused_obstacles:
                    fused_obstacles[key] = (pos, radius, conf, source)
                else:
                    # Grid (processed LIDAR) always wins - it's the most reliable
                    existing_source = fused_obstacles[key][3]
                    if 'grid' in source and 'grid' not in existing_source:
                        fused_obstacles[key] = (pos, radius, conf, source)
                    elif 'grid' not in existing_source:  # Don't override grid
                        # Priority: lidar > scan > camera
                        source_priority = {'lidar': 3, 'scan': 2, 'camera': 1}
                        existing_priority = source_priority.get(existing_source.split('+')[0], 0)
                        new_priority = source_priority.get(source.split('+')[0], 0)
                        if new_priority > existing_priority or (new_priority == existing_priority and conf > fused_obstacles[key][2]):
                            fused_obstacles[key] = (pos, radius, conf, source)
        
        # STEP 3: Convert to list - ALL obstacles are important, sort by distance to robot
        obstacles = [(pos, radius) for pos, radius, conf, source in fused_obstacles.values()]
        # Sort by distance to robot (closest first) - prioritize nearby obstacles
        if len(obstacles) > 0:
            obstacles.sort(key=lambda obs: np.linalg.norm(obs[0] - robot_pos))
        
        # Log confidence distribution
        if len(obstacles) > 0 and not hasattr(self, '_confidence_logged'):
            sources = [source for _, _, _, source in fused_obstacles.values()]
            camera_count = sources.count('camera')
            grid_count = sources.count('grid')
            self.get_logger().info(
                f'🎯 Fused obstacles: {len(obstacles)} total '
                f'(Camera: {camera_count}, Grid: {grid_count})'
            )
            self._confidence_logged = True
        
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

                ux = dx / dist
                uy = dy / dist

                # AGGRESSIVE: Add safety inflation to scan-matched obstacles
                safety_inflation = 0.15  # 15cm additional safety margin
                corrected_x = px + ux * (obstacle_radius + safety_inflation)
                corrected_y = py + uy * (obstacle_radius + safety_inflation)
                
                # Only within 2m
                if 0.1 < dist < 2.0:
                    inflated_radius = obstacle_radius + safety_inflation
                    obstacles.append((np.array([corrected_x, corrected_y]), inflated_radius))
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
            # AGGRESSIVE: Add safety inflation to LIDAR obstacles
            safety_inflation = 0.15  # 15cm additional safety margin
            world_angle = robot_theta + ray_angle + self.lidar_angle_offset
            obstacle_x = robot_x + (r + obstacle_radius + safety_inflation) * np.cos(world_angle)
            obstacle_y = robot_y + (r + obstacle_radius + safety_inflation) * np.sin(world_angle)
            
            obstacle_pos = np.array([obstacle_x, obstacle_y])
            inflated_radius = obstacle_radius + safety_inflation
            obstacles.append((obstacle_pos, inflated_radius))
            
            # Add to persistent storage
            self.persistent_obstacles.append((obstacle_pos, obstacle_radius, current_time))
        
        # Clean up old obstacles (older than timeout)
        self.persistent_obstacles = [
            (pos, radius, t) for pos, radius, t in self.persistent_obstacles
            if current_time - t < self.obstacle_timeout
        ]
        
        return obstacles
    
    def extract_map_obstacles_from_grid(self, grid_map):
        """Extract obstacles from occupancy grid with clustering to reduce noise
        ROBUST FRAME HANDLING: Validates grid origin against current robot pose and recomputes if needed.
        This ensures obstacles are always correctly positioned in world/map frame even as robot moves.
        """
        if grid_map is None or self.seeker_state is None:
            return []
        
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        
        # CRITICAL: Verify frame consistency
        if grid_map.header.frame_id != 'map':
            self.get_logger().warn(
                f"Grid frame_id is '{grid_map.header.frame_id}', expected 'map'. "
                f"Obstacles may be in wrong coordinate frame!"
            )
            return []  # Reject if wrong frame
        
        # Extract grid parameters
        width = grid_map.info.width
        height = grid_map.info.height
        resolution = grid_map.info.resolution
        origin_x = grid_map.info.origin.position.x
        origin_y = grid_map.info.origin.position.y
        
        # ROBUST FRAME VALIDATION: Check if grid origin matches expected robot-centric window
        # Expected origin: robot_x - grid_size/2, robot_y - grid_size/2
        # Grid size = width * resolution (assuming square grid)
        grid_size = width * resolution
        expected_origin_x = robot_x - grid_size / 2
        expected_origin_y = robot_y - grid_size / 2
        
        # Check if grid origin is synchronized with current robot pose
        origin_error = np.sqrt((origin_x - expected_origin_x)**2 + (origin_y - expected_origin_y)**2)
        max_origin_error = 0.1  # Allow 10cm error (grid might be slightly stale)
        
        # TIMESTAMP VALIDATION: Check if grid is recent (within 1 second)
        grid_age = (self.get_clock().now().nanoseconds / 1e9) - (
            grid_map.header.stamp.sec + grid_map.header.stamp.nanosec * 1e-9
        )
        max_grid_age = 1.0  # Reject grids older than 1 second
        
        if grid_age > max_grid_age:
            # Grid is too old - reject to avoid using stale data
            if not hasattr(self, '_grid_age_warn_count'):
                self._grid_age_warn_count = 0
            self._grid_age_warn_count += 1
            if self._grid_age_warn_count % 50 == 0:  # Log every 5 seconds
                self.get_logger().warn(
                    f"⚠️ Grid is stale! Age: {grid_age:.2f}s (max: {max_grid_age}s). "
                    f"Rejecting to avoid incorrect obstacle positions."
                )
            return []  # Reject stale grid
        
        if origin_error > max_origin_error:
            # Grid origin is desynchronized - recompute using CURRENT robot pose
            # This ensures obstacles are always in correct world frame
            if not hasattr(self, '_grid_sync_warn_count'):
                self._grid_sync_warn_count = 0
            self._grid_sync_warn_count += 1
            if self._grid_sync_warn_count % 50 == 0:  # Log every 5 seconds
                self.get_logger().warn(
                    f"⚠️ Grid origin desynchronized! Error: {origin_error:.3f}m. "
                    f"Grid origin: ({origin_x:.3f}, {origin_y:.3f}), "
                    f"Expected: ({expected_origin_x:.3f}, {expected_origin_y:.3f}), "
                    f"Robot: ({robot_x:.3f}, {robot_y:.3f}), Grid age: {grid_age:.2f}s. "
                    f"Recomputing using current robot pose..."
                )
            # Use expected origin (current robot pose) instead of stale grid origin
            origin_x = expected_origin_x
            origin_y = expected_origin_y
        
        # CRITICAL: Grid origin is robot-centric (moves with robot), but we convert to world coordinates
        # The grid origin represents the bottom-left corner of the robot-centric window in world frame
        # So: world_x = grid_x * resolution + origin_x (where origin_x = robot_x - grid_size/2)
        # This ensures obstacles are always in world/map frame, correctly positioned
        
        # First pass: find all occupied cells within range
        occupied_cells = []
        for i in range(width * height):
            if grid_map.data[i] > 65:  # Higher threshold to reduce noise
                gx = i % width
                gy = i // width
                # Convert grid coordinates to world coordinates using validated origin
                # Origin is in 'map' frame, so result is in 'map' frame
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
                
                # Normalize direction vector FROM robot TO obstacle center
                robot_x = self.seeker_state[0]
                robot_y = self.seeker_state[1]
                dx = center_x - robot_x
                dy = center_y - robot_y
                dist = np.sqrt(dx*dx + dy*dy)
                
                if dist > 0.01 and dist < 2.0:  # Avoid division by zero, within range
                    ux = dx / dist
                    uy = dy / dist
                    
                    # AGGRESSIVE: Push obstacle center AWAY from robot by radius + safety margin
                    # This inflates obstacles to ensure we never get too close
                    safety_inflation = 0.15  # 15cm additional safety margin
                    corrected_x = center_x + ux * (obstacle_radius + safety_inflation)
                    corrected_y = center_y + uy * (obstacle_radius + safety_inflation)
                    # Use inflated radius for obstacle
                    inflated_radius = obstacle_radius + safety_inflation
                    obstacles.append((np.array([corrected_x, corrected_y]), inflated_radius))
                else:
                    # Too close or too far, use center with inflated radius
                    inflated_radius = obstacle_radius + 0.15  # Safety inflation
                    obstacles.append((np.array([center_x, center_y]), inflated_radius))

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

                ux = dx / dist
                uy = dy / dist

                corrected_x = world_x + ux * obstacle_radius
                corrected_y = world_y + uy * obstacle_radius
                
                # Only within 1.5m of robot (reduced range for performance)
                if dist < 1.5 and dist > 0.02:
                    obstacles.append((np.array([corrected_x, corrected_y]), obstacle_radius))
        
        # Limit to closest 20 obstacles (for performance / MPC max_obstacles=20)
        if len(obstacles) > 20:
            obstacles.sort(key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
            obstacles = obstacles[:20]
        
        # DEBUG
        if not hasattr(self, '_obstacle_count'):
            self._obstacle_count = 0
        self._obstacle_count += 1
        if self._obstacle_count % 20 == 0:
            # Reduced logging - only log occasionally
            if self._obstacle_count % 100 == 0:  # Every 10 seconds
                self.get_logger().info(
                    f"OBSTACLE CELLS: Found {len(obstacles)} occupied cells within 1.5m, "
                    f"robot=({robot_x:.3f}, {robot_y:.3f})"
                )
                if len(obstacles) > 0:
                    closest = min(obstacles, key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
                    dist_closest = np.sqrt((closest[0][0]-robot_x)**2 + (closest[0][1]-robot_y)**2)
                    self.get_logger().info(
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
        
        emergency_dist = 0.15  # 15cm emergency threshold - only trigger for real danger (was 25cm - too aggressive)
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
        """Publish comprehensive MPC trajectory visualization with 15 steps, obstacles, and goal"""
        if self.seeker_state is None:
            return
        
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Get trajectory from MPC's last solution
        trajectory = self.mpc.get_predicted_trajectory()
        x0 = self.seeker_state
        
        # 1. TRAJECTORY LINE (green, thick)
        traj_marker = Marker()
        traj_marker.header.frame_id = 'map'
        traj_marker.header.stamp = now
        traj_marker.ns = 'mpc_trajectory'
        traj_marker.id = 0
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD
        traj_marker.scale.x = 0.08  # Thick line
        traj_marker.color.r = 0.0
        traj_marker.color.g = 1.0
        traj_marker.color.b = 0.0
        traj_marker.color.a = 1.0
        
        if trajectory is not None and len(trajectory) > 0:
            for i, point in enumerate(trajectory):
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.05
                traj_marker.points.append(p)
        else:
            # Fallback: just show current position
            p = Point()
            p.x = float(x0[0])
            p.y = float(x0[1])
            p.z = 0.05
            traj_marker.points.append(p)
        
        marker_array.markers.append(traj_marker)
        
        # 2. TRAJECTORY STEP MARKERS (15 steps, numbered)
        if trajectory is not None and len(trajectory) > 0:
            for i, point in enumerate(trajectory):
                step_marker = Marker()
                step_marker.header.frame_id = 'map'
                step_marker.header.stamp = now
                step_marker.ns = 'mpc_steps'
                step_marker.id = i
                step_marker.type = Marker.SPHERE
                step_marker.action = Marker.ADD
                step_marker.pose.position.x = float(point[0])
                step_marker.pose.position.y = float(point[1])
                step_marker.pose.position.z = 0.1
                step_marker.scale.x = 0.12
                step_marker.scale.y = 0.12
                step_marker.scale.z = 0.12
                # Color gradient: green (start) to yellow (end)
                step_marker.color.r = min(1.0, i / 15.0)
                step_marker.color.g = 1.0
                step_marker.color.b = 0.0
                step_marker.color.a = 0.8
                marker_array.markers.append(step_marker)
        
        # 3. ROBOT CURRENT POSITION (blue sphere)
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = now
        robot_marker.ns = 'robot_current'
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = float(x0[0])
        robot_marker.pose.position.y = float(x0[1])
        robot_marker.pose.position.z = 0.15
        robot_marker.scale.x = 0.20
        robot_marker.scale.y = 0.20
        robot_marker.scale.z = 0.20
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.5
        robot_marker.color.b = 1.0
        robot_marker.color.a = 1.0
        marker_array.markers.append(robot_marker)
        
        # 4. ROBOT ORIENTATION ARROW
        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'map'
        arrow_marker.header.stamp = now
        arrow_marker.ns = 'robot_arrow'
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = float(x0[0])
        arrow_marker.pose.position.y = float(x0[1])
        arrow_marker.pose.position.z = 0.15
        # Orientation
        import math
        qx = 0.0
        qy = 0.0
        qz = math.sin(x0[2] / 2.0)
        qw = math.cos(x0[2] / 2.0)
        arrow_marker.pose.orientation.x = qx
        arrow_marker.pose.orientation.y = qy
        arrow_marker.pose.orientation.z = qz
        arrow_marker.pose.orientation.w = qw
        arrow_marker.scale.x = 0.3  # Length
        arrow_marker.scale.y = 0.08  # Shaft diameter
        arrow_marker.scale.z = 0.08  # Head diameter
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 1.0
        arrow_marker.color.a = 1.0
        marker_array.markers.append(arrow_marker)
        
        # 5. GOAL MARKER (red sphere)
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = now
        goal_marker.ns = 'goal'
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(self.goal_x)
        goal_marker.pose.position.y = float(self.goal_y)
        goal_marker.pose.position.z = 0.15
        goal_marker.scale.x = 0.25
        goal_marker.scale.y = 0.25
        goal_marker.scale.z = 0.25
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        marker_array.markers.append(goal_marker)
        
        # 6. OBSTACLES (yellow/red cylinders with radii)
        obstacles = self.compute_obstacles()
        for i, (center, radius) in enumerate(obstacles):
            obs_marker = Marker()
            obs_marker.header.frame_id = 'map'
            obs_marker.header.stamp = now
            obs_marker.ns = 'mpc_obstacles'
            obs_marker.id = i
            obs_marker.type = Marker.CYLINDER
            obs_marker.action = Marker.ADD
            obs_marker.pose.position.x = float(center[0])
            obs_marker.pose.position.y = float(center[1])
            obs_marker.pose.position.z = 0.1
            obs_marker.scale.x = radius * 2.0  # Diameter
            obs_marker.scale.y = radius * 2.0
            obs_marker.scale.z = 0.2  # Height
            # Color by distance to robot
            dist = np.linalg.norm(center - x0[:2])
            if dist < 0.5:
                obs_marker.color.r = 1.0  # Red (close)
                obs_marker.color.g = 0.0
                obs_marker.color.b = 0.0
            else:
                obs_marker.color.r = 1.0  # Yellow (far)
                obs_marker.color.g = 1.0
                obs_marker.color.b = 0.0
            obs_marker.color.a = 0.6
            marker_array.markers.append(obs_marker)
        
        # 7. TEXT INFO (showing MPC status)
        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = now
        text_marker.ns = 'mpc_info'
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(x0[0])
        text_marker.pose.position.y = float(x0[1])
        text_marker.pose.position.z = 0.5
        text_marker.scale.z = 0.15
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        # Get current MPC command if available
        try:
            twist_cmd = self.mpc.get_twist_command(x0, np.array([[self.goal_x], [self.goal_y]]), obstacles)
            v = twist_cmd['linear']['x']
            omega = twist_cmd['angular']['z']
            dist_to_goal = np.sqrt((self.goal_x - x0[0])**2 + (self.goal_y - x0[1])**2)
            text_marker.text = (
                f"MPC Status\n"
                f"v: {v:.2f} m/s\n"
                f"ω: {np.degrees(omega):.1f} deg/s\n"
                f"Goal dist: {dist_to_goal:.2f} m\n"
                f"Obstacles: {len(obstacles)}\n"
                f"Steps: {len(trajectory) if trajectory is not None else 0}"
            )
        except:
            text_marker.text = "MPC Status\nComputing..."
        
        marker_array.markers.append(text_marker)
        
        # Publish all markers
        self.traj_pub.publish(marker_array.markers[0])  # Keep backward compat
        self.traj_array_pub.publish(marker_array)  # Full visualization
    
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
        
        # Remove duplicates (obstacles within 0.2m of each other) - more aggressive merging
        unique_obstacles = []
        for obs in obstacles:
            # Check if this obstacle is too close to any existing one
            is_duplicate = False
            obs_center = obs[0]
            obs_radius = obs[1]
            
            # Convert to tuple for comparison (numpy arrays can't be compared directly)
            obs_center_tuple = (float(obs_center[0]), float(obs_center[1]))
            
            for i, existing in enumerate(unique_obstacles):
                existing_center = existing[0]
                existing_radius = existing[1]
                dist = np.sqrt((obs_center[0]-existing_center[0])**2 + (obs_center[1]-existing_center[1])**2)
                if dist < 0.20:  # Within 20cm = duplicate (more aggressive)
                    # Keep the one with larger radius (more conservative)
                    if obs_radius > existing_radius:
                        unique_obstacles[i] = obs  # Replace existing with new one
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
        # CRITICAL: Use target pose in map frame (transformed if needed)
        if self.target_pose_map_frame is None:
            if self.target_pose is None:
                return None
            # Fallback: use original pose if transformation not available
            self.get_logger().warn('Using untransformed target pose - may be in wrong frame!')
            target_pose_to_use = self.target_pose
        else:
            target_pose_to_use = self.target_pose_map_frame
        
        px_tgt = target_pose_to_use.position.x
        py_tgt = target_pose_to_use.position.y
        
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
        """SIMPLE PROPORTIONAL CONTROL - GO STRAIGHT TO GOAL"""
        elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9

        # Warmup to let EKF/UKF settle before commanding
        if elapsed < self.startup_warmup:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            if int(elapsed) % 5 == 0:
                remaining = max(self.startup_warmup - elapsed, 0.0)
                self.get_logger().info(f'⏳ Warmup ({remaining:.0f}s left) - holding position')
            return
        
        # NO POSE - can't navigate
        if self.seeker_state is None:
            twist = Twist()
            self.cmd_pub.publish(twist)
            return
        
        # BUILD STATE FOR MPC
        x0 = self.seeker_state.copy()
        
        # COMPUTE OBSTACLES - ONLY NEARBY ONES (within 1.5m) AND NOT BEHIND ROBOT
        all_obstacles = self.compute_obstacles()
        obstacles = []
        robot_pos = x0[:2]
        robot_theta = x0[2]
        forward_dir = np.array([np.cos(robot_theta), np.sin(robot_theta)])
        
        for center, radius in all_obstacles:
            dist = np.linalg.norm(center - robot_pos)
            if dist > 1.5:  # Only obstacles within 1.5m
                continue
            
            # CRITICAL: Ignore obstacles BEHIND the robot (they're not blocking forward progress)
            to_obstacle = center - robot_pos
            if np.linalg.norm(to_obstacle) > 0.1:
                to_obstacle_norm = to_obstacle / np.linalg.norm(to_obstacle)
                forward_alignment = np.dot(to_obstacle_norm, forward_dir)
                if forward_alignment < -0.3:  # Behind robot (more than ~110°)
                    continue  # Skip obstacles behind us
            
            obstacles.append((center, radius))

        # WAYPOINT LOGIC: if blocked ahead, generate a short waypoint to thread gaps
        if self.target_pose is not None:
            goal_vec = np.array([self.target_pose.position.x, self.target_pose.position.y])
        else:
            goal_vec = np.array([self.goal_x, self.goal_y])
        if self.current_waypoint is None:
            wp = self.generate_waypoint_if_blocked(x0, goal_vec, obstacles)
            if wp is not None:
                self.current_waypoint = wp
                self.get_logger().info(
                    f"🎯 Using waypoint ({wp[0]:.2f}, {wp[1]:.2f}) to bypass obstacle"
                )
        else:
            # Clear waypoint if reached
            if np.linalg.norm(self.current_waypoint - robot_pos) < self.waypoint_reached_threshold:
                self.get_logger().info("✅ Waypoint reached, resuming goal")
                self.current_waypoint = None
                self.waypoint_cleared_time = self.get_clock().now()

        # BUILD TARGET SEQUENCE (prefer EKF target, fall back to waypoint/goal)
        target_seq = self.predict_target_trajectory()
        if target_seq is None:
            target_seq = np.zeros((2, self.N + 1))
            if self.current_waypoint is not None:
                target_seq[0, :] = self.current_waypoint[0]
                target_seq[1, :] = self.current_waypoint[1]
            else:
                target_seq[0, :] = self.goal_x
                target_seq[1, :] = self.goal_y

        # Primary target for heading/arrival checks
        tgt_x = float(target_seq[0, 0])
        tgt_y = float(target_seq[1, 0])
        if self.target_pose is not None:
            # use actual target pose for proximity stop
            tgt_x = self.target_pose.position.x
            tgt_y = self.target_pose.position.y

        # Compute proximity for adaptive speed scaling
        min_obs_dist = self.compute_min_obstacle_distance(x0, obstacles)
        self.velocity_scale_factor = self.compute_velocity_scale(min_obs_dist)
        # Softer angular limit everywhere (reduces over-rotation)
        self.soft_omega_limit = max(
            0.5,
            min(self.omega_max * 0.6, 0.7 + 0.3 * max(0.0, min_obs_dist))
        )
        
        # SET MPC VELOCITY LIMITS
        self.mpc.v_max = self.v_max_base
        # Slow down if target covariance is inflated (uncertainty)
        if self.target_cov is not None:
            pos_cov_trace = np.trace(self.target_cov[:2, :2])
            if pos_cov_trace > 0.5:
                scale = max(0.4, min(1.0, 1.0 / (pos_cov_trace)))
                self.mpc.v_max = self.v_max_base * scale
                self.get_logger().debug(f'Target covariance trace={pos_cov_trace:.2f}, scaling v_max -> {self.mpc.v_max:.2f}')

        # COMPUTE GOAL DIRECTION FIRST
        dx_goal = tgt_x - x0[0]
        dy_goal = tgt_y - x0[1]
        dist_to_goal = np.sqrt(dx_goal**2 + dy_goal**2)
        angle_to_goal = np.arctan2(dy_goal, dx_goal)
        angle_err = angle_to_goal - x0[2]
        angle_err = np.mod(angle_err + np.pi, 2*np.pi) - np.pi

        # If straight line is clear and heading is reasonable, drive straight to avoid needless weaving
        omega_limit = self.soft_omega_limit  # soften turning globally
        if self.line_of_sight_clear(x0[:2], np.array([tgt_x, tgt_y]), obstacles, safety_margin=0.2) \
           and abs(angle_err) < np.pi / 2 and min_obs_dist > 0.25:
            twist = Twist()
            twist.linear.x = min(self.mpc.v_max, self.v_max_base) * self.velocity_scale_factor
            twist.angular.z = np.clip(self.Kp_w * angle_err, -omega_limit, omega_limit)
            if self.is_command_safe(twist.linear.x, twist.angular.z):
                self.cmd_pub.publish(twist)
                return

        # If we're facing more than ~120° away from the goal, rotate in place before driving (more permissive).
        if abs(angle_err) > (2*np.pi / 3):
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = np.clip(angle_err, -omega_limit, omega_limit)
            self.cmd_pub.publish(twist)
            return
        
        # REACHED GOAL?
        if dist_to_goal < 0.6:
            twist = Twist()
            self.cmd_pub.publish(twist)
            return
        
        # SOLVE MPC - THIS IS THE PRIMARY CONTROLLER
        try:
            twist_cmd = self.mpc.get_twist_command(x0, target_seq, obstacles)
            v_cmd = twist_cmd['linear']['x']
            omega_cmd = twist_cmd['angular']['z']
            
            # Validate MPC solution
            if math.isnan(v_cmd) or math.isnan(omega_cmd) or \
               math.isinf(v_cmd) or math.isinf(omega_cmd):
                raise ValueError("MPC solution contains NaN or Inf")
            
            # Clip to safe limits
            v_cmd = np.clip(v_cmd, self.v_min, self.mpc.v_max)
            v_cmd = max(0.0, v_cmd)  # avoid backing toward target
            omega_cmd = np.clip(omega_cmd, -self.soft_omega_limit, self.soft_omega_limit)
            # Adaptive slowdown near obstacles (less aggressive now)
            v_cmd *= self.velocity_scale_factor
            
        except Exception as e:
            # Fallback to proportional control only if MPC completely fails
            if not hasattr(self, '_mpc_error_count'):
                self._mpc_error_count = 0
            self._mpc_error_count += 1
            if self._mpc_error_count % 20 == 0:
                self.get_logger().warn(f"MPC solve failed: {e}, using fallback")
            # Fallback to proportional control
            v_cmd, omega_cmd = self.fallback_control(x0, target_seq)

        # PUBLISH MPC COMMAND
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(omega_cmd)
        # Safety check using current scan; if unsafe, trim speed
        if not self.is_command_safe(twist.linear.x, twist.angular.z):
            twist.linear.x *= 0.5
            twist.angular.z *= 0.7
            self.get_logger().warn('MPC command trimmed by safety check')

        self.cmd_pub.publish(twist)
        
        # Record trajectory history
        if self.seeker_state is not None:
            self.trajectory_history.append({
                'x': self.seeker_state[0],
                'y': self.seeker_state[1],
                'theta': self.seeker_state[2]
            })
            if len(self.trajectory_history) > self.max_history_length:
                self.trajectory_history.pop(0)
        
        # Visualize MPC predicted trajectory
        self.visualize_trajectory()
    
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

    def line_of_sight_clear(self, start, goal, obstacles, safety_margin=0.2):
        """Check if straight path from start to goal is free of obstacles (circle models)."""
        if goal is None or start is None:
            return False
        if np.allclose(start, goal):
            return True
        for center, radius in obstacles:
            dist = self.point_to_segment_distance(center, start, goal)
            if dist < (radius + safety_margin):
                return False
        return True

    def point_to_segment_distance(self, point, a, b):
        """Compute distance from point to line segment ab."""
        a = np.array(a)
        b = np.array(b)
        p = np.array(point)
        ab = b - a
        denom = np.dot(ab, ab)
        if denom < 1e-9:
            return np.linalg.norm(p - a)
        t = np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0)
        proj = a + t * ab
        return np.linalg.norm(p - proj)
    
    def compute_velocity_scale(self, min_obs_dist):
        """
        ALGORITHMIC IMPROVEMENT: Adaptive velocity scaling based on obstacle proximity.
        Less aggressive - allow progress even near obstacles.
        
        Returns: scale factor in [0.5, 1.0] to keep speed up but slow more when very close
        """
        if min_obs_dist >= 0.8:
            # Far from obstacles - full speed
            return 1.0
        elif min_obs_dist >= 0.5:
            # 0.8m -> 1.0, 0.5m -> 0.9
            return 0.9 + 0.1 * (min_obs_dist - 0.5) / 0.3
        elif min_obs_dist >= 0.3:
            # 0.5m -> 0.9, 0.3m -> 0.75
            return 0.75 + 0.15 * (min_obs_dist - 0.3) / 0.2
        elif min_obs_dist >= 0.15:
            # 0.3m -> 0.75, 0.15m -> 0.6
            return 0.6 + 0.15 * (min_obs_dist - 0.15) / 0.15
        else:
            # Extremely close
            return 0.5
    
    def verify_full_trajectory_safety(self, x0, v, omega, obstacles, horizon_steps=15):
        """
        ALGORITHMIC IMPROVEMENT: Verify safety of ENTIRE predicted trajectory, not just first step.
        AGGRESSIVE: Uses larger safety margin and longer horizon.
        Simulates robot motion forward and checks for collisions at each step.
        
        Returns: (is_safe, min_clearance_along_path)
        """
        if not obstacles or len(obstacles) == 0:
            return True, float('inf')
        
        # Simulate forward motion
        dt = 0.1  # 100ms steps
        x, y, theta, v_curr = x0[0], x0[1], x0[2], x0[3]
        min_clearance = float('inf')
        robot_radius = 0.105  # Robot radius
        safety_margin = 0.12  # modest safety margin to prevent clipping
        
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
                # Clearance = distance to obstacle surface - robot radius - safety margin
                clearance = dist_to_center - obs_radius - robot_radius - safety_margin
                min_clearance = min(min_clearance, clearance)
                
                # If collision imminent, trajectory is unsafe
                if clearance < 0.0:  # AGGRESSIVE: Any negative clearance = unsafe
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
        
        safety_dist = 0.25  # 25cm safety threshold - AGGRESSIVE to prevent collisions (was 12cm)
        
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
        omega_limit = getattr(self, 'soft_omega_limit', self.omega_max * 0.6)
        omega_cmd = np.clip(omega_cmd, -omega_limit, omega_limit)
        
        return v_cmd, omega_cmd


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
