import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker
import numpy as np
import math
import transforms3d.euler as euler
from scipy.ndimage import shift, rotate
import tf2_ros
from collections import deque


class RobustTargetEstimator(Node):
    def __init__(self):
        super().__init__('robust_target_estimator')
        print("Map-Correlation Target Estimator initialized!")

<<<<<<< HEAD
        self.declare_parameter('obstacle_radius', 0.06)  # Default obstacle radius (meters)
        self.obstacle_radius_param = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        self.obstacle_decay_rate = .02

        # Noise handling parameters for noisy lidar
        self.min_obstacle_observations = 3  # Minimum times obstacle must be seen before trusting it
        self.obstacle_observation_counts = {}  # Track how many times each obstacle has been seen
        self.noise_filter_threshold = 0.15  # Distance threshold for considering points as same obstacle (15cm)
        self.min_cluster_size = 5  # Minimum points per cluster (increased for noisy data)
        self.median_filter_window = 5  # Window size for median filtering of obstacle positions

        self.seeker_obs = []
        self.target_obs = []
        
        # Robot positions (extracted from map origins)
        self.seeker_robot_x = None
        self.seeker_robot_y = None
        self.target_robot_x = None
        self.target_robot_y = None
        
        # Temporal filtering for robust estimates (more aggressive for noisy lidar)
        self.alpha_position = 0.15  # Lower = more smoothing (was 0.3, reduced for noisy data)
        self.alpha_orientation = 0.1  # Lower = more smoothing (was 0.2, reduced for noisy data)
        self.filtered_target_x = None
        self.filtered_target_y = None
        self.filtered_target_yaw = None
        self.last_update_time = None
        
        # Transform history for validation and median filtering
        self.transform_history = []  # Store last N transforms for validation
        self.max_history = 15  # Increased for better median filtering
        self.position_history = []  # Separate history for median filtering
        self.orientation_history = []
        
        # Initialize obstacle observation tracking
        self.seeker_obstacle_counts = {}
        self.target_obstacle_counts = {}
        
        # Confidence tracking
        self.last_confidence = 0.0
        self.min_confidence_to_publish = 0.3  # Minimum confidence before publishing
        self.consecutive_low_confidence = 0  # Track consecutive low confidence estimates

        self.seeker_sub = self.create_subscription(
            OccupancyGrid,
            '/local_map',
            self.seeker_callback,
            10
=======
        # Parameters
        self.declare_parameter('search_range_x', 3.0)  # meters
        self.declare_parameter('search_range_y', 3.0)
        self.declare_parameter('search_step', 0.1)  # 10cm resolution
        self.declare_parameter('angle_range', 180.0)  # degrees
        self.declare_parameter('angle_step', 5.0)  # degrees
        self.declare_parameter('guess_window_xy', 1.5)  # meters around odom guess
        self.declare_parameter('guess_window_yaw', 30.0)  # degrees around odom guess
        self.declare_parameter('max_jump_xy', 1.0)  # meters allowed jump
        self.declare_parameter('max_jump_yaw_deg', 25.0)  # degrees allowed jump
        self.declare_parameter('low_confidence_threshold', 0.25)
        self.declare_parameter('pyramid_levels', 3)
        self.declare_parameter('min_accept_confidence', 0.05)
        self.declare_parameter('min_overlap_accept', 0.02)  # fraction of map area
        
        self.search_range_x = self.get_parameter('search_range_x').get_parameter_value().double_value
        self.search_range_y = self.get_parameter('search_range_y').get_parameter_value().double_value
        self.search_step = self.get_parameter('search_step').get_parameter_value().double_value
        self.angle_range = self.get_parameter('angle_range').get_parameter_value().double_value
        self.angle_step = self.get_parameter('angle_step').get_parameter_value().double_value
        self.guess_window_xy = self.get_parameter('guess_window_xy').get_parameter_value().double_value
        self.guess_window_yaw = self.get_parameter('guess_window_yaw').get_parameter_value().double_value
        self.max_jump_xy = self.get_parameter('max_jump_xy').get_parameter_value().double_value
        self.max_jump_yaw = math.radians(self.get_parameter('max_jump_yaw_deg').get_parameter_value().double_value)
        self.low_conf = self.get_parameter('low_confidence_threshold').get_parameter_value().double_value
        self.pyramid_levels = max(1, int(self.get_parameter('pyramid_levels').get_parameter_value().integer_value))
        self.min_accept_conf = self.get_parameter('min_accept_confidence').get_parameter_value().double_value
        self.min_overlap_accept = self.get_parameter('min_overlap_accept').get_parameter_value().double_value
        self.low_conf_streak = 0
        
        # Data storage
        self.seeker_map = None
        self.target_map = None
        self.seeker_odom = None
        self.target_odom = None
        
        # Pose estimation history
        self.pose_history = deque(maxlen=5)
        self.last_valid_pose = None
        self.last_confidence = 0.0
        
        # Subscribers
        self.seeker_map_sub = self.create_subscription(
            OccupancyGrid, '/local_map', self.seeker_map_callback, 10
>>>>>>> 36bbd91 (Whatever it takes ahh project)
        )
        self.target_map_sub = self.create_subscription(
            OccupancyGrid, '/target/local_map', self.target_map_callback, 10
        )
        
        # Optional: use odometry for better initial guess
        self.seeker_odom_sub = self.create_subscription(
            Odometry, '/odom', self.seeker_odom_callback, 10
        )
        self.target_odom_sub = self.create_subscription(
            Odometry, '/target/odom', self.target_odom_callback, 10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_est', 1)
<<<<<<< HEAD
        self.goal_pub2 = self.create_publisher(PoseWithCovarianceStamped, '/target_est_from_maps', 1)
        self.timer = self.create_timer(1, self.pub_callback)

    def seeker_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.map = msg
        obstacles = self.extract_occupied_points(self.map)
        self.seeker_obs = self.improved_obstacle_clustering(self.seeker_obs, obstacles)
        # Extract seeker robot position from map origin (center of grid)
        grid_size = self.map.info.width * self.map.info.resolution
        self.seeker_robot_x = self.map.info.origin.position.x + grid_size / 2
        self.seeker_robot_y = self.map.info.origin.position.y + grid_size / 2
=======
        self.meas_pub = self.create_publisher(PoseStamped, '/target_pose_measurement', 1)
        self.correlation_pub = self.create_publisher(Marker, '/correlation_score', 10)
>>>>>>> 36bbd91 (Whatever it takes ahh project)
        
        # Timer for estimation
        self.timer = self.create_timer(0.5, self.estimation_callback)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def seeker_map_callback(self, msg: OccupancyGrid):
        self.seeker_map = msg
        
    def target_map_callback(self, msg: OccupancyGrid):
        self.target_map = msg
<<<<<<< HEAD
        obstacles = self.extract_occupied_points(self.target_map)
        self.target_obs = self.improved_obstacle_clustering(self.target_obs, obstacles)
        # Extract target robot position from map origin (center of grid)
        grid_size = self.target_map.info.width * self.target_map.info.resolution
        self.target_robot_x = self.target_map.info.origin.position.x + grid_size / 2
        self.target_robot_y = self.target_map.info.origin.position.y + grid_size / 2

    def extract_occupied_points(self, map_data):
        """
        Extracts occupied points (x, y) from the OccupancyGrid.
        Returns points in map frame (absolute coordinates).
        """
        occupied_points = []
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        for i in range(width * height):
            if map_data.data[i] > 30:  # Occupied cell
                gx = i % width
                gy = i // width
                # Convert grid coordinates to world coordinates (map frame)
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                occupied_points.append([world_x, world_y])
        
        return occupied_points

    def improved_obstacle_clustering(self, curr_obs, occupied_points, eps=0.15, min_samples=3):
        """
        ROBUST obstacle clustering for noisy lidar data.
        Uses DBSCAN-like algorithm with noise filtering and temporal tracking.
        
        Args:
            occupied_points: List of (x, y) coordinates
            eps: Maximum distance between points in same cluster (15cm)
            min_samples: Minimum points to form a cluster (increased for noisy data)
            constant_radius: The fixed radius for obstacles
            
        Returns: List of (center, radius) tuples
        """
        if len(occupied_points) == 0:
            return []

        points = np.array(occupied_points)
        n_points = len(points)

        constant_radius = self.obstacle_radius_param
        
        # Use increased min_samples for noisy lidar
        effective_min_samples = max(min_samples, self.min_cluster_size)

        # Initialize labels (-1 means unassigned)
        labels = -np.ones(n_points, dtype=int)
        cluster_id = 0

        # DBSCAN-like clustering with tighter constraints for noisy data
        for i in range(n_points):
            if labels[i] != -1:
                continue  # Already assigned
            
            # Find all points within eps distance
            dists = np.linalg.norm(points - points[i], axis=1)
            neighbors = np.where(dists <= eps)[0]

            if len(neighbors) < effective_min_samples:
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
                
                if len(new_neighbors) >= effective_min_samples:
                    seed_set.extend(new_neighbors)
            
            cluster_id += 1

        # Convert clusters to obstacles with noise filtering
        obstacles = []
        for cid in range(cluster_id):
            cluster_points = points[labels == cid]
            if len(cluster_points) < effective_min_samples:
                continue  # Ignore small clusters (noise)
            
            # Use median instead of mean for more robust center estimation
            center_x = np.median(cluster_points[:, 0])
            center_y = np.median(cluster_points[:, 1])
            center = np.array([center_x, center_y])
            
            # Compute robust radius estimate (use median distance from center)
            distances_from_center = np.linalg.norm(cluster_points - center, axis=1)
            robust_radius = np.median(distances_from_center) + constant_radius
            
            obstacles.append((center, robust_radius))
        
        # Sort by distance and take top obstacles
        obstacles.sort(key=lambda obs: np.linalg.norm(obs[0]))  # Sort by distance from (0,0)
        new_obs = obstacles[:3]  # Take top 3 obstacles
        
        # Temporal filtering: only keep obstacles that appear consistently
        if len(curr_obs) == 0:
            # First time: initialize observation counts
            for obs_center, obs_radius in new_obs:
                obs_key = tuple(np.round(obs_center, 2))  # Round for matching
                self.obstacle_observation_counts[obs_key] = 1
            return new_obs
        else:
            # Match new obstacles with current obstacles
            updated_obs = []
            matched_new_indices = set()
            
            for curr_center, curr_radius in curr_obs:
                best_match_idx = None
                best_distance = float('inf')
                
                for i, (new_center, new_radius) in enumerate(new_obs):
                    if i in matched_new_indices:
                        continue
                    
                    distance = np.linalg.norm(np.array(curr_center) - np.array(new_center))
                    if distance < self.noise_filter_threshold and distance < best_distance:
                        best_distance = distance
                        best_match_idx = i
                
                if best_match_idx is not None:
                    # Match found: update with heavy temporal filtering
                    new_center, new_radius = new_obs[best_match_idx]
                    updated_center = (1 - self.obstacle_decay_rate * 0.5) * np.array(curr_center) + (self.obstacle_decay_rate * 0.5) * np.array(new_center)
                    updated_radius = (1 - self.obstacle_decay_rate * 0.5) * curr_radius + (self.obstacle_decay_rate * 0.5) * new_radius
                    
                    # Increment observation count
                    obs_key = tuple(np.round(updated_center, 2))
                    self.obstacle_observation_counts[obs_key] = self.obstacle_observation_counts.get(obs_key, 0) + 1
                    
                    updated_obs.append((updated_center, updated_radius))
                    matched_new_indices.add(best_match_idx)
                else:
                    # No match: check if obstacle has been seen enough times
                    obs_key = tuple(np.round(curr_center, 2))
                    count = self.obstacle_observation_counts.get(obs_key, 0)
                    if count >= self.min_obstacle_observations:
                        # Keep it but decay it
                        updated_obs.append((curr_center, curr_radius))
            
            # Add new obstacles that weren't matched (but only if we have room)
            for i, (new_center, new_radius) in enumerate(new_obs):
                if i not in matched_new_indices and len(updated_obs) < 3:
                    obs_key = tuple(np.round(new_center, 2))
                    self.obstacle_observation_counts[obs_key] = 1
                    updated_obs.append((new_center, new_radius))
            
            return updated_obs

    def pub_callback(self):
        # Check if we have robot positions (preferred method)
        has_robot_positions = (hasattr(self, 'seeker_robot_x') and 
                              self.seeker_robot_x is not None and
                              hasattr(self, 'target_robot_x') and 
                              self.target_robot_x is not None)
        
        # If we don't have robot positions, we need obstacles for transform estimation
        if not has_robot_positions:
            if len(self.seeker_obs) == 0 or len(self.target_obs) == 0:
                self.get_logger().warn("No obstacles seen and no robot positions available")
            return
=======
    
    def seeker_odom_callback(self, msg: Odometry):
        self.seeker_odom = msg
    
    def target_odom_callback(self, msg: Odometry):
        self.target_odom = msg

    def estimation_callback(self):
        """Main estimation using map correlation"""
        if self.seeker_map is None or self.target_map is None:
            self.get_logger().warning("Waiting for both maps...")
            return
        
        # Convert maps to numpy arrays
        seeker_grid = self.occupancy_grid_to_numpy(self.seeker_map)
        target_grid = self.occupancy_grid_to_numpy(self.target_map)
        
        # Get map info
        resolution = self.seeker_map.info.resolution
        
        # Estimate pose using 2D correlation
        best_pose, confidence = self.correlate_maps(seeker_grid, target_grid, resolution)
        
        if best_pose is None:
            self.get_logger().error("Correlation failed!")
            self.low_conf_streak += 1
            return
        
        if confidence < self.min_accept_conf:
            self.low_conf_streak += 1
        else:
            self.low_conf_streak = 0

        # If confidence is bad for several cycles, temporarily widen search windows next time
        if self.low_conf_streak > 3:
            self.guess_window_xy = min(self.search_range_x, self.guess_window_xy * 1.5)
            self.guess_window_yaw = min(self.angle_range, self.guess_window_yaw * 1.5)
        else:
            # Slowly shrink back toward defaults
            self.guess_window_xy = max(self.guess_window_xy * 0.95, 0.5)
            self.guess_window_yaw = max(self.guess_window_yaw * 0.95, 10.0)
            return
        
        # Apply temporal filtering
        filtered_pose = self.temporal_filter(best_pose)
>>>>>>> 36bbd91 (Whatever it takes ahh project)
        
        # Publish result
        self.publish_pose(filtered_pose, confidence)
        
        self.get_logger().info(f"Target pose: ({filtered_pose[0]:.2f}, {filtered_pose[1]:.2f}, {math.degrees(filtered_pose[2]):.1f}°) conf={confidence:.3f}")

    def occupancy_grid_to_numpy(self, grid_msg):
        """Convert OccupancyGrid to numpy array"""
        width = grid_msg.info.width
        height = grid_msg.info.height
        
        # Reshape to 2D array
        grid = np.array(grid_msg.data).reshape((height, width))
        
        # Binarize: occupied (>30) = 1, free/unknown = 0 (more sensitive)
        binary_grid = (grid > 30).astype(np.float32)
        
        return binary_grid

    def correlate_maps(self, seeker_grid, target_grid, resolution):
        """
        Pyramid search: coarse→fine correlation to align target map to seeker map.
        """
        # Initial window
        guess = self.get_initial_guess()
        if guess is not None:
            cx, cy, cyaw = guess
            win_xy = self.guess_window_xy
            win_yaw = self.guess_window_yaw
        else:
            cx, cy, cyaw = 0.0, 0.0, 0.0
            win_xy = max(self.search_range_x, self.search_range_y)
            win_yaw = self.angle_range

<<<<<<< HEAD
    def match_obstacles(self, seeker_obs, target_obs):
        """
        Robust obstacle matching with geometric validation.
        Uses nearest neighbor with distance and angle constraints.
        Returns matched pairs: (seeker_obs_idx, target_obs_idx, distance)
        """
        if len(seeker_obs) == 0 or len(target_obs) == 0:
            return []
        
        seeker_positions = np.array([obs[0] for obs in seeker_obs])
        target_positions = np.array([obs[0] for obs in target_obs])
        
        # Compute relative positions from robot centers if available
        seeker_center = np.array([self.seeker_robot_x, self.seeker_robot_y]) if (
            self.seeker_robot_x is not None and self.seeker_robot_y is not None
        ) else np.array([0.0, 0.0])
        target_center = np.array([self.target_robot_x, self.target_robot_y]) if (
            self.target_robot_x is not None and self.target_robot_y is not None
        ) else np.array([0.0, 0.0])
        
        # Compute relative vectors (from robot center to obstacle)
        seeker_vectors = seeker_positions - seeker_center
        target_vectors = target_positions - target_center
        
        # Normalize vectors for angle comparison
        seeker_norms = np.linalg.norm(seeker_vectors, axis=1, keepdims=True)
        target_norms = np.linalg.norm(target_vectors, axis=1, keepdims=True)
        seeker_normalized = seeker_vectors / (seeker_norms + 1e-6)
        target_normalized = target_vectors / (target_norms + 1e-6)
        
        matched_pairs = []
        used_target_indices = set()
        max_match_distance = 2.0  # Increased to 2m for noisy lidar (was 1.5m)
        max_angle_diff = 0.7  # Increased to ~40 degrees for noisy lidar (was 0.5, ~30 degrees)
        max_relative_distance_diff = 0.5  # Increased to 50cm for noisy lidar (was 30cm)
        
        for i, seeker_pos in enumerate(seeker_positions):
            best_target_idx = None
            best_score = float('inf')
            
            for j, target_pos in enumerate(target_positions):
                if j in used_target_indices:
                    continue
                
                # Distance constraint
                distance = np.linalg.norm(seeker_pos - target_pos)
                if distance > max_match_distance:
                    continue
                
                # Angle constraint (if we have robot positions)
                if (self.seeker_robot_x is not None and self.target_robot_x is not None):
                    seeker_vec = seeker_normalized[i]
                    target_vec = target_normalized[j]
                    angle_diff = np.arccos(np.clip(np.dot(seeker_vec, target_vec), -1.0, 1.0))
                    if angle_diff > max_angle_diff:
                        continue
                    
                    # Relative distance constraint
                    seeker_dist = seeker_norms[i, 0]
                    target_dist = target_norms[j, 0]
                    dist_diff = abs(seeker_dist - target_dist)
                    if dist_diff > max_relative_distance_diff:
                        continue
                
                # Score combines distance and angle (lower is better)
                score = distance
                if self.seeker_robot_x is not None:
                    score += 0.5 * angle_diff  # Weight angle difference
                
                if score < best_score:
                    best_score = score
                    best_target_idx = j
            
            # Only match if score is reasonable
            if best_target_idx is not None and best_score < max_match_distance:
                matched_pairs.append((i, best_target_idx, best_score))
                used_target_indices.add(best_target_idx)
        
        # Sort by score (best matches first)
        matched_pairs.sort(key=lambda x: x[2])
        
        return matched_pairs

    def compute_transform_robust(self, A, B, max_outlier_ratio=0.6):
        """
        Compute transform using RANSAC-like approach for robustness.
        Enhanced for noisy lidar data with more aggressive outlier rejection.
        Returns: (R, t, inliers, confidence)
        """
        if len(A) < 2 or len(B) < 2:
            return None, None, [], 0.0
        
        n_points = len(A)
        min_inliers = max(2, int(n_points * (1 - max_outlier_ratio)))
        best_R = None
        best_t = None
        best_inliers = []
        best_error = float('inf')
        inlier_threshold = 0.25  # Increased to 25cm for noisy lidar (was 20cm)
        
        # Try more trials for noisy data
        n_trials = min(50, n_points * (n_points - 1) // 2) if n_points <= 5 else 50
        
        for trial in range(n_trials):
            # Sample 2 points
            if n_points <= 2:
                sample_indices = [0, 1]
            else:
                sample_indices = np.random.choice(n_points, size=2, replace=False)
            
            A_sample = A[sample_indices]
            B_sample = B[sample_indices]
            
            # Compute transform from 2 points
            # For 2 points: compute translation and rotation
            B_center = B_sample.mean(axis=0)
            A_center = A_sample.mean(axis=0)
            
            B_vec = B_sample[1] - B_sample[0]
            A_vec = A_sample[1] - A_sample[0]
            
            # Compute rotation angle
            angle_B = math.atan2(B_vec[1], B_vec[0])
            angle_A = math.atan2(A_vec[1], A_vec[0])
            angle_diff = angle_A - angle_B
            
            # Build rotation matrix
            R_trial = np.array([
                [math.cos(angle_diff), -math.sin(angle_diff)],
                [math.sin(angle_diff), math.cos(angle_diff)]
            ])
            
            # Compute translation
            t_trial = A_center - R_trial @ B_center
            
            # Count inliers
            transformed_B = (R_trial @ B.T).T + t_trial
            errors = np.linalg.norm(A - transformed_B, axis=1)
            inliers = np.where(errors < inlier_threshold)[0]
            
            if len(inliers) >= min_inliers:
                # Refine using all inliers
                A_inliers = A[inliers]
                B_inliers = B[inliers]
                
                A_mean = A_inliers.mean(axis=0)
                B_mean = B_inliers.mean(axis=0)
                A_center = A_inliers - A_mean
                B_center = B_inliers - B_mean
                
                H = A_center.T @ B_center
                U, S, Vt = np.linalg.svd(H)
                R_refined = Vt.T @ U.T
                
                if np.linalg.det(R_refined) < 0:
                    Vt[-1, :] *= -1
                    R_refined = Vt.T @ U.T
                
                t_refined = A_mean - R_refined @ B_mean
                
                # Compute error
                transformed_B_refined = (R_refined @ B_inliers.T).T + t_refined
                error = np.mean(np.linalg.norm(A_inliers - transformed_B_refined, axis=1))
                
                if error < best_error:
                    best_R = R_refined
                    best_t = t_refined
                    best_inliers = inliers.tolist()
                    best_error = error
        
        # If no good solution found, use all points
        if best_R is None:
            A_mean = A.mean(axis=0)
            B_mean = B.mean(axis=0)
            A_center = A - A_mean
            B_center = B - B_mean
            H = A_center.T @ B_center
            U, S, Vt = np.linalg.svd(H)
            best_R = Vt.T @ U.T
            if np.linalg.det(best_R) < 0:
                Vt[-1, :] *= -1
                best_R = Vt.T @ U.T
            best_t = A_mean - best_R @ B_mean
            best_inliers = list(range(n_points))
            best_error = np.mean(np.linalg.norm(A - ((best_R @ B.T).T + best_t), axis=1))
        
        # Compute confidence based on inlier ratio and error (more conservative for noisy data)
        inlier_ratio = len(best_inliers) / n_points
        error_penalty = min(best_error / inlier_threshold, 1.0)
        confidence = inlier_ratio * (1.0 - error_penalty)
        
        # Penalize low inlier counts more aggressively
        if len(best_inliers) < 3:
            confidence *= 0.5  # Halve confidence if less than 3 inliers
        
        # Require minimum confidence
        if confidence < 0.2:
            confidence = 0.0
        
        return best_R, best_t, best_inliers, confidence

    def validate_transform(self, target_x, target_y, target_yaw):
        """
        Validate transform reasonableness based on history and constraints.
        Returns: (is_valid, reason)
        """
        # Check for NaN or Inf
        if not (np.isfinite(target_x) and np.isfinite(target_y) and np.isfinite(target_yaw)):
            return False, "Non-finite values"
        
        # Check reasonable bounds (adjust based on your environment)
        max_distance = 10.0  # Maximum expected distance between robots (meters)
        if abs(target_x) > max_distance or abs(target_y) > max_distance:
            return False, f"Distance too large: ({target_x:.2f}, {target_y:.2f})"
        
        # Check history for sudden jumps
        if len(self.transform_history) > 0:
            last_transform = self.transform_history[-1]
            dx = target_x - last_transform['x']
            dy = target_y - last_transform['y']
            jump_distance = math.sqrt(dx**2 + dy**2)
            max_jump = 1.0  # Maximum expected jump per update (meters)
            
            if jump_distance > max_jump:
                return False, f"Sudden jump: {jump_distance:.2f}m"
        
        return True, "OK"

    def publish_pose_estimate(self):
        """
        Compute transform from seeker to target using matched obstacles.
        Uses robust RANSAC-like approach with temporal filtering.
        """
        # Method 1: Use direct robot positions (most reliable)
        if (self.seeker_robot_x is not None and self.seeker_robot_y is not None and
            self.target_robot_x is not None and self.target_robot_y is not None):
            
            target_pos_x = self.target_robot_x
            target_pos_y = self.target_robot_y
            
            # Compute relative position and orientation
            relative_x = self.target_robot_x - self.seeker_robot_x
            relative_y = self.target_robot_y - self.seeker_robot_y
            est_yaw_rad = math.atan2(relative_y, relative_x)
            
            # Cross-validate with obstacle matching if available
            confidence = 0.9  # High confidence for direct positions
            if len(self.seeker_obs) > 0 and len(self.target_obs) > 0:
                matched_pairs = self.match_obstacles(self.seeker_obs, self.target_obs)
                if len(matched_pairs) >= 2:
                    # Use obstacle matching to validate
                    A = np.array([self.target_obs[j][0] for _, j, _ in matched_pairs])
                    B = np.array([self.seeker_obs[i][0] for i, _, _ in matched_pairs])
                    R_obs, t_obs, inliers, obs_confidence = self.compute_transform_robust(A, B)
                    
                    if R_obs is not None:
                        # Compare obstacle-based transform with direct positions
                        t_from_obs = t_obs
                        t_from_direct = np.array([relative_x, relative_y])
                        diff = np.linalg.norm(t_from_obs - t_from_direct)
                        
                        if diff < 0.5:  # Within 50cm agreement
                            confidence = 0.95
                        else:
                            confidence = 0.7  # Lower confidence if disagreement
                            self.get_logger().warn(
                                f"⚠️ Transform disagreement: direct=({relative_x:.2f}, {relative_y:.2f}), "
                                f"obstacle=({t_from_obs[0]:.2f}, {t_from_obs[1]:.2f}), diff={diff:.2f}m"
                            )
            
            # Apply temporal filtering with median filtering for noisy data
            self.position_history.append((target_pos_x, target_pos_y))
            self.orientation_history.append(est_yaw_rad)
            
            # Keep only recent history
            if len(self.position_history) > self.median_filter_window:
                self.position_history.pop(0)
            if len(self.orientation_history) > self.median_filter_window:
                self.orientation_history.pop(0)
            
            # Use median filtering if we have enough history
            if len(self.position_history) >= 3:
                # Median filter for position (more robust to outliers)
                positions_array = np.array(self.position_history)
                median_x = np.median(positions_array[:, 0])
                median_y = np.median(positions_array[:, 1])
                
                # Combine median with exponential smoothing
                if self.filtered_target_x is None:
                    self.filtered_target_x = median_x
                    self.filtered_target_y = median_y
                else:
                    # Use median as target, but smooth towards it
                    self.filtered_target_x = (1 - self.alpha_position) * self.filtered_target_x + self.alpha_position * median_x
                    self.filtered_target_y = (1 - self.alpha_position) * self.filtered_target_y + self.alpha_position * median_y
            else:
                # Not enough history yet, use regular smoothing
                if self.filtered_target_x is None:
                    self.filtered_target_x = target_pos_x
                    self.filtered_target_y = target_pos_y
                else:
                    self.filtered_target_x = (1 - self.alpha_position) * self.filtered_target_x + self.alpha_position * target_pos_x
                    self.filtered_target_y = (1 - self.alpha_position) * self.filtered_target_y + self.alpha_position * target_pos_y
            
            # Median filter for orientation
            if len(self.orientation_history) >= 3:
                # Handle angle wrapping for median
                angles = np.array(self.orientation_history)
                # Convert to unit vectors for proper median
                unit_vectors = np.column_stack([np.cos(angles), np.sin(angles)])
                median_vector = np.median(unit_vectors, axis=0)
                median_angle = math.atan2(median_vector[1], median_vector[0])
                
                if self.filtered_target_yaw is None:
                    self.filtered_target_yaw = median_angle
                else:
                    angle_diff = median_angle - self.filtered_target_yaw
                    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    self.filtered_target_yaw = self.filtered_target_yaw + self.alpha_orientation * angle_diff
                    self.filtered_target_yaw = math.atan2(math.sin(self.filtered_target_yaw), math.cos(self.filtered_target_yaw))
            else:
                if self.filtered_target_yaw is None:
                    self.filtered_target_yaw = est_yaw_rad
                else:
                    angle_diff = est_yaw_rad - self.filtered_target_yaw
                    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    self.filtered_target_yaw = self.filtered_target_yaw + self.alpha_orientation * angle_diff
                    self.filtered_target_yaw = math.atan2(math.sin(self.filtered_target_yaw), math.cos(self.filtered_target_yaw))
            
            # Validate
            is_valid, reason = self.validate_transform(self.filtered_target_x, self.filtered_target_y, self.filtered_target_yaw)
            if not is_valid:
                self.get_logger().warn(f"⚠️ Transform validation failed: {reason}, using last valid")
                if len(self.transform_history) > 0:
                    last = self.transform_history[-1]
                    self.filtered_target_x = last['x']
                    self.filtered_target_y = last['y']
                    self.filtered_target_yaw = last['yaw']
                    confidence = last['confidence'] * 0.8  # Reduce confidence
                else:
                    return  # Can't proceed without valid transform
            
            final_x = self.filtered_target_x
            final_y = self.filtered_target_y
            final_yaw = self.filtered_target_yaw
            
        else:
            # Method 2: Fallback to obstacle-based transform
            if len(self.seeker_obs) == 0 or len(self.target_obs) == 0:
                self.get_logger().warn("No obstacles available and no robot positions")
                return
            
            matched_pairs = self.match_obstacles(self.seeker_obs, self.target_obs)
            
            if len(matched_pairs) < 2:
                self.get_logger().warn(f"Not enough matched obstacles ({len(matched_pairs)}), need at least 2")
                return
            
            # Extract matched obstacle positions
            A = np.array([self.target_obs[j][0] for _, j, _ in matched_pairs])
            B = np.array([self.seeker_obs[i][0] for i, _, _ in matched_pairs])
            
            # Compute robust transform
            R, t, inliers, confidence = self.compute_transform_robust(A, B)
            
            if R is None:
                self.get_logger().error("Failed to compute transform")
                return
            
            # Extract yaw
            est_yaw_rad = math.atan2(R[1, 0], R[0, 0])
            
            # Use translation as relative position
            target_pos_x = t[0]
            target_pos_y = t[1]
            
            # Apply temporal filtering with median filtering (same as above)
            self.position_history.append((target_pos_x, target_pos_y))
            self.orientation_history.append(est_yaw_rad)
            
            if len(self.position_history) > self.median_filter_window:
                self.position_history.pop(0)
            if len(self.orientation_history) > self.median_filter_window:
                self.orientation_history.pop(0)
            
            if len(self.position_history) >= 3:
                positions_array = np.array(self.position_history)
                median_x = np.median(positions_array[:, 0])
                median_y = np.median(positions_array[:, 1])
                
                if self.filtered_target_x is None:
                    self.filtered_target_x = median_x
                    self.filtered_target_y = median_y
                else:
                    self.filtered_target_x = (1 - self.alpha_position) * self.filtered_target_x + self.alpha_position * median_x
                    self.filtered_target_y = (1 - self.alpha_position) * self.filtered_target_y + self.alpha_position * median_y
            else:
                if self.filtered_target_x is None:
                    self.filtered_target_x = target_pos_x
                    self.filtered_target_y = target_pos_y
                else:
                    self.filtered_target_x = (1 - self.alpha_position) * self.filtered_target_x + self.alpha_position * target_pos_x
                    self.filtered_target_y = (1 - self.alpha_position) * self.filtered_target_y + self.alpha_position * target_pos_y
            
            if len(self.orientation_history) >= 3:
                angles = np.array(self.orientation_history)
                unit_vectors = np.column_stack([np.cos(angles), np.sin(angles)])
                median_vector = np.median(unit_vectors, axis=0)
                median_angle = math.atan2(median_vector[1], median_vector[0])
                
                if self.filtered_target_yaw is None:
                    self.filtered_target_yaw = median_angle
                else:
                    angle_diff = median_angle - self.filtered_target_yaw
                    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    self.filtered_target_yaw = self.filtered_target_yaw + self.alpha_orientation * angle_diff
                    self.filtered_target_yaw = math.atan2(math.sin(self.filtered_target_yaw), math.cos(self.filtered_target_yaw))
            else:
                if self.filtered_target_yaw is None:
                    self.filtered_target_yaw = est_yaw_rad
                else:
                    angle_diff = est_yaw_rad - self.filtered_target_yaw
                    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    self.filtered_target_yaw = self.filtered_target_yaw + self.alpha_orientation * angle_diff
                    self.filtered_target_yaw = math.atan2(math.sin(self.filtered_target_yaw), math.cos(self.filtered_target_yaw))
            
            # Validate
            is_valid, reason = self.validate_transform(self.filtered_target_x, self.filtered_target_y, self.filtered_target_yaw)
            if not is_valid:
                self.get_logger().warn(f"⚠️ Transform validation failed: {reason}")
                if len(self.transform_history) > 0:
                    last = self.transform_history[-1]
                    self.filtered_target_x = last['x']
                    self.filtered_target_y = last['y']
                    self.filtered_target_yaw = last['yaw']
                    confidence = last['confidence'] * 0.8
                else:
                    return
            
            final_x = self.filtered_target_x
            final_y = self.filtered_target_y
            final_yaw = self.filtered_target_yaw
        
        # Check confidence threshold before publishing
        if confidence < self.min_confidence_to_publish:
            self.consecutive_low_confidence += 1
            if self.consecutive_low_confidence > 3:
                self.get_logger().warn(
                    f"⚠️ Low confidence ({confidence:.2f}) for {self.consecutive_low_confidence} updates, "
                    f"not publishing transform"
                )
                return  # Don't publish low-confidence estimates
        else:
            self.consecutive_low_confidence = 0
        
        # Store transform in history
        transform_entry = {
            'x': final_x,
            'y': final_y,
            'yaw': final_yaw,
            'confidence': confidence,
            'time': self.get_clock().now()
        }
        self.transform_history.append(transform_entry)
        if len(self.transform_history) > self.max_history:
            self.transform_history.pop(0)
        
        self.last_confidence = confidence
        est_yaw_deg = math.degrees(final_yaw)
        
        # Prepare the pose message - target's estimated pose in map frame
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = final_x
        pose_msg.pose.pose.position.y = final_y
        pose_msg.pose.pose.position.z = 0.0

        # Set quaternion orientation (yaw only for 2D)
        orientation = euler.euler2quat(0, 0, final_yaw)
        pose_msg.pose.pose.orientation.x = orientation[1]  # transforms3d uses [w, x, y, z]
        pose_msg.pose.pose.orientation.y = orientation[2]
        pose_msg.pose.pose.orientation.z = orientation[3]
        pose_msg.pose.pose.orientation.w = orientation[0]

        # Adaptive covariance based on confidence (more conservative for noisy data)
        base_pos_cov = 0.15  # Increased base covariance for noisy lidar
        base_rot_cov = 0.3   # Increased base covariance for noisy lidar
        # Lower confidence = higher covariance (more aggressive scaling)
        pos_cov = base_pos_cov / max(confidence, 0.05)  # Cap at 0.05 to prevent extreme values
        rot_cov = base_rot_cov / max(confidence, 0.05)
        
        # Cap maximum covariance to prevent controller instability
        pos_cov = min(pos_cov, 1.0)
        rot_cov = min(rot_cov, 2.0)
        
        pose_msg.pose.covariance = [
            pos_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, rot_cov
        ]

        # Store for TF publishing
        self.last_target_pose = pose_msg
        
        # Store transform components for reference
        # Compute relative transform from seeker to target
        if self.seeker_robot_x is not None:
            relative_x = final_x - self.seeker_robot_x
            relative_y = final_y - self.seeker_robot_y
            self.seeker_to_target_t = np.array([relative_x, relative_y])
            # Rotation matrix from yaw
            self.seeker_to_target_R = np.array([
                [math.cos(final_yaw), -math.sin(final_yaw)],
                [math.sin(final_yaw), math.cos(final_yaw)]
            ])
        else:
            self.seeker_to_target_R = np.eye(2)
            self.seeker_to_target_t = np.array([final_x, final_y])

        # Log and publish the estimated pose
        est_yaw_deg = math.degrees(final_yaw)
        self.get_logger().info(
            f"✅ Target pose: ({final_x:.3f}, {final_y:.3f}), yaw={est_yaw_deg:.1f}°, "
            f"confidence={confidence:.2f}"
        )
=======
        best_pose = None
        best_score = -np.inf
        best_overlap = 0.0

        # Pyramid levels: coarse to fine
        factor = 1
        for level in range(self.pyramid_levels):
            # Downsample
            if level > 0:
                seeker_lvl = seeker_grid[::2, ::2]
                target_lvl = target_grid[::2, ::2]
                res_lvl = resolution * 2
                factor *= 2
            else:
                seeker_lvl = seeker_grid
                target_lvl = target_grid
                res_lvl = resolution

            step_xy = max(self.search_step / factor, 0.05)
            step_yaw = max(self.angle_step / factor, 1.0)
            win_xy_lvl = max(win_xy / factor, step_xy * 4)
            win_yaw_lvl = max(win_yaw / factor, step_yaw * 4)

            dx_range = np.arange(cx - win_xy_lvl, cx + win_xy_lvl + step_xy, step_xy)
            dy_range = np.arange(cy - win_xy_lvl, cy + win_xy_lvl + step_xy, step_xy)
            dtheta_range = np.arange(cyaw - win_yaw_lvl, cyaw + win_yaw_lvl + step_yaw, step_yaw)

            # Coarse angle ranking
            angle_scores = []
            for dtheta in dtheta_range:
                rotated_target = rotate(target_lvl, dtheta, reshape=False, order=1, mode='constant', cval=0)
                score, overlap = self.compute_overlap_score(seeker_lvl, rotated_target, 0, 0)
                angle_scores.append((dtheta, score, overlap))
            angle_scores.sort(key=lambda x: x[1], reverse=True)
            top_angles = [a[0] for a in angle_scores[:5]]

            for dtheta in top_angles:
                rotated_target = rotate(target_lvl, dtheta, reshape=False, order=1, mode='constant', cval=0)
                for dx_meters in dx_range:
                    for dy_meters in dy_range:
                        dx_pixels = int(dx_meters / res_lvl)
                        dy_pixels = int(dy_meters / res_lvl)
                        score, overlap = self.compute_overlap_score(seeker_lvl, rotated_target, dx_pixels, dy_pixels)
                        if score > best_score:
                            best_score = score
                            best_overlap = overlap
                            best_pose = (dx_meters, dy_meters, math.radians(dtheta))

            # center next level on current best
            if best_pose is not None:
                cx, cy, cyaw = best_pose
                cyaw = math.degrees(cyaw)

        if best_pose is None:
            return None, 0.0

        bx, by, byaw = best_pose
        confidence = min(best_score * max(best_overlap, 0.1), 1.0)
        byaw = self.wrap_angle(byaw)

        # Consistency gate: reject wild jumps unless score is much better
        if self.last_valid_pose is not None:
            lx, ly, lyaw = self.last_valid_pose
            jump = math.hypot(bx - lx, by - ly)
            yaw_jump = abs((byaw - lyaw + math.pi) % (2 * math.pi) - math.pi)
        if (jump > self.max_jump_xy or yaw_jump > self.max_jump_yaw) and confidence < (self.last_confidence * 1.2):
            self.get_logger().warn(f"Rejecting jump (Δ={jump:.2f}m, yaw={math.degrees(yaw_jump):.1f}°) with low conf {confidence:.2f}")
            return self.last_valid_pose, self.last_confidence

        # Reject very low confidence or tiny overlap outright if we have a previous good pose
        if (confidence < self.min_accept_conf or best_overlap < self.min_overlap_accept) and self.last_valid_pose is not None:
            self.get_logger().warn(f"Low confidence {confidence:.3f}, holding last pose")
            return self.last_valid_pose, self.last_confidence

        self.get_logger().info(f"Best score {best_score:.3f}, overlap {best_overlap:.2f}, conf {confidence:.3f}")
        return (bx, by, byaw), confidence

    def compute_overlap_score(self, seeker_grid, target_grid, dx, dy):
        """
        Compute overlap score between two grids with translation.
        Returns (score, overlap_ratio).
        """
        shifted_target = shift(target_grid, [dy, dx], order=0, mode='constant', cval=0)

        overlap_mask = (seeker_grid > 0) | (shifted_target > 0)
        if np.sum(overlap_mask) == 0:
            return 0.0, 0.0

        seeker_masked = seeker_grid[overlap_mask]
        target_masked = shifted_target[overlap_mask]

        if len(seeker_masked) < 10:
            return 0.0, 0.0

        agreement = np.sum(seeker_masked * target_masked)
        seeker_norm = np.sum(seeker_masked * seeker_masked)
        target_norm = np.sum(target_masked * target_masked)
        if seeker_norm == 0 or target_norm == 0:
            return 0.0, 0.0

        correlation = agreement / np.sqrt(seeker_norm * target_norm)
        overlap_ratio = np.sum(overlap_mask) / (seeker_grid.shape[0] * seeker_grid.shape[1])
        # Penalize tiny overlaps more aggressively
        weight = min(overlap_ratio * 8.0, 1.0)
        score = correlation * weight
        return score, overlap_ratio

    def wrap_angle(self, ang):
        """Wrap angle to [-pi, pi]."""
        return (ang + math.pi) % (2 * math.pi) - math.pi

    def get_initial_guess(self):
        """Use odometry frames to seed search space if both seeker and target odom are available."""
        if self.seeker_odom is None or self.target_odom is None:
            return None
        sx = self.seeker_odom.pose.pose.position.x
        sy = self.seeker_odom.pose.pose.position.y
        tx = self.target_odom.pose.pose.position.x
        ty = self.target_odom.pose.pose.position.y
        sq = self.seeker_odom.pose.pose.orientation
        tq = self.target_odom.pose.pose.orientation
        _, _, syaw = euler.quat2euler([sq.w, sq.x, sq.y, sq.z])
        _, _, tyaw = euler.quat2euler([tq.w, tq.x, tq.y, tq.z])
        dx = tx - sx
        dy = ty - sy
        dyaw = math.degrees(tyaw - syaw)
        return (dx, dy, dyaw)

    def temporal_filter(self, pose):
        """Apply temporal smoothing"""
        self.pose_history.append(pose)
        
        if len(self.pose_history) < 3:
            return pose
        
        # Get recent poses
        recent_poses = list(self.pose_history)[-3:]
        
        # Median filter for position
        x_vals = [p[0] for p in recent_poses]
        y_vals = [p[1] for p in recent_poses]
        
        x_filtered = np.median(x_vals)
        y_filtered = np.median(y_vals)
        
        # Circular mean for angle
        sin_vals = [math.sin(p[2]) for p in recent_poses]
        cos_vals = [math.cos(p[2]) for p in recent_poses]
        yaw_filtered = math.atan2(np.mean(sin_vals), np.mean(cos_vals))
        
        # Don't filter if jump is too large (probably correct sudden change)
        if np.linalg.norm([x_filtered - pose[0], y_filtered - pose[1]]) > 0.5:
            return pose
        
        return (x_filtered, y_filtered, yaw_filtered)

    def publish_pose(self, pose, confidence):
        """Publish estimated target pose in map frame"""
        x, y, yaw = pose
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (transforms3d returns [w, x, y, z])
        quat = euler.euler2quat(0, 0, yaw)
        pose_msg.pose.pose.orientation.w = quat[0]
        pose_msg.pose.pose.orientation.x = quat[1]
        pose_msg.pose.pose.orientation.y = quat[2]
        pose_msg.pose.pose.orientation.z = quat[3]
        
        # Set covariance based on confidence
        pos_var = 0.05 / (confidence + 0.1)
        ang_var = 0.1 / (confidence + 0.1)
        
        pose_msg.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ang_var
        ]
        
>>>>>>> 36bbd91 (Whatever it takes ahh project)
        self.goal_pub.publish(pose_msg)
        # Emit measurement for EKF/UKF
        meas = PoseStamped()
        meas.header = pose_msg.header
        meas.pose = pose_msg.pose.pose
        self.meas_pub.publish(meas)
        self.last_valid_pose = pose
        self.last_confidence = confidence
        
        # Broadcast TF
        self.broadcast_tf(pose)
        
        # Visualize confidence
        self.visualize_confidence(confidence)

    def broadcast_tf(self, pose):
        """Broadcast TF transform"""
        x, y, yaw = pose
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'target_map'
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        
        quat = euler.euler2quat(0, 0, yaw)
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)

    def visualize_confidence(self, confidence):
        """Visualize confidence as a colored marker"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = "correlation_confidence"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        if self.last_valid_pose:
            marker.pose.position.x = self.last_valid_pose[0]
            marker.pose.position.y = self.last_valid_pose[1]
            marker.pose.position.z = 0.5
        else:
            marker.pose.position.z = 0.5
        
        marker.scale.z = 0.2
        
        # Color based on confidence: red (low) -> yellow -> green (high)
        if confidence < 0.3:
            marker.color.r = 1.0
            marker.color.g = 0.0
        elif confidence < 0.6:
            marker.color.r = 1.0
            marker.color.g = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
        
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = f"Confidence: {confidence:.1%}"
        
        self.correlation_pub.publish(marker)


class HybridTargetEstimator(RobustTargetEstimator):
    """
    Enhanced version that combines map correlation with velocity-based prediction
    """
    
    def __init__(self):
        super().__init__()
        self.velocity_history = deque(maxlen=10)
        
    def estimation_callback(self):
        """Enhanced estimation with velocity prediction"""
        if self.seeker_map is None or self.target_map is None:
            self.get_logger().warning("Waiting for both maps...")
            return
        
        # Get initial guess from velocity if available
        initial_guess = self.predict_from_velocity()
        
        # Convert maps
        seeker_grid = self.occupancy_grid_to_numpy(self.seeker_map)
        target_grid = self.occupancy_grid_to_numpy(self.target_map)
        resolution = self.seeker_map.info.resolution
        
        # Run correlation with initial guess to narrow search
        if initial_guess is not None:
            # Narrow search around prediction
            best_pose, confidence = self.correlate_near_guess(
                seeker_grid, target_grid, resolution, initial_guess
            )
        else:
            # Full search
            best_pose, confidence = self.correlate_maps(seeker_grid, target_grid, resolution)
        
        if best_pose is None:
            self.get_logger().error("Correlation failed!")
            return
        
        # Update velocity history
        if self.last_valid_pose is not None:
            dt = 0.5  # Timer period
            vel_x = (best_pose[0] - self.last_valid_pose[0]) / dt
            vel_y = (best_pose[1] - self.last_valid_pose[1]) / dt
            vel_yaw = self.angle_diff(best_pose[2], self.last_valid_pose[2]) / dt
            self.velocity_history.append((vel_x, vel_y, vel_yaw))
        
        # Apply temporal filtering
        filtered_pose = self.temporal_filter(best_pose)
        
        # Publish
        self.publish_pose(filtered_pose, confidence)
        
        self.get_logger().info(
            f"Target: ({filtered_pose[0]:.2f}, {filtered_pose[1]:.2f}, "
            f"{math.degrees(filtered_pose[2]):.1f}°) conf={confidence:.3f}"
        )
    
    def predict_from_velocity(self):
        """Predict next pose from velocity history"""
        if len(self.velocity_history) < 3 or self.last_valid_pose is None:
            return None
        
        # Average recent velocities
        recent_vels = list(self.velocity_history)[-5:]
        avg_vel_x = np.mean([v[0] for v in recent_vels])
        avg_vel_y = np.mean([v[1] for v in recent_vels])
        avg_vel_yaw = np.mean([v[2] for v in recent_vels])
        
        # Predict forward
        dt = 0.5
        pred_x = self.last_valid_pose[0] + avg_vel_x * dt
        pred_y = self.last_valid_pose[1] + avg_vel_y * dt
        pred_yaw = self.last_valid_pose[2] + avg_vel_yaw * dt
        
        return (pred_x, pred_y, pred_yaw)
    
    def correlate_near_guess(self, seeker_grid, target_grid, resolution, guess):
        """Narrow correlation search around an initial guess"""
        # Smaller search range
        dx_range = np.arange(-0.5, 0.5, self.search_step)
        dy_range = np.arange(-0.5, 0.5, self.search_step)
        dtheta_range = np.arange(-30, 30, self.angle_step)
        
        best_score = -np.inf
        best_transform = None
        best_overlap = 0.0
        
        guess_angle_deg = math.degrees(guess[2])
        
        for dtheta in dtheta_range:
            angle = guess_angle_deg + dtheta
            rotated_target = rotate(target_grid, angle, reshape=False, order=1, mode='constant', cval=0)
            
            for dx_offset in dx_range:
                for dy_offset in dy_range:
                    dx_total = guess[0] + dx_offset
                    dy_total = guess[1] + dy_offset
                    
                    dx_pixels = int(dx_total / resolution)
                    dy_pixels = int(dy_total / resolution)
                    
                    score, overlap = self.compute_overlap_score(seeker_grid, rotated_target, dx_pixels, dy_pixels)
                    
                    if score > best_score:
                        best_score = score
                        best_overlap = overlap
                        best_transform = (dx_total, dy_total, math.radians(angle))
        
        confidence = min(best_score * max(best_overlap, 0.1), 1.0)
        return best_transform, confidence
    
    def angle_diff(self, a, b):
        """Compute smallest angle difference"""
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    
    # Use the hybrid estimator for best performance
    node = HybridTargetEstimator()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
