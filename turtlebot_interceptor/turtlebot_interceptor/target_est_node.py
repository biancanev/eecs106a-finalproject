import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
import math
import transforms3d.euler as euler
from scipy.ndimage import correlate


class TargetEstimator(Node):
    def __init__(self):
        super().__init__('mpc_node')

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
        )

        self.target_sub = self.create_subscription(
            OccupancyGrid,
            '/target/local_map',
            self.target_callback,
            10
        )

        self.obs_pub_seeker = self.create_publisher(Marker, '/obstacles', 10)
        self.obs_pub_target = self.create_publisher(Marker, '/obstacles_target', 10)

        self.goal_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_est', 1)
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
        
    def target_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.target_map = msg
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
        
        # Publish seeker obstacles
        for i, (pos, radius) in enumerate(self.seeker_obs):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = "seeker_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.0  # 2D, so z is always 0
            marker.scale.x = 2 * radius  # Diameter of the cylinder
            marker.scale.y = 2 * radius  # Diameter of the cylinder
            marker.scale.z = 0.1  # Small height for visualization
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            self.obs_pub_seeker.publish(marker)

        # Publish target obstacles
        for i, (pos, radius) in enumerate(self.target_obs):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = "target_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.0  # 2D, so z is always 0
            marker.scale.x = 2 * radius  # Diameter of the cylinder
            marker.scale.y = 2 * radius  # Diameter of the cylinder
            marker.scale.z = 0.1  # Small height for visualization
            marker.color.r = 0.0  # Green
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            self.obs_pub_target.publish(marker)

        # You can also publish the estimated pose (optional, already present in your code)
        self.publish_pose_estimate()
        # self.estimate_pose_from_maps()

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
        self.goal_pub.publish(pose_msg)
    
    def estimate_pose_from_maps(self):
        """
        Estimate the pose by aligning the local_map with the target_local_map
        using a normalized cross-correlation approach to determine the best
        rotation and translation for map alignment.
        """
        # Extract the occupancy grids from the maps
        local_map_data = np.array(self.map.data).reshape(self.map.info.height, self.map.info.width)
        target_map_data = np.array(self.target_map.data).reshape(self.target_map.info.height, self.target_map.info.width)
        
        # Extract map resolution and origin information
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        target_origin_x = self.target_map.info.origin.position.x
        target_origin_y = self.target_map.info.origin.position.y
        
        # Convert maps to binary (occupied vs free space) for NCC
        local_map_bin = local_map_data > 30  # Occupied cells
        target_map_bin = target_map_data > 30  # Occupied cells

        def downsample_map(map_data, factor=2):
            return map_data[::factor, ::factor]

        local_map_bin = downsample_map(local_map_bin, factor=4)
        target_map_bin = downsample_map(target_map_bin, factor=4)

        
        # Define the search range for translation and rotation (small values for coarse alignment)
        translation_range = np.arange(0.0, 1.5, 0.1)  # ±1.5 meter in 10 cm steps
        rotation_range = np.arange(-180.0, 180.0, 10.0)  # ±180 degrees in 10 degree steps

        best_score = -np.inf
        best_translation = (0, 0)
        best_rotation = 0

        self.get_logger().info("starting map estimate pose")

        for dx in translation_range:
            for dy in translation_range:
                for rotation in rotation_range:
                    # Rotate and translate the target map
                    rotated_target_map = self.rotate_map(target_map_bin, rotation)
                    translated_target_map = self.translate_map(rotated_target_map, dx, dy)
                    
                    # Calculate the similarity score using normalized cross-correlation
                    score = self.compute_ncc(local_map_bin, translated_target_map)
                    
                    # Update the best transformation if the score improves
                    if score > best_score:
                        best_score = score
                        best_translation = (dx, dy)
                        best_rotation = rotation
                    # self.get_logger().info(f"checking: {dx}, {dy}, {rotation}")

        # After finding the best transformation, apply it to the pose estimation
        t_x, t_y = best_translation
        est_yaw = best_rotation
        
        # Estimate the pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = t_x + origin_x - target_origin_x
        pose_msg.pose.pose.position.y = t_y + origin_y - target_origin_y
        pose_msg.pose.pose.position.z = 0.0

        # Set quaternion orientation (yaw only)
        orientation = euler.euler2quat(0, 0, math.radians(est_yaw))  # Roll, pitch, yaw to quaternion
        pose_msg.pose.pose.orientation.x = orientation[0]
        pose_msg.pose.pose.orientation.y = orientation[1]
        pose_msg.pose.pose.orientation.z = orientation[2]
        pose_msg.pose.pose.orientation.w = orientation[3]

        # Set covariance (small uncertainty)
        pose_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        # Log and publish the estimated pose
        self.get_logger().info(f"Publishing estimated pose: ({pose_msg.pose.pose.position.x}, {pose_msg.pose.pose.position.y}), yaw={est_yaw}")
        self.goal_pub2.publish(pose_msg)

    def rotate_map(self, map_data, angle_deg):
        """Rotate the map data by a given angle in degrees (counterclockwise)."""
        angle_rad = math.radians(angle_deg)
        return np.rot90(map_data, k=int(angle_rad // (math.pi / 2)))  # Approximate rotation in 90° steps

    def translate_map(self, map_data, dx, dy):
        """Translate the map by (dx, dy)."""
        shifted_map = np.roll(map_data, shift=(int(dy / self.map.info.resolution), int(dx / self.map.info.resolution)), axis=(0, 1))
        return shifted_map

    def compute_ncc(self, map1, map2):
        """Compute Normalized Cross-Correlation (NCC) between two maps."""
        correlation = correlate(map1.astype(float), map2.astype(float), mode='constant')
        max_correlation = np.max(correlation)
        norm_factor = np.sqrt(np.sum(map1**2) * np.sum(map2**2))
        return max_correlation / norm_factor if norm_factor > 0 else 0


def main(args=None):
    rclpy.init(args=args)
    node = TargetEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()