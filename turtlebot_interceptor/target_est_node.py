import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
import math
import transforms3d.euler as euler
from scipy.ndimage import correlate
import tf2_ros


class TargetEstimator(Node):
    def __init__(self):
        super().__init__('mpc_node')
        print("I'm alive!!!!!")

        self.declare_parameter('obstacle_radius', 0.06)  # Default obstacle radius (meters)
        self.obstacle_radius_param = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        self.obstacle_decay_rate = .02

        self.seeker_obs = []
        self.target_obs = []

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

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to publish TF after 30 seconds
        self.timer2 = self.create_timer(30, self.publish_target_tf)

        self.start = False

    def publish_target_tf(self):
        if self.start:
            return
        # Get the target's pose from the calculated estimate
        if not hasattr(self, 'last_target_pose'):
            self.get_logger().warning("No target pose available for TF broadcast.")
            return

        pose_msg = self.last_target_pose  # Assuming this is the latest calculated pose
        
        # Create TransformStamped message
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'  # This is the parent frame
        transform.child_frame_id = 'target/odom'  # This is the child frame

        # Set position from pose (translation part)
        transform.transform.translation.x = pose_msg.pose.pose.position.x
        transform.transform.translation.y = pose_msg.pose.pose.position.y
        transform.transform.translation.z = pose_msg.pose.pose.position.z

        # Set orientation from pose (rotation part)
        transform.transform.rotation = pose_msg.pose.pose.orientation
        q = pose_msg.pose.pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])

        # Send the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Publishing transform from /odom to /target/odom: (x={transform.transform.translation.x}, y={transform.transform.translation.y}, yaw={yaw})")
        self.start = True


    def seeker_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.map = msg
        obstacles = self.extract_occupied_points(self.map)
        self.seeker_obs = self.improved_obstacle_clustering(self.seeker_obs, obstacles)
        
    def target_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.target_map = msg
        obstacles = self.extract_occupied_points(self.target_map)
        self.target_obs = self.improved_obstacle_clustering(self.target_obs, obstacles)

    def extract_occupied_points(self, map_data):
        """Extracts occupied points (x, y) from the OccupancyGrid"""
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
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                occupied_points.append([world_x, world_y])
        
        return occupied_points

    def improved_obstacle_clustering(self, curr_obs, occupied_points, eps=0.15, min_samples=3):
        """
        ALGORITHMIC IMPROVEMENT: Better obstacle clustering using DBSCAN-like algorithm.
        Groups nearby occupied cells into coherent obstacles with better representation.
        
        Args:
            occupied_points: List of (x, y) coordinates
            eps: Maximum distance between points in same cluster (15cm)
            min_samples: Minimum points to form a cluster
            constant_radius: The fixed radius for obstacles
            
        Returns: List of (center, radius) tuples
        """
        if len(occupied_points) == 0:
            return []

        points = np.array(occupied_points)
        n_points = len(points)

        constant_radius = self.obstacle_radius_param

        # Initialize labels (-1 means unassigned)
        labels = -np.ones(n_points, dtype=int)
        cluster_id = 0

        # DBSCAN-like clustering
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
            if len(cluster_points) < 0:
                continue  # don't Ignore small clusters
            
            center = np.mean(cluster_points, axis=0)
            unit_vector = center / np.linalg.norm(center)           
            new_center = center + unit_vector * constant_radius            
            radius = constant_radius
            
            obstacles.append((new_center, radius))
        
        obstacles.sort(key=lambda obs: np.linalg.norm(obs[0]))  # Sort by distance from (0,0)
        new_obs = obstacles[:3]
        if len(curr_obs) == 0:
            return new_obs
        else:
            updated_obs = []
            
            for (curr_center, curr_radius), (new_center, new_radius) in zip(curr_obs, new_obs):
                updated_center = (1 - self.obstacle_decay_rate) * np.array(curr_center) + self.obstacle_decay_rate * np.array(new_center)
                updated_radius = (1 - self.obstacle_decay_rate) * curr_radius + self.obstacle_decay_rate * new_radius
                updated_obs.append((updated_center, updated_radius))
            
            return updated_obs

    def pub_callback(self):
        if len(self.seeker_obs) == 0 or len(self.target_obs) == 0:
            self.get_logger().error("No obstacles seen")
            return
        
        if len(self.seeker_obs) != len(self.target_obs):
            self.get_logger().error("Diff number of obstacles seen")
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

    def publish_pose_estimate(self):
        # Extract the positions (x, y) from the obstacle data
        A = np.array([p[0] for p in self.target_obs])  # p[0] gives position (x, y)
        B = np.array([p[0] for p in self.seeker_obs])  # p[0] gives position (x, y)

        # Ensure the arrays are of shape (N, 2), where N is the number of obstacles
        if A.shape[1] != 2 or B.shape[1] != 2:
            self.get_logger().error(f"Shape mismatch: A.shape={A.shape}, B.shape={B.shape}")
            return

        # Calculate means of the points
        A_mean = A.mean(axis=0)
        B_mean = B.mean(axis=0)

        # Center the points around their mean
        A_center = A - A_mean
        B_center = B - B_mean

        # Compute the cross-covariance matrix H
        H = A_center.T @ B_center
        U, S, Vt = np.linalg.svd(H)

        # Estimate rotation and translation using SVD
        R_t2w_est = Vt.T @ U.T
        if np.linalg.det(R_t2w_est) < 0:
            Vt[-1, :] *= -1  # Ensure proper rotation matrix
            R_t2w_est = Vt.T @ U.T

        # Calculate translation
        t_t2w_est = B_mean - R_t2w_est @ A_mean

        # Estimate yaw angle
        est_yaw = math.degrees(math.atan2(R_t2w_est[1, 0], R_t2w_est[0, 0]))

        # Prepare the pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = t_t2w_est[0]
        pose_msg.pose.pose.position.y = t_t2w_est[1]
        pose_msg.pose.pose.position.z = 0.0

        # Set quaternion orientation (yaw only for simplicity)
        orientation = euler.euler2quat(0, 0, math.radians(est_yaw))  # Roll, pitch, yaw to quaternion
        pose_msg.pose.pose.orientation.x = orientation[0]
        pose_msg.pose.pose.orientation.y = orientation[1]
        pose_msg.pose.pose.orientation.z = orientation[2]
        pose_msg.pose.pose.orientation.w = orientation[3]

        # Set small covariance
        pose_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]
        self.last_target_pose = pose_msg

        # Log and publish the estimated pose
        self.get_logger().info(
            f"Publishing: ({t_t2w_est[0]}, {t_t2w_est[1]})"
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