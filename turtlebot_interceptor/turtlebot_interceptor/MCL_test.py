#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import transforms3d.euler as euler

class MCL(Node):
    """
    Minimal Monte Carlo Localization implementation.
    State = [x, y, theta]
    """

    def __init__(self, N=300, motion_noise=[0.02, 0.02, 0.01], use_sim=False):
        super().__init__('mcl_node')

        self.N = N
        self.motion_noise = np.array(motion_noise)
        self.particles = None
        self.weights = None
        self.map = None
        self.initialized = False
        self.use_sim = use_sim

        # Subscriptions
        if use_sim:
            # For simulation: use fake lidar or voxel grid
            self.lidar_sub = self.create_subscription(
                LaserScan,
                '/scan',  # Standard ROS2 topic
                self.lidar_callback,
                10
            )
        else:
            # For hardware: use actual lidar
            self.lidar_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.lidar_callback,
                10
            )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            10
        )

        self.last_cmd = None
        self.last_cmd_time = None

    def map_callback(self, msg: OccupancyGrid):
        """Store the map for likelihood computation"""
        self.map = msg
        if not self.initialized and self.map is not None:
            # Initialize particles uniformly over free space
            self.initialize_from_map()

    def cmd_callback(self, msg: Twist):
        """Store last command for motion model"""
        self.last_cmd = [msg.linear.x, msg.angular.z]
        self.last_cmd_time = self.get_clock().now()

    def initialize_from_map(self):
        """Initialize particles uniformly in free space"""
        if self.map is None:
            return
        
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # Find free cells
        free_cells = []
        for i in range(width * height):
            if self.map.data[i] == 0:  # Free space
                x = (i % width) * resolution + origin_x
                y = (i // width) * resolution + origin_y
                free_cells.append((x, y))
        
        if len(free_cells) == 0:
            self.get_logger().warn("No free cells found in map!")
            return
        
        # Sample particles from free space
        self.particles = np.zeros((self.N, 3))
        indices = np.random.choice(len(free_cells), self.N)
        for i, idx in enumerate(indices):
            self.particles[i, 0] = free_cells[idx][0]
            self.particles[i, 1] = free_cells[idx][1]
            self.particles[i, 2] = np.random.uniform(0, 2 * np.pi)
        
        self.weights = np.ones(self.N) / self.N
        self.initialized = True
        self.get_logger().info(f"MCL initialized with {self.N} particles")

    def initialize_uniform(self, x_range, y_range, theta_range):
        """Initialize particles uniformly in given ranges"""
        self.particles = np.zeros((self.N, 3))
        self.particles[:, 0] = np.random.uniform(*x_range, self.N)
        self.particles[:, 1] = np.random.uniform(*y_range, self.N)
        self.particles[:, 2] = np.random.uniform(*theta_range, self.N)
        self.weights = np.ones(self.N) / self.N
        self.initialized = True

    def predict(self, u, dt):
        """
        Motion model prediction step
        u = [v, omega]
        """
        if not self.initialized:
            return
        
        v, w = u
        noise = np.random.randn(self.N, 3) * self.motion_noise

        self.particles[:, 0] += dt * v * np.cos(self.particles[:, 2]) + noise[:, 0]
        self.particles[:, 1] += dt * v * np.sin(self.particles[:, 2]) + noise[:, 1]
        self.particles[:, 2] += dt * w + noise[:, 2]
        self.particles[:, 2] = np.mod(self.particles[:, 2] + np.pi, 2 * np.pi) - np.pi  # Wrap to [-pi, pi]

    def compute_likelihood(self, particle, lidar_scan):
        """Compute likelihood of particle given lidar scan"""
        if self.map is None:
            return 1e-10
        
        # Simple likelihood: compare expected vs actual ranges
        # This is a simplified version - full implementation would raycast
        px, py, ptheta = particle
        
        # For now, return uniform likelihood (will be improved)
        # TODO: Implement proper raycasting likelihood
        return 1.0 / self.N

    def lidar_callback(self, msg: LaserScan):
        """Update step when new lidar scan arrives"""
        if not self.initialized or self.map is None:
            return
        
        # Predict step (if we have a command)
        if self.last_cmd is not None and self.last_cmd_time is not None:
            dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if dt > 0 and dt < 1.0:  # Reasonable dt
                self.predict(self.last_cmd, dt)
        
        # Update step: compute weights
        for i in range(self.N):
            self.weights[i] = self.compute_likelihood(self.particles[i], msg)

        # Normalize weights
        self.weights += 1e-12
        self.weights /= np.sum(self.weights)
        
        # Resample
        self.resample()
        
        # Publish pose estimate
        self.publish_pose()

    def resample(self):
        """Resample particles based on weights"""
        idx = np.random.choice(self.N, self.N, p=self.weights)
        self.particles = self.particles[idx]
        self.weights = np.ones(self.N) / self.N

    def mean_and_covariance(self):
        """Compute mean and covariance from particles"""
        if not self.initialized:
            return None, None
        
        mean = np.average(self.particles, axis=0, weights=self.weights)
        # Weighted covariance
        diff = self.particles - mean
        cov = np.cov(diff.T, aweights=self.weights)
        return mean, cov

    def publish_pose(self):
        """Publish current pose estimate"""
        mean, cov = self.mean_and_covariance()
        if mean is None:
            return
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(mean[0])
        msg.pose.pose.position.y = float(mean[1])
        
        # Convert theta to quaternion
        quat = [0.0, 0.0, np.sin(mean[2] / 2.0), np.cos(mean[2] / 2.0)]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        # Convert 3x3 covariance to 6x6 pose covariance
        pose_cov = np.zeros((6, 6))
        pose_cov[0, 0] = cov[0, 0]
        pose_cov[1, 1] = cov[1, 1]
        pose_cov[5, 5] = cov[2, 2]
        pose_cov[0, 1] = cov[0, 1]
        pose_cov[1, 0] = cov[1, 0]
        msg.pose.covariance = pose_cov.flatten().tolist()
        
        self.pose_pub.publish(msg)
