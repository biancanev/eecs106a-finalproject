#!/usr/bin/env python3
"""
UKF Node for Target Tracking
Hardware-ready ROS2 node for tracking target using Unscented Kalman Filter
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import math


class SimpleUKF:
    """Unscented Kalman Filter for target tracking"""
    def __init__(self, dt=0.1):
        self.dt = dt
        self.n = 4  # State dimension [px, py, vx, vy]
        self.x = np.zeros((self.n, 1))
        self.P = np.eye(self.n) * 10.0
        self.initialized = False
        
        # UKF parameters
        self.alpha = 0.001
        self.beta = 2.0
        self.kappa = 0.0
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n
        
        # Weights
        self.Wm0 = self.lambda_ / (self.n + self.lambda_)
        self.Wc0 = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)
        self.Wi = 1.0 / (2.0 * (self.n + self.lambda_))
        
        # Process and measurement noise
        self.Q = np.eye(self.n) * 0.02
        self.R = np.eye(2) * 0.03
    
    def _generate_sigma_points(self):
        """Generate sigma points for UKF"""
        n = self.n
        P_safe = (self.P + self.P.T) / 2
        P_safe += np.eye(n) * 1e-6
        try:
            sqrt_P = np.linalg.cholesky((n + self.lambda_) * P_safe)
        except:
            eigenvals, eigenvecs = np.linalg.eig(P_safe)
            eigenvals = np.maximum(eigenvals, 1e-6)
            sqrt_P = eigenvecs @ np.diag(np.sqrt(eigenvals * (n + self.lambda_)))
        sigma_points = np.zeros((n, 2*n + 1))
        sigma_points[:, 0] = self.x.flatten()
        for i in range(n):
            sigma_points[:, i+1] = (self.x + sqrt_P[:, i]).flatten()
            sigma_points[:, i+n+1] = (self.x - sqrt_P[:, i]).flatten()
        return sigma_points
    
    def _f(self, x):
        """Nonlinear process model (constant velocity)"""
        px, py, vx, vy = x[0, 0], x[1, 0], x[2, 0], x[3, 0]
        return np.array([[px + self.dt * vx],
                         [py + self.dt * vy],
                         [vx],
                         [vy]])
    
    def _h(self, x):
        """Measurement model (position only)"""
        return x[:2]
    
    def predict(self):
        """UKF prediction step"""
        if not self.initialized:
            return
        
        sigma_points = self._generate_sigma_points()
        n = self.n
        sigma_points_pred = np.zeros((n, 2*n + 1))
        for i in range(2*n + 1):
            sigma_points_pred[:, i] = self._f(sigma_points[:, i].reshape(n, 1)).flatten()
        
        x_pred = self.Wm0 * sigma_points_pred[:, 0]
        for i in range(1, 2*n + 1):
            x_pred += self.Wi * sigma_points_pred[:, i]
        x_pred = x_pred.reshape(n, 1)
        
        P_pred = self.Wc0 * (sigma_points_pred[:, 0:1] - x_pred) @ (sigma_points_pred[:, 0:1] - x_pred).T
        for i in range(1, 2*n + 1):
            diff = (sigma_points_pred[:, i:i+1] - x_pred)
            P_pred += self.Wi * diff @ diff.T
        P_pred += self.Q
        
        self.x = x_pred
        self.P = P_pred
    
    def update(self, z):
        """UKF update step with measurement"""
        if not self.initialized:
            self.x[0, 0] = z[0]
            self.x[1, 0] = z[1]
            self.x[2, 0] = 0.0
            self.x[3, 0] = 0.0
            self.P = np.eye(4) * 0.1
            self.initialized = True
            return
        
        sigma_points = self._generate_sigma_points()
        n = self.n
        z_sigma = np.zeros((2, 2*n + 1))
        for i in range(2*n + 1):
            z_sigma[:, i] = self._h(sigma_points[:, i].reshape(n, 1)).flatten()
        
        z_pred = self.Wm0 * z_sigma[:, 0]
        for i in range(1, 2*n + 1):
            z_pred += self.Wi * z_sigma[:, i]
        z_pred = z_pred.reshape(2, 1)
        
        P_pred = self.P.copy()
        S = self.Wc0 * (z_sigma[:, 0:1] - z_pred) @ (z_sigma[:, 0:1] - z_pred).T
        for i in range(1, 2*n + 1):
            diff = (z_sigma[:, i:i+1] - z_pred)
            S += self.Wi * diff @ diff.T
        S += self.R
        
        S = (S + S.T) / 2
        S += np.eye(2) * 1e-5
        
        Pxz = self.Wc0 * (sigma_points[:, 0:1] - self.x) @ (z_sigma[:, 0:1] - z_pred).T
        for i in range(1, 2*n + 1):
            Pxz += self.Wi * (sigma_points[:, i:i+1] - self.x) @ (z_sigma[:, i:i+1] - z_pred).T
        
        try:
            K = Pxz @ np.linalg.inv(S)
        except:
            K = Pxz @ np.linalg.pinv(S)
        
        z = z.reshape(2, 1)
        innovation = z - z_pred
        innovation_norm = np.linalg.norm(innovation)
        if innovation_norm > 1.5:
            scale = 1.5 / innovation_norm
            innovation = innovation * scale
        
        self.x = self.x + K @ innovation
        
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        
        self.P = (self.P + self.P.T) / 2
        self.P += np.eye(self.n) * 1e-5
        self.P = np.clip(self.P, -5.0, 5.0)
        diag = np.diag(self.P)
        diag = np.clip(diag, 0.0001, 5.0)
        np.fill_diagonal(self.P, diag)
        
        self.x[0, 0] = np.clip(self.x[0, 0], -10.0, 10.0)
        self.x[1, 0] = np.clip(self.x[1, 0], -10.0, 10.0)
        self.x[2, 0] = np.clip(self.x[2, 0], -1.0, 1.0)
        self.x[3, 0] = np.clip(self.x[3, 0], -1.0, 1.0)
    
    def get_state(self):
        """Get current state estimate"""
        return self.x.copy()
    
    def get_uncertainty(self):
        """Get position uncertainty (trace of position covariance)"""
        if not self.initialized:
            return 1.0
        pos_cov = self.P[:2, :2]
        return np.sqrt(np.trace(pos_cov))


class UKFNode(Node):
    """UKF Node for target tracking - hardware ready"""
    
    def __init__(self):
        super().__init__('ukf_node')
        
        # Declare parameters (lab4 pattern)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('process_noise', 0.02)
        self.declare_parameter('measurement_noise', 0.03)
        self.declare_parameter('use_sim_time', False)
        
        # Get parameters
        dt = self.get_parameter('dt').get_parameter_value().double_value
        process_noise = self.get_parameter('process_noise').get_parameter_value().double_value
        measurement_noise = self.get_parameter('measurement_noise').get_parameter_value().double_value
        
        # UKF instance with parameters
        self.ukf = SimpleUKF(dt=dt)
        self.ukf.Q = np.eye(4) * process_noise
        self.ukf.R = np.eye(2) * measurement_noise
        
        # Subscriptions
        self.target_meas_sub = self.create_subscription(
            PoseStamped,
            '/target_pose_measurement',  # External source provides target measurements
            self.target_measurement_callback,
            10
        )
        
        # Publishers
        self.target_est_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/target_estimate',
            10
        )
        
        # Timer for prediction (UKF always predicts, even without measurements)
        self.prediction_timer = self.create_timer(dt, self.prediction_timer_callback)
        
        self.get_logger().info('UKF node initialized - ready for hardware')
    
    def target_measurement_callback(self, msg: PoseStamped):
        """Process target measurement from external source"""
        # Extract position measurement
        z = np.array([msg.pose.position.x, msg.pose.position.y])
        
        # Update UKF with measurement
        self.ukf.update(z)
        
        # Publish updated estimate
        self.publish_estimate()
    
    def prediction_timer_callback(self):
        """Predict step - always runs, even without measurements"""
        # Always predict (UKF continues estimating even without measurements)
        self.ukf.predict()
        
        # Publish current estimate
        if self.ukf.initialized:
            self.publish_estimate()
    
    def publish_estimate(self):
        """Publish target estimate"""
        if not self.ukf.initialized:
            return
        
        state = self.ukf.get_state()
        cov = self.ukf.P
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = float(state[0, 0])
        msg.pose.pose.position.y = float(state[1, 0])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Flatten covariance matrix (6x6 for PoseWithCovariance)
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[:2, :2] = cov[:2, :2]  # Position covariance
        cov_6x6[3:5, 3:5] = cov[2:, 2:]  # Velocity covariance (in orientation field)
        msg.pose.covariance = cov_6x6.flatten().tolist()
        
        self.target_est_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

