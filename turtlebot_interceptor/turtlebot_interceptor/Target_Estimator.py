#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class TargetKF(Node):
    """
    Kalman Filter for target with state [px, py, vx, vy]
    Based on paper: constant velocity model
    """

    def __init__(self, dt=0.1):
        super().__init__('target_kf_node')

        # state: [px, py, vx, vy]^T
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 1.0
        self.dt = dt

        # Dynamics matrix (constant velocity model)
        self.A = np.array([
            [1.0, 0.0, self.dt, 0.0],
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self.Q = np.eye(4) * 0.01  # Process noise

        # Observation matrix (only position observed)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])
        self.R = np.eye(2) * 0.05  # Measurement noise

        # Subscriptions and publishers
        self.meas_sub = self.create_subscription(
            PoseStamped,
            '/target_pose_measurement',
            self.measurement_callback,
            10
        )

        self.est_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/target_estimate',
            10
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def measurement_callback(self, msg: PoseStamped):
        """Update step when new measurement arrives"""
        z = np.array([[msg.pose.position.x],
                      [msg.pose.position.y]])
        
        # Prediction step
        x_pred = self.A @ self.x
        P_pred = self.A @ self.P @ self.A.T + self.Q

        # Update step
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.x = x_pred + K @ (z - self.H @ x_pred)
        self.P = (np.eye(4) - K @ self.H) @ P_pred

    def timer_callback(self):
        """Publish current estimate and predict forward if no new measurement"""
        # Prediction-only if no new measurement
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

        # Publish estimate
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])
        
        # Convert covariance (4x4) to 6x6 pose covariance
        cov = np.zeros((6, 6))
        cov[0, 0] = self.P[0, 0]
        cov[1, 1] = self.P[1, 1]
        cov[0, 1] = self.P[0, 1]
        cov[1, 0] = self.P[1, 0]
        msg.pose.covariance = cov.flatten().tolist()
        self.est_pub.publish(msg)

    def get_state_and_cov(self):
        """Get current state and covariance"""
        return self.x.copy(), self.P.copy()

    def predict_horizon(self, N):
        """Predict target states for MPC horizon"""
        x_pred = self.x.copy()
        trajectory = []
        for k in range(N + 1):
            pos = np.array([x_pred[0, 0], x_pred[1, 0]])
            trajectory.append(pos)
            x_pred = self.A @ x_pred
        return np.array(trajectory).T  # Shape: (2, N+1)
