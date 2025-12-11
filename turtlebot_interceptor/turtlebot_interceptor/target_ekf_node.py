#!/usr/bin/env python3
"""
TargetEKF Node
- Constant-velocity EKF for target tracking (state: [x, y, vx, vy]).
- Subscribes to /target_pose_measurement.
- Publishes /target_estimate (fast) and /target_estimate_slow (inflated, throttled) for the seeker.
"""
import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


class TargetEKF(Node):
    def __init__(self):
        super().__init__('target_ekf_node')

        # Parameters
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('process_noise', 0.02)
        self.declare_parameter('measurement_noise', 0.03)
        self.declare_parameter('warmup_duration', 65.0)
        self.declare_parameter('seeker_pub_period', 5.0)
        self.declare_parameter('seeker_covariance_scale', 4.0)
        try:
            self.declare_parameter('use_sim_time', False)
        except Exception:
            pass  # already declared via launch

        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        q = self.get_parameter('process_noise').get_parameter_value().double_value
        r = self.get_parameter('measurement_noise').get_parameter_value().double_value
        self.warmup_duration = self.get_parameter('warmup_duration').get_parameter_value().double_value
        self.seeker_pub_period = self.get_parameter('seeker_pub_period').get_parameter_value().double_value
        self.seeker_cov_scale = self.get_parameter('seeker_covariance_scale').get_parameter_value().double_value

        # State and covariance
        self.x = np.zeros((4, 1))  # [x, y, vx, vy]
        self.P = np.eye(4) * 0.5
        self.initialized = False

        # Constant-velocity model
        self.A = np.array([
            [1.0, 0.0, self.dt, 0.0],
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])
        self.Q = np.eye(4) * q
        self.R = np.eye(2) * r

        # Publishers
        self.est_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_estimate', 10)
        self.est_pub_slow = self.create_publisher(PoseWithCovarianceStamped, '/target_estimate_slow', 10)

        # Subscriptions
        self.meas_sub = self.create_subscription(PoseStamped, '/target_pose_measurement', self.measurement_cb, 10)

        # Timers
        self.timer = self.create_timer(self.dt, self.predict_timer)
        self.start_time = self.get_clock().now()
        self.slow_timer = self.create_timer(self.seeker_pub_period, self.publish_inflated_estimate)

        self.get_logger().info('TargetEKF initialized (CV model)')

    def measurement_cb(self, msg: PoseStamped):
        z = np.array([[msg.pose.position.x], [msg.pose.position.y]])

        if not self.initialized:
            self.x[:2, 0] = z.flatten()
            self.P = np.eye(4) * 0.1
            self.initialized = True
        else:
            self.predict()
            # Kalman update
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            y = z - self.H @ self.x
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ self.H) @ self.P

        self.publish_estimate()

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def predict_timer(self):
        if not self.initialized:
            return
        self.predict()
        self.publish_estimate()

    def publish_estimate(self):
        if not self.initialized:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])

        vx = float(self.x[2, 0])
        vy = float(self.x[3, 0])
        speed = math.hypot(vx, vy)
        if speed > 1e-3:
            yaw = math.atan2(vy, vx)
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            msg.pose.pose.orientation.w = 1.0

        cov6 = np.zeros((6, 6))
        cov6[:2, :2] = self.P[:2, :2]
        cov6[3:5, 3:5] = self.P[2:, 2:]
        msg.pose.covariance = cov6.flatten().tolist()

        self.est_pub.publish(msg)
        self.publish_inflated_estimate(immediate_msg=msg)

    def publish_inflated_estimate(self, immediate_msg=None):
        if not self.initialized:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.warmup_duration:
            return

        if immediate_msg is not None:
            msg = PoseWithCovarianceStamped()
            msg.header = immediate_msg.header
            msg.pose = immediate_msg.pose
        else:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = float(self.x[0, 0])
            msg.pose.pose.position.y = float(self.x[1, 0])
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.w = 1.0
            cov6 = np.zeros((6, 6))
            cov6[:2, :2] = self.P[:2, :2]
            cov6[3:5, 3:5] = self.P[2:, 2:]
            msg.pose.covariance = cov6.flatten().tolist()

        cov_matrix = np.array(msg.pose.covariance).reshape((6, 6))
        cov_matrix[:2, :2] *= self.seeker_cov_scale
        cov_matrix[3:5, 3:5] *= self.seeker_cov_scale
        msg.pose.covariance = cov_matrix.flatten().tolist()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.est_pub_slow.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
