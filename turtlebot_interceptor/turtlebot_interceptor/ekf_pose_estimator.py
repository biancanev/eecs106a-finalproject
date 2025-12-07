#!/usr/bin/env python3
"""
EKF-based Pose Estimator for TurtleBot3
Fuses IMU (accelerometer + gyroscope), magnetometer, and wheel encoders
for accurate state estimation with gravity compensation.

State: [x, y, theta, vx, vy, omega]
- x, y: position in map frame
- theta: heading angle
- vx, vy: linear velocities
- omega: angular velocity

Sensors:
- IMU: Linear acceleration (gravity compensated) + angular velocity
- Magnetometer: Absolute heading reference
- Wheel encoders: Linear and angular velocity estimates
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import transforms3d.euler as euler


class EKFPoseEstimator(Node):
    """Extended Kalman Filter for sensor fusion state estimation"""
    
    def __init__(self):
        super().__init__('ekf_pose_estimator')
        
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        
        # State covariance
        self.P = np.eye(6) * 0.1
        self.P[0:2, 0:2] = 0.01  # Low initial position uncertainty
        self.P[2, 2] = 0.05  # Low initial heading uncertainty
        self.P[3:6, 3:6] = 0.1  # Higher velocity uncertainty
        
        # Process noise covariance
        self.Q = np.eye(6)
        self.Q[0:2, 0:2] = 0.001  # Position process noise
        self.Q[2, 2] = 0.01  # Heading process noise
        self.Q[3:5, 3:5] = 0.05  # Linear velocity process noise
        self.Q[5, 5] = 0.05  # Angular velocity process noise
        
        # Measurement noise covariances
        self.R_imu_gyro = 0.01  # IMU gyroscope (omega)
        self.R_imu_accel = 0.5  # IMU accelerometer (ax, ay)
        self.R_mag = 0.1  # Magnetometer (heading)
        self.R_odom_v = 0.02  # Odometry linear velocity
        self.R_odom_w = 0.02  # Odometry angular velocity
        
        # Gravity vector (for compensation)
        self.gravity_mag = 9.81  # m/s^2
        self.gravity_world = np.array([0.0, 0.0, self.gravity_mag])
        
        # Magnetometer calibration (will be auto-calibrated)
        self.mag_offset = np.zeros(3)
        self.mag_calibrated = False
        self.mag_samples = []
        self.mag_calibration_samples = 100
        
        # Timing
        self.last_time = None
        self.dt = 0.01  # Initial dt, will be updated
        
        # Data flags
        self.imu_data = None
        self.mag_data = None
        self.odom_data = None
        
        # QoS profile for sensor data (BEST_EFFORT for hardware compatibility)
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            sensor_qos
        )
        
        self.mag_sub = self.create_subscription(
            MagneticField,
            '/magnetic_field',
            self.mag_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # For MPC node
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_ekf',  # For Cartographer (refined odometry)
            10
        )
        
        # Timer for EKF prediction step (100Hz)
        self.timer = self.create_timer(0.01, self.prediction_step)
        
        self.get_logger().info('EKF Pose Estimator initialized - fusing IMU + Magnetometer + Encoders')
        self.get_logger().info('Calibrating magnetometer... (move robot in circle slowly)')
    
    def quaternion_to_euler(self, q: Quaternion):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        return euler.quat2euler([q.w, q.x, q.y, q.z])
    
    def imu_callback(self, msg: Imu):
        """Process IMU data - angular velocity and linear acceleration"""
        # Angular velocity (body frame)
        omega_body = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Linear acceleration (body frame, includes gravity)
        accel_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # Get orientation from IMU (for gravity compensation)
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        
        # Gravity compensation: remove gravity component from acceleration
        # Rotate gravity vector from world to body frame
        R_world_to_body = self.rotation_matrix(-yaw, -pitch, -roll)
        gravity_body = R_world_to_body @ self.gravity_world
        
        # True acceleration (gravity removed)
        accel_true = accel_body - gravity_body
        
        self.imu_data = {
            'omega': omega_body[2],  # Z-axis angular velocity (yaw rate)
            'accel': accel_true[0:2],  # X-Y acceleration (2D motion)
            'timestamp': self.get_clock().now()
        }
        
        # Update measurement (angular velocity)
        self.update_imu_gyro(omega_body[2])
        
        # DISABLED: Acceleration updates cause drift - rely on wheel odometry instead
        # if np.linalg.norm(accel_true[0:2]) > 0.1:
        #     self.update_imu_accel(accel_true[0:2])
    
    def mag_callback(self, msg: MagneticField):
        """Process magnetometer data for absolute heading"""
        mag_vec = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ])
        
        # Auto-calibration: collect samples
        if not self.mag_calibrated:
            self.mag_samples.append(mag_vec)
            if len(self.mag_samples) >= self.mag_calibration_samples:
                # Compute offset as mean (assumes robot rotated through full circle)
                self.mag_offset = np.mean(self.mag_samples, axis=0)
                self.mag_calibrated = True
                self.get_logger().info(f'Magnetometer calibrated! Offset: {self.mag_offset}')
            return
        
        # Apply calibration
        mag_calibrated = mag_vec - self.mag_offset
        
        # Compute heading from magnetometer (2D)
        heading = np.arctan2(mag_calibrated[1], mag_calibrated[0])
        
        self.mag_data = {
            'heading': heading,
            'timestamp': self.get_clock().now()
        }
        
        # Update measurement
        self.update_magnetometer(heading)
    
    def odom_callback(self, msg: Odometry):
        """Process wheel odometry data"""
        # Extract velocities
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # CRITICAL: Sanity check - if velocities are unreasonably high, robot might be stationary
        # This prevents integrating noise when robot hasn't started moving yet
        max_reasonable_v = 1.0  # m/s (TurtleBot max is ~0.6)
        if abs(v_x) > max_reasonable_v or abs(v_y) > max_reasonable_v:
            self.get_logger().warn(f'Odom velocity unreasonable: vx={v_x:.2f}, vy={v_y:.2f} - ignoring')
            return
        
        self.odom_data = {
            'vx': v_x,
            'vy': v_y,
            'omega': omega,
            'timestamp': self.get_clock().now()
        }
        
        # Update measurement
        self.update_odometry(v_x, v_y, omega)
    
    def rotation_matrix(self, yaw, pitch, roll):
        """Construct 3D rotation matrix from Euler angles (ZYX convention)"""
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        return R_z @ R_y @ R_x
    
    def prediction_step(self):
        """EKF prediction step - propagate state forward using motion model"""
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Compute dt
        self.dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.dt <= 0 or self.dt > 0.5:  # Sanity check
            return
        
        # State: [x, y, theta, vx, vy, omega]
        x, y, theta, vx, vy, omega = self.state
        
        # Motion model (unicycle with velocity state)
        # Predict position using velocity
        x_new = x + (vx * np.cos(theta) - vy * np.sin(theta)) * self.dt
        y_new = y + (vx * np.sin(theta) + vy * np.cos(theta)) * self.dt
        theta_new = theta + omega * self.dt
        
        # Normalize theta to [-pi, pi]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        
        # Velocity stays constant (will be updated by measurements)
        vx_new = vx
        vy_new = vy
        omega_new = omega
        
        # Update state
        self.state = np.array([x_new, y_new, theta_new, vx_new, vy_new, omega_new])
        
        # Jacobian of motion model
        F = np.eye(6)
        F[0, 2] = -(vx * np.sin(theta) + vy * np.cos(theta)) * self.dt  # dx/dtheta
        F[0, 3] = np.cos(theta) * self.dt  # dx/dvx
        F[0, 4] = -np.sin(theta) * self.dt  # dx/dvy
        F[1, 2] = (vx * np.cos(theta) - vy * np.sin(theta)) * self.dt  # dy/dtheta
        F[1, 3] = np.sin(theta) * self.dt  # dy/dvx
        F[1, 4] = np.cos(theta) * self.dt  # dy/dvy
        F[2, 5] = self.dt  # dtheta/domega
        
        # Propagate covariance
        self.P = F @ self.P @ F.T + self.Q * self.dt
        
        # Publish pose
        self.publish_pose()
    
    def update_imu_gyro(self, omega_measured):
        """EKF update step for IMU gyroscope (angular velocity)"""
        # Measurement model: z = omega (direct measurement of state[5])
        H = np.zeros((1, 6))
        H[0, 5] = 1.0  # Measures omega
        
        # Innovation
        z = np.array([omega_measured])
        z_pred = H @ self.state
        y = z - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu_gyro
        
        # Kalman gain
        K = self.P @ H.T / S
        
        # Update state and covariance
        self.state = self.state + K.flatten() * y[0]
        self.P = (np.eye(6) - np.outer(K, H)) @ self.P
    
    def update_imu_accel(self, accel_measured):
        """EKF update step for IMU accelerometer (linear acceleration)"""
        # DISABLED: Acceleration integration causes massive drift
        # The double integration of noisy accelerometer data leads to unbounded position error
        # We rely on wheel odometry for velocity instead
        return
    
    def update_magnetometer(self, heading_measured):
        """EKF update step for magnetometer (absolute heading)"""
        # Measurement model: z = theta (direct measurement of state[2])
        H = np.zeros((1, 6))
        H[0, 2] = 1.0  # Measures theta
        
        # Innovation (with angle wrapping)
        z = np.array([heading_measured])
        z_pred = H @ self.state
        y = z - z_pred
        
        # Wrap angle difference to [-pi, pi]
        y[0] = np.arctan2(np.sin(y[0]), np.cos(y[0]))
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_mag
        
        # Kalman gain
        K = self.P @ H.T / S
        
        # Update state and covariance
        self.state = self.state + K.flatten() * y[0]
        
        # Normalize theta
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        self.P = (np.eye(6) - np.outer(K, H)) @ self.P
    
    def update_odometry(self, vx_measured, vy_measured, omega_measured):
        """EKF update step for wheel odometry (velocities)"""
        # Measurement model: z = [vx, vy, omega] (direct measurement of state[3:6])
        H = np.zeros((3, 6))
        H[0, 3] = 1.0  # vx
        H[1, 4] = 1.0  # vy
        H[2, 5] = 1.0  # omega
        
        # Innovation
        z = np.array([vx_measured, vy_measured, omega_measured])
        z_pred = H @ self.state
        y = z - z_pred
        
        # Innovation covariance
        R_odom = np.diag([self.R_odom_v, self.R_odom_v, self.R_odom_w])
        S = H @ self.P @ H.T + R_odom
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # Skip if singular
        
        # Update state and covariance
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def publish_pose(self):
        """Publish current state estimate as PoseWithCovarianceStamped and Odometry"""
        stamp = self.get_clock().now().to_msg()
        
        # 1. Publish as PoseWithCovarianceStamped (for MPC)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        
        # Position
        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation (theta to quaternion)
        theta = self.state[2]
        quat = euler.euler2quat(0, 0, theta)
        pose_msg.pose.pose.orientation.w = quat[0]
        pose_msg.pose.pose.orientation.x = quat[1]
        pose_msg.pose.pose.orientation.y = quat[2]
        pose_msg.pose.pose.orientation.z = quat[3]
        
        # Covariance (6x6 -> flatten to 36 elements)
        # ROS expects: [x, y, z, rot_x, rot_y, rot_z]
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.P[0:2, 0:2]  # x, y
        cov[5, 5] = self.P[2, 2]  # theta (maps to rot_z)
        pose_msg.pose.covariance = cov.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
        
        # 2. Publish as Odometry (for Cartographer)
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Pose (same as above)
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = quat[0]
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.covariance = pose_msg.pose.covariance
        
        # Twist (velocities in body frame)
        # Convert from world frame (vx, vy) to body frame
        vx_world = self.state[3]
        vy_world = self.state[4]
        omega = self.state[5]
        
        # Rotate to body frame
        vx_body = vx_world * np.cos(theta) + vy_world * np.sin(theta)
        vy_body = -vx_world * np.sin(theta) + vy_world * np.cos(theta)
        
        odom_msg.twist.twist.linear.x = vx_body
        odom_msg.twist.twist.linear.y = vy_body
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        
        # Twist covariance
        twist_cov = np.zeros((6, 6))
        twist_cov[0:2, 0:2] = self.P[3:5, 3:5]  # vx, vy
        twist_cov[5, 5] = self.P[5, 5]  # omega
        odom_msg.twist.covariance = twist_cov.flatten().tolist()
        
        self.odom_pub.publish(odom_msg)
        
        # Log periodically
        if not hasattr(self, '_pose_count'):
            self._pose_count = 0
        self._pose_count += 1
        if self._pose_count % 100 == 0:  # Every 1 second at 100Hz
            self.get_logger().info(
                f'EKF State: pos=({self.state[0]:.3f}, {self.state[1]:.3f}), '
                f'θ={np.degrees(self.state[2]):.1f}°, '
                f'vel=({self.state[3]:.2f}, {self.state[4]:.2f}), '
                f'ω={self.state[5]:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = EKFPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

