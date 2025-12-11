#!/usr/bin/env python3
"""
Target estimator (vision + warm start)
- User provides warm-start guess; we publish that immediately.
- When a blue cone is detected (/camera_cone_positions), we snap goal to it.
- While the cone is visible, we maintain a constant-velocity estimate.
- When the cone disappears, we extrapolate for a short horizon then hold.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import transforms3d.euler as euler
from collections import deque


class TargetEstimator(Node):
    def __init__(self):
        super().__init__('target_estimator')

        # Parameters
        self.declare_parameter('warm_start_x', 0.0)
        self.declare_parameter('warm_start_y', 0.0)
        self.declare_parameter('warm_start_yaw', 0.0)
        self.declare_parameter('visibility_timeout', 3.0)  # seconds to extrapolate after last detection
        self.declare_parameter('max_extrap_speed', 1.0)  # m/s cap on extrapolated speed
        self.declare_parameter('cov_pos_base', 0.05)
        self.declare_parameter('cov_yaw_base', 0.1)
        self.declare_parameter('use_odom_measurement', False)  # default off to avoid drifting translation when no vision
        self.declare_parameter('lost_cov_scale', 4.0)  # inflate covariance when target is out of sight
        self.declare_parameter('lock_confidence', 0.8)  # once reached, freeze estimate
        self.declare_parameter('warmup_duration', 65.0)  # seconds before force-lock

        # Cache frequently used parameter values
        self.cov_pos_base = self.get_parameter('cov_pos_base').get_parameter_value().double_value
        self.cov_yaw_base = self.get_parameter('cov_yaw_base').get_parameter_value().double_value
        self.max_extrap_speed = self.get_parameter('max_extrap_speed').get_parameter_value().double_value
        self.visibility_timeout = self.get_parameter('visibility_timeout').get_parameter_value().double_value

        self.state = np.array([self.get_parameter('warm_start_x').get_parameter_value().double_value,
                               self.get_parameter('warm_start_y').get_parameter_value().double_value], dtype=float)
        self.yaw = self.get_parameter('warm_start_yaw').get_parameter_value().double_value
        self.velocity = np.zeros(2)
        self.last_detection_time = None
        self.history = deque(maxlen=5)
        self.seeker_odom = None
        self.target_odom = None
        self.lost_cov_scale = self.get_parameter('lost_cov_scale').get_parameter_value().double_value
        self.lock_conf = self.get_parameter('lock_confidence').get_parameter_value().double_value
        self.warmup_duration = self.get_parameter('warmup_duration').get_parameter_value().double_value
        self.locked_pose = None
        self.start_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_est', 10)
        self.pose_pub2 = self.create_publisher(PoseWithCovarianceStamped, '/target_est_from_maps', 10)
        self.meas_pub = self.create_publisher(PoseStamped, '/target_pose_measurement', 10)

        self.create_subscription(PointStamped, '/camera_cone_positions', self.cone_cb, 10)
        self.create_subscription(PointStamped, '/camera_target_positions', self.cone_cb, 10)  # blue cone target
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/target/odom', self.target_odom_cb, 10)

        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info("Target estimator (vision + warm start) ready")

    def odom_cb(self, msg: Odometry):
        self.seeker_odom = msg

    def target_odom_cb(self, msg: Odometry):
        self.target_odom = msg

    def cone_cb(self, msg: PointStamped):
        if self.locked_pose is not None:
            # Already locked; ignore further updates
            return
        now = self.get_clock().now().to_msg()
        if self.last_detection_time is not None:
            dt = (self.get_clock().now().nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            if dt > 0.0:
                vel = np.array([msg.point.x - self.state[0], msg.point.y - self.state[1]]) / dt
                speed = np.linalg.norm(vel)
                if speed > self.max_extrap_speed:
                    vel = vel * (self.max_extrap_speed / speed)
                self.velocity = 0.5 * self.velocity + 0.5 * vel
        self.state = np.array([msg.point.x, msg.point.y])
        self.last_detection_time = self.get_clock().now()
        self.history.append(self.state.copy())

    def tick(self):
        # Force-lock after warmup duration if not already locked
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.locked_pose is None and elapsed >= self.warmup_duration:
            yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
            self.locked_pose = (self.state[0], self.state[1], yaw_use)
            self.get_logger().info(f"🔒 Warmup elapsed ({elapsed:.1f}s). Locking target pose at ({self.state[0]:.2f}, {self.state[1]:.2f}).")

        # If we have recent detection, publish it
        if self.last_detection_time is not None:
            dt = (self.get_clock().now().nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            if dt < self.visibility_timeout:
                yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
                pose = (self.state[0], self.state[1], yaw_use)
                self.publish_pose(pose, confidence=0.9)
                return
            else:
                # Lost sight: hold last measured position (no extrapolation to avoid drift)
                yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
                pose = (self.state[0], self.state[1], yaw_use)
                self.velocity = np.zeros(2)
                self.publish_pose(pose, confidence=0.3)
                return

        # No detection yet: publish warm start
        yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
        pose = (self.state[0], self.state[1], yaw_use)
        self.publish_pose(pose, confidence=0.2)

    def yaw_from_odom(self):
        """Compute relative yaw from odom frames if available."""
        if self.seeker_odom is None or self.target_odom is None:
            return None
        syaw = self.odom_yaw(self.seeker_odom.pose.pose.orientation)
        tyaw = self.odom_yaw(self.target_odom.pose.pose.orientation)
        return self.wrap_angle(tyaw - syaw)

    def odom_yaw(self, q):
        """Extract yaw from geometry_msgs/Quaternion."""
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def wrap_angle(self, ang):
        """Wrap angle to [-pi, pi]."""
        return (ang + math.pi) % (2 * math.pi) - math.pi

    def publish_pose(self, pose, confidence):
        # If locked, hold that pose regardless of input confidence
        if self.locked_pose is not None:
            pose = self.locked_pose
            confidence = max(confidence, self.lock_conf)

        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        q = euler.euler2quat(0, 0, yaw)
        msg.pose.pose.orientation.x = q[1]
        msg.pose.pose.orientation.y = q[2]
        msg.pose.pose.orientation.z = q[3]
        msg.pose.pose.orientation.w = q[0]

        pos_var = self.get_parameter('cov_pos_base').get_parameter_value().double_value / max(confidence, 0.05)
        yaw_var = self.get_parameter('cov_yaw_base').get_parameter_value().double_value / max(confidence, 0.05)
        # Inflate covariance if target not recently seen
        if self.last_detection_time is None:
            pos_var *= self.lost_cov_scale
            yaw_var *= self.lost_cov_scale
        else:
            dt_since = (self.get_clock().now().nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            if dt_since > self.visibility_timeout:
                pos_var *= self.lost_cov_scale
                yaw_var *= self.lost_cov_scale
        # If locked, shrink covariance (high confidence) and set lock
        if confidence >= self.lock_conf and self.locked_pose is None:
            self.locked_pose = (x, y, yaw)
        if self.locked_pose is not None:
            pos_var = min(pos_var, self.cov_pos_base * 0.1)
            yaw_var = min(yaw_var, self.cov_yaw_base * 0.1)
        msg.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_var
        ]

        self.pose_pub.publish(msg)
        self.pose_pub2.publish(msg)
        meas = PoseStamped()
        meas.header = msg.header
        meas.pose = msg.pose.pose
        self.meas_pub.publish(meas)


def main(args=None):
    rclpy.init(args=args)
    node = TargetEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
