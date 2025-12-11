#!/usr/bin/env python3
"""
DriveTester: drives the robot in simple patterns (fwd/back/left/right/diagonals)
and logs pose error between simple pose (/amcl_pose) and EKF (/odom_ekf).
Useful for comparing estimators.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np


class DriveTester(Node):
    def __init__(self):
        super().__init__('drive_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/drive_test_markers', 10)

        self.raw_odom = None
        self.ekf_odom = None
        self.raw_cov = None
        self.ekf_cov = None
        self.raw_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.ekf_sub = self.create_subscription(
            Odometry, '/odom_ekf', self.ekf_cb, 10)

        # Simple sequence: forward, left, backward, right, diag left fwd, diag right back, stop
        self.sequence = [
            {'vx': 0.15, 'vy': 0.0, 'wz': 0.0, 'duration': 2.0},
            {'vx': 0.0, 'vy': 0.0, 'wz': 0.8, 'duration': 1.5},
            {'vx': -0.15, 'vy': 0.0, 'wz': 0.0, 'duration': 2.0},
            {'vx': 0.0, 'vy': 0.0, 'wz': -0.8, 'duration': 1.5},
            {'vx': 0.10, 'vy': 0.10, 'wz': 0.0, 'duration': 2.0},
            {'vx': -0.10, 'vy': -0.10, 'wz': 0.0, 'duration': 2.0},
            {'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'duration': 0.0},
        ]
        self.seq_index = 0
        self.seq_start = self.get_clock().now()
        # Delay before starting commands (seconds)
        self.start_delay = 30
        self.started = False

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("DriveTester started: comparing /odom vs /odom_ekf")
    def odom_cb(self, msg):
        self.raw_odom = msg.pose.pose
        try:
            self.raw_cov = np.array(msg.pose.covariance).reshape((6, 6))
        except Exception:
            self.raw_cov = None

    def ekf_cb(self, msg):
        self.ekf_pose = msg.pose.pose
        try:
            self.ekf_cov = np.array(msg.pose.covariance).reshape((6, 6))
        except Exception:
            self.ekf_cov = None

    def loop(self):
        # publish next command in sequence
        now = self.get_clock().now()
        if not self.started:
            if (now - self.seq_start).nanoseconds / 1e9 < self.start_delay:
                return
            self.started = True
            self.seq_start = now

        current = self.sequence[self.seq_index]
        elapsed = (now - self.seq_start).nanoseconds / 1e9
        if elapsed > current['duration']:
            self.seq_index += 1
            self.seq_start = now
            if self.seq_index >= len(self.sequence):
                self.seq_index = len(self.sequence) - 1  # hold last command (stop)
            current = self.sequence[self.seq_index]

        cmd = Twist()
        cmd.linear.x = float(current['vx'])
        cmd.linear.y = float(current['vy'])
        cmd.angular.z = float(current['wz'])
        self.cmd_pub.publish(cmd)

        # log pose/yaw error if both estimates available
        if self.raw_odom and self.ekf_pose:
            rx, ry = self.raw_odom.position.x, self.raw_odom.position.y
            ex, ey = self.ekf_pose.position.x, self.ekf_pose.position.y
            pos_err = np.hypot(rx - ex, ry - ey)
            raw_yaw = self.yaw_from_quat(self.raw_odom.orientation)
            ekf_yaw = self.yaw_from_quat(self.ekf_pose.orientation)
            yaw_err = np.arctan2(np.sin(raw_yaw - ekf_yaw), np.cos(raw_yaw - ekf_yaw))
            self.get_logger().info(
                f"Seq {self.seq_index}: cmd(vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}) "
                f"pos_err={pos_err:.3f}m yaw_err={np.degrees(yaw_err):.2f}deg "
                f"odom=({rx:.2f},{ry:.2f},{np.degrees(raw_yaw):.1f}°) "
                f"ekf=({ex:.2f},{ey:.2f},{np.degrees(ekf_yaw):.1f}°)"
            )
            self.publish_markers()

    def yaw_from_quat(self, q):
        import math
        # q is geometry_msgs/Quaternion
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        w, x, y, z = q.w, q.x, q.y, q.z
        return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def publish_markers(self):
        markers = MarkerArray()
        mid = 0

        # Raw odom pose marker
        if self.raw_odom:
            m = self.pose_marker(self.raw_odom.position.x, self.raw_odom.position.y,
                                 ns='odom_raw', mid=mid, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
            markers.markers.append(m); mid += 1
            if self.raw_cov is not None:
                markers.markers.append(self.ellipse_marker(self.raw_cov[:2, :2],
                                                           self.raw_odom.position.x,
                                                           self.raw_odom.position.y,
                                                           ns='odom_raw_cov',
                                                           mid=mid,
                                                           color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.4)))
                mid += 1

        # EKF pose marker
        if self.ekf_pose:
            m = self.pose_marker(self.ekf_pose.position.x, self.ekf_pose.position.y,
                                 ns='odom_ekf', mid=mid, color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0))
            markers.markers.append(m); mid += 1
            if self.ekf_cov is not None:
                markers.markers.append(self.ellipse_marker(self.ekf_cov[:2, :2],
                                                           self.ekf_pose.position.x,
                                                           self.ekf_pose.position.y,
                                                           ns='odom_ekf_cov',
                                                           mid=mid,
                                                           color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.4)))
                mid += 1

        self.marker_pub.publish(markers)

    def pose_marker(self, x, y, ns, mid, color):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color = color
        m.lifetime.sec = 0  # persist until overwritten
        return m

    def ellipse_marker(self, cov_2d, cx, cy, ns, mid, color):
        eigenvals, eigenvecs = np.linalg.eig(cov_2d)
        eigenvals = np.maximum(eigenvals, 1e-6)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        num_points = 30
        angle = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])
        width = 2 * np.sqrt(eigenvals[0]) * 2.0
        height = 2 * np.sqrt(eigenvals[1]) * 2.0
        m.points = []
        for i in range(num_points + 1):
            theta = 2 * np.pi * i / num_points
            x_local = width / 2 * np.cos(theta)
            y_local = height / 2 * np.sin(theta)
            x_rot = x_local * np.cos(angle) - y_local * np.sin(angle)
            y_rot = x_local * np.sin(angle) + y_local * np.cos(angle)
            p = Point()
            p.x = float(cx + x_rot)
            p.y = float(cy + y_rot)
            p.z = 0.05
            m.points.append(p)
        m.scale.x = 0.02
        m.color = color
        m.lifetime.sec = 0  # persist until overwritten
        return m


def main(args=None):
    rclpy.init(args=args)
    node = DriveTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
