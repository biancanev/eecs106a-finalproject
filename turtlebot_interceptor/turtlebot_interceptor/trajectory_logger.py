#!/usr/bin/env python3
"""
TrajectoryLogger: lightweight post-processing helper.
- Subscribes to seeker EKF odometry, target estimate, and commanded twist.
- Logs to CSV for offline analysis and emits a short metrics summary at shutdown.
"""
import atexit
import csv
import math
import os
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        self.declare_parameter('log_path', 'log/trajectory_log.csv')
        self.declare_parameter('summary_path', 'log/trajectory_metrics.txt')
        self.declare_parameter('log_period', 0.2)

        self.log_path = self.get_parameter('log_path').get_parameter_value().string_value
        self.summary_path = self.get_parameter('summary_path').get_parameter_value().string_value
        self.log_period = self.get_parameter('log_period').get_parameter_value().double_value

        # Data stores
        self.seeker: Optional[Odometry] = None
        self.target: Optional[PoseWithCovarianceStamped] = None
        self.cmd: Optional[Twist] = None
        self.records = []
        self.last_log_time = time.time()

        # Subscriptions
        self.seeker_sub = self.create_subscription(Odometry, '/odom_ekf', self.seeker_cb, 10)
        self.target_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/target_estimate_slow', self.target_cb, 10
        )
        self.target_sub_fast = self.create_subscription(
            PoseWithCovarianceStamped, '/target_estimate', self.target_cb, 10
        )
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.timer = self.create_timer(self.log_period, self.tick)
        atexit.register(self.save_logs)
        self.get_logger().info(
            f'TrajectoryLogger writing to {self.log_path} (period={self.log_period}s)'
        )

    def seeker_cb(self, msg: Odometry):
        self.seeker = msg

    def target_cb(self, msg: PoseWithCovarianceStamped):
        self.target = msg

    def cmd_cb(self, msg: Twist):
        self.cmd = msg

    def tick(self):
        if self.seeker is None:
            return

        now = self.get_clock().now().to_msg()
        seeker_pose = self.seeker.pose.pose
        target_pose = self.target.pose.pose if self.target is not None else None

        yaw = self._yaw_from_quat(seeker_pose.orientation)
        vx_cmd = self.cmd.linear.x if self.cmd else 0.0
        wz_cmd = self.cmd.angular.z if self.cmd else 0.0

        if target_pose is not None:
            dx = target_pose.position.x - seeker_pose.position.x
            dy = target_pose.position.y - seeker_pose.position.y
            separation = math.hypot(dx, dy)
        else:
            separation = float('nan')

        record = {
            'stamp_sec': now.sec,
            'stamp_nanosec': now.nanosec,
            'seeker_x': seeker_pose.position.x,
            'seeker_y': seeker_pose.position.y,
            'seeker_yaw': yaw,
            'target_x': target_pose.position.x if target_pose else float('nan'),
            'target_y': target_pose.position.y if target_pose else float('nan'),
            'separation': separation,
            'cmd_vx': vx_cmd,
            'cmd_wz': wz_cmd,
        }
        self.records.append(record)

    def save_logs(self):
        if not self.records:
            return

        os.makedirs(os.path.dirname(self.log_path) or '.', exist_ok=True)
        with open(self.log_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=list(self.records[0].keys()))
            writer.writeheader()
            writer.writerows(self.records)

        metrics = self._compute_metrics()
        os.makedirs(os.path.dirname(self.summary_path) or '.', exist_ok=True)
        with open(self.summary_path, 'w') as f:
            for k, v in metrics.items():
                f.write(f'{k}: {v}\n')

        self.get_logger().info(
            f'TrajectoryLogger wrote {len(self.records)} samples to {self.log_path}. '
            f'Metrics: {metrics}'
        )

    def _compute_metrics(self):
        arr = self.records
        positions = np.array([[r['seeker_x'], r['seeker_y']] for r in arr])
        separations = np.array([r['separation'] for r in arr])
        vx = np.array([r['cmd_vx'] for r in arr])
        wz = np.array([r['cmd_wz'] for r in arr])

        path_length = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))) if len(positions) > 1 else 0.0
        mean_separation = float(np.nanmean(separations)) if np.any(np.isfinite(separations)) else float('nan')
        final_separation = float(separations[-1]) if len(separations) else float('nan')
        control_effort = float(np.sum(np.abs(vx)) * self.log_period)
        yaw_effort = float(np.sum(np.abs(wz)) * self.log_period)

        return {
            'path_length_m': round(path_length, 3),
            'mean_separation_m': round(mean_separation, 3) if math.isfinite(mean_separation) else 'nan',
            'final_separation_m': round(final_separation, 3) if math.isfinite(final_separation) else 'nan',
            'control_effort_vx_int': round(control_effort, 3),
            'control_effort_wz_int': round(yaw_effort, 3),
        }

    def _yaw_from_quat(self, q):
        # q is geometry_msgs/Quaternion
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_logs()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
