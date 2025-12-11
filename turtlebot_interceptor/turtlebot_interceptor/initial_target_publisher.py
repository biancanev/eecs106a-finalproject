#!/usr/bin/env python3
"""
InitialTargetPublisher
- Publishes an initial target pose so the seeker has a goal immediately.
- Publishes to /target_estimate and /target_estimate_slow a few times at startup.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialTargetPublisher(Node):
    def __init__(self):
        super().__init__('initial_target_publisher')

        # Parameters
        self.declare_parameter('target_init_x', 0.0)
        self.declare_parameter('target_init_y', 0.0)
        self.declare_parameter('target_init_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('repeat_count', 5)
        self.declare_parameter('repeat_period', 0.5)

        self.x = self.get_parameter('target_init_x').get_parameter_value().double_value
        self.y = self.get_parameter('target_init_y').get_parameter_value().double_value
        self.yaw = self.get_parameter('target_init_yaw').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.repeat_count = int(self.get_parameter('repeat_count').get_parameter_value().integer_value)
        self.repeat_period = self.get_parameter('repeat_period').get_parameter_value().double_value

        # Publishers
        self.pub_fast = self.create_publisher(PoseWithCovarianceStamped, '/target_estimate', 10)
        self.pub_slow = self.create_publisher(PoseWithCovarianceStamped, '/target_estimate_slow', 10)

        # Publish immediately and then repeat a few times
        self.publish_pose()
        self.remaining = self.repeat_count - 1
        self.timer = self.create_timer(self.repeat_period, self.timer_cb)
        self.get_logger().info(
            f'Publishing initial target pose x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad'
        )

    def timer_cb(self):
        if self.remaining <= 0:
            self.timer.cancel()
            return
        self.publish_pose()
        self.remaining -= 1

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0

        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        cov = [0.0] * 36
        cov[0] = 0.05  # x
        cov[7] = 0.05  # y
        cov[35] = 0.1  # yaw
        msg.pose.covariance = cov

        self.pub_fast.publish(msg)
        self.pub_slow.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InitialTargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
