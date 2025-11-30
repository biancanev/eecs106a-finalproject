#!/usr/bin/env python3
"""
Target UKF Node (Legacy name - now uses UKF)
Hardware-ready ROS2 node for target tracking
"""
import rclpy
from turtlebot_interceptor.ukf_node import UKFNode


def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

