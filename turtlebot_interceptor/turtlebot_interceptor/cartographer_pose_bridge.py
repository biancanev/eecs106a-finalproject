#!/usr/bin/env python3
"""
Cartographer Pose Bridge
Republishes Cartographer's tracked pose as /amcl_pose for compatibility with MPC node.

Cartographer publishes pose on /tracked_pose (PoseStamped)
MPC expects /amcl_pose (PoseWithCovarianceStamped)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class CartographerPoseBridge(Node):
    """Bridge Cartographer pose to AMCL pose topic"""
    
    def __init__(self):
        super().__init__('cartographer_pose_bridge')
        
        # Subscribe to Cartographer's tracked pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )
        
        # Publish as AMCL pose (for MPC compatibility)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            10
        )
        
        self.get_logger().info('Cartographer Pose Bridge: /tracked_pose -> /amcl_pose')
    
    def pose_callback(self, msg: PoseStamped):
        """Convert PoseStamped to PoseWithCovarianceStamped"""
        pose_with_cov = PoseWithCovarianceStamped()
        pose_with_cov.header = msg.header
        pose_with_cov.pose.pose = msg.pose
        
        # Set covariance (Cartographer is pretty accurate)
        # Diagonal: [x, y, z, roll, pitch, yaw]
        pose_with_cov.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance = 1cm
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # y variance = 1cm
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll, pitch (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05   # yaw variance = ~13 degrees
        ]
        
        self.pose_pub.publish(pose_with_cov)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

