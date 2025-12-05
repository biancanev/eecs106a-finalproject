#!/usr/bin/env python3
"""
Simple Pose Publisher for LIDAR Validation
Publishes a static or odometry-based pose for SLAM node

For LIDAR validation, we can use:
- Static pose (robot at origin)
- Odometry pose (from /odom topic)
- Manual pose (for testing)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import transforms3d.euler as euler


class SimplePosePublisher(Node):
    """Publishes robot pose for SLAM node - uses odometry or static pose"""
    
    def __init__(self):
        super().__init__('simple_pose_publisher')
        
        # Declare parameters
        self.declare_parameter('use_odom', True)  # Use odometry if available
        self.declare_parameter('static_pose', False)  # Use static pose at origin
        self.declare_parameter('use_sim_time', False)
        
        use_odom = self.get_parameter('use_odom').get_parameter_value().bool_value
        static_pose = self.get_parameter('static_pose').get_parameter_value().bool_value
        
        # Publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # SLAM node expects this topic
            10
        )
        
        # State
        self.current_pose = None
        
        if use_odom and not static_pose:
            # Subscribe to odometry
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
            self.get_logger().info('Simple pose publisher: Using /odom for pose')
        else:
            # Use static pose at origin
            self.publish_static_pose()
            self.get_logger().info('Simple pose publisher: Using static pose at origin')
    
    def odom_callback(self, msg: Odometry):
        """Convert odometry to PoseWithCovarianceStamped"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'  # SLAM expects map frame
        pose_msg.pose = msg.pose
        self.pose_pub.publish(pose_msg)
    
    def publish_static_pose(self):
        """Publish static pose at origin (for testing)"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        # Small covariance
        pose_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        self.pose_pub.publish(pose_msg)
        
        # Timer to keep publishing
        self.timer = self.create_timer(0.1, self.publish_static_pose)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

