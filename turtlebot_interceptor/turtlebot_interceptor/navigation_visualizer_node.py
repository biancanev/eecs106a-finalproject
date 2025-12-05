#!/usr/bin/env python3
"""
Navigation Visualizer Node
Publishes RViz markers to visualize robot navigation progress:
- Robot pose and orientation
- Goal point
- Path history
- Distance to goal
- Planned trajectory (if available)
Overlays on top of voxel grid map in RViz
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import transforms3d.euler as euler


class NavigationVisualizerNode(Node):
    """Visualizes robot navigation progress in RViz"""
    
    def __init__(self):
        super().__init__('navigation_visualizer_node')
        
        # Declare parameters
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('robot_radius', 0.15)  # Robot footprint radius
        self.declare_parameter('path_history_length', 100)  # Number of poses to keep
        # use_sim_time may be passed from launch file - declare only if not already set
        try:
            self.declare_parameter('use_sim_time', False)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass  # Parameter already declared by launch file
        
        # Get parameters
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.path_history_length = self.get_parameter('path_history_length').get_parameter_value().integer_value
        
        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # MCL estimated pose
            self.pose_callback,
            10
        )
        
        # Subscribe to odometry for true robot pose (if available)
        from nav_msgs.msg import Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/navigation_visualization',
            10
        )
        
        # State
        self.current_pose = None  # MCL estimated pose
        self.current_pose_cov = None  # MCL pose covariance
        self.true_pose = None  # True robot pose from odometry
        self.path_history = []  # List of (x, y) tuples
        self.marker_id = 0
        
        # Timer for publishing markers
        self.timer = self.create_timer(0.1, self.publish_markers)  # 10Hz
        
        self.get_logger().info(
            f'Navigation visualizer initialized - goal: ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update MCL estimated pose and path history"""
        self.current_pose = msg.pose.pose
        # Store covariance for uncertainty visualization
        self.current_pose_cov = np.array(msg.pose.covariance).reshape((6, 6))
        
        # Add to path history
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.path_history.append((x, y))
        
        # Limit path history length
        if len(self.path_history) > self.path_history_length:
            self.path_history.pop(0)
    
    def odom_callback(self, msg):
        """Update true robot pose from odometry"""
        # Store true pose for comparison
        self.true_pose = msg.pose.pose
    
    def map_callback(self, msg: OccupancyGrid):
        """Store map for reference (if needed)"""
        pass  # Map is already displayed, we just overlay on top
    
    def publish_markers(self):
        """Publish all visualization markers"""
        markers = MarkerArray()
        self.marker_id = 0
        
        # 1. Robot pose marker (arrow showing position and orientation)
        if self.current_pose is not None:
            robot_marker = self.create_robot_marker()
            if robot_marker:
                markers.markers.append(robot_marker)
        
        # 2. Goal point marker (sphere)
        goal_marker = self.create_goal_marker()
        markers.markers.append(goal_marker)
        
        # 3. Path history (line strip)
        if len(self.path_history) > 1:
            path_marker = self.create_path_marker()
            markers.markers.append(path_marker)
        
        # 4. Distance line (line from robot to goal)
        if self.current_pose is not None:
            distance_marker = self.create_distance_marker()
            if distance_marker:
                markers.markers.append(distance_marker)
        
        # 5. Text marker (distance to goal)
        if self.current_pose is not None:
            text_marker = self.create_text_marker()
            if text_marker:
                markers.markers.append(text_marker)
        
        # 6. MCL uncertainty ellipse
        if self.current_pose is not None and self.current_pose_cov is not None:
            uncertainty_marker = self.create_uncertainty_ellipse()
            if uncertainty_marker:
                markers.markers.append(uncertainty_marker)
        
        # 7. True robot pose (if available from odometry)
        if self.true_pose is not None:
            true_pose_marker = self.create_true_pose_marker()
            if true_pose_marker:
                markers.markers.append(true_pose_marker)
        
        # Publish all markers
        self.marker_pub.publish(markers)
    
    def create_robot_marker(self):
        """Create robot pose marker (arrow)"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position = self.current_pose.position
        marker.pose.orientation = self.current_pose.orientation
        
        # Scale (arrow size)
        marker.scale.x = self.robot_radius * 2.0  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height
        
        # Color (green for robot)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def create_goal_marker(self):
        """Create goal point marker (red sphere)"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(self.goal_x)
        marker.pose.position.y = float(self.goal_y)
        marker.pose.position.z = 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0
        
        # Scale (sphere size)
        marker.scale.x = 0.2  # Diameter
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color (red for goal)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def create_path_marker(self):
        """Create path history marker (blue line)"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'path'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Points along path
        marker.points = []
        for x, y in self.path_history:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.05  # Slightly above ground
            marker.points.append(point)
        
        # Scale (line width)
        marker.scale.x = 0.05  # Line width
        
        # Color (blue for path)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def create_distance_marker(self):
        """Create line from robot to goal"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'distance'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Points: robot to goal
        marker.points = []
        
        # Robot point
        robot_point = Point()
        robot_point.x = float(self.current_pose.position.x)
        robot_point.y = float(self.current_pose.position.y)
        robot_point.z = 0.1
        marker.points.append(robot_point)
        
        # Goal point
        goal_point = Point()
        goal_point.x = float(self.goal_x)
        goal_point.y = float(self.goal_y)
        goal_point.z = 0.1
        marker.points.append(goal_point)
        
        # Scale (line width)
        marker.scale.x = 0.03  # Thin line
        
        # Color (yellow for distance line)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def create_text_marker(self):
        """Create text marker showing distance to goal"""
        # Calculate distance
        dx = self.goal_x - self.current_pose.position.x
        dy = self.goal_y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'text'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (above robot)
        marker.pose.position.x = float(self.current_pose.position.x)
        marker.pose.position.y = float(self.current_pose.position.y)
        marker.pose.position.z = 0.5  # Above robot
        marker.pose.orientation.w = 1.0
        
        # Text
        marker.text = f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})\nDistance: {distance:.2f}m'
        
        # Scale (text size)
        marker.scale.z = 0.2  # Text height
        
        # Color (white text)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def create_uncertainty_ellipse(self):
        """Create uncertainty ellipse from MCL covariance"""
        # Extract 2D position covariance
        cov_2d = self.current_pose_cov[:2, :2]
        
        # Compute eigenvalues and eigenvectors
        eigenvals, eigenvecs = np.linalg.eig(cov_2d)
        eigenvals = np.maximum(eigenvals, 1e-6)  # Ensure positive
        
        # Create ellipse as line strip
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mcl_uncertainty'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Generate ellipse points
        num_points = 30
        angle = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])
        width = 2 * np.sqrt(eigenvals[0]) * 2.0  # 2-sigma
        height = 2 * np.sqrt(eigenvals[1]) * 2.0
        
        marker.points = []
        for i in range(num_points + 1):
            theta = 2 * np.pi * i / num_points
            x_local = width/2 * np.cos(theta)
            y_local = height/2 * np.sin(theta)
            
            # Rotate
            x_rot = x_local * np.cos(angle) - y_local * np.sin(angle)
            y_rot = x_local * np.sin(angle) + y_local * np.cos(angle)
            
            point = Point()
            point.x = float(self.current_pose.position.x + x_rot)
            point.y = float(self.current_pose.position.y + y_rot)
            point.z = 0.05
            marker.points.append(point)
        
        # Scale (line width)
        marker.scale.x = 0.02
        
        # Color (cyan for MCL uncertainty)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def create_true_pose_marker(self):
        """Create marker for true robot pose (from odometry)"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'true_pose'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position = self.true_pose.position
        marker.pose.orientation = self.true_pose.orientation
        
        # Scale (smaller than MCL estimate)
        marker.scale.x = self.robot_radius * 1.5
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        # Color (magenta for true pose)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.6)
        
        # Lifetime
        marker.lifetime.sec = 1
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = NavigationVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

