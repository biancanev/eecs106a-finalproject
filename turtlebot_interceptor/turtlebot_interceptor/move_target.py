import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time  # For time tracking
import transforms3d.euler as euler
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class MoveTarget(Node):
    def __init__(self):
        super().__init__('move_target')

        # Publishers
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.cmd_pub = self.create_publisher(Twist, '/target/cmd_vel', reliable_qos)
        
        # Timer
        self.timer = self.create_timer(2.0, self.move)  # Check every second
        self.start_time = time.time()  # Store the start time
        self.warmup_duration = 65.0
        self.moving = False  # Hold still until warmup finishes
        self.mode = 'circle'  # Default mode, change this to 'line' or 'square' as needed
        self.state = 'forwards'
        self.side_length = 2.0  # Side length for square movement (in meters)
        self.square_step = 0  # To track which side of the square we're on

    def move(self):
        current_time = time.time()

        # Only start moving after warmup
        if current_time - self.start_time >= self.warmup_duration and not self.moving:
            self.moving = True
            self.get_logger().info(f"{int(self.warmup_duration)} seconds have passed, starting movement.")

        if self.moving:
            if self.mode == 'circle':
                self.move_circle()
            elif self.mode == 'line':
                self.move_line()
            elif self.mode == 'square':
                self.move_square()
        else:
            remaining = max(int(self.warmup_duration - (current_time - self.start_time)), 0)
            self.get_logger().info(f"Waiting to start movement... {remaining} seconds remaining.")

    def move_circle(self):
        twist = Twist()
        twist.linear.x = 0.0  # Move forward with a constant speed 1.0
        twist.angular.z = 0.5  # Rotate at a constant rate 0.2
        self.cmd_pub.publish(twist)
        self.get_logger().info("Moving in a circle.")

    def move_line(self):
        twist = Twist()

        if self.state == 'forwards':
            twist.linear.x = 0.8
            twist.angular.z = 0.0 
            self.cmd_pub.publish(twist)
            self.state = 'backwards'


        elif self.state == 'backwards':
            twist.linear.x = -0.8
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.state = 'forwards'


    def move_square(self):
        # Move along one side of the square
        if self.square_step == 0:  # Moving forward on the first side
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Moving side {self.square_step + 1} of square.")
            # Stop and turn after a certain distance
            # You can implement a distance check here to stop and turn
            self.square_step += 1
        elif self.square_step == 1:  # Turn 90 degrees
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Rotate 90 degrees
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Turning at corner {self.square_step + 1} of square.")
            # After turning, we proceed to the next side
            self.square_step += 1
        elif self.square_step == 2:  # Moving second side
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Moving side {self.square_step + 1} of square.")
            self.square_step += 1
        elif self.square_step == 3:  # Turn 90 degrees again
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Rotate 90 degrees
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Turning at corner {self.square_step + 1} of square.")
            self.square_step += 1
        elif self.square_step == 4:  # Moving third side
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Moving side {self.square_step + 1} of square.")
            self.square_step += 1
        elif self.square_step == 5:  # Last turn and complete square
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Rotate 90 degrees
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Turning at corner {self.square_step + 1} of square.")
            self.square_step = 0  # Reset to start a new square

def main(args=None):
    rclpy.init(args=args)
    node = MoveTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
