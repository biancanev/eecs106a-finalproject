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
        self.timer = self.create_timer(1.0, self.move)  # Check every second
        self.start_time = time.time()  # Store the start time
        self.warmup_duration = 65.0
        self.moving = False  # Hold still until warmup finishes
        self.mode = 'circle'  # Default mode: slow circle so seeker can chase
        self.state = 'forwards'
        self.side_length = 2.0  # Side length for square movement (in meters)
        self.square_step = 0  # To track which side of the square we're on
        # Safety stop on obstacles
        self.min_range = 0.45  # more conservative clearance
        self.last_scan = None
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # Wander parameters
        self.wander_heading = 0.0
        self.wander_speed = 0.015
        self.wander_last_change = time.time()
        self.wander_interval = 3.0  # seconds between heading changes

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

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
            elif self.mode == 'wander':
                self.move_wander()
        else:
            remaining = max(int(self.warmup_duration - (current_time - self.start_time)), 0)
            self.get_logger().info(f"Waiting to start movement... {remaining} seconds remaining.")
            # Ensure stop
            self.cmd_pub.publish(Twist())

    def safe_to_move(self):
        if self.last_scan is None:
            return True
        rngs = [r for r in self.last_scan.ranges if np.isfinite(r)]
        if len(rngs) == 0:
            return True
        return min(rngs) > self.min_range

    def move_circle(self):
        if not self.safe_to_move():
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("Obstacle too close; stopping target.")
            return
        twist = Twist()
        # Slow circle to stay within view
        twist.linear.x = 0.0
        twist.angular.z = 0.0
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

    def move_wander(self):
        """Gentle random walk to make the chase interesting but safe."""
        if not self.safe_to_move():
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("Obstacle too close; stopping target.")
            return

        now = time.time()
        if now - self.wander_last_change > self.wander_interval:
            delta_heading = np.deg2rad(np.random.uniform(-4, 4))
            # Damp old heading so we don't drift far
            self.wander_heading = 0.3 * self.wander_heading + delta_heading
            # Keep heading very small to stay within a tight radius and bias outward if near seeker
            self.wander_heading = float(np.clip(self.wander_heading, -0.10, 0.10))
            self.wander_speed = float(np.clip(self.wander_speed + np.random.uniform(-0.005, 0.005), 0.012, 0.025))
            self.wander_last_change = now
            self.get_logger().info(
                f"Wander update: heading delta={math.degrees(delta_heading):.1f}°, speed={self.wander_speed:.2f} m/s"
            )

        twist = Twist()
        twist.linear.x = self.wander_speed
        twist.angular.z = float(np.clip(self.wander_heading, -0.2, 0.2))
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
