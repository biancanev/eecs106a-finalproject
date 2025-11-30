#!/usr/bin/env python3
"""
MPC Node for real-time trajectory planning
Based on the paper implementation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np
import transforms3d.euler as euler
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # Subscriptions
        self.seeker_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.seeker_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/target_estimate',
            self.target_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.seeker_pose = None
        self.seeker_cov = None
        self.target_pose = None
        self.target_cov = None
        self.map = None
        self.seeker_state = None  # [px, py, theta, v]

        # MPC parameters
        self.dt = 0.1
        self.N = 15  # MPC horizon
        self.mpc = SimpleUnicycleMPC(horizon=self.N, dt=self.dt)

        # Control limits
        self.v_max_base = 0.6
        self.v_min = 0.0

        # Timer for MPC updates
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('MPC node initialized')

    def map_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.map = msg

    def seeker_callback(self, msg: PoseWithCovarianceStamped):
        """Update seeker state"""
        self.seeker_pose = msg.pose.pose
        self.seeker_cov = np.array(msg.pose.covariance).reshape((6, 6))
        
        # Extract state [px, py, theta, v]
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Estimate velocity from previous state (simple approach)
        if self.seeker_state is not None:
            dt = self.dt
            dx = msg.pose.pose.position.x - self.seeker_state[0]
            dy = msg.pose.pose.position.y - self.seeker_state[1]
            v = np.sqrt(dx**2 + dy**2) / dt if dt > 0 else 0.0
            v = np.clip(v, self.v_min, self.v_max_base)
        else:
            v = 0.0
        
        self.seeker_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            v
        ])

    def target_callback(self, msg: PoseWithCovarianceStamped):
        """Update target state"""
        self.target_pose = msg.pose.pose
        self.target_cov = np.array(msg.pose.covariance).reshape((6, 6))

    def compute_obstacles(self):
        """Extract obstacles from map and inflate based on uncertainty"""
        obstacles = []
        if self.map is None:
            return obstacles
        
        # Get uncertainty measure
        if self.seeker_cov is not None:
            sigma_seek = np.sqrt(np.max(np.linalg.eigvals(self.seeker_cov[:2, :2])))
        else:
            sigma_seek = 0.1
        
        k_sigma = 2.0  # Tuning parameter from paper
        inflation = k_sigma * sigma_seek
        
        # Extract obstacles from occupancy grid
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # Simple obstacle extraction: find occupied cells
        for i in range(width * height):
            if self.map.data[i] > 50:  # Occupied
                x = (i % width) * resolution + origin_x
                y = (i // width) * resolution + origin_y
                r_eff = resolution * np.sqrt(2) + inflation  # Base radius + inflation
                obstacles.append((np.array([x, y]), r_eff))
        
        return obstacles

    def predict_target_trajectory(self):
        """Predict target trajectory over MPC horizon"""
        if self.target_pose is None:
            return None
        
        # Simple constant velocity prediction
        # In full implementation, would use KF prediction
        px_tgt = self.target_pose.position.x
        py_tgt = self.target_pose.position.y
        
        # If we have target covariance, we could predict velocity
        # For now, assume target is stationary
        target_seq = np.zeros((2, self.N + 1))
        target_seq[0, :] = px_tgt
        target_seq[1, :] = py_tgt
        
        return target_seq

    def timer_callback(self):
        """Main MPC control loop"""
        if self.seeker_state is None or self.target_pose is None:
            return

        # Build initial state
        x0 = self.seeker_state.copy()

        # Predict target trajectory
        target_seq = self.predict_target_trajectory()
        if target_seq is None:
            return

        # Compute obstacles with uncertainty inflation
        obstacles = self.compute_obstacles()

        # Adjust speed limit based on uncertainty (from paper)
        if self.seeker_cov is not None:
            sigma_seek = np.sqrt(np.max(np.linalg.eigvals(self.seeker_cov[:2, :2])))
            alpha = 1.0
            v_max = self.v_max_base * np.exp(-alpha * sigma_seek)
            self.mpc.v_max = np.clip(v_max, self.v_min, self.v_max_base)
        else:
            self.mpc.v_max = self.v_max_base

        # Solve MPC - use Twist command format
        try:
            twist_cmd = self.mpc.get_twist_command(x0, target_seq, obstacles)
            v_cmd = twist_cmd['linear']['x']
            omega_cmd = twist_cmd['angular']['z']
        except Exception as e:
            self.get_logger().error(f"MPC solve failed: {e}")
            # Fallback: stop
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        # Publish command in Twist format
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(omega_cmd)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

