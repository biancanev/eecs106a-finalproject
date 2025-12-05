#!/usr/bin/env python3
"""
MPC Node for real-time trajectory planning
Based on the paper implementation
"""
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import transforms3d.euler as euler
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # Declare parameters (lab4 + lab8 pattern)
        self.declare_parameter('mpc_horizon', 15)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('v_max_base', 0.6)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('omega_max', 1.5)
        # use_sim_time may be passed from launch file - declare only if not already set
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass  # Parameter already declared by launch file
        # Fallback control gains (lab8 pattern)
        self.declare_parameter('Kp_v', 2.0)
        self.declare_parameter('Kp_w', 0.8)
        self.declare_parameter('Kd_w', 0.5)
        
        # Get parameters
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.N = self.get_parameter('mpc_horizon').get_parameter_value().integer_value
        self.v_max_base = self.get_parameter('v_max_base').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.omega_max = self.get_parameter('omega_max').get_parameter_value().double_value
        # Fallback control gains
        self.Kp_v = self.get_parameter('Kp_v').get_parameter_value().double_value
        self.Kp_w = self.get_parameter('Kp_w').get_parameter_value().double_value
        self.Kd_w = self.get_parameter('Kd_w').get_parameter_value().double_value

        # Subscriptions
        self.seeker_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.seeker_callback,
            10
        )

        # Optional: Target subscription (for interception mode)
        # For single robot navigation, use goal point instead
        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.5)
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        
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
        self.prev_state = None  # Previous state for velocity estimation
        self.prev_velocity = 0.0  # Previous velocity (lab8 pattern)

        # Initialize MPC
        self.mpc = SimpleUnicycleMPC(horizon=self.N, dt=self.dt)

        # Timer for MPC updates
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('MPC node initialized (lab8 control patterns)')

    def map_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.map = msg

    def seeker_callback(self, msg: PoseWithCovarianceStamped):
        """Update seeker state (lab8 pattern - improved velocity estimation)"""
        self.seeker_pose = msg.pose.pose
        self.seeker_cov = np.array(msg.pose.covariance).reshape((6, 6))
        
        # Extract state [px, py, theta, v]
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
        
        # Store pose for state update
        self.seeker_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            0.0  # Velocity will be updated below
        ])
        
        # Estimate velocity from previous state (lab8 pattern - more robust)
        if self.prev_state is not None:
            dt = self.dt
            dx = msg.pose.pose.position.x - self.prev_state[0]
            dy = msg.pose.pose.position.y - self.prev_state[1]
            # Use velocity in direction of motion (more accurate)
            v = np.sqrt(dx**2 + dy**2) / dt if dt > 0 else 0.0
            # Smooth velocity estimate (exponential moving average)
            v = 0.7 * v + 0.3 * self.prev_velocity
            v = np.clip(v, self.v_min, self.v_max_base)
        else:
            v = 0.0
        
        # Store previous state for next iteration
        self.prev_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ])
        self.prev_velocity = v
        
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

    def extract_cones_from_map(self):
        """Extract circular obstacles (cones) from occupancy grid - based on lab6 patterns"""
        obstacles = []
        if self.map is None:
            return obstacles
        
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # Extract occupied cells (not walls)
        occupied_cells = []
        wall_margin = 0.35  # Cells near boundaries are walls
        
        for i in range(width * height):
            if self.map.data[i] > 50:  # Occupied
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x
                world_y = gy * resolution + origin_y
                
                # Skip walls
                if (world_x < origin_x + wall_margin or 
                    world_x > origin_x + width * resolution - wall_margin or
                    world_y < origin_y + wall_margin or 
                    world_y > origin_y + height * resolution - wall_margin):
                    continue
                
                occupied_cells.append((world_x, world_y))
        
        if len(occupied_cells) == 0:
            return obstacles
        
        # Cluster nearby cells (cones are compact circular clusters)
        clusters = []
        for x, y in occupied_cells:
            assigned = False
            for cluster in clusters:
                for cx, cy in cluster:
                    if np.sqrt((x - cx)**2 + (y - cy)**2) < resolution * 3:
                        cluster.append((x, y))
                        assigned = True
                        break
                if assigned:
                    break
            if not assigned:
                clusters.append([(x, y)])
        
        # Extract circular obstacles from clusters
        for cluster in clusters:
            if len(cluster) < 3:  # Too small
                continue
            
            points = np.array(cluster)
            min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
            min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
            width_cluster = max_x - min_x
            height_cluster = max_y - min_y
            
            # Skip elongated clusters (walls)
            if width_cluster < 0.01 or height_cluster < 0.01:
                continue
            aspect_ratio = max(width_cluster, height_cluster) / min(width_cluster, height_cluster)
            if aspect_ratio > 2.5:
                continue
            
            # Compute center and radius
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            radius = max(width_cluster, height_cluster) / 2 + 0.02
            
            # Accept reasonable cone sizes
            if 0.06 < radius < 0.5:
                obstacles.append((np.array([center_x, center_y]), radius))
        
        return obstacles[:10]  # Limit to 10 cones max
    
    def compute_obstacles(self):
        """Extract obstacles from map - use cone extraction for better detection"""
        return self.extract_cones_from_map()

    def predict_target_trajectory(self):
        """Predict target trajectory over MPC horizon (lab8 pattern - improved prediction)"""
        if self.target_pose is None:
            return None
        
        px_tgt = self.target_pose.position.x
        py_tgt = self.target_pose.position.y
        
        # If we have target covariance, extract velocity estimate
        # Target state from UKF: [px, py, vx, vy] in covariance
        # Check if velocity information is available in covariance
        if self.target_cov is not None:
            # Try to extract velocity from covariance (if UKF provides it)
            # For now, use simple constant velocity prediction
            # In full implementation, would use UKF predicted trajectory
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = px_tgt
            target_seq[1, :] = py_tgt
        else:
            # No velocity info - assume stationary
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = px_tgt
            target_seq[1, :] = py_tgt
        
        return target_seq

    def timer_callback(self):
        """Main MPC control loop (lab8 pattern - with fallback control)"""
        if self.seeker_state is None:
            return
        
        # For single robot navigation, use goal point if target not available
        use_goal = (self.target_pose is None)

        # Build initial state
        x0 = self.seeker_state.copy()

        # Get target/goal position
        if use_goal:
            # Use goal point for navigation
            target_pos = np.array([self.goal_x, self.goal_y])
            # Create constant target sequence
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = target_pos[0]
            target_seq[1, :] = target_pos[1]
        else:
            # Predict target trajectory
            target_seq = self.predict_target_trajectory()
            if target_seq is None:
                return

        # Compute obstacles with uncertainty inflation
        # TEMPORARY: Disable obstacles to get MPC working
        # The obstacle cost rebuilding causes solver issues
        obstacles = None  # self.compute_obstacles()  # Disabled for now

        # Adjust speed limit based on uncertainty (from paper)
        if self.seeker_cov is not None:
            sigma_seek = np.sqrt(np.max(np.linalg.eigvals(self.seeker_cov[:2, :2])))
            alpha = 1.0
            v_max = self.v_max_base * np.exp(-alpha * sigma_seek)
            self.mpc.v_max = np.clip(v_max, self.v_min, self.v_max_base)
        else:
            self.mpc.v_max = self.v_max_base

        # Solve MPC (lab8 pattern - with fallback to proportional control)
        try:
            # Try MPC solve
            twist_cmd = self.mpc.get_twist_command(x0, target_seq, obstacles)
            v_cmd = twist_cmd['linear']['x']
            omega_cmd = twist_cmd['angular']['z']
            
            # Validate MPC solution
            if math.isnan(v_cmd) or math.isnan(omega_cmd) or \
               math.isinf(v_cmd) or math.isinf(omega_cmd):
                raise ValueError("MPC solution contains NaN or Inf")
            
            # Clip commands to safe limits
            v_cmd = np.clip(v_cmd, self.v_min, self.v_max_base)
            omega_cmd = np.clip(omega_cmd, -self.omega_max, self.omega_max)
            
        except Exception as e:
            # Log error details for debugging (but not every time to avoid spam)
            if not hasattr(self, '_mpc_error_count'):
                self._mpc_error_count = 0
            self._mpc_error_count += 1
            if self._mpc_error_count % 20 == 0:  # Every 2 seconds at 10Hz
                self.get_logger().warn(
                    f"MPC solve failed: {e}, using fallback control. "
                    f"Robot: ({x0[0]:.2f}, {x0[1]:.2f}, {np.degrees(x0[2]):.1f}°), "
                    f"Goal: ({target_seq[0,0]:.2f}, {target_seq[1,0]:.2f})"
                )
            # Fallback to proportional control (lab8 pattern)
            v_cmd, omega_cmd = self.fallback_control(x0, target_seq)

        # Publish command in Twist format
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(omega_cmd)
        self.cmd_pub.publish(twist)
    
    def fallback_control(self, x0, target_seq):
        """Fallback proportional control (lab8 pattern)"""
        px, py, theta, v = x0
        
        # Get target position
        if target_seq.ndim == 2:
            tgt_x = target_seq[0, 0]
            tgt_y = target_seq[1, 0]
        else:
            tgt_x = target_seq[0]
            tgt_y = target_seq[1]
        
        # Compute error in world frame (map frame)
        dx = tgt_x - px
        dy = tgt_y - py
        dist = np.sqrt(dx**2 + dy**2)
        angle_to_target = np.arctan2(dy, dx)
        
        # Angle error (difference between desired heading and current heading)
        angle_err = angle_to_target - theta
        # Wrap to [-pi, pi] - CRITICAL: This ensures shortest rotation
        angle_err = np.mod(angle_err + np.pi, 2*np.pi) - np.pi
        
        # Debug logging (periodic)
        if not hasattr(self, '_fallback_log_count'):
            self._fallback_log_count = 0
        self._fallback_log_count += 1
        if self._fallback_log_count % 20 == 0:  # Every 2 seconds at 10Hz
            self.get_logger().info(
                f'Fallback: robot=({px:.2f}, {py:.2f}, {np.degrees(theta):.1f}°), '
                f'goal=({tgt_x:.2f}, {tgt_y:.2f}), dist={dist:.2f}m, '
                f'angle_to_target={np.degrees(angle_to_target):.1f}°, '
                f'angle_err={np.degrees(angle_err):.1f}°, '
                f'omega_cmd={np.degrees(self.Kp_w * angle_err):.1f}°/s'
            )
        
        # Proportional control
        # CRITICAL: Don't move forward if angle error is large (turn first)
        if abs(angle_err) > np.pi / 4:  # More than 45° off
            v_cmd = 0.0  # Stop and turn first
            omega_cmd = self.Kp_w * angle_err
        else:
            # Move forward and turn simultaneously
            v_cmd = self.Kp_v * min(dist, 1.0)  # Cap distance influence
            omega_cmd = self.Kp_w * angle_err
        
        # Clip to limits
        v_cmd = np.clip(v_cmd, self.v_min, self.v_max_base)
        omega_cmd = np.clip(omega_cmd, -self.omega_max, self.omega_max)
        
        return v_cmd, omega_cmd


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

