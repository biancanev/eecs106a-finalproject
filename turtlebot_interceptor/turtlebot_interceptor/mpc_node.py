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
        self.prev_time = None  # Previous timestamp for accurate velocity estimation

        # Initialize MPC
        self.mpc = SimpleUnicycleMPC(horizon=self.N, dt=self.dt)

        # Startup delay: Wait 10 seconds for LIDAR, SLAM, and MCL to initialize
        self.startup_time = self.get_clock().now()
        self.startup_delay = 10.0  # 10 seconds delay

        # Timer for MPC updates
        # Start timer immediately, but check startup delay in callback
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'MPC node initialized (lab8 control patterns) - '
            f'Waiting {self.startup_delay}s for LIDAR/SLAM/MCL initialization...'
        )

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
            # Use actual time difference from message timestamps for more accurate velocity
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if hasattr(self, 'prev_time') and self.prev_time is not None:
                dt_actual = current_time - self.prev_time
            else:
                dt_actual = self.dt  # Fallback to nominal dt
            
            dx = msg.pose.pose.position.x - self.prev_state[0]
            dy = msg.pose.pose.position.y - self.prev_state[1]
            # Use velocity in direction of motion (more accurate)
            v = np.sqrt(dx**2 + dy**2) / dt_actual if dt_actual > 0.01 else 0.0  # Min dt to avoid division by zero
            # Smooth velocity estimate (exponential moving average) - more aggressive smoothing
            v = 0.5 * v + 0.5 * self.prev_velocity  # More smoothing to reduce noise
            v = np.clip(v, self.v_min, self.v_max_base)
            self.prev_time = current_time
        else:
            v = 0.0
            self.prev_time = None
        
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
        """Extract circular obstacles (cones) from occupancy grid - based on lab6 patterns
        Returns obstacles in map frame (same frame as robot pose and goal)
        """
        obstacles = []
        if self.map is None:
            return obstacles
        
        # CRITICAL: Verify map is in 'map' frame
        if self.map.header.frame_id != 'map':
            self.get_logger().warn(
                f"Map frame_id is '{self.map.header.frame_id}', expected 'map'. "
                f"Obstacles may be in wrong coordinate frame!"
            )
        
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # DEBUG: Log map origin once
        if not hasattr(self, '_map_origin_logged'):
            self.get_logger().info(
                f"Map origin: ({origin_x:.3f}, {origin_y:.3f}), "
                f"resolution: {resolution:.3f}m, size: {width}x{height}, "
                f"frame_id: {self.map.header.frame_id}"
            )
            self._map_origin_logged = True
        
        # Extract occupied cells (not walls)
        # CRITICAL: Use lower threshold to catch more obstacles (50 might miss some)
        occupied_cells = []
        wall_margin = 0.35  # Cells near boundaries are walls
        occupancy_threshold = 30  # Lower threshold - catch more obstacles (was 50)
        
        for i in range(width * height):
            # Use lower threshold to catch more obstacles
            if self.map.data[i] > occupancy_threshold:  # Occupied (lowered from 50)
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
        
        # Count total occupied cells for debugging
        total_occupied = sum(1 for i in range(width * height) if self.map.data[i] > 50)
        
        # DEBUG: Log occupied cell count
        if not hasattr(self, '_occupied_cells_logged'):
            self.get_logger().info(
                f"Map has {total_occupied} occupied cells out of {width * height} total cells, "
                f"{len(occupied_cells)} non-wall occupied cells"
            )
            self._occupied_cells_logged = True
        
        if len(occupied_cells) == 0:
            if total_occupied > 0:
                self.get_logger().warn(
                    f"All {total_occupied} occupied cells are near walls (wall_margin={wall_margin}m). "
                    f"Consider reducing wall_margin or checking map origin."
                )
            return obstacles
        
        # Cluster nearby cells (cones are compact circular clusters)
        # CRITICAL: Use larger clustering distance for better obstacle detection
        # With 0.02m resolution, resolution*3 = 0.06m is too small
        # Use 0.15m clustering distance (about 7-8 cells at 0.02m resolution)
        cluster_distance = 0.15  # 15cm - good for small obstacles like cones
        
        clusters = []
        for x, y in occupied_cells:
            assigned = False
            for cluster in clusters:
                # Check distance to cluster center (more efficient)
                cluster_points = np.array(cluster)
                cluster_center = np.mean(cluster_points, axis=0)
                dist = np.sqrt((x - cluster_center[0])**2 + (y - cluster_center[1])**2)
                if dist < cluster_distance:
                    cluster.append((x, y))
                    assigned = True
                    break
            if not assigned:
                clusters.append([(x, y)])
        
        # Extract circular obstacles from clusters
        # CRITICAL: Improved obstacle extraction with better radius calculation
        for cluster in clusters:
            if len(cluster) < 2:  # Reduced from 3 - allow smaller obstacles
                continue
            
            points = np.array(cluster)
            min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
            min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
            width_cluster = max_x - min_x
            height_cluster = max_y - min_y
            
            # Skip very small clusters (noise)
            if width_cluster < 0.005 or height_cluster < 0.005:  # 5mm minimum
                continue
            
            # Skip elongated clusters (walls) - but be less strict
            if width_cluster > 0.01 and height_cluster > 0.01:  # Both dimensions significant
                aspect_ratio = max(width_cluster, height_cluster) / min(width_cluster, height_cluster)
                if aspect_ratio > 3.0:  # Increased from 2.5 - be less strict
                    continue
            
            # Compute center (use mean for better accuracy)
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            
            # Compute radius more accurately - use distance from center to farthest point
            distances_from_center = np.sqrt((points[:, 0] - center_x)**2 + (points[:, 1] - center_y)**2)
            max_dist = np.max(distances_from_center)
            radius = max_dist + resolution * 2  # Add 2 cells margin for safety
            
            # Also use bounding box as fallback
            radius_bbox = max(width_cluster, height_cluster) / 2 + resolution
            radius = max(radius, radius_bbox)  # Use larger of the two
            
            # Accept reasonable obstacle sizes (cones are typically 0.1-0.2m radius)
            if 0.05 < radius < 0.6:  # Expanded range - was 0.06-0.5
                obstacles.append((np.array([center_x, center_y]), radius))
        
        return obstacles[:10]  # Limit to 10 cones max
    
    def get_occupied_cells_as_obstacles(self, robot_pos, lookahead_dist=2.0):
        """
        Get ALL occupied cells near the robot as obstacles.
        This ensures the robot never hits any occupied cell.
        
        Args:
            robot_pos: (x, y) robot position in map frame
            lookahead_dist: Only consider cells within this distance (m)
        
        Returns:
            List of (center, radius) tuples for each occupied cell
        """
        obstacles = []
        if self.map is None:
            return obstacles
        
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # Robot safety radius (robot radius + margin)
        robot_radius = 0.15  # Robot radius
        safety_margin = 0.1  # Additional safety margin
        cell_radius = resolution * np.sqrt(2) / 2  # Half diagonal of cell (worst case)
        min_obstacle_radius = robot_radius + safety_margin + cell_radius
        
        # Extract occupied cells near robot
        occupancy_threshold = 30  # Same as extract_cones_from_map
        
        for i in range(width * height):
            if self.map.data[i] > occupancy_threshold:  # Occupied
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                
                # Only consider cells within lookahead distance
                dist_to_robot = np.sqrt((world_x - robot_pos[0])**2 + (world_y - robot_pos[1])**2)
                if dist_to_robot > lookahead_dist:
                    continue
                
                # Add as obstacle with minimum radius to ensure robot doesn't hit it
                obstacles.append((np.array([world_x, world_y]), min_obstacle_radius))
        
        return obstacles
    
    def compute_obstacles(self):
        """Extract obstacles from map - use occupied cells directly for guaranteed avoidance"""
        if self.seeker_state is None:
            return []
        
        # Get robot position from seeker_state [px, py, theta, v]
        robot_x = self.seeker_state[0]
        robot_y = self.seeker_state[1]
        
        # Get all occupied cells as obstacles
        return self.get_occupied_cells_as_obstacles((robot_x, robot_y), lookahead_dist=3.0)

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
        # Check startup delay - wait 10 seconds for initialization
        elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            # Log countdown every 2 seconds
            if not hasattr(self, '_startup_log_count'):
                self._startup_log_count = 0
            self._startup_log_count += 1
            if self._startup_log_count % 20 == 0:  # Every 2 seconds at 10Hz
                remaining = self.startup_delay - elapsed
                self.get_logger().info(
                    f'MPC startup delay: {remaining:.1f}s remaining for LIDAR/SLAM/MCL initialization...'
                )
            return  # Don't run MPC until startup delay is over
        
        if self.seeker_state is None:
            return
        
        # For single robot navigation, use goal point if target not available
        use_goal = (self.target_pose is None)

        # Build initial state
        x0 = self.seeker_state.copy()
        
        # CRITICAL DEBUG: Log state and goal
        if not hasattr(self, '_debug_count'):
            self._debug_count = 0
        self._debug_count += 1
        if self._debug_count % 20 == 0:  # Every 2 seconds
            self.get_logger().error(
                f"DEBUG: x0=[{x0[0]:.3f}, {x0[1]:.3f}, {np.degrees(x0[2]):.1f}°, {x0[3]:.3f}m/s], "
                f"goal_x={self.goal_x}, goal_y={self.goal_y}"
            )

        # Get target/goal position
        if use_goal:
            # Use goal point for navigation
            target_pos = np.array([self.goal_x, self.goal_y])
            # Create constant target sequence
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = target_pos[0]
            target_seq[1, :] = target_pos[1]
            
            # CRITICAL DEBUG: Verify target sequence
            if self._debug_count % 20 == 0:
                self.get_logger().error(
                    f"DEBUG: target_seq[0,0]={target_seq[0,0]:.3f}, target_seq[1,0]={target_seq[1,0]:.3f}, "
                    f"dx={target_seq[0,0]-x0[0]:.3f}, dy={target_seq[1,0]-x0[1]:.3f}"
                )
        else:
            # Predict target trajectory
            target_seq = self.predict_target_trajectory()
            if target_seq is None:
                return

        # Compute obstacles with uncertainty inflation
        obstacles = self.compute_obstacles()  # Enable obstacle avoidance
        
        # DEBUG: Log obstacles periodically - MORE FREQUENT
        if not hasattr(self, '_obstacle_debug_count'):
            self._obstacle_debug_count = 0
        self._obstacle_debug_count += 1
        if self._obstacle_debug_count % 10 == 0:  # Every 1 second
            total_occupied = np.sum(np.array(self.map.data) > 30) if self.map else 0
            if obstacles and len(obstacles) > 0:
                self.get_logger().error(  # ERROR level so it's visible
                    f"OCCUPIED CELL OBSTACLES: Found {len(obstacles)} occupied cells as obstacles. "
                    f"Total occupied cells in map: {total_occupied}, "
                    f"Map frame_id: {self.map.header.frame_id if self.map else 'None'}, "
                    f"Robot pose: ({x0[0]:.3f}, {x0[1]:.3f})"
                )
                # Log closest obstacles
                obstacle_dists = [(np.sqrt((center[0] - x0[0])**2 + (center[1] - x0[1])**2), center, radius) 
                                 for center, radius in obstacles]
                obstacle_dists.sort()
                for i, (dist, center, radius) in enumerate(obstacle_dists[:5]):  # Log 5 closest
                    self.get_logger().error(
                        f"  Obstacle {i+1}: center=({center[0]:.3f}, {center[1]:.3f}), "
                        f"radius={radius:.3f}m, dist_to_robot={dist:.3f}m"
                    )
            else:
                self.get_logger().warn(
                    f"NO OCCUPIED CELLS NEAR ROBOT! Map has {total_occupied} total occupied cells "
                    f"(threshold=30, lookahead=3.0m)"
                )

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
            
            # CRITICAL DEBUG: Log everything to find the bug
            if not hasattr(self, '_mpc_cmd_count'):
                self._mpc_cmd_count = 0
            self._mpc_cmd_count += 1
            if self._mpc_cmd_count % 10 == 0:  # Every 1 second - MORE FREQUENT
                angle_to_goal = np.arctan2(target_seq[1,0] - x0[1], target_seq[0,0] - x0[0])
                angle_err = angle_to_goal - x0[2]
                angle_err = np.mod(angle_err + np.pi, 2*np.pi) - np.pi
                dist = np.sqrt((target_seq[0,0] - x0[0])**2 + (target_seq[1,0] - x0[1])**2)
                dx = target_seq[0,0] - x0[0]
                dy = target_seq[1,0] - x0[1]
                # Check if commands make sense
                self.get_logger().error(  # ERROR level so it's visible
                    f"MPC: robot=({x0[0]:.3f}, {x0[1]:.3f}, {np.degrees(x0[2]):.1f}°), v={x0[3]:.3f}, "
                    f"goal=({target_seq[0,0]:.3f}, {target_seq[1,0]:.3f}), "
                    f"dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f}m, "
                    f"angle_to_goal={np.degrees(angle_to_goal):.1f}°, angle_err={np.degrees(angle_err):.1f}°, "
                    f"v_cmd={v_cmd:.3f}, omega_cmd={np.degrees(omega_cmd):.1f}°"
                )
            
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

