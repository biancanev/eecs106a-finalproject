#!/usr/bin/env python3
"""
Robust Target Estimator (Vision + Odometry Fusion)
- Fuses camera detections with odometry for accurate target pose
- Robust filtering and outlier rejection for noisy CV measurements
- Temporal smoothing and validation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import transforms3d.euler as euler
from collections import deque
import tf2_ros


class TargetEstimator(Node):
    def __init__(self):
        super().__init__('target_estimator')

        # Parameters
        self.declare_parameter('warm_start_x', 0.0)
        self.declare_parameter('warm_start_y', 0.0)
        self.declare_parameter('warm_start_yaw', 0.0)
        self.declare_parameter('visibility_timeout', 3.0)  # seconds to extrapolate after last detection
        self.declare_parameter('max_extrap_speed', 1.0)  # m/s cap on extrapolated speed
        self.declare_parameter('cov_pos_base', 0.05)
        self.declare_parameter('cov_yaw_base', 0.1)
        self.declare_parameter('use_odom_measurement', False)  # default off to avoid drifting translation when no vision
        self.declare_parameter('lost_cov_scale', 4.0)  # inflate covariance when target is out of sight
        self.declare_parameter('lock_confidence', 0.8)  # once reached, freeze estimate
        self.declare_parameter('warmup_duration', 65.0)  # seconds before force-lock

        # Cache frequently used parameter values
        self.cov_pos_base = self.get_parameter('cov_pos_base').get_parameter_value().double_value
        self.cov_yaw_base = self.get_parameter('cov_yaw_base').get_parameter_value().double_value
        self.max_extrap_speed = self.get_parameter('max_extrap_speed').get_parameter_value().double_value
        self.visibility_timeout = self.get_parameter('visibility_timeout').get_parameter_value().double_value

        # State variables
        self.state = np.array([self.get_parameter('warm_start_x').get_parameter_value().double_value,
                               self.get_parameter('warm_start_y').get_parameter_value().double_value], dtype=float)
        self.yaw = self.get_parameter('warm_start_yaw').get_parameter_value().double_value
        self.velocity = np.zeros(2)  # [vx, vy] in world frame
        
        # Full state vector for propagation: [x, y, vx, vy, theta, omega]
        # This allows us to propagate state forward when target is out of frame
        self.full_state = None  # Will be initialized on first detection
        self.state_covariance = None  # 6x6 covariance matrix
        self.last_propagation_time = None
        
        # Detection tracking
        self.last_detection_time = None
        self.last_detection_pos = None
        self.detection_history = deque(maxlen=10)  # Store recent detections for filtering
        self.history = deque(maxlen=5)
        
        # Odometry
        self.seeker_odom = None
        self.target_odom = None
        
        # Filtering parameters
        self.lost_cov_scale = self.get_parameter('lost_cov_scale').get_parameter_value().double_value
        self.lock_conf = self.get_parameter('lock_confidence').get_parameter_value().double_value
        self.warmup_duration = self.get_parameter('warmup_duration').get_parameter_value().double_value
        self.locked_pose = None
        self.start_time = self.get_clock().now()
        
        # Robust filtering
        self.alpha_position = 0.4  # Position smoothing
        self.alpha_orientation = 0.3  # Orientation smoothing
        self.filtered_state = None
        self.filtered_yaw = None
        
        # Outlier rejection
        self.max_jump_distance = 1.5  # meters - reject jumps larger than this
        self.min_detection_confidence = 0.3  # Minimum confidence to accept detection
        
        # Debug counters
        self.detection_count = 0
        self.rejected_count = 0

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_est', 10)
        self.pose_pub2 = self.create_publisher(PoseWithCovarianceStamped, '/target_est_from_maps', 10)
        self.meas_pub = self.create_publisher(PoseStamped, '/target_pose_measurement', 10)

        self.create_subscription(PointStamped, '/camera_cone_positions', self.cone_cb, 10)
        self.create_subscription(PointStamped, '/camera_target_positions', self.cone_cb, 10)  # blue cone target
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/target/odom', self.target_odom_cb, 10)

        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info("Target estimator (vision + warm start) ready")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to publish TF after 30 seconds
        self.timer2 = self.create_timer(30, self.publish_target_tf)

        self.start = False

    def publish_target_tf(self):
        if self.start:
            return
        # Get the target's pose from the calculated estimate
        if not hasattr(self, 'last_target_pose'):
            self.get_logger().warning("No target pose available for TF broadcast.")
            return

        pose_msg = self.last_target_pose  # Assuming this is the latest calculated pose
        
        # Create TransformStamped message
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'  # This is the parent frame
        transform.child_frame_id = 'target/odom'  # This is the child frame

        # Set position from pose (translation part)
        transform.transform.translation.x = pose_msg.pose.pose.position.x
        transform.transform.translation.y = pose_msg.pose.pose.position.y
        transform.transform.translation.z = pose_msg.pose.pose.position.z

        # Set orientation from pose (rotation part)
        transform.transform.rotation = pose_msg.pose.pose.orientation
        q = pose_msg.pose.pose.orientation
        roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])

        # Send the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Publishing transform from /odom to /target/odom: (x={transform.transform.translation.x}, y={transform.transform.translation.y}, yaw={yaw})")
        self.start = True

    def odom_cb(self, msg: Odometry):
        self.seeker_odom = msg

    def target_odom_cb(self, msg: Odometry):
        self.target_odom = msg

    def cone_cb(self, msg: PointStamped):
        """Handle camera cone detection with robust filtering"""
        # Allow updates even if locked - CV detections can refine the estimate
        # (removed early return to allow CV to update locked pose)
        
        # Extract detection position
        detection_pos = np.array([msg.point.x, msg.point.y])
        detection_time = self.get_clock().now()
        
        # Validate detection (check for outliers)
        if self.last_detection_pos is not None:
            jump_distance = np.linalg.norm(detection_pos - self.last_detection_pos)
            if jump_distance > self.max_jump_distance:
                self.rejected_count += 1
                self.get_logger().warn(
                    f"⚠️ Rejected CV detection: jump={jump_distance:.2f}m "
                    f"(max={self.max_jump_distance:.2f}m), rejected={self.rejected_count}"
                )
                return  # Reject outlier
        
        # Valid detection - update state
        self.detection_count += 1
        
        # Compute velocity if we have previous detection
        if self.last_detection_time is not None:
            dt = (detection_time.nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            if dt > 0.0 and dt < 5.0:  # Reasonable time window
                vel = (detection_pos - self.last_detection_pos) / dt
                speed = np.linalg.norm(vel)
                if speed > self.max_extrap_speed:
                    vel = vel * (self.max_extrap_speed / speed)
                # Smooth velocity estimate
                self.velocity = 0.6 * self.velocity + 0.4 * vel
        
        # Update state with temporal filtering
        if self.filtered_state is None:
            self.filtered_state = detection_pos.copy()
        else:
            # Apply exponential smoothing
            self.filtered_state = (1 - self.alpha_position) * self.filtered_state + self.alpha_position * detection_pos
        
        # Store detection
        self.state = detection_pos.copy()
        self.last_detection_pos = detection_pos.copy()
        self.last_detection_time = detection_time
        self.detection_history.append((detection_pos.copy(), detection_time))
        self.history.append(self.state.copy())
        
        # Update full state vector for propagation
        # Estimate heading from velocity direction if available
        if np.linalg.norm(self.velocity) > 0.1:
            estimated_heading = math.atan2(self.velocity[1], self.velocity[0])
        else:
            estimated_heading = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
        
        # Initialize or update full state: [x, y, vx, vy, theta, omega]
        if self.full_state is None:
            # Initialize full state
            self.full_state = np.array([
                detection_pos[0],
                detection_pos[1],
                self.velocity[0],
                self.velocity[1],
                estimated_heading,
                0.0  # omega (angular velocity) - will be estimated from heading changes
            ])
            # Initialize covariance (high uncertainty initially)
            self.state_covariance = np.diag([0.1, 0.1, 0.2, 0.2, 0.2, 0.1])
        else:
            # Update position and velocity
            self.full_state[0] = detection_pos[0]
            self.full_state[1] = detection_pos[1]
            self.full_state[2] = self.velocity[0]
            self.full_state[3] = self.velocity[1]
            
            # Update heading (smooth transition)
            heading_diff = estimated_heading - self.full_state[4]
            heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
            self.full_state[4] = self.full_state[4] + 0.3 * heading_diff
            self.full_state[4] = math.atan2(math.sin(self.full_state[4]), math.cos(self.full_state[4]))
            
            # Estimate angular velocity from heading change
            if self.last_propagation_time is not None:
                dt_prop = (detection_time.nanoseconds - self.last_propagation_time.nanoseconds) / 1e9
                if dt_prop > 0.0:
                    omega_est = heading_diff / dt_prop
                    self.full_state[5] = 0.7 * self.full_state[5] + 0.3 * omega_est
        
        self.last_propagation_time = detection_time
        
        # Log periodically
        if self.detection_count % 10 == 0:
            self.get_logger().info(
                f"✅ CV detection #{self.detection_count}: pos=({detection_pos[0]:.3f}, {detection_pos[1]:.3f}), "
                f"filtered=({self.filtered_state[0]:.3f}, {self.filtered_state[1]:.3f}), "
                f"vel=({self.velocity[0]:.3f}, {self.velocity[1]:.3f}), "
                f"heading={math.degrees(self.full_state[4]):.1f}°, rejected={self.rejected_count}"
            )

    def get_smoothed_position(self):
        """Return a robust position estimate using median of recent detections."""
        if len(self.detection_history) == 0:
            return None
        points = np.array([p for p, _ in self.detection_history])
        return np.median(points, axis=0)
    
    def propagate_state(self, dt):
        """
        Propagate target state forward using constant velocity model.
        Used when target is out of frame to maintain continuous tracking.
        
        State: [x, y, vx, vy, theta, omega]
        """
        if self.full_state is None:
            return None, None
        
        # Constant velocity propagation model
        x, y, vx, vy, theta, omega = self.full_state
        
        # Propagate position
        x_new = x + dt * vx
        y_new = y + dt * vy
        
        # Propagate heading
        theta_new = theta + dt * omega
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
        
        # Velocity and omega remain constant (constant velocity model)
        # In reality, they may change, but we model this as process noise
        
        propagated_state = np.array([x_new, y_new, vx, vy, theta_new, omega])
        
        # Propagate covariance: P_{k+1} = F_k P_k F_k^T + Q_k
        if self.state_covariance is not None:
            # State transition matrix (Jacobian of propagation model)
            F = np.array([
                [1, 0, dt, 0, 0, 0],
                [0, 1, 0, dt, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 1]
            ])
            
            # Process noise (increases with time since last detection)
            # Base noise scales
            base_noise = np.array([0.01, 0.01, 0.05, 0.05, 0.02, 0.05])
            # Increase noise over time (uncertainty grows when target is lost)
            time_since_detection = dt
            noise_scale = 1.0 + 0.5 * time_since_detection  # Grow uncertainty over time
            Q = np.diag(base_noise * noise_scale)
            
            # Covariance propagation
            propagated_cov = F @ self.state_covariance @ F.T + Q
        else:
            propagated_cov = None
        
        return propagated_state, propagated_cov

    def tick(self):
        """Main update loop with robust pose estimation"""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # CRITICAL: Trust warm start - user gave us a good guess!
        # Check if warm start is non-zero (user actually provided it)
        warm_start_provided = (abs(self.state[0]) > 0.01 or abs(self.state[1]) > 0.01)
        
        # Determine which state to use (filtered if available, otherwise raw/warm start)
        pos_use = self.filtered_state if self.filtered_state is not None else self.state
        
        # If we have recent detection, publish it
        if self.last_detection_time is not None:
            dt = (now.nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            
            if dt < self.visibility_timeout:
                # Recent detection - VERY HIGH confidence for stable detections
                yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
                
                # Update filtered yaw
                if self.filtered_yaw is None:
                    self.filtered_yaw = yaw_use
                else:
                    angle_diff = yaw_use - self.filtered_yaw
                    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                    self.filtered_yaw = self.filtered_yaw + self.alpha_orientation * angle_diff
                    self.filtered_yaw = math.atan2(math.sin(self.filtered_yaw), math.cos(self.filtered_yaw))
                
                pose = (pos_use[0], pos_use[1], self.filtered_yaw)
                
                # VERY HIGH confidence if we have multiple detections
                detection_stability = min(len(self.detection_history) / 10.0, 1.0)  # More detections = more stable
                base_confidence = 0.92  # Higher base confidence
                confidence = base_confidence + (1.0 - base_confidence) * detection_stability
                confidence = min(confidence, 0.98)  # Cap at 98% for safety
                
                # If we have many consistent detections, lock it
                if len(self.detection_history) >= 10 and detection_stability > 0.7:
                    if self.locked_pose is None:
                        self.locked_pose = pose
                        self.get_logger().info(
                            f"🔒 Locked target pose: ({pose[0]:.3f}, {pose[1]:.3f}), "
                            f"yaw={math.degrees(pose[2]):.1f}°, confidence={confidence:.2f}, "
                            f"detections={len(self.detection_history)}"
                        )
                
                self.publish_pose(pose, confidence=confidence)
                return
            else:
                # Lost sight: PROPAGATE STATE FORWARD using constant velocity model
                # This is critical for robust tracking when target wanders out of frame
                if self.full_state is not None and self.last_propagation_time is not None:
                    # Compute time since last detection
                    dt_prop = (now.nanoseconds - self.last_propagation_time.nanoseconds) / 1e9
                    
                    if dt_prop > 0.0 and dt_prop < 10.0:  # Reasonable propagation window
                        # Propagate state forward
                        propagated_state, propagated_cov = self.propagate_state(dt_prop)
                        
                        if propagated_state is not None:
                            # Update full state with propagated values
                            self.full_state = propagated_state
                            if propagated_cov is not None:
                                self.state_covariance = propagated_cov
                            
                            # Extract position and heading from propagated state
                            pos_propagated = np.array([propagated_state[0], propagated_state[1]])
                            heading_propagated = propagated_state[4]
                            
                            # Blend propagated position with last known position (smooth transition)
                            blend_factor = min(dt_prop / 2.0, 0.7)  # Gradually trust propagation more over time
                            pos_use = (1 - blend_factor) * pos_use + blend_factor * pos_propagated
                            
                            # Use propagated heading
                            yaw_use = heading_propagated
                            
                            # Update velocity estimate from propagated state
                            self.velocity = np.array([propagated_state[2], propagated_state[3]])
                            
                            # Confidence decays exponentially with time
                            confidence_decay_rate = 0.15  # per second
                            base_confidence = 0.6  # Start with moderate confidence
                            confidence = base_confidence * math.exp(-confidence_decay_rate * dt_prop)
                            confidence = max(confidence, 0.2)  # Minimum confidence floor
                            
                            self.get_logger().info(
                                f"🔄 State propagation: dt={dt_prop:.2f}s, "
                                f"propagated=({pos_propagated[0]:.3f}, {pos_propagated[1]:.3f}), "
                                f"vel=({self.velocity[0]:.3f}, {self.velocity[1]:.3f}), "
                                f"heading={math.degrees(heading_propagated):.1f}°, "
                                f"confidence={confidence:.2f}"
                            )
                        else:
                            # Fallback: use last known position
                            yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else (
                                self.filtered_yaw if self.filtered_yaw is not None else self.yaw
                            )
                            confidence = 0.3
                    else:
                        # Too long since last detection - use last known position
                        yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else (
                            self.filtered_yaw if self.filtered_yaw is not None else self.yaw
                        )
                        confidence = 0.2  # Very low confidence after long time
                else:
                    # No state to propagate - use last known position
                    yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else (
                        self.filtered_yaw if self.filtered_yaw is not None else self.yaw
                    )
                    confidence = 0.3
                
                pose = (pos_use[0], pos_use[1], yaw_use)
                self.publish_pose(pose, confidence=confidence)
                return

        # No detection yet: publish warm start with HIGH confidence if user provided it
        yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
        pose = (pos_use[0], pos_use[1], yaw_use)
        
        # Trust warm start - user knows where target is!
        if warm_start_provided:
            confidence = 0.75  # HIGH confidence for user-provided warm start
            self.get_logger().info(
                f"🔥 Using warm start: ({pos_use[0]:.3f}, {pos_use[1]:.3f}), confidence={confidence:.2f}"
            )
        else:
            confidence = 0.15  # Low confidence if no warm start provided
        
        self.publish_pose(pose, confidence=confidence)
        
        # Force-lock after warmup duration if not already locked
        if self.locked_pose is None and elapsed >= self.warmup_duration:
            # Use filtered state if available, otherwise raw/warm start
            pos_use = self.filtered_state if self.filtered_state is not None else self.state
            yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else self.yaw
            self.locked_pose = (pos_use[0], pos_use[1], yaw_use)
            self.get_logger().info(
                f"🔒 Warmup elapsed ({elapsed:.1f}s). Locking target pose at "
                f"({pos_use[0]:.2f}, {pos_use[1]:.2f}), yaw={math.degrees(yaw_use):.1f}°"
            )

    def yaw_from_odom(self):
        """Compute relative yaw from odom frames if available."""
        if self.seeker_odom is None or self.target_odom is None:
            return None
        
        try:
            syaw = self.odom_yaw(self.seeker_odom.pose.pose.orientation)
            tyaw = self.odom_yaw(self.target_odom.pose.pose.orientation)
            relative_yaw = self.wrap_angle(tyaw - syaw)
            
            # Also compute direction from seeker to target position for better accuracy
            seeker_pos = np.array([
                self.seeker_odom.pose.pose.position.x,
                self.seeker_odom.pose.pose.position.y
            ])
            target_pos = self.filtered_state if self.filtered_state is not None else self.state
            if target_pos is not None and len(target_pos) >= 2:
                direction = target_pos[:2] - seeker_pos
                direction_yaw = math.atan2(direction[1], direction[0])
                # Blend odometry yaw with direction yaw (70% odom, 30% direction)
                relative_yaw = 0.7 * relative_yaw + 0.3 * direction_yaw
                relative_yaw = self.wrap_angle(relative_yaw)
            
            return relative_yaw
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error computing yaw from odom: {e}")
            return None

    def odom_yaw(self, q):
        """Extract yaw from geometry_msgs/Quaternion."""
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def wrap_angle(self, ang):
        """Wrap angle to [-pi, pi]."""
        return (ang + math.pi) % (2 * math.pi) - math.pi

    def publish_pose(self, pose, confidence):
        # If locked, hold that pose regardless of input confidence
        if self.locked_pose is not None:
            pose = self.locked_pose
            confidence = max(confidence, self.lock_conf)

        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        q = euler.euler2quat(0, 0, yaw)
        msg.pose.pose.orientation.x = q[1]
        msg.pose.pose.orientation.y = q[2]
        msg.pose.pose.orientation.z = q[3]
        msg.pose.pose.orientation.w = q[0]

        pos_var = self.get_parameter('cov_pos_base').get_parameter_value().double_value / max(confidence, 0.05)
        yaw_var = self.get_parameter('cov_yaw_base').get_parameter_value().double_value / max(confidence, 0.05)
        # Inflate covariance if target not recently seen
        if self.last_detection_time is None:
            pos_var *= self.lost_cov_scale
            yaw_var *= self.lost_cov_scale
        else:
            dt_since = (self.get_clock().now().nanoseconds - self.last_detection_time.nanoseconds) / 1e9
            if dt_since > self.visibility_timeout:
                pos_var *= self.lost_cov_scale
                yaw_var *= self.lost_cov_scale
        # If locked, shrink covariance (high confidence) and set lock
        if confidence >= self.lock_conf and self.locked_pose is None:
            self.locked_pose = (x, y, yaw)
        if self.locked_pose is not None:
            pos_var = min(pos_var, self.cov_pos_base * 0.1)
            yaw_var = min(yaw_var, self.cov_yaw_base * 0.1)
        msg.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_var
        ]

        self.pose_pub.publish(msg)
        self.pose_pub2.publish(msg)
        meas = PoseStamped()
        meas.header = msg.header
        meas.pose = msg.pose.pose
        self.meas_pub.publish(meas)


def main(args=None):
    rclpy.init(args=args)
    node = TargetEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
