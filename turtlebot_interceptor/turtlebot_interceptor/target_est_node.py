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
        self.declare_parameter('enable_lock', False)  # default: keep updating from CV even after warmup

        # Cache frequently used parameter values
        self.cov_pos_base = self.get_parameter('cov_pos_base').get_parameter_value().double_value
        self.cov_yaw_base = self.get_parameter('cov_yaw_base').get_parameter_value().double_value
        self.max_extrap_speed = self.get_parameter('max_extrap_speed').get_parameter_value().double_value
        self.visibility_timeout = self.get_parameter('visibility_timeout').get_parameter_value().double_value
        self.enable_lock = self.get_parameter('enable_lock').get_parameter_value().bool_value

        # State variables
        self.state = np.array([self.get_parameter('warm_start_x').get_parameter_value().double_value,
                               self.get_parameter('warm_start_y').get_parameter_value().double_value], dtype=float)
        self.yaw = self.get_parameter('warm_start_yaw').get_parameter_value().double_value
        self.velocity = np.zeros(2)
        
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
        self.alpha_position = 0.8  # Position smoothing (favor latest detection strongly)
        self.alpha_orientation = 0.3  # Orientation smoothing
        self.filtered_state = None
        self.filtered_yaw = None
        self.min_target_dist = None  # Track closest seen distance for monotonic approach
        self.last_robot_dist = None  # Last robot-to-detection distance
        
        # Outlier rejection
        self.max_jump_distance = 1.0  # meters - reject jumps larger than this (tighter for accuracy)
        self.min_detection_confidence = 0.3  # Minimum confidence to accept detection
        
        # Debug counters
        self.detection_count = 0
        self.rejected_count = 0

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_est', 10)
        self.pose_pub2 = self.create_publisher(PoseWithCovarianceStamped, '/target_est_from_maps', 10)
        self.meas_pub = self.create_publisher(PoseStamped, '/target_pose_measurement', 10)

        # Only use blue target detections for the target pose (yellow are obstacles)
        self.create_subscription(PointStamped, '/camera_target_positions', self.cone_cb, 10)  # blue cone target
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/target/odom', self.target_odom_cb, 10)

        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info("Target estimator (vision + warm start) ready")

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
        
        # If we have robot pose, enforce monotonic approach: distance should not increase as robot gets closer
        if self.seeker_odom is not None:
            robot_pos = np.array([self.seeker_odom.pose.pose.position.x,
                                  self.seeker_odom.pose.pose.position.y])
            dist_robot_det = np.linalg.norm(detection_pos - robot_pos)
            # Initialize trackers
            if self.min_target_dist is None:
                self.min_target_dist = dist_robot_det
            if self.last_robot_dist is None:
                self.last_robot_dist = dist_robot_det
            # If detection jumps farther while robot is closer, clamp along the line toward robot
            if dist_robot_det > self.last_robot_dist + 0.05:
                desired_dist = max(0.2, min(self.min_target_dist, self.last_robot_dist - 0.05))
                if np.linalg.norm(detection_pos - robot_pos) > 1e-3:
                    direction = (detection_pos - robot_pos) / np.linalg.norm(detection_pos - robot_pos)
                    detection_pos = robot_pos + direction * desired_dist
                    dist_robot_det = desired_dist
                    self.get_logger().info(
                        f"🔧 Clamped target range: was {dist_robot_det:.2f}m, now {desired_dist:.2f}m"
                    )
            # Update trackers
            self.min_target_dist = min(self.min_target_dist, dist_robot_det)
            self.last_robot_dist = dist_robot_det

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
        
        # Update state with temporal filtering (bias toward latest detection)
        if self.filtered_state is None:
            self.filtered_state = detection_pos.copy()
        else:
            # Apply faster exponential smoothing for quicker convergence
            self.filtered_state = (1 - self.alpha_position) * self.filtered_state + self.alpha_position * detection_pos
        
        # Store detection
        self.state = detection_pos.copy()
        self.last_detection_pos = detection_pos.copy()
        self.last_detection_time = detection_time
        self.detection_history.append((detection_pos.copy(), detection_time))
        self.history.append(self.state.copy())
        
        # If very close to the detection, aggressively pull lock toward it (final 0.8m)
        if self.seeker_odom is not None:
            robot_pos = np.array([
                self.seeker_odom.pose.pose.position.x,
                self.seeker_odom.pose.pose.position.y
            ])
            dist_robot_det = np.linalg.norm(detection_pos - robot_pos)
            if dist_robot_det < 0.8:
                # Strong blend toward detection for final approach
                if self.locked_pose is None:
                    self.locked_pose = (detection_pos[0], detection_pos[1], self.yaw)
                else:
                    lock_xy = np.array([self.locked_pose[0], self.locked_pose[1]])
                    new_lock = 0.8 * detection_pos + 0.2 * lock_xy
                    self.locked_pose = (new_lock[0], new_lock[1], self.locked_pose[2])
                # Also bias filtered state strongly to this detection
                self.filtered_state = detection_pos.copy()
                self.get_logger().info(
                    f"🎯 Final-approach snap: dist={dist_robot_det:.2f}m -> lock=({self.locked_pose[0]:.2f}, {self.locked_pose[1]:.2f})"
                )
        
        # If we already locked, allow small corrective nudges when closer (prevents freeze drift)
        if self.locked_pose is not None and self.seeker_odom is not None:
            robot_pos = np.array([
                self.seeker_odom.pose.pose.position.x,
                self.seeker_odom.pose.pose.position.y
            ])
            lock_xy = np.array([self.locked_pose[0], self.locked_pose[1]])
            lock_dist = np.linalg.norm(lock_xy - robot_pos)
            new_dist = np.linalg.norm(detection_pos - robot_pos)
            shift = np.linalg.norm(detection_pos - lock_xy)
            # Only nudge if detection is closer and within a reasonable jump
            if new_dist + 0.05 < lock_dist and shift < 0.8:
                blend = 0.6  # stronger nudge toward new detection
                new_lock = (1 - blend) * lock_xy + blend * detection_pos
                self.locked_pose = (new_lock[0], new_lock[1], self.locked_pose[2])
                self.get_logger().info(
                    f"🔧 Nudge locked pose toward CV: shift={shift:.2f}m, dist→{new_dist:.2f}m"
                )
        
        # Log periodically
        if self.detection_count % 10 == 0:
            self.get_logger().info(
                f"✅ CV detection #{self.detection_count}: pos=({detection_pos[0]:.3f}, {detection_pos[1]:.3f}), "
                f"filtered=({self.filtered_state[0]:.3f}, {self.filtered_state[1]:.3f}), "
                f"rejected={self.rejected_count}"
            )

    def get_smoothed_position(self):
        """Return a robust position estimate using median of recent detections."""
        if len(self.detection_history) == 0:
            return None
        points = np.array([p for p, _ in self.detection_history])
        return np.median(points, axis=0)

    def tick(self):
        """Main update loop with robust pose estimation"""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # CRITICAL: Trust warm start - user gave us a good guess!
        # Check if warm start is non-zero (user actually provided it)
        warm_start_provided = (abs(self.state[0]) > 0.01 or abs(self.state[1]) > 0.01)
        
        # Determine which state to use: prefer median of recent detections (robust), then filtered, then state
        pos_use = self.get_smoothed_position()
        if pos_use is None:
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
                # Lost sight: hold last measured position with lower confidence
                yaw_use = self.yaw_from_odom() if self.yaw_from_odom() is not None else (
                    self.filtered_yaw if self.filtered_yaw is not None else self.yaw
                )
                pose = (pos_use[0], pos_use[1], yaw_use)
                self.velocity = np.zeros(2)
                confidence = 0.4  # Lower confidence when lost sight
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
        if self.enable_lock and self.locked_pose is None and elapsed >= self.warmup_duration:
            # Use robust median if available, otherwise filtered/raw
            pos_use = self.get_smoothed_position()
            if pos_use is None:
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
        # If locked (and locking enabled), hold that pose regardless of input confidence
        if self.enable_lock and self.locked_pose is not None:
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
        if self.enable_lock and confidence >= self.lock_conf and self.locked_pose is None:
            self.locked_pose = (x, y, yaw)
        if self.enable_lock and self.locked_pose is not None:
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
