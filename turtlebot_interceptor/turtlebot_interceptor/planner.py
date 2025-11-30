#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from turtlebot_interceptor.trajectory import plan_probabilistic_trajectory, NoiseModel
import time
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('probabilistic_controller')
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_noisy_pose = self.create_publisher(PoseWithCovarianceStamped, '/noisy_pose', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.Kp = np.diag([2.0, 0.8])
        self.Kd = np.diag([-0.5, 0.5])
        self.Ki = np.diag([-0.1, 0.1])

        # New MPC Variables
        self.prev_uv = 0
        self.mpc = None  # Will be initialized when needed
        
        self.noise_model = NoiseModel()
        
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)
        
        self.timer = self.create_timer(0.1, self.publish_noisy_pose)
        
        self.get_logger().info('Probabilistic TurtleBot controller initialized.')
    
    def publish_noisy_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
            
            true_pose = np.array([x, y, yaw])
            cov = self.noise_model.get_pose_covariance(np.linalg.norm([x, y]))
            noisy_pose = self.noise_model.sample_measurement(true_pose, cov)
            
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.pose.position.x = noisy_pose[0]
            msg.pose.pose.position.y = noisy_pose[1]
            
            quat = self._quat_from_yaw(noisy_pose[2])
            msg.pose.pose.orientation.x = quat[0]
            msg.pose.pose.orientation.y = quat[1]
            msg.pose.pose.orientation.z = quat[2]
            msg.pose.pose.orientation.w = quat[3]
            
            msg.pose.covariance = np.zeros(36)
            msg.pose.covariance[0] = cov[0, 0]
            msg.pose.covariance[7] = cov[1, 1]
            msg.pose.covariance[35] = cov[2, 2]
            
            self.pub_noisy_pose.publish(msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    
    ### NEW MPC Contoller ###
    def mpc_controller(self, waypoint_with_cov):
        nominal_waypoint = waypoint_with_cov[0]
        covariance = waypoint_with_cov[1]
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            try:
                trans = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time())
                
                wp_pose = PoseStamped()
                wp_pose.header.frame_id = 'odom'
                wp_pose.pose.position.x = nominal_waypoint[0]
                wp_pose.pose.position.y = nominal_waypoint[1]
                quat = self._quat_from_yaw(nominal_waypoint[2])
                wp_pose.pose.orientation.x = quat[0]
                wp_pose.pose.orientation.y = quat[1]
                wp_pose.pose.orientation.z = quat[2]
                wp_pose.pose.orientation.w = quat[3]
                
                wp_base = do_transform_pose(wp_pose.pose, trans)
                
                x_err = wp_base.position.x
                q = wp_base.orientation
                roll, pitch, yaw_err = euler.quat2euler([q.w, q.x, q.y, q.z])

                if abs(x_err) < 0.03 and abs(yaw_err) < 0.2:
                    self.get_logger().info(f"Waypoint reached (cov trace: {np.trace(covariance[:2,:2]):.4f})")
                    return

                # Initialize MPC if needed
                if self.mpc is None:
                    self.mpc = SimpleUnicycleMPC(horizon=15, dt=0.1)

                # Get current state in base frame
                x0 = [0, 0, yaw_err, self.prev_uv]  # in base frame
                target = [wp_base.position.x, wp_base.position.y]
                
                try:
                    a_cmd, omega_cmd = self.mpc.solve(x0, target)
                    # Convert acceleration to velocity
                    v_cmd = np.clip(self.prev_uv + a_cmd * 0.1, 0.0, 0.6)
                    self.prev_uv = v_cmd
                    u0v = v_cmd
                    u0w = omega_cmd
                except Exception as e:
                    self.get_logger().error(f"MPC solve failed: {e}")
                    # Fallback to proportional control
                    u0v = self.Kp[0, 0] * x_err
                    u0w = self.Kp[1, 1] * yaw_err
                
                cmd = Twist()
                cmd.linear.x = u0v
                self.prev_uv = u0v
                cmd.angular.z = u0w
                self.pub_cmd.publish(cmd)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            
            time.sleep(0.1)

    ### Basic Proportional Gain controller ##
    def controller(self, waypoint_with_cov):
        nominal_waypoint = waypoint_with_cov[0]
        covariance = waypoint_with_cov[1]
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            try:
                trans = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time())
                
                wp_pose = PoseStamped()
                wp_pose.header.frame_id = 'odom'
                wp_pose.pose.position.x = nominal_waypoint[0]
                wp_pose.pose.position.y = nominal_waypoint[1]
                quat = self._quat_from_yaw(nominal_waypoint[2])
                wp_pose.pose.orientation.x = quat[0]
                wp_pose.pose.orientation.y = quat[1]
                wp_pose.pose.orientation.z = quat[2]
                wp_pose.pose.orientation.w = quat[3]
                
                wp_base = do_transform_pose(wp_pose.pose, trans)
                
                x_err = wp_base.position.x
                q = wp_base.orientation
                roll, pitch, yaw_err = euler.quat2euler([q.w, q.x, q.y, q.z])
                
                if abs(x_err) < 0.03 and abs(yaw_err) < 0.2:
                    self.get_logger().info(f"Waypoint reached (cov trace: {np.trace(covariance[:2,:2]):.4f})")
                    return
                
                cmd = Twist()
                cmd.linear.x = self.Kp[0, 0] * x_err
                cmd.angular.z = self.Kp[1, 1] * yaw_err
                self.pub_cmd.publish(cmd)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            
            time.sleep(0.1)
    
    def planning_callback(self, msg: PointStamped):
        self.get_logger().info(f'Planning probabilistic trajectory to ({msg.point.x:.2f}, {msg.point.y:.2f})')
        
        waypoints_with_cov, sampled_trajectories = plan_probabilistic_trajectory((msg.point.x, msg.point.y))
        
        for waypoint_with_cov in waypoints_with_cov:
            self.controller(waypoint_with_cov)
        
    @staticmethod
    def _quat_from_yaw(yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()