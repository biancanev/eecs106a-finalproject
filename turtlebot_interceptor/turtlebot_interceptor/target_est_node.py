import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import transforms3d.euler as euler
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC
from visualization_msgs.msg import Marker, MarkerArray

class TargetEstimator(Node):
    def __init__(self):
        super().__init__('mpc_node')

        self.seeker_obs = []
        self.target_obs = []

        self.seeker_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.seeker_callback,
            10
        )

        self.target_sub = self.create_subscription(
            OccupancyGrid,
            '/target/map',
            self.target_callback,
            10
        )

        self.goal_pub = self.create_publisher(PoseWithCovarianceStamped, '/target_est', 10)
        self.timer = self.create_timer(0.1, self.pub_callback)

    def seeker_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.map = msg
        
        # Log map reception periodically
        if not hasattr(self, '_map_callback_count'):
            self._map_callback_count = 0
        self._map_callback_count += 1

        if self.map is None:
            return []
        
        robot_x = 0.0
        robot_y = 0.0
        
        # Extract ALL occupied cells within range as individual obstacles
        obstacles = []
        width = self.map.info.width
        height = self.map.info.height
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        
        # CRITICAL: Large obstacle radius to ensure avoidance
        obstacle_radius = self.obstacle_radius_param * 1.2  # Slightly larger for map obstacles
        
        for i in range(width * height):
            if self.map.data[i] > 30:  # Occupied
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                
                # Distance from robot
                dx = world_x - robot_x
                dy = world_y - robot_y
                dist = np.sqrt(dx*dx + dy*dy)

                ux = dx / dist
                uy = dy / dist

                corrected_x = world_x + ux * obstacle_radius
                corrected_y = world_y + uy * obstacle_radius
                
                # Only within 1.5m of robot (reduced range for performance)
                if dist < 1.5 and dist > 0.02:
                    obstacles.append((np.array([corrected_x, corrected_y]), obstacle_radius))
        
        # Limit to closest 30 obstacles (for performance)
        if len(obstacles) > 30:
            obstacles.sort(key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
            obstacles = obstacles[:30]
        
        # DEBUG
        if not hasattr(self, '_obstacle_count'):
            self._obstacle_count = 0
        self._obstacle_count += 1
        if self._obstacle_count % 20 == 0:
            self.get_logger().error(
                f"OBSTACLE CELLS: Found {len(obstacles)} occupied cells within 1.5m, "
                f"robot=({robot_x:.3f}, {robot_y:.3f})"
            )
            if len(obstacles) > 0:
                closest = min(obstacles, key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
                dist_closest = np.sqrt((closest[0][0]-robot_x)**2 + (closest[0][1]-robot_y)**2)
                self.get_logger().error(
                    f"  Closest cell: ({closest[0][0]:.3f}, {closest[0][1]:.3f}), "
                    f"dist={dist_closest:.3f}m, radius={closest[1]:.3f}m"
                )
        
        self.seeker_obs = obstacles
        return
    
    def target_callback(self, msg: OccupancyGrid):
        """Store map for obstacle avoidance"""
        self.target_map = msg
        
        # Log map reception periodically
        if not hasattr(self, '_map_callback_count'):
            self.t_map_callback_count = 0
        self.t_map_callback_count += 1

        if self.target_map is None:
            return []
        
        robot_x = 0.0
        robot_y = 0.0
        
        # Extract ALL occupied cells within range as individual obstacles
        obstacles = []
        width = self.target_map.info.width
        height = self.target_map.info.height
        resolution = self.target_map.info.resolution
        origin_x = self.target_map.info.origin.position.x
        origin_y = self.target_map.info.origin.position.y
        
        # CRITICAL: Large obstacle radius to ensure avoidance
        obstacle_radius = self.obstacle_radius_param * 1.2  # Slightly larger for map obstacles
        
        for i in range(width * height):
            if self.target_map.data[i] > 30:  # Occupied
                gx = i % width
                gy = i // width
                world_x = gx * resolution + origin_x + resolution / 2  # Cell center
                world_y = gy * resolution + origin_y + resolution / 2
                
                # Distance from robot
                dx = world_x - robot_x
                dy = world_y - robot_y
                dist = np.sqrt(dx*dx + dy*dy)

                ux = dx / dist
                uy = dy / dist

                corrected_x = world_x + ux * obstacle_radius
                corrected_y = world_y + uy * obstacle_radius
                
                # Only within 1.5m of robot (reduced range for performance)
                if dist < 1.5 and dist > 0.02:
                    obstacles.append((np.array([corrected_x, corrected_y]), obstacle_radius))
        
        # Limit to closest 30 obstacles (for performance)
        if len(obstacles) > 30:
            obstacles.sort(key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
            obstacles = obstacles[:30]
        
        # DEBUG
        if not hasattr(self, '_obstacle_count'):
            self._obstacle_count = 0
        self._obstacle_count += 1
        if self._obstacle_count % 20 == 0:
            self.get_logger().error(
                f"OBSTACLE CELLS: Found {len(obstacles)} occupied cells within 1.5m, "
                f"robot=({robot_x:.3f}, {robot_y:.3f})"
            )
            if len(obstacles) > 0:
                closest = min(obstacles, key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
                dist_closest = np.sqrt((closest[0][0]-robot_x)**2 + (closest[0][1]-robot_y)**2)
                self.get_logger().error(
                    f"  Closest cell: ({closest[0][0]:.3f}, {closest[0][1]:.3f}), "
                    f"dist={dist_closest:.3f}m, radius={closest[1]:.3f}m"
                )
        
        self.target_obs = obstacles
        return

    def pub_callback(self):
        if len(self.seeker_obs) == 0 or len(self.target_obs) == 0:
            self.get_logger().error(
                f"no obstacles seen"
            )
            return
        
        A = np.array([p[:2] for p in self.target_obs])
        B = np.array([p[:2] for p in self.seeker_obs])

        A_mean = A.mean(axis=0)
        B_mean = B.mean(axis=0)

        A_center = A - A_mean
        B_center = B - B_mean

        H = A_center.T @ B_center
        U, S, Vt = np.linalg.svd(H)

        R_t2w_est = Vt.T @ U.T
        if np.linalg.det(R_t2w_est) < 0:
            Vt[-1, :] *= -1
            R_t2w_est = Vt.T @ U.T
        
        t_t2w_est = B_mean - R_t2w_est @ A_mean
        
        est_yaw = math.degrees(math.atan2(R_t2w_est[1,0], R_t2w_est[0,0]))

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = t_t2w_est[0]
        pose_msg.pose.pose.position.y = t_t2w_est[1]
        pose_msg.pose.pose.position.z = 0.0

        pose_msg.pose.pose.orientation.w = 1.0
        # Small covariance
        pose_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        self.goal_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()