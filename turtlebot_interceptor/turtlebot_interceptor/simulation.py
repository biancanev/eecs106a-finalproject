#!/usr/bin/env python3
"""
Simulation support for testing without hardware
Provides fake lidar scans and voxel grid maps
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


class FakeLidarNode(Node):
    """
    Generates fake lidar scans for simulation
    Can simulate obstacles and provide realistic scan data
    """
    
    def __init__(self):
        super().__init__('fake_lidar_node')
        
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        self.map = None
        self.robot_pose = None
        
        # Lidar parameters
        self.num_beams = 360
        self.max_range = 3.5
        self.min_range = 0.12
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 2 * math.pi / self.num_beams
        self.range_noise_std = 0.02
        
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('Fake lidar node initialized')
    
    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
    
    def pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg
    
    def raycast(self, x, y, theta, max_range):
        """Simple raycast in occupancy grid"""
        if self.map is None:
            return max_range
        
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        width = self.map.info.width
        height = self.map.info.height
        
        # Raycast
        dx = math.cos(theta) * resolution
        dy = math.sin(theta) * resolution
        
        current_x = x
        current_y = y
        distance = 0.0
        
        while distance < max_range:
            # Convert to grid coordinates
            grid_x = int((current_x - origin_x) / resolution)
            grid_y = int((current_y - origin_y) / resolution)
            
            if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
                return max_range
            
            idx = grid_y * width + grid_x
            if idx >= len(self.map.data):
                return max_range
            
            if self.map.data[idx] > 50:  # Occupied
                return distance
            
            current_x += dx
            current_y += dy
            distance += resolution
        
        return max_range
    
    def publish_scan(self):
        if self.robot_pose is None:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        
        x = self.robot_pose.pose.position.x
        y = self.robot_pose.pose.position.y
        
        # Extract yaw from quaternion
        q = self.robot_pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        ranges = []
        for i in range(self.num_beams):
            angle = self.angle_min + i * self.angle_increment
            ray_angle = yaw + angle
            
            # Raycast
            range_val = self.raycast(x, y, ray_angle, self.max_range)
            
            # Add noise
            if range_val < self.max_range:
                range_val += np.random.normal(0, self.range_noise_std)
                range_val = np.clip(range_val, self.min_range, self.max_range)
            
            ranges.append(float(range_val))
        
        scan.ranges = ranges
        self.pub.publish(scan)


class VoxelGridMapNode(Node):
    """
    Generates a simple voxel grid / occupancy grid map for simulation
    Creates a test environment with obstacles
    """
    
    def __init__(self):
        super().__init__('voxel_grid_map_node')
        
        self.pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Map parameters
        self.width = 100
        self.height = 100
        self.resolution = 0.05  # 5cm per cell
        self.origin_x = -2.5
        self.origin_y = -2.5
        
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Voxel grid map node initialized')
    
    def create_test_map(self):
        """Create a test map with obstacles"""
        map_data = [0] * (self.width * self.height)
        
        # Add walls
        wall_thickness = 2
        # Bottom wall
        for x in range(self.width):
            for y in range(wall_thickness):
                idx = y * self.width + x
                map_data[idx] = 100
        
        # Top wall
        for x in range(self.width):
            for y in range(self.height - wall_thickness, self.height):
                idx = y * self.width + x
                map_data[idx] = 100
        
        # Left wall
        for x in range(wall_thickness):
            for y in range(self.height):
                idx = y * self.width + x
                map_data[idx] = 100
        
        # Right wall
        for x in range(self.width - wall_thickness, self.width):
            for y in range(self.height):
                idx = y * self.width + x
                map_data[idx] = 100
        
        # Add some obstacles
        obstacles = [
            (30, 30, 5),  # (x, y, radius)
            (70, 30, 5),
            (50, 70, 8),
        ]
        
        for ox, oy, r in obstacles:
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if dx*dx + dy*dy <= r*r:
                        x = ox + dx
                        y = oy + dy
                        if 0 <= x < self.width and 0 <= y < self.height:
                            idx = y * self.width + x
                            map_data[idx] = 100
        
        return map_data
    
    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = self.create_test_map()
        
        self.pub.publish(map_msg)
        self.get_logger().info('Published test map')


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    if '--map-only' in sys.argv:
        node = VoxelGridMapNode()
    elif '--lidar-only' in sys.argv:
        node = FakeLidarNode()
    else:
        # Run both
        map_node = VoxelGridMapNode()
        lidar_node = FakeLidarNode()
        
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(map_node)
        executor.add_node(lidar_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            map_node.destroy_node()
            lidar_node.destroy_node()
            rclpy.shutdown()
        return
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

