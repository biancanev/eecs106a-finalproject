#!/usr/bin/env python3
"""
Standalone simulation that runs without ROS2
Simulates the entire system using pure Python
"""
import numpy as np
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import math


def wrap_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Import our core modules (without ROS2 dependencies)
try:
    # Keep optional path for older imports; no MPC used in this stripped sim
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
except Exception:
    pass


@dataclass
class Pose:
    """Simple pose representation"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    def to_array(self):
        return np.array([self.x, self.y, self.theta])


@dataclass
class LaserScan:
    """Simple LIDAR scan representation"""
    ranges: List[float]
    angles: List[float]
    max_range: float = 3.5
    min_range: float = 0.12


@dataclass
class OccupancyGrid:
    """Simple occupancy grid representation"""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: List[int]  # 0=free, 100=occupied, -1=unknown


class TopicManager:
    """Simulates ROS2 topic system"""
    def __init__(self):
        self.topics: Dict[str, any] = {}
        self.subscribers: Dict[str, List[callable]] = {}
        self.lock = threading.Lock()
    
    def publish(self, topic: str, message: any):
        """Publish a message to a topic"""
        with self.lock:
            self.topics[topic] = message
            # Notify subscribers
            if topic in self.subscribers:
                for callback in self.subscribers[topic]:
                    try:
                        callback(message)
                    except Exception as e:
                        print(f"Error in subscriber callback: {e}")
    
    def subscribe(self, topic: str, callback: callable):
        """Subscribe to a topic"""
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(callback)
    
    def get(self, topic: str, default=None):
        """Get latest message from topic"""
        with self.lock:
            return self.topics.get(topic, default)


class MapGenerator:
    """Generates a test map"""
    def __init__(self, width=100, height=100, resolution=0.05):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -2.5
        self.origin_y = -2.5
    
    def create_map(self, obstacle_centers: List[Tuple[float, float]] = None) -> OccupancyGrid:
        """
        Create a test map with obstacles.
        If obstacle_centers is provided, place circular obstacles at those centers; otherwise
        a single obstacle is placed at (0,0).
        """
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
        
        # Add circular obstacles
        centers = obstacle_centers if obstacle_centers is not None else [(0.0, 0.0)]
        obstacle_radius_m = 0.30
        margin = obstacle_radius_m + 0.05
        world_max_x = self.origin_x + self.width * self.resolution - margin
        world_max_y = self.origin_y + self.height * self.resolution - margin
        obstacle_radius_cells = max(3, int(round(obstacle_radius_m / self.resolution)))
        
        for cx, cy in centers:
            # Keep obstacle inside the map bounds with a small margin
            cx = min(max(cx, self.origin_x + margin), world_max_x)
            cy = min(max(cy, self.origin_y + margin), world_max_y)
            center_gx, center_gy = self.world_to_grid(cx, cy)
            for dx in range(-obstacle_radius_cells, obstacle_radius_cells+1):
                for dy in range(-obstacle_radius_cells, obstacle_radius_cells+1):
                    if dx*dx + dy*dy <= obstacle_radius_cells*obstacle_radius_cells:
                        x = center_gx + dx
                        y = center_gy + dy
                        if 0 <= x < self.width and 0 <= y < self.height:
                            idx = y * self.width + x
                            map_data[idx] = 100
        
        return OccupancyGrid(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            data=map_data
        )
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def is_occupied(self, x: float, y: float, map_grid: OccupancyGrid, radius: float = 0.0) -> bool:
        """
        Check if a world coordinate is occupied.
        Mirrors the animated sim: bounds + nearby cells + wall margin.
        """
        # Bounds check
        if x < self.origin_x or x >= self.origin_x + map_grid.width * self.resolution:
            return True
        if y < self.origin_y or y >= self.origin_y + map_grid.height * self.resolution:
            return True

        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < map_grid.width and 0 <= gy < map_grid.height:
            idx = gy * map_grid.width + gx
            if map_grid.data[idx] > 50:
                return True

        # Radius check against nearby cells
        if radius > 0:
            radius_cells = int(radius / self.resolution) + 1
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    check_gx = gx + dx
                    check_gy = gy + dy
                    if 0 <= check_gx < map_grid.width and 0 <= check_gy < map_grid.height:
                        idx = check_gy * map_grid.width + check_gx
                        if map_grid.data[idx] > 50:
                            cell_x = check_gx * self.resolution + self.origin_x
                            cell_y = check_gy * self.resolution + self.origin_y
                            dist = math.hypot(x - cell_x, y - cell_y)
                            if dist < radius + self.resolution:
                                return True

        # Wall margin: keep small gap from borders
        wall_thickness = 0.1
        if (
            x < self.origin_x + wall_thickness + radius
            or x > self.origin_x + map_grid.width * self.resolution - wall_thickness - radius
            or y < self.origin_y + wall_thickness + radius
            or y > self.origin_y + map_grid.height * self.resolution - wall_thickness - radius
        ):
            return True

        return False


class FakeLidar:
    """Generates fake LIDAR scans"""
    def __init__(self, map_gen: MapGenerator, hit_dropout_prob: float = 0.0):
        self.map_gen = map_gen
        self.num_beams = 360
        self.max_range = 3.5
        self.min_range = 0.12
        self.range_noise_std = 0.02
        # Probability that a true hit is reported as a miss (simulated bad lidar)
        self.hit_dropout_prob = hit_dropout_prob
    
    def raycast(self, x: float, y: float, theta: float, map_grid: OccupancyGrid) -> float:
        """Raycast from position in direction theta"""
        resolution = map_grid.resolution
        step_size = resolution * 0.1  # finer steps to avoid missing obstacles
        
        current_x = x
        current_y = y
        distance = 0.0
        
        dx = math.cos(theta) * step_size
        dy = math.sin(theta) * step_size
        
        while distance < self.max_range:
            # Check with small radius so we don't slip through thin obstacles
            if self.map_gen.is_occupied(current_x, current_y, map_grid, radius=0.05):
                return distance
            
            current_x += dx
            current_y += dy
            distance += step_size
        
        return self.max_range
    
    def generate_scan(self, pose: Pose, map_grid: OccupancyGrid) -> LaserScan:
        """Generate a LIDAR scan from given pose"""
        ranges = []
        angles = []
        
        for i in range(self.num_beams):
            angle = -math.pi + i * (2 * math.pi / self.num_beams)
            ray_angle = pose.theta + angle
            
            range_val = self.raycast(pose.x, pose.y, ray_angle, map_grid)
            
            # Add noise
            if range_val < self.max_range:
                # Randomly drop true hits to simulate lower-quality returns
                if self.hit_dropout_prob > 0 and np.random.rand() < self.hit_dropout_prob:
                    range_val = self.max_range
                else:
                    range_val += np.random.normal(0, self.range_noise_std)
                    range_val = np.clip(range_val, self.min_range, self.max_range)
            
            ranges.append(range_val)
            angles.append(angle)
        
        return LaserScan(ranges=ranges, angles=angles)


class SimpleMCL:
    """Simplified MCL for standalone simulation"""
    def __init__(self, N=100):
        self.N = N
        self.particles = None
        self.weights = None
        self.initialized = False
        self.map = None
        self.motion_noise = np.array([0.02, 0.02, 0.01])
    
    def initialize(self, map_grid: OccupancyGrid):
        """Initialize particles uniformly in free space"""
        self.map = map_grid
        self.particles = np.zeros((self.N, 3))
        
        # Find free cells
        free_cells = []
        for i in range(map_grid.width * map_grid.height):
            if map_grid.data[i] == 0:
                x = (i % map_grid.width) * map_grid.resolution + map_grid.origin_x
                y = (i // map_grid.width) * map_grid.resolution + map_grid.origin_y
                free_cells.append((x, y))
        
        if len(free_cells) == 0:
            print("Warning: No free cells in map!")
            return
        
        # Sample particles
        indices = np.random.choice(len(free_cells), self.N)
        for i, idx in enumerate(indices):
            self.particles[i, 0] = free_cells[idx][0]
            self.particles[i, 1] = free_cells[idx][1]
            self.particles[i, 2] = np.random.uniform(0, 2 * math.pi)
        
        self.weights = np.ones(self.N) / self.N
        self.initialized = True
    
    def predict(self, v: float, omega: float, dt: float):
        """Motion model prediction"""
        if not self.initialized:
            return
        
        noise = np.random.randn(self.N, 3) * self.motion_noise
        
        self.particles[:, 0] += dt * v * np.cos(self.particles[:, 2]) + noise[:, 0]
        self.particles[:, 1] += dt * v * np.sin(self.particles[:, 2]) + noise[:, 1]
        self.particles[:, 2] += dt * omega + noise[:, 2]
        self.particles[:, 2] = np.mod(self.particles[:, 2] + math.pi, 2 * math.pi) - math.pi
    
    def update(self, scan: LaserScan):
        """Update weights based on scan (simplified)"""
        if not self.initialized:
            return
        
        # Simple likelihood: uniform for now (can be improved)
        self.weights = np.ones(self.N) / self.N
        self.resample()
    
    def resample(self):
        """Resample particles"""
        idx = np.random.choice(self.N, self.N, p=self.weights)
        self.particles = self.particles[idx]
        self.weights = np.ones(self.N) / self.N
    
    def get_estimate(self) -> Tuple[Pose, np.ndarray]:
        """Get mean pose and covariance"""
        if not self.initialized:
            return None, None
        
        mean = np.average(self.particles, axis=0, weights=self.weights)
        diff = self.particles - mean
        cov = np.cov(diff.T, aweights=self.weights)
        
        pose = Pose(x=mean[0], y=mean[1], theta=mean[2])
        return pose, cov


class SimpleKF:
    """Simplified Kalman Filter for target tracking"""
    def __init__(self, dt=0.1):
        self.dt = dt
        self.x = np.zeros((4, 1))  # [px, py, vx, vy]
        self.P = np.eye(4) * 1.0
        
        self.A = np.array([
            [1.0, 0.0, self.dt, 0.0],
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        self.Q = np.eye(4) * 0.01
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])
        self.R = np.eye(2) * 0.05
    
    def predict(self):
        """Prediction step"""
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q
    
    def update(self, z: np.ndarray):
        """Update step with measurement"""
        x_pred = self.A @ self.x
        P_pred = self.A @ self.P @ self.A.T + self.Q
        
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        
        z = z.reshape(2, 1)
        self.x = x_pred + K @ (z - self.H @ x_pred)
        self.P = (np.eye(4) - K @ self.H) @ P_pred
    
    def get_position(self) -> Tuple[float, float]:
        """Get current position estimate"""
        return float(self.x[0, 0]), float(self.x[1, 0])


@dataclass
class RobotState:
    """Lightweight robot container for multi-robot sim"""
    name: str
    pose: Pose
    velocity: float = 0.0
    last_cmd: List[float] = None  # [v, omega]
    color: str = 'blue'

    def __post_init__(self):
        if self.last_cmd is None:
            self.last_cmd = [0.0, 0.0]


class StandaloneSimulation:
    """Main simulation class"""
    def __init__(self, randomize: bool = False, num_objects: int = 6):
        self.topics = TopicManager()
        self.map_gen = MapGenerator()
        # Drop ~50% of true hits to lower perceived object voxels
        self.lidar = FakeLidar(self.map_gen, hit_dropout_prob=0.5)
        # Per-robot localization
        self.robot_localizers: Dict[str, SimpleMCL] = {}
        self.num_objects = max(1, num_objects)
        
        # Two simple turtlebots
        self.robots: Dict[str, RobotState] = {
            'tb1': RobotState(
                name='tb1',
                pose=Pose(x=-1.5, y=0.0, theta=0.0),
                color='blue',
                last_cmd=[0.0, 0.0],
            ),
            'tb2': RobotState(
                name='tb2',
                pose=Pose(x=1.5, y=0.0, theta=math.pi),
                color='green',
                last_cmd=[0.0, 0.0],
            ),
        }
        
        # Distinct colors for visualizing what each robot's lidar observes
        self.voxel_colors: Dict[str, str] = {
            'tb1': 'orange',
            'tb2': 'purple',
        }
        
        # Start with a map and optionally randomize placements
        self.setup(randomize=randomize)
    
    def setup(self, randomize: bool = False):
        """Initialize simulation"""
        # Per-robot detected occupied voxels from lidar hits
        self.detected_voxels: Dict[str, set] = {name: set() for name in self.robots.keys()}
        # Reset any previous estimates
        self.object_estimates: Dict[str, Tuple[float, float, float]] = {}
        
        # If randomizing, place robots first so we can ensure objects block line of sight
        if randomize:
            # Create a temporary empty map for robot placement
            temp_map = self.map_gen.create_map(None)
            self.robot_localizers = {}
            placed_poses: List[Pose] = []
            for name, robot in self.robots.items():
                robot.pose = self._sample_free_pose(temp_map, placed_poses)
                robot.last_cmd = [0.0, 0.0]
                placed_poses.append(robot.pose)
        
        # Place obstacles ensuring line of sight between robots is blocked
        obstacle_centers = None
        if randomize:
            # Get robot positions to ensure objects block line of sight
            tb1_pos = self.robots.get('tb1')
            tb2_pos = self.robots.get('tb2')
            robot_positions = None
            if tb1_pos and tb2_pos:
                robot_positions = ((tb1_pos.pose.x, tb1_pos.pose.y), (tb2_pos.pose.x, tb2_pos.pose.y))
            # Ensure at least one object blocks the line between robots
            obstacle_centers = self._sample_object_centers(self.num_objects, robot_positions=robot_positions)
        
        # Generate map with obstacles
        map_grid = self.map_gen.create_map(obstacle_centers)
        self.topics.publish('/map', map_grid)
        # Also publish per-robot map topics for convenience
        for name in self.robots.keys():
            self.topics.publish(f'/{name}/map', map_grid)
        
        # Initialize MCL for each robot (robots already placed if randomize was True)
        self.robot_localizers = {}
        for name, robot in self.robots.items():
            robot.last_cmd = [0.0, 0.0]
            
            mcl = SimpleMCL(N=100)
            mcl.initialize(map_grid)
            self.robot_localizers[name] = mcl
            # Subscribe to per-robot cmd_vel
            self.topics.subscribe(
                f'/{name}/cmd_vel',
                lambda cmd, n=name: self.cmd_vel_callback(n, cmd)
            )
    
    def _sample_object_centers(self, count: int, robot_positions: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None) -> List[Tuple[float, float]]:
        """
        Sample multiple random object centers well inside the map.
        If robot_positions is provided, ensures at least one object blocks the line of sight between robots.
        """
        radius = 0.30
        margin = radius + 0.15
        map_width_m = self.map_gen.width * self.map_gen.resolution
        map_height_m = self.map_gen.height * self.map_gen.resolution
        min_x = self.map_gen.origin_x + margin
        max_x = self.map_gen.origin_x + map_width_m - margin
        min_y = self.map_gen.origin_y + margin
        max_y = self.map_gen.origin_y + map_height_m - margin
        
        centers: List[Tuple[float, float]] = []
        min_sep = radius * 2.2  # keep obstacles from overlapping too much
        max_attempts = 500
        
        # First, place an object along the line between robots to block line of sight
        if robot_positions:
            tb1_pos, tb2_pos = robot_positions
            tb1_x, tb1_y = tb1_pos
            tb2_x, tb2_y = tb2_pos
            
            # Calculate the line between robots
            line_length = math.hypot(tb2_x - tb1_x, tb2_y - tb1_y)
            if line_length > radius * 2:  # Only place if robots are far enough apart
                # Place object at a random point along the line (but not too close to robots)
                # Keep object away from robot positions by at least robot_radius + object_radius
                robot_margin = 0.15 + radius + 0.1  # robot radius + object radius + buffer
                min_t = robot_margin / line_length
                max_t = 1.0 - robot_margin / line_length
                
                if max_t > min_t:
                    # Random position along the line between robots
                    t = np.random.uniform(min_t, max_t)
                    blocking_x = tb1_x + t * (tb2_x - tb1_x)
                    blocking_y = tb1_y + t * (tb2_y - tb1_y)
                    
                    # Ensure it's within map bounds
                    blocking_x = max(min_x, min(max_x, blocking_x))
                    blocking_y = max(min_y, min(max_y, blocking_y))
                    
                    centers.append((blocking_x, blocking_y))
        
        # Place remaining objects randomly
        for _ in range(max_attempts):
            if len(centers) >= count:
                break
            cx = np.random.uniform(min_x, max_x)
            cy = np.random.uniform(min_y, max_y)
            too_close = False
            for ox, oy in centers:
                if math.hypot(cx - ox, cy - oy) < min_sep:
                    too_close = True
                    break
            
            # Also check distance from robots if they're placed
            if robot_positions and not too_close:
                tb1_pos, tb2_pos = robot_positions
                tb1_x, tb1_y = tb1_pos
                tb2_x, tb2_y = tb2_pos
                # Ensure objects aren't too close to robots
                robot_clearance = 0.15 + radius + 0.05  # robot radius + object radius + small buffer
                if (math.hypot(cx - tb1_x, cy - tb1_y) < robot_clearance or
                    math.hypot(cx - tb2_x, cy - tb2_y) < robot_clearance):
                    too_close = True
            
            if too_close:
                continue
            centers.append((cx, cy))
        
        # If we failed to place enough, just fill remaining with whatever we have
        if len(centers) < count:
            print(f"Warning: requested {count} objects, placed {len(centers)} due to space limits.")
        return centers
    
    def _sample_free_pose(self, map_grid: OccupancyGrid, placed: List[Pose], robot_radius: float = 0.15) -> Pose:
        """
        Sample a random collision-free pose for a robot.
        Ensures separation from existing placed poses.
        """
        max_attempts = 300
        separation = 0.6  # keep robots apart a bit
        map_width_m = map_grid.width * map_grid.resolution
        map_height_m = map_grid.height * map_grid.resolution
        min_x = map_grid.origin_x + robot_radius + 0.1
        max_x = map_grid.origin_x + map_width_m - robot_radius - 0.1
        min_y = map_grid.origin_y + robot_radius + 0.1
        max_y = map_grid.origin_y + map_height_m - robot_radius - 0.1
        
        for _ in range(max_attempts):
            x = np.random.uniform(min_x, max_x)
            y = np.random.uniform(min_y, max_y)
            theta = np.random.uniform(-math.pi, math.pi)
            
            if self.map_gen.is_occupied(x, y, map_grid, radius=robot_radius):
                continue
            
            too_close = False
            for pose in placed:
                if math.hypot(pose.x - x, pose.y - y) < separation:
                    too_close = True
                    break
            if too_close:
                continue
            
            return Pose(x=x, y=y, theta=theta)
        
        # Fallback to origin if random sampling fails (should be rare)
        print("Warning: could not find collision-free pose after many attempts; using default.")
        return Pose(x=self.map_gen.origin_x + 0.5, y=self.map_gen.origin_y + 0.5, theta=0.0)
    
    def cmd_vel_callback(self, robot_name: str, cmd: Dict[str, float]):
        """Handle velocity commands per robot"""
        if robot_name in self.robots:
            self.robots[robot_name].last_cmd = [cmd.get('linear_x', 0.0),
                                                cmd.get('angular_z', 0.0)]
    
    def _circle_overlap_area(self, cx1: float, cy1: float, r1: float, 
                             cx2: float, cy2: float, r2: float) -> float:
        """Calculate the area of intersection between two circles."""
        d = math.hypot(cx2 - cx1, cy2 - cy1)
        
        # No overlap
        if d >= r1 + r2:
            return 0.0
        
        # One circle completely inside the other
        if d <= abs(r1 - r2):
            # Return area of smaller circle
            return math.pi * min(r1, r2) ** 2
        
        # Calculate intersection area using standard formula
        # Area = r1^2 * arccos((d^2 + r1^2 - r2^2) / (2*d*r1)) 
        #       + r2^2 * arccos((d^2 + r2^2 - r1^2) / (2*d*r2))
        #       - 0.5 * sqrt((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2))
        
        d_sq = d * d
        r1_sq = r1 * r1
        r2_sq = r2 * r2
        
        term1 = (d_sq + r1_sq - r2_sq) / (2 * d * r1)
        term1 = max(-1.0, min(1.0, term1))  # Clamp to valid range for arccos
        a1 = r1_sq * math.acos(term1)
        
        term2 = (d_sq + r2_sq - r1_sq) / (2 * d * r2)
        term2 = max(-1.0, min(1.0, term2))  # Clamp to valid range for arccos
        a2 = r2_sq * math.acos(term2)
        
        s = (-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)
        if s < 0:
            s = 0
        a3 = -0.5 * math.sqrt(s)
        
        return a1 + a2 + a3
    
    def _merge_overlapping_circles(self, circles: List[Tuple[float, float, float]], 
                                   overlap_threshold: float = 0.5,
                                   robot_position: Optional[Tuple[float, float]] = None) -> List[Tuple[float, float, float]]:
        """
        Merge circles that overlap by more than the threshold percentage.
        Applies exponential distance weighting - further objects have lower quality readings.
        
        Args:
            circles: List of (cx, cy, radius) tuples
            overlap_threshold: Overlap percentage (0.0-1.0) above which circles are merged
            robot_position: Optional (x, y) position of robot for distance-based weighting
        
        Returns:
            List of merged circles
        """
        # Exponential decay parameters for distance weighting
        # Quality decreases exponentially with distance
        # weight = exp(-distance / decay_distance)
        decay_distance = 1.5  # meters - distance at which weight drops to 1/e (~37%)
        
        def distance_weight(distance: float) -> float:
            """Calculate exponential weight based on distance from robot."""
            if robot_position is None:
                return 1.0  # No weighting if robot position not provided
            return math.exp(-distance / decay_distance)
        if len(circles) <= 1:
            return circles
        
        merged = []
        used = set()
        
        for i, (cx1, cy1, r1) in enumerate(circles):
            if i in used:
                continue
            
            # Find all circles that overlap significantly with this group (handles transitive overlaps)
            to_merge = [(cx1, cy1, r1)]
            changed = True
            
            # Keep iterating until no new overlapping circles are found (handles transitive overlaps)
            while changed:
                changed = False
                for j, (cx2, cy2, r2) in enumerate(circles):
                    if j in used or j == i:
                        continue
                    
                    # Check if this circle overlaps with any circle in the current merge group
                    overlaps = False
                    for cx_ref, cy_ref, r_ref in to_merge:
                        overlap_area = self._circle_overlap_area(cx_ref, cy_ref, r_ref, cx2, cy2, r2)
                        
                        # Calculate overlap as percentage of smaller circle
                        smaller_area = math.pi * min(r_ref, r2) ** 2
                        if smaller_area > 0:
                            overlap_percent = overlap_area / smaller_area
                        else:
                            overlap_percent = 0.0
                        
                        # Apply exponential distance weighting to overlap threshold
                        # Further objects need higher overlap percentage to merge (lower quality readings)
                        effective_threshold = overlap_threshold
                        if robot_position:
                            rx, ry = robot_position
                            # Use average distance of the two circles from robot
                            dist_ref = math.hypot(cx_ref - rx, cy_ref - ry)
                            dist_curr = math.hypot(cx2 - rx, cy2 - ry)
                            avg_dist = (dist_ref + dist_curr) / 2.0
                            
                            # Further objects require higher overlap to merge
                            # weight decreases exponentially, so threshold increases
                            weight = distance_weight(avg_dist)
                            # Inverse relationship: lower weight = higher required overlap
                            # Scale threshold so that at distance 0, threshold is normal
                            # At larger distances, threshold approaches 1.0 (very hard to merge)
                            effective_threshold = overlap_threshold + (1.0 - overlap_threshold) * (1.0 - weight)
                        
                        # If overlap > effective threshold, mark for merging
                        if overlap_percent > effective_threshold:
                            overlaps = True
                            break
                    
                    if overlaps:
                        to_merge.append((cx2, cy2, r2))
                        used.add(j)
                        changed = True
            
            # Merge all overlapping circles into one
            if len(to_merge) == 1:
                merged.append(to_merge[0])
            else:
                # Combine multiple circles: use weighted average of centers
                # Weight by circle area AND exponential distance weighting (closer = higher quality)
                total_weight = 0.0
                weighted_cx = 0.0
                weighted_cy = 0.0
                max_radius = 0.0
                
                for cx, cy, r in to_merge:
                    area = math.pi * r * r
                    # Base weight on area
                    weight = area
                    
                    # Apply exponential distance weighting
                    if robot_position:
                        rx, ry = robot_position
                        dist = math.hypot(cx - rx, cy - ry)
                        # Further objects contribute less to the merged center
                        dist_weight = distance_weight(dist)
                        weight *= dist_weight
                    
                    weighted_cx += cx * weight
                    weighted_cy += cy * weight
                    total_weight += weight
                    max_radius = max(max_radius, r)
                
                if total_weight > 0:
                    merged_cx = weighted_cx / total_weight
                    merged_cy = weighted_cy / total_weight
                    # Use the maximum radius to ensure all merged circles are covered
                    merged.append((merged_cx, merged_cy, max_radius))
            
            used.add(i)
        
        return merged
    
    def compute_object_estimates(self, map_grid: OccupancyGrid):
        """Compute object position estimates from detected voxels for each robot.

        Uses iterative refinement algorithm that fits circle edge to detected voxels:
        - Knows the object size (radius = 0.30m) + 5 voxel sizes for easier fitting
        - LIDAR only detects edges, so voxels should lie on/near the circle perimeter
        - Leverages fact that voxels are between robot and object center for better initialization
        - Initializes object center along ray from robot through detected voxel
        - Iteratively refines the center to minimize distance from voxels to circle edge
        - Prefers voxels aligned with robot-center line during refinement
        - Continues until convergence or all nearby edge voxels are accounted for
        """
        if not hasattr(self, 'object_estimates'):
            self.object_estimates: Dict[str, List[Tuple[float, float, float]]] = {}
        
        # Known object size (radius in meters) + 5 voxel sizes (2 originally + 3 additional)
        object_radius = 0.30
        expanded_radius = object_radius + 5 * map_grid.resolution
        
        # Tolerance for considering a voxel to be "on the edge" of the circle
        edge_tolerance = map_grid.resolution * 1.5
        
        for name in self.robots.keys():
            hits = self.detected_voxels.get(name, set())
            wall_margin = 0.35  # discard voxels that are actually walls
            occupied_cells = []

            for gx, gy in hits:
                x = gx * map_grid.resolution + map_grid.origin_x
                y = gy * map_grid.resolution + map_grid.origin_y
                
                # Skip wall hits (near map boundary)
                if (
                    x < map_grid.origin_x + wall_margin
                    or x > map_grid.origin_x + map_grid.width * map_grid.resolution - wall_margin
                    or y < map_grid.origin_y + wall_margin
                    or y > map_grid.origin_y + map_grid.height * map_grid.resolution - wall_margin
                ):
                    continue

                # Keep track of voxel centers for object detection
                occupied_cells.append((x + map_grid.resolution * 0.5,
                                       y + map_grid.resolution * 0.5))
            
            if not occupied_cells:
                self.object_estimates.pop(name, None)
                continue

            # Get robot pose to leverage voxel position relative to robot
            robot = self.robots.get(name)
            if not robot:
                continue
            robot_x = robot.pose.x
            robot_y = robot.pose.y

            # Iterative refinement algorithm - fitting circle edge to voxels
            circles: List[Tuple[float, float, float]] = []
            processed = set()  # Track which voxels have been assigned to an object
            
            for seed_idx, (seed_x, seed_y) in enumerate(occupied_cells):
                # Skip if this voxel is already part of another object
                if seed_idx in processed:
                    continue
                
                # Leverage fact that voxels are between robot and object center
                # The object center should be roughly in the direction from robot through voxel
                # Vector from robot to seed voxel
                vec_robot_to_voxel_x = seed_x - robot_x
                vec_robot_to_voxel_y = seed_y - robot_y
                vec_robot_to_voxel_len = math.hypot(vec_robot_to_voxel_x, vec_robot_to_voxel_y)
                
                # Start with initial guess: object center is along the ray from robot through voxel
                # Since voxel is on the edge, center is approximately at: voxel + normalized_vector * expanded_radius
                if vec_robot_to_voxel_len > 1e-6:
                    # Normalize the direction vector
                    dir_x = vec_robot_to_voxel_x / vec_robot_to_voxel_len
                    dir_y = vec_robot_to_voxel_y / vec_robot_to_voxel_len
                    # Place center beyond the voxel in the direction away from robot
                    center_x = seed_x + dir_x * expanded_radius
                    center_y = seed_y + dir_y * expanded_radius
                else:
                    # Fallback if robot and voxel are too close
                    center_x = seed_x + expanded_radius * 0.7
                    center_y = seed_y + expanded_radius * 0.7
                max_iterations = 30
                convergence_threshold = 0.005  # meters - tighter for edge fitting
                
                for iteration in range(max_iterations):
                    # Find all voxels near the edge of the current circle guess
                    # Leverage fact that voxels should be between robot and object center
                    edge_voxels = []
                    for voxel_idx, (vx, vy) in enumerate(occupied_cells):
                        if voxel_idx in processed:
                            continue
                        dist_to_center = math.hypot(vx - center_x, vy - center_y)
                        # Check if voxel is near the circle edge (within tolerance)
                        if abs(dist_to_center - expanded_radius) <= edge_tolerance:
                            # Verify voxel is between robot and object center
                            # Vector from robot to voxel
                            vec_rv_x = vx - robot_x
                            vec_rv_y = vy - robot_y
                            # Vector from robot to object center
                            vec_rc_x = center_x - robot_x
                            vec_rc_y = center_y - robot_y
                            
                            # Check if voxel is in the direction from robot to center
                            # Using dot product: if positive, voxel is in forward direction
                            dot_product = vec_rv_x * vec_rc_x + vec_rv_y * vec_rc_y
                            # Also check that voxel is closer to robot than center is
                            dist_robot_to_voxel = math.hypot(vec_rv_x, vec_rv_y)
                            dist_robot_to_center = math.hypot(vec_rc_x, vec_rc_y)
                            
                            # Voxel should be between robot and center (or at least in similar direction)
                            if dot_product > 0 and dist_robot_to_voxel < dist_robot_to_center * 1.1:
                                edge_voxels.append((voxel_idx, vx, vy, dist_to_center))
                    
                    # If we didn't find enough with strict filtering, relax the constraint
                    if len(edge_voxels) < 3:
                        edge_voxels = []
                        for voxel_idx, (vx, vy) in enumerate(occupied_cells):
                            if voxel_idx in processed:
                                continue
                            dist_to_center = math.hypot(vx - center_x, vy - center_y)
                            if abs(dist_to_center - expanded_radius) <= edge_tolerance:
                                edge_voxels.append((voxel_idx, vx, vy, dist_to_center))
                    
                    if len(edge_voxels) < 3:
                        break  # Need at least 3 points to fit a circle reliably
                    
                    # Refine center using least-squares approach
                    # We want to minimize sum of (distance_from_center - expanded_radius)^2
                    # For each edge voxel, compute the gradient contribution
                    # The ideal center would make all edge voxels exactly at expanded_radius distance
                    
                    # Use geometric center adjustment based on edge voxels
                    # For each edge voxel, the ideal center direction is towards/away from it
                    # to make the distance equal to expanded_radius
                    # Weight by proximity to robot-center line for better accuracy
                    sum_dx = 0.0
                    sum_dy = 0.0
                    total_weight = 0.0
                    
                    for voxel_idx, vx, vy, dist in edge_voxels:
                        if dist < 1e-6:  # Avoid division by zero
                            continue
                        
                        # Compute unit vector from center to voxel
                        dx = (vx - center_x) / dist
                        dy = (vy - center_y) / dist
                        
                        # Error: how far is this voxel from the desired radius?
                        error = dist - expanded_radius
                        
                        # Weight by inverse distance error (closer to edge = higher weight)
                        weight = 1.0 / (abs(error) + 0.01)
                        
                        # Additional weight: prefer voxels that are aligned with robot-center line
                        # Vector from robot to voxel
                        vec_rv_x = vx - robot_x
                        vec_rv_y = vy - robot_y
                        # Vector from robot to center
                        vec_rc_x = center_x - robot_x
                        vec_rc_y = center_y - robot_y
                        
                        # Normalize vectors for dot product
                        len_rv = math.hypot(vec_rv_x, vec_rv_y)
                        len_rc = math.hypot(vec_rc_x, vec_rc_y)
                        if len_rv > 1e-6 and len_rc > 1e-6:
                            # Cosine of angle between robot-voxel and robot-center vectors
                            cos_angle = (vec_rv_x * vec_rc_x + vec_rv_y * vec_rc_y) / (len_rv * len_rc)
                            # Increase weight if voxel is aligned with robot-center line
                            alignment_weight = max(0.5, cos_angle)  # Prefer aligned voxels
                            weight *= alignment_weight
                        
                        # Apply exponential distance weighting - further readings are lower quality
                        # Use distance from robot to object center (where the reading is coming from)
                        dist_to_robot = math.hypot(center_x - robot_x, center_y - robot_y)
                        decay_distance = 1.5  # meters - same as merge function
                        dist_weight = math.exp(-dist_to_robot / decay_distance)
                        weight *= dist_weight
                        
                        # Adjustment direction: move center to reduce error
                        # If voxel is too far, move center towards it
                        # If voxel is too close, move center away from it
                        adjustment_x = -error * dx * weight
                        adjustment_y = -error * dy * weight
                        
                        sum_dx += adjustment_x
                        sum_dy += adjustment_y
                        total_weight += weight
                    
                    if total_weight < 1e-6:
                        break
                    
                    # Compute new center
                    avg_dx = sum_dx / total_weight
                    avg_dy = sum_dy / total_weight
                    
                    # Apply damping for stability
                    damping = 0.5
                    new_center_x = center_x + damping * avg_dx
                    new_center_y = center_y + damping * avg_dy
                    
                    # Check for convergence
                    center_shift = math.hypot(new_center_x - center_x, new_center_y - center_y)
                    if center_shift < convergence_threshold:
                        # Converged! Check if we have enough edge voxels for a valid object
                        if len(edge_voxels) >= 3:
                            circles.append((center_x, center_y, object_radius))
                            # Mark all edge voxels as processed
                            for voxel_idx, _, _, _ in edge_voxels:
                                processed.add(voxel_idx)
                        break
                    
                    # Update center for next iteration
                    center_x = new_center_x
                    center_y = new_center_y
                    
                    # Also check if we've captured all nearby voxels
                    # Look for unprocessed voxels that could be part of this object
                    nearby_voxels = []
                    for voxel_idx, (vx, vy) in enumerate(occupied_cells):
                        if voxel_idx in processed:
                            continue
                        dist_to_center = math.hypot(vx - center_x, vy - center_y)
                        # Consider voxels within reasonable distance of the edge
                        if abs(dist_to_center - expanded_radius) <= edge_tolerance * 2.0:
                            nearby_voxels.append(voxel_idx)
                    
                    # If no new nearby voxels, we've captured all relevant edge points
                    if len(nearby_voxels) == 0 and len(edge_voxels) >= 3:
                        circles.append((center_x, center_y, object_radius))
                        for voxel_idx, _, _, _ in edge_voxels:
                            processed.add(voxel_idx)
                        break

            # Merge overlapping circles (>50% overlap indicates same object)
            # Apply exponential distance weighting - further objects have lower quality
            robot_x = robot.pose.x
            robot_y = robot.pose.y
            circles = self._merge_overlapping_circles(circles, overlap_threshold=0.5, 
                                                     robot_position=(robot_x, robot_y))
            
            if circles:
                self.object_estimates[name] = circles
            else:
                self.object_estimates.pop(name, None)
    
    def calculate_tb1_to_tb2_transform(self):
        """
        Calculate transform from tb1 to tb2 using:
        - Distance from tb1 to object
        - Distance from object to tb2
        - Rotation matrix between tb1 and tb2
        
        Returns: (translation, rotation_matrix, rotation_angle) or None if insufficient data
        """
        if 'tb1' not in self.robots or 'tb2' not in self.robots:
            return None
        
        tb1 = self.robots['tb1']
        tb2 = self.robots['tb2']
        
        # Check if we have object estimates for both robots
        if not hasattr(self, 'object_estimates'):
            return None
        
        obj1_list = self.object_estimates.get('tb1', [])
        obj2_list = self.object_estimates.get('tb2', [])
        
        if not obj1_list or not obj2_list:
            return None
        
        # Find the closest pair of circles between tb1 and tb2 that plausibly
        # represent the same object (must overlap or be within a small margin).
        margin = 0.10  # meters; allow slight separation tolerance
        best_pair = None
        best_dist = float('inf')
        for c1 in obj1_list:
            for c2 in obj2_list:
                dist = math.hypot(c1[0] - c2[0], c1[1] - c2[1])
                # Require overlap (distance <= sum of radii + margin)
                if dist <= (c1[2] + c2[2] + margin) and dist < best_dist:
                    best_dist = dist
                    best_pair = (c1, c2)

        if best_pair is None:
            return None

        c1, c2 = best_pair
        # Use midpoint between matched centers as the shared object position
        obj_x = (c1[0] + c2[0]) * 0.5
        obj_y = (c1[1] + c2[1]) * 0.5
        
        # Distance from tb1 to object
        d1 = math.hypot(obj_x - tb1.pose.x, obj_y - tb1.pose.y)
        
        # Distance from object to tb2
        d2 = math.hypot(tb2.pose.x - obj_x, tb2.pose.y - obj_y)
        
        # Compute rotation matrix from tb1 to tb2
        # Rotation matrix from world to tb1 frame
        R_world_to_tb1 = np.array([
            [math.cos(tb1.pose.theta), math.sin(tb1.pose.theta), 0],
            [-math.sin(tb1.pose.theta), math.cos(tb1.pose.theta), 0],
            [0, 0, 1]
        ])
        
        # Rotation matrix from world to tb2 frame
        R_world_to_tb2 = np.array([
            [math.cos(tb2.pose.theta), math.sin(tb2.pose.theta), 0],
            [-math.sin(tb2.pose.theta), math.cos(tb2.pose.theta), 0],
            [0, 0, 1]
        ])
        
        # Rotation matrix from tb1 to tb2 = R_world_to_tb2 @ R_world_to_tb1^T
        R_tb1_to_tb2 = R_world_to_tb2 @ R_world_to_tb1.T
        
        # Compute translation from tb1 to tb2 in tb1's frame
        # Vector from tb1 to object in world frame
        vec_tb1_to_obj_world = np.array([obj_x - tb1.pose.x, obj_y - tb1.pose.y, 0])
        
        # Vector from object to tb2 in world frame
        vec_obj_to_tb2_world = np.array([tb2.pose.x - obj_x, tb2.pose.y - obj_y, 0])
        
        # Transform to tb1 frame
        vec_tb1_to_obj_tb1 = R_world_to_tb1[:2, :2] @ vec_tb1_to_obj_world[:2]
        vec_obj_to_tb2_tb1 = R_world_to_tb1[:2, :2] @ vec_obj_to_tb2_world[:2]
        
        # Translation from tb1 to tb2 in tb1 frame = vec_tb1_to_obj_tb1 + vec_obj_to_tb2_tb1
        translation_tb1 = vec_tb1_to_obj_tb1 + vec_obj_to_tb2_tb1
        
        # Also compute direct translation for comparison
        vec_tb1_to_tb2_world = np.array([tb2.pose.x - tb1.pose.x, tb2.pose.y - tb1.pose.y])
        translation_tb1_direct = R_world_to_tb1[:2, :2] @ vec_tb1_to_tb2_world
        
        # Extract rotation angle from rotation matrix
        rotation_angle = math.atan2(R_tb1_to_tb2[1, 0], R_tb1_to_tb2[0, 0])
        
        # Construct homogeneous transformation matrix (3x3)
        # T = [R  t]
        #     [0  1]
        # where R is 2x2 rotation matrix and t is 2x1 translation vector
        T_tb1_to_tb2 = np.zeros((3, 3))
        T_tb1_to_tb2[0:2, 0:2] = R_tb1_to_tb2[:2, :2]  # 2x2 rotation part
        T_tb1_to_tb2[0:2, 2] = translation_tb1  # 2x1 translation part
        T_tb1_to_tb2[2, 2] = 1.0  # bottom right corner
        
        return {
            'translation': translation_tb1,
            'translation_direct': translation_tb1_direct,
            'rotation_matrix': R_tb1_to_tb2,
            'rotation_angle': rotation_angle,
            'transformation_matrix': T_tb1_to_tb2,
            'd1': d1,
            'd2': d2,
            'object_pos': (obj_x, obj_y)
        }
    
    def step(self, dt=0.1):
        """Run one simulation step"""
        map_grid = self.topics.get('/map')
        if not map_grid:
            return

        for name, robot in self.robots.items():
            v, omega = robot.last_cmd
            robot.velocity = v

            # Apply commanded motion with obstacle/path checking
            robot_radius = 0.15
            new_x = robot.pose.x + dt * v * math.cos(robot.pose.theta)
            new_y = robot.pose.y + dt * v * math.sin(robot.pose.theta)
            new_theta = wrap_angle(robot.pose.theta + dt * omega)

            collision_detected = False
            if abs(v) > 1e-3:
                num_checks = max(5, int(abs(v) * dt / 0.05))  # sample every 5cm along path
                for i in range(num_checks + 1):
                    alpha = i / num_checks if num_checks > 0 else 1.0
                    check_x = robot.pose.x + alpha * (new_x - robot.pose.x)
                    check_y = robot.pose.y + alpha * (new_y - robot.pose.y)
                    if self.map_gen.is_occupied(check_x, check_y, map_grid, radius=robot_radius):
                        collision_detected = True
                        break

            if collision_detected:
                # Block translation if path is obstructed; allow in-place rotation
                new_x, new_y = robot.pose.x, robot.pose.y
                v = 0.0

            robot.pose.x = new_x
            robot.pose.y = new_y
            robot.pose.theta = new_theta

            # Generate LIDAR scan for this robot
            scan = self.lidar.generate_scan(robot.pose, map_grid)
            self.topics.publish(f'/{name}/scan', scan)

            # Record occupied voxels from scan endpoints (object hits)
            detected = self.detected_voxels.get(name, set())
            for r, a in zip(scan.ranges, scan.angles):
                if r < scan.max_range:  # Hit something
                    hit_x = robot.pose.x + r * math.cos(robot.pose.theta + a)
                    hit_y = robot.pose.y + r * math.sin(robot.pose.theta + a)
                    gx = int((hit_x - map_grid.origin_x) / map_grid.resolution)
                    gy = int((hit_y - map_grid.origin_y) / map_grid.resolution)
                    if 0 <= gx < map_grid.width and 0 <= gy < map_grid.height:
                        detected.add((gx, gy))
            self.detected_voxels[name] = detected

            # Update localization
            mcl = self.robot_localizers.get(name)
            if mcl:
                # Predict with zero motion to keep filter stable
                mcl.predict(0.0, 0.0, dt)
                mcl.update(scan)
                pose_est, cov = mcl.get_estimate()
                if pose_est:
                    self.topics.publish(f'/{name}/amcl_pose', {'pose': pose_est, 'cov': cov})
        
        # Compute object estimates from detected voxels
        self.compute_object_estimates(map_grid)
        
        # Calculate and print transform from tb1 to tb2 (after all robots updated)
        transform_info = self.calculate_tb1_to_tb2_transform()
        if transform_info:
            print("\n" + "="*60)
            print("Transform from tb1 to tb2 (calculated via object distances):")
            print(f"  Distance tb1 → object (d1): {transform_info['d1']:.4f} m")
            print(f"  Distance object → tb2 (d2): {transform_info['d2']:.4f} m")
            print(f"  Translation in tb1 frame: [{transform_info['translation'][0]:.4f}, {transform_info['translation'][1]:.4f}] m")
            print(f"  Rotation angle: {math.degrees(transform_info['rotation_angle']):.4f}° ({transform_info['rotation_angle']:.4f} rad)")
            print(f"  Rotation matrix R_tb1_to_tb2:")
            R = transform_info['rotation_matrix']
            print(f"    [{R[0,0]:8.4f}, {R[0,1]:8.4f}]")
            print(f"    [{R[1,0]:8.4f}, {R[1,1]:8.4f}]")
            print(f"  Homogeneous transformation matrix T_tb1_to_tb2 (3x3):")
            T = transform_info['transformation_matrix']
            print(f"    [{T[0,0]:8.4f}, {T[0,1]:8.4f}, {T[0,2]:8.4f}]")
            print(f"    [{T[1,0]:8.4f}, {T[1,1]:8.4f}, {T[1,2]:8.4f}]")
            print(f"    [{T[2,0]:8.4f}, {T[2,1]:8.4f}, {T[2,2]:8.4f}]")
            print(f"  Object position (world): ({transform_info['object_pos'][0]:.4f}, {transform_info['object_pos'][1]:.4f})")
            print("="*60)
    
    def visualize(self, ax=None):
        """Visualize current state"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        
        def draw_frame(origin_x: float, origin_y: float, theta: float, color: str, label: str):
            """Draw a tiny coordinate frame at given pose."""
            axis_len = 0.15
            # x-axis
            ax.arrow(
                origin_x, origin_y,
                axis_len * math.cos(theta),
                axis_len * math.sin(theta),
                head_width=0.04, head_length=0.025, fc=color, ec=color,
                alpha=0.9, linewidth=1.2, label=label
            )
            # y-axis (theta + 90 deg)
            ax.arrow(
                origin_x, origin_y,
                axis_len * math.cos(theta + math.pi / 2),
                axis_len * math.sin(theta + math.pi / 2),
                head_width=0.04, head_length=0.025, fc=color, ec=color,
                alpha=0.7, linewidth=1.0
            )
        
        # Draw map
        map_grid = self.topics.get('/map')
        if map_grid:
            for i in range(map_grid.width * map_grid.height):
                if map_grid.data[i] > 50:
                    x = (i % map_grid.width) * map_grid.resolution + map_grid.origin_x
                    y = (i // map_grid.width) * map_grid.resolution + map_grid.origin_y
                    rect = Rectangle((x, y), map_grid.resolution, map_grid.resolution,
                                    facecolor='black', edgecolor='none')
                    ax.add_patch(rect)
        
        # Draw robots and their localization
        for name, robot in self.robots.items():
            robot_pose = robot.pose
            ax.plot(robot_pose.x, robot_pose.y, marker='o', color=robot.color,
                    markersize=10, label=f'{name} pose')
            ax.arrow(robot_pose.x, robot_pose.y,
                     0.1 * math.cos(robot_pose.theta),
                     0.1 * math.sin(robot_pose.theta),
                     head_width=0.05, head_length=0.03, fc=robot.color, ec=robot.color)
            # Robot body frame
            draw_frame(robot_pose.x, robot_pose.y, robot_pose.theta, robot.color, label=f'{name} frame')

            mcl = self.robot_localizers.get(name)
            if mcl and mcl.initialized:
                particles = mcl.particles
                ax.scatter(particles[:, 0], particles[:, 1], c=robot.color, s=1, alpha=0.2,
                           label=f'{name} particles')

                pose_est_data = self.topics.get(f'/{name}/amcl_pose')
                if pose_est_data:
                    pose_est = pose_est_data['pose']
                    ax.plot(pose_est.x, pose_est.y, marker='x', color=robot.color,
                            markersize=8, label=f'{name} estimate')

            # Draw sampled LIDAR beams
            scan = self.topics.get(f'/{name}/scan')
            if scan:
                for i in range(0, len(scan.ranges), 10):  # Sample every 10th beam
                    angle = scan.angles[i] + robot_pose.theta
                    range_val = scan.ranges[i]
                    if range_val < scan.max_range:
                        end_x = robot_pose.x + range_val * math.cos(angle)
                        end_y = robot_pose.y + range_val * math.sin(angle)
                        ax.plot([robot_pose.x, end_x], [robot_pose.y, end_y], 'r-', alpha=0.2, linewidth=0.5)

            # Draw voxels detected as occupied by this robot's lidar
            hits = self.detected_voxels.get(name, set())
            voxel_color = self.voxel_colors.get(name, 'orange')
            wall_margin = 0.35  # discard voxels that are actually walls
            
            # Ensure object estimates are computed (should already be done in step())
            if not hasattr(self, 'object_estimates') or name not in self.object_estimates:
                self.compute_object_estimates(map_grid)
            
            for hit_idx, (gx, gy) in enumerate(hits):
                x = gx * map_grid.resolution + map_grid.origin_x
                y = gy * map_grid.resolution + map_grid.origin_y

                # Skip wall hits (near map boundary) to avoid giant circles
                if (
                    x < map_grid.origin_x + wall_margin
                    or x > map_grid.origin_x + map_grid.width * map_grid.resolution - wall_margin
                    or y < map_grid.origin_y + wall_margin
                    or y > map_grid.origin_y + map_grid.height * map_grid.resolution - wall_margin
                ):
                    continue

                rect = Rectangle((x, y), map_grid.resolution, map_grid.resolution,
                                 facecolor=voxel_color, edgecolor='none', alpha=0.8,
                                 label=f'{name} detected voxels' if hit_idx == 0 else None)
                ax.add_patch(rect)
            
            # Draw circle fits if available (can be multiple)
            for idx, (cx, cy, radius) in enumerate(self.object_estimates.get(name, [])):
                circle = Circle(
                    (cx, cy),
                    radius,
                    edgecolor=robot.color,
                    facecolor=robot.color,
                    alpha=0.15,
                    linewidth=2.0,
                    label=f'{name} seen object' if idx == 0 else None
                )
                ax.add_patch(circle)

        # Highlight overlaps between object estimates (circle fits)
        overlap_frame_origin = None
        if hasattr(self, 'object_estimates') and len(self.object_estimates) >= 2:
            entries = []
            for robot_name, circles in self.object_estimates.items():
                for cx, cy, r in circles:
                    entries.append((robot_name, cx, cy, r))

            added_label = False
            for i in range(len(entries)):
                for j in range(i + 1, len(entries)):
                    robot_a, cx_a, cy_a, r_a = entries[i]
                    robot_b, cx_b, cy_b, r_b = entries[j]
                    if robot_a == robot_b:
                        continue  # only highlight overlaps across robots
                    dx = cx_b - cx_a
                    dy = cy_b - cy_a
                    d = math.hypot(dx, dy)
                    if d >= r_a + r_b or d == 0:
                        continue  # no overlap or concentric (skip zero division)
                    # Approximate overlap region as a circle centered between the two centers
                    shared_radius = max(0.05, (r_a + r_b - d) * 0.5)
                    ox = cx_a + dx * 0.5
                    oy = cy_a + dy * 0.5
                    overlap_circle = Circle(
                        (ox, oy),
                        shared_radius,
                        edgecolor='red',
                        facecolor='red',
                        alpha=0.25,
                        linewidth=2.0,
                        label='estimate overlap' if not added_label else None,
                    )
                    ax.add_patch(overlap_circle)
                    overlap_frame_origin = (ox, oy)
                    added_label = True
        # Draw a frame at the center of the overlap estimate (world-aligned)
        if overlap_frame_origin:
            draw_frame(overlap_frame_origin[0], overlap_frame_origin[1], 0.0, 'red', label='overlap frame')
        
        # Calculate and display transform from tb1 to tb2
        transform_info = self.calculate_tb1_to_tb2_transform()
        if transform_info:
            # Draw arrow representing tb1 → tb2 transform (translation in world frame)
            tb1_pose = self.robots['tb1'].pose
            trans_tb1 = transform_info['translation']
            # Rotate translation from tb1 frame to world frame
            dx_world = math.cos(tb1_pose.theta) * trans_tb1[0] - math.sin(tb1_pose.theta) * trans_tb1[1]
            dy_world = math.sin(tb1_pose.theta) * trans_tb1[0] + math.cos(tb1_pose.theta) * trans_tb1[1]
            arrow_len = math.hypot(dx_world, dy_world)
            if arrow_len > 1e-3:
                arrow_scale = 1.0  # keep real scale; change to <1 to shorten if desired
                ax.arrow(
                    tb1_pose.x, tb1_pose.y,
                    dx_world * arrow_scale, dy_world * arrow_scale,
                    head_width=0.08, head_length=0.05,
                    fc='magenta', ec='magenta', alpha=0.8,
                    linewidth=2.0, length_includes_head=True,
                    label='tb1 → tb2 transform'
                )

            # Visualize distance vectors to the shared object (world frame)
            obj_x, obj_y = transform_info['object_pos']
            tb2_pose = self.robots['tb2'].pose
            # tb1 → object
            ax.arrow(
                tb1_pose.x, tb1_pose.y,
                obj_x - tb1_pose.x, obj_y - tb1_pose.y,
                head_width=0.06, head_length=0.04,
                fc='blue', ec='blue', alpha=0.7, linewidth=1.8,
                length_includes_head=True, label='tb1 → object'
            )
            # object → tb2
            ax.arrow(
                obj_x, obj_y,
                tb2_pose.x - obj_x, tb2_pose.y - obj_y,
                head_width=0.06, head_length=0.04,
                fc='green', ec='green', alpha=0.7, linewidth=1.8,
                length_includes_head=True, label='object → tb2'
            )

            # Create text box with transform information
            T = transform_info['transformation_matrix']
            # Use the translation stored in the homogeneous matrix to keep the
            # display consistent with the matrix contents.
            t_x, t_y = T[0, 2], T[1, 2]
            text_lines = [
                "Transform tb1 → tb2 (via object):",
                f"  d1 (tb1→obj): {transform_info['d1']:.3f} m",
                f"  d2 (obj→tb2): {transform_info['d2']:.3f} m",
                f"  Translation (tb1 frame):",
                f"    x: {t_x:.3f} m",
                f"    y: {t_y:.3f} m",
                f"  Rotation angle: {math.degrees(transform_info['rotation_angle']):.2f}°",
                f"  Transformation matrix T:",
                f"    [{T[0,0]:6.3f}, {T[0,1]:6.3f}, {T[0,2]:6.3f}]",
                f"    [{T[1,0]:6.3f}, {T[1,1]:6.3f}, {T[1,2]:6.3f}]",
                f"    [{T[2,0]:6.3f}, {T[2,1]:6.3f}, {T[2,2]:6.3f}]",
                f"  Object pos: ({transform_info['object_pos'][0]:.3f}, {transform_info['object_pos'][1]:.3f})"
            ]
            
            # Add text box to plot
            textstr = '\n'.join(text_lines)
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.8, edgecolor='black', linewidth=1.5)
            ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
                   verticalalignment='top', bbox=props, family='monospace')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Standalone Simulation')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        
        return ax


def main():
    """Run standalone simulation"""
    print("=" * 50)
    print("Standalone Simulation (No ROS2 Required)")
    print("=" * 50)
    dt = 0.1
    iteration = 1
    while True:
        print(f"\nRun {iteration}: generating random map, obstacle, and robot poses...")
        sim = StandaloneSimulation(randomize=True, num_objects=6)
        
        # Run a single lidar sweep step, then visualize detections
        print("Running a single-step lidar sweep...")
        sim.step(dt)
        positions = ", ".join(
            [f"{name}=({robot.pose.x:.2f}, {robot.pose.y:.2f})"
             for name, robot in sim.robots.items()]
        )
        print(f"Step 0: {positions}")
        print("Simulation complete!")
        
        # Visualize final state with detected voxels overlay
        print("Generating visualization (orange = voxels detected as objects)...")
        sim.visualize()
        print("Close the plot window to randomize and regenerate (Ctrl+C to exit).")
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nExiting standalone simulation loop.")
            break
        finally:
            plt.close('all')
        iteration += 1


if __name__ == '__main__':
    main()

