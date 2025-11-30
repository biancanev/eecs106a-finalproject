#!/usr/bin/env python3
"""
Standalone simulation that runs without ROS2
Simulates the entire system using pure Python
"""
import numpy as np
import time
import threading
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import math

# Import our core modules (without ROS2 dependencies)
try:
    from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC
except ImportError:
    # If running as script directly
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC


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
    
    def create_map(self) -> OccupancyGrid:
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
        
        # Add obstacles
        obstacles = [
            (30, 30, 5),
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
    
    def is_occupied(self, x: float, y: float, map_grid: OccupancyGrid) -> bool:
        """Check if a world coordinate is occupied"""
        gx, gy = self.world_to_grid(x, y)
        if gx < 0 or gx >= map_grid.width or gy < 0 or gy >= map_grid.height:
            return True  # Out of bounds = occupied
        idx = gy * map_grid.width + gx
        return map_grid.data[idx] > 50


class FakeLidar:
    """Generates fake LIDAR scans"""
    def __init__(self, map_gen: MapGenerator):
        self.map_gen = map_gen
        self.num_beams = 360
        self.max_range = 3.5
        self.min_range = 0.12
        self.range_noise_std = 0.02
    
    def raycast(self, x: float, y: float, theta: float, map_grid: OccupancyGrid) -> float:
        """Raycast from position in direction theta"""
        resolution = map_grid.resolution
        step_size = resolution * 0.5
        
        current_x = x
        current_y = y
        distance = 0.0
        
        dx = math.cos(theta) * step_size
        dy = math.sin(theta) * step_size
        
        while distance < self.max_range:
            if self.map_gen.is_occupied(current_x, current_y, map_grid):
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


class StandaloneSimulation:
    """Main simulation class"""
    def __init__(self):
        self.topics = TopicManager()
        self.map_gen = MapGenerator()
        self.lidar = FakeLidar(self.map_gen)
        self.mcl = SimpleMCL(N=100)
        self.kf = SimpleKF()
        self.mpc = SimpleUnicycleMPC(horizon=10, dt=0.1)
        
        # Robot state
        self.robot_pose = Pose(x=0.0, y=0.0, theta=0.0)
        self.robot_velocity = 0.0
        self.target_position = np.array([1.5, 1.5])
        
        # Control
        self.last_cmd = [0.0, 0.0]  # [v, omega]
        
        # Setup
        self.setup()
    
    def setup(self):
        """Initialize simulation"""
        # Generate map
        map_grid = self.map_gen.create_map()
        self.topics.publish('/map', map_grid)
        
        # Initialize MCL
        self.mcl.initialize(map_grid)
        
        # Subscribe to topics
        self.topics.subscribe('/cmd_vel', self.cmd_vel_callback)
    
    def cmd_vel_callback(self, cmd):
        """Handle velocity commands"""
        self.last_cmd = [cmd['linear_x'], cmd['angular_z']]
    
    def step(self, dt=0.1):
        """Run one simulation step"""
        # Update robot pose based on last command
        v, omega = self.last_cmd
        self.robot_pose.x += dt * v * math.cos(self.robot_pose.theta)
        self.robot_pose.y += dt * v * math.sin(self.robot_pose.theta)
        self.robot_pose.theta += dt * omega
        self.robot_pose.theta = np.mod(self.robot_pose.theta + math.pi, 2 * math.pi) - math.pi
        self.robot_velocity = v
        
        # Generate LIDAR scan
        map_grid = self.topics.get('/map')
        if map_grid:
            scan = self.lidar.generate_scan(self.robot_pose, map_grid)
            self.topics.publish('/scan', scan)
            
            # Update MCL
            self.mcl.predict(v, omega, dt)
            self.mcl.update(scan)
            pose_est, cov = self.mcl.get_estimate()
            if pose_est:
                self.topics.publish('/amcl_pose', {'pose': pose_est, 'cov': cov})
        
        # Update target KF (simulate target movement)
        self.kf.predict()
        # Simulate target measurement (with noise)
        target_meas = self.target_position + np.random.normal(0, 0.05, 2)
        self.kf.update(target_meas)
        target_pos = self.kf.get_position()
        self.topics.publish('/target_estimate', {'x': target_pos[0], 'y': target_pos[1]})
        
        # Run MPC
        pose_est_data = self.topics.get('/amcl_pose')
        target_est_data = self.topics.get('/target_estimate')
        
        if pose_est_data and target_est_data:
            pose_est = pose_est_data['pose']
            target_x = target_est_data['x']
            target_y = target_est_data['y']
            
            # Build state: [px, py, theta, v]
            x0 = [pose_est.x, pose_est.y, pose_est.theta, self.robot_velocity]
            target = [target_x, target_y]
            
            try:
                a_cmd, omega_cmd = self.mpc.solve(x0, target)
                v_cmd = np.clip(self.robot_velocity + a_cmd * dt, 0.0, 0.6)
                
                cmd = {'linear_x': float(v_cmd), 'angular_z': float(omega_cmd)}
                self.topics.publish('/cmd_vel', cmd)
                self.last_cmd = [v_cmd, omega_cmd]
            except Exception as e:
                print(f"MPC error: {e}")
    
    def visualize(self, ax=None):
        """Visualize current state"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        
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
        
        # Draw robot
        robot_pose = self.robot_pose
        ax.plot(robot_pose.x, robot_pose.y, 'bo', markersize=10, label='Robot')
        ax.arrow(robot_pose.x, robot_pose.y,
                0.1 * math.cos(robot_pose.theta),
                0.1 * math.sin(robot_pose.theta),
                head_width=0.05, head_length=0.03, fc='blue', ec='blue')
        
        # Draw MCL particles
        if self.mcl.initialized:
            particles = self.mcl.particles
            ax.scatter(particles[:, 0], particles[:, 1], c='cyan', s=1, alpha=0.3, label='MCL Particles')
            
            # Draw pose estimate
            pose_est_data = self.topics.get('/amcl_pose')
            if pose_est_data:
                pose_est = pose_est_data['pose']
                ax.plot(pose_est.x, pose_est.y, 'go', markersize=8, label='MCL Estimate')
        
        # Draw target
        target_est_data = self.topics.get('/target_estimate')
        if target_est_data:
            tx = target_est_data['x']
            ty = target_est_data['y']
            ax.plot(tx, ty, 'ro', markersize=10, label='Target')
        
        # Draw LIDAR scan
        scan = self.topics.get('/scan')
        if scan:
            for i in range(0, len(scan.ranges), 10):  # Sample every 10th beam
                angle = scan.angles[i] + robot_pose.theta
                range_val = scan.ranges[i]
                if range_val < scan.max_range:
                    end_x = robot_pose.x + range_val * math.cos(angle)
                    end_y = robot_pose.y + range_val * math.sin(angle)
                    ax.plot([robot_pose.x, end_x], [robot_pose.y, end_y], 'r-', alpha=0.3, linewidth=0.5)
        
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
    
    sim = StandaloneSimulation()
    
    # Run simulation
    dt = 0.1
    num_steps = 100
    
    print(f"\nRunning simulation for {num_steps} steps ({num_steps * dt:.1f} seconds)...")
    print("Press Ctrl+C to stop early\n")
    
    try:
        for step in range(num_steps):
            sim.step(dt)
            
            if step % 10 == 0:
                robot_pose = sim.robot_pose
                target_data = sim.topics.get('/target_estimate')
                if target_data:
                    dist = math.sqrt((robot_pose.x - target_data['x'])**2 + 
                                    (robot_pose.y - target_data['y'])**2)
                    print(f"Step {step:3d}: Robot=({robot_pose.x:.2f}, {robot_pose.y:.2f}), "
                          f"Target=({target_data['x']:.2f}, {target_data['y']:.2f}), "
                          f"Dist={dist:.2f}m")
            
            time.sleep(dt * 0.1)  # Slow down for visualization
        
        print("\nSimulation complete!")
        
        # Visualize final state
        print("\nGenerating visualization...")
        sim.visualize()
        plt.show()
        
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
        sim.visualize()
        plt.show()


if __name__ == '__main__':
    main()

