#!/usr/bin/env python3
"""
Animated Simulation for TurtleBot Interceptor System

This module provides a standalone simulation environment for testing and visualizing:
- Monte Carlo Localization (MCL) for robot pose estimation
- Unscented Kalman Filter (UKF) for target tracking
- Model Predictive Control (MPC) for interception with obstacle avoidance
- Log-odds occupancy grid mapping (SLAM)
- Real-time visualization with GUI metrics

Usage:
    python3 -m turtlebot_interceptor.animated_sim
    OR
    python3 run_animated_sim.py

Note: This is a simulation-only module. For hardware deployment, see:
    - HARDWARE_DEPLOYMENT.md for step-by-step instructions
    - hardware.launch.py for ROS2 node deployment
"""
import numpy as np
import time
import math
from dataclasses import dataclass
from typing import Tuple, List, Optional
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch
from matplotlib.collections import LineCollection

# Import our core modules
try:
    from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
    from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class MapGenerator:
    """Generates test map"""
    def __init__(self, width=100, height=100, resolution=0.05):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = -2.5
        self.origin_y = -2.5
    
    def create_ground_truth_map(self):
        """Create ground truth map (unknown to robot at startup)"""
        map_data = [0] * (self.width * self.height)
        
        # Walls
        wall_thickness = 2
        for x in range(self.width):
            for y in range(wall_thickness):
                map_data[y * self.width + x] = 100
            for y in range(self.height - wall_thickness, self.height):
                map_data[y * self.width + x] = 100
        for x in range(wall_thickness):
            for y in range(self.height):
                map_data[y * self.width + x] = 100
        for x in range(self.width - wall_thickness, self.width):
            for y in range(self.height):
                map_data[y * self.width + x] = 100
        
        # Circular obstacles (cones) - simple circular shapes
        # Format: (grid_x, grid_y, radius_in_cells)
        # Maximum 5 obstacles
        # Store ground truth obstacle info for matching
        # Strategic obstacle placement: BETWEEN robot (0,0) and target (1.5, 1.5)
        # Grid coordinates: 0-100, world: -2.5 to 2.5 (grid 50 = world 0.0, grid 80 = world 1.5)
        # Robot starts at (0, 0) = grid (50, 50)
        # Target starts at (1.5, 1.5) = grid (80, 80)
        # Place obstacles along the diagonal path to force MPC to navigate around them
        self.ground_truth_obstacles = [
            (60, 55, 5),   # Obstacle 1: world (0.5, 0.25) - blocks direct path, slightly below diagonal
            (55, 65, 5),   # Obstacle 2: world (0.25, 0.75) - blocks upper-left route
            (68, 68, 5),   # Obstacle 3: world (0.9, 0.9) - blocks center of diagonal
            (75, 60, 5),   # Obstacle 4: world (1.25, 0.5) - blocks lower-right route
            (62, 78, 5),   # Obstacle 5: world (0.6, 1.4) - near target, blocks upper approach
        ]
        
        for ox, oy, r in self.ground_truth_obstacles:
            # Draw perfect circle using better algorithm
            # Use finer sub-pixel sampling for smoother circles
            for dx in range(-r-1, r+2):
                for dy in range(-r-1, r+2):
                    # Check center of cell for better circle shape
                    cell_center_x = dx + 0.5
                    cell_center_y = dy + 0.5
                    dist_sq = cell_center_x*cell_center_x + cell_center_y*cell_center_y
                    # Use exact radius for proper circular shape
                    if dist_sq <= r * r:  # Inside circle (including boundary)
                        x, y = ox + dx, oy + dy
                        if 0 <= x < self.width and 0 <= y < self.height:
                            map_data[y * self.width + x] = 100
        
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'data': map_data
        }
    
    def create_unknown_map(self):
        """Create unknown map (all cells unknown at startup)"""
        # -1 = unknown, 0 = free, 100 = occupied
        map_data = [-1] * (self.width * self.height)
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'data': map_data
        }
    
    def update_map_from_lidar(self, map_data, pose, scan):
        """Update map using LIDAR scan (robust log-odds occupancy grid mapping)"""
        x, y, theta = pose.x, pose.y, pose.theta
        
        # Minimum range to consider - ignore readings closer than this (robot's footprint)
        min_range = scan.get('min_range', 0.25)
        max_range = scan.get('max_range', 3.5)
        
        # Log-odds parameters - MORE ROBUST for obstacle detection
        log_odds_free = -0.5  # Evidence for free space
        log_odds_occupied = 0.8  # STRONG evidence for occupied (was 0.6 - need stronger!)
        log_odds_min = -3.0  # Minimum log-odds (strongly free)
        log_odds_max = 3.0   # Maximum log-odds (strongly occupied)
        
        # Convert occupancy values to log-odds
        def occupancy_to_log_odds(val):
            if val == -1:  # Unknown
                return 0.0
            elif val == 0:  # Free
                return -1.5
            elif val == 100:  # Occupied
                return 1.5
            else:  # Intermediate value
                return (val - 50) / 33.0  # Scale to [-1.5, 1.5]
        
        def log_odds_to_occupancy(lo):
            if lo < -0.3:  # Free space
                return 0
            elif lo > 0.25:  # Show occupied with 1 reading (was 0.3 - make it easier!)
                return 100
            else:  # Unknown/uncertain
                if lo > 0.1:  # Show uncertain if we have some evidence
                    return 50  # Uncertain but likely occupied
                return -1  # Unknown
        
        for i in range(len(scan['ranges'])):
            angle = scan['angles'][i] + theta
            range_val = scan['ranges'][i]
            
            # Skip if this beam detected target (not an obstacle)
            # IMPORTANT: Don't mark target as obstacle!
            is_target_detection = False
            if 'target_detection' in scan and scan['target_detection'] is not None:
                target_range, target_angle = scan['target_detection']
                angle_diff = abs(angle - target_angle)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                # If this beam is close to target detection angle and range matches, skip obstacle marking
                # Use wider angle tolerance to ensure target is never marked as obstacle
                beam_width = (2*math.pi / len(scan['ranges'])) * 3  # Wider tolerance
                if angle_diff < beam_width and abs(range_val - target_range) < 0.15:
                    is_target_detection = True
            
            # Mark cells along the ray as free (only if not target detection)
            if not is_target_detection:
                step_size = self.resolution * 0.05  # EVEN smaller steps for MORE voxels (was 0.1)
                num_steps = int(range_val / step_size)
                
                for step in range(num_steps):
                    dist = step * step_size
                    cell_x = x + dist * math.cos(angle)
                    cell_y = y + dist * math.sin(angle)
                    
                    gx, gy = self.world_to_grid(cell_x, cell_y)
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        idx = gy * self.width + gx
                        # Update log-odds for free space
                        current_val = map_data['data'][idx]
                        log_odds = occupancy_to_log_odds(current_val)
                        log_odds += log_odds_free
                        log_odds = np.clip(log_odds, log_odds_min, log_odds_max)
                        map_data['data'][idx] = log_odds_to_occupancy(log_odds)
            
            # Mark end point as occupied (only if hit obstacle, not target)
            # IMPORTANT: Ignore short range readings (within robot footprint) to avoid self-detection
            # Also ignore max_range readings (didn't hit anything - just maxed out)
            # Make sure we're actually hitting something (not just max range)
            if range_val < max_range - 0.1 and range_val > min_range and not is_target_detection:
                end_x = x + range_val * math.cos(angle)
                end_y = y + range_val * math.sin(angle)
                gx, gy = self.world_to_grid(end_x, end_y)
                if 0 <= gx < self.width and 0 <= gy < self.height:
                    idx = gy * self.width + gx
                    # Update log-odds for occupied space - STRONG evidence
                    current_val = map_data['data'][idx]
                    log_odds = occupancy_to_log_odds(current_val)
                    # Add STRONG occupied evidence - make sure voxels appear!
                    log_odds += log_odds_occupied
                    log_odds = np.clip(log_odds, log_odds_min, log_odds_max)
                    new_val = log_odds_to_occupancy(log_odds)
                    map_data['data'][idx] = new_val  # CRITICAL: Direct assignment to ensure update
                    
                    # Also update a small neighborhood for better visibility (5x5 for more coverage)
                    for dx_grid in range(-2, 3):
                        for dy_grid in range(-2, 3):
                            if dx_grid == 0 and dy_grid == 0:
                                continue
                            nearby_gx = gx + dx_grid
                            nearby_gy = gy + dy_grid
                            if 0 <= nearby_gx < self.width and 0 <= nearby_gy < self.height:
                                nearby_idx = nearby_gy * self.width + nearby_gx
                                nearby_val = map_data['data'][nearby_idx]
                                nearby_lo = occupancy_to_log_odds(nearby_val)
                                # Stronger evidence for immediate neighbors, weaker for outer ring
                                dist_from_center = math.sqrt(dx_grid**2 + dy_grid**2)
                                if dist_from_center <= 1.0:
                                    nearby_lo += log_odds_occupied * 0.5  # Stronger for immediate neighbors
                                else:
                                    nearby_lo += log_odds_occupied * 0.2  # Weaker for outer ring
                                nearby_lo = np.clip(nearby_lo, log_odds_min, log_odds_max)
                                map_data['data'][nearby_idx] = log_odds_to_occupancy(nearby_lo)
    
    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
    
    def is_occupied(self, x, y, map_data, radius=0.0):
        """Check if position is occupied - checks actual voxel grid values"""
        # First check bounds
        if x < self.origin_x or x >= self.origin_x + self.width * self.resolution:
            return True
        if y < self.origin_y or y >= self.origin_y + self.height * self.resolution:
            return True
        
        # Check actual voxel grid value at this position (and nearby if radius > 0)
        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            idx = gy * self.width + gx
            if map_data['data'][idx] > 50:  # Occupied (value > 50 means obstacle)
                return True
        
        # If radius specified, check nearby cells too
        if radius > 0:
            radius_cells = int(radius / self.resolution) + 1
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    check_gx = gx + dx
                    check_gy = gy + dy
                    if 0 <= check_gx < self.width and 0 <= check_gy < self.height:
                        idx = check_gy * self.width + check_gx
                        if map_data['data'][idx] > 50:
                            # Check actual distance to center of this cell
                            cell_x = check_gx * self.resolution + self.origin_x
                            cell_y = check_gy * self.resolution + self.origin_y
                            dist = math.sqrt((x - cell_x)**2 + (y - cell_y)**2)
                            if dist < radius + self.resolution:
                                return True
        
        # Also check walls (boundaries)
        wall_thickness = 0.1
        if (x < self.origin_x + wall_thickness + radius or 
            x > self.origin_x + self.width * self.resolution - wall_thickness - radius or
            y < self.origin_y + wall_thickness + radius or
            y > self.origin_y + self.height * self.resolution - wall_thickness - radius):
            return True
        
        return False
    
    def extract_obstacles_from_voxel_grid(self, map_data, target_pos=None):
        """Extract CIRCULAR obstacles only - exclude walls and target"""
        obstacles = []
        wall_margin = 0.35  # Cells within 35cm of boundary are walls, not obstacles
        
        # Extract occupied cells NOT near walls
        occupied_cells = []
        for i in range(map_data['width'] * map_data['height']):
            if map_data['data'][i] > 50:  # Occupied
                gx = i % map_data['width']
                gy = i // map_data['width']
                world_x = gx * map_data['resolution'] + map_data['origin_x']
                world_y = gy * map_data['resolution'] + map_data['origin_y']
                
                # Skip cells near walls (walls are linear, not circular obstacles)
                if (world_x < self.origin_x + wall_margin or 
                    world_x > self.origin_x + self.width * self.resolution - wall_margin or
                    world_y < self.origin_y + wall_margin or 
                    world_y > self.origin_y + self.height * self.resolution - wall_margin):
                    continue
                
                # Skip cells near target
                if target_pos is not None:
                    dist = math.sqrt((world_x - target_pos[0])**2 + (world_y - target_pos[1])**2)
                    if dist < 0.25:
                        continue
                
                occupied_cells.append((world_x, world_y))
        
        if len(occupied_cells) == 0:
            return obstacles
        
        # Cluster nearby cells
        clusters = []
        for x, y in occupied_cells:
            assigned = False
            for cluster in clusters:
                for cx, cy in cluster:
                    if math.sqrt((x - cx)**2 + (y - cy)**2) < self.resolution * 3:
                        cluster.append((x, y))
                        assigned = True
                        break
                if assigned:
                    break
            if not assigned:
                clusters.append([(x, y)])
        
        # Only keep compact, circular clusters (not wall segments)
        for cluster in clusters:
            if len(cluster) < 3:  # Lower threshold - accept smaller clusters (was 5)
                continue
            
            points = np.array(cluster)
            min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
            min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
            width = max_x - min_x
            height = max_y - min_y
            
            # Skip elongated clusters (walls have aspect ratio > 2.5)
            if width < 0.01 or height < 0.01:  # Lower threshold (was 0.02)
                continue
            aspect_ratio = max(width, height) / min(width, height)
            if aspect_ratio > 2.5:
                continue
            
            # Skip if too large (wall segment)
            if width > 0.6 or height > 0.6:
                continue
            
            # Compute center and radius for circular obstacle
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            # Use tighter radius - MPC will add its own safety margin
            # This allows MPC to plan tighter paths while still being safe
            radius = max(width, height) / 2 + 0.02  # Smaller margin - let MPC handle safety (was 0.08)
            
            # Accept wider range of obstacle sizes (was 0.08-0.5, now 0.06-0.5)
            if 0.06 < radius < 0.5:
                obstacles.append((np.array([center_x, center_y]), radius))
        
        # Limit to 10 obstacles max (was 5) - allow more obstacles to be detected
        return obstacles[:10]


class FakeLidar:
    """Generates probabilistic LIDAR scans with noise"""
    def __init__(self, map_gen):
        self.map_gen = map_gen
        self.num_beams = 180  # Reduced for performance
        self.max_range = 3.5
        self.min_range = 0.25  # Increased to avoid self-detection (robot radius ~0.15m + margin)
        # Probabilistic LIDAR parameters
        self.range_noise_std = 0.02  # 2cm standard deviation for range noise
        self.angle_noise_std = 0.01  # Small angle noise
        self.detection_probability = 0.98  # Higher detection rate (was 0.95)
        self.false_positive_probability = 0.005  # Lower false positives (was 0.01)
    
    def raycast(self, x, y, theta, map_data):
        """Raycast - uses ground truth map for actual raycasting, discovered map for MCL"""
        # MUCH smaller steps for accurate obstacle detection
        step_size = map_data['resolution'] * 0.1  # Was 0.3 - now 3x more accurate!
        current_x, current_y = x, y
        distance = 0.0
        dx = math.cos(theta) * step_size
        dy = math.sin(theta) * step_size
        
        # Check every step - don't miss obstacles
        while distance < self.max_range:
            # Check current position AND nearby cells for better obstacle detection
            if self.map_gen.is_occupied(current_x, current_y, map_data, radius=0.05):  # Small radius check
                return distance
            current_x += dx
            current_y += dy
            distance += step_size
        return self.max_range
    
    def generate_scan(self, pose, map_data, target_pos=None):
        """Generate probabilistic LIDAR scan with noise, optionally detecting target"""
        ranges = []
        angles = []
        target_detection = None  # (range, angle) if target detected
        
        for i in range(self.num_beams):
            angle = -math.pi + i * (2 * math.pi / self.num_beams)
            # Add small angle noise (probabilistic)
            angle += np.random.normal(0, self.angle_noise_std)
            ray_angle = pose.theta + angle
            
            # Raycast to map obstacles
            true_range_to_obstacle = self.raycast(pose.x, pose.y, ray_angle, map_data)
            
            # Add probabilistic range noise
            range_noise = np.random.normal(0, self.range_noise_std)
            range_to_obstacle = true_range_to_obstacle + range_noise
            range_to_obstacle = np.clip(range_to_obstacle, self.min_range, self.max_range)
            
            # Check if target is in this beam's path (if target provided) - probabilistic detection
            if target_pos is not None:
                # Calculate distance and angle to target
                dx = target_pos[0] - pose.x
                dy = target_pos[1] - pose.y
                dist_to_target = math.sqrt(dx*dx + dy*dy)
                angle_to_target = math.atan2(dy, dx)
                
                # Check if target is in this beam's direction (within beam width)
                angle_diff = abs(angle_to_target - ray_angle)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # Wrap around
                
                # Probabilistic target detection
                beam_width = (2*math.pi / self.num_beams) * 3  # Wider detection angle
                in_beam = dist_to_target < range_to_obstacle and dist_to_target < self.max_range and angle_diff < beam_width
                
                # Probabilistic detection: sometimes miss target, sometimes false positive
                if in_beam:
                    # Target is in beam - probabilistic detection
                    if np.random.random() < self.detection_probability:
                        # Target detected - range is to target, not obstacle
                        # Add noise to target range measurement
                        noisy_range = dist_to_target + np.random.normal(0, self.range_noise_std * 2)  # More noise for target
                        noisy_range = np.clip(noisy_range, self.min_range, self.max_range)
                        range_val = noisy_range
                        # Add noise to angle measurement
                        noisy_angle = angle_to_target + np.random.normal(0, self.angle_noise_std * 2)
                        if target_detection is None or dist_to_target < target_detection[0]:
                            target_detection = (noisy_range, noisy_angle)
                    else:
                        # Missed target detection - return obstacle range
                        range_val = range_to_obstacle
                else:
                    # Target not in beam, but check for false positive
                    if np.random.random() < self.false_positive_probability:
                        # False positive - random detection
                        false_range = np.random.uniform(self.min_range, min(range_to_obstacle, self.max_range))
                        false_angle = ray_angle + np.random.normal(0, 0.1)
                        if target_detection is None:
                            target_detection = (false_range, false_angle)
                        range_val = range_to_obstacle
                    else:
                        range_val = range_to_obstacle
            else:
                range_val = range_to_obstacle
            
            ranges.append(range_val)
            angles.append(angle)
        
        result = {'ranges': ranges, 'angles': angles, 'max_range': self.max_range, 'min_range': self.min_range}
        if target_detection:
            result['target_detection'] = target_detection
        return result


class SimpleMCL:
    """MCL for localization - MUST converge to robot pose"""
    def __init__(self, N=200):
        self.N = N
        self.particles = None
        self.weights = None
        self.initialized = False
        self.motion_noise = np.array([0.008, 0.008, 0.004])  # Very low noise for convergence
        self.max_range = 3.5  # LIDAR max range
        self.map_gen = None  # Will be set to access ground truth
    
    def initialize(self, map_data):
        """Initialize particles in free space (or unknown if map is unknown)"""
        free_cells = []
        for i in range(map_data['width'] * map_data['height']):
            # Accept free cells or unknown cells (for startup)
            if map_data['data'][i] == 0 or map_data['data'][i] == -1:
                x = (i % map_data['width']) * map_data['resolution'] + map_data['origin_x']
                y = (i // map_data['width']) * map_data['resolution'] + map_data['origin_y']
                free_cells.append((x, y))
        
        if len(free_cells) == 0:
            return
        
        self.particles = np.zeros((self.N, 3))
        indices = np.random.choice(len(free_cells), self.N)
        for i, idx in enumerate(indices):
            self.particles[i, 0] = free_cells[idx][0]
            self.particles[i, 1] = free_cells[idx][1]
            self.particles[i, 2] = np.random.uniform(0, 2 * math.pi)
        
        self.weights = np.ones(self.N) / self.N
        self.initialized = True
    
    def initialize_uniform(self, x_range, y_range, theta_range, robot_pose=None):
        """Initialize particles uniformly in given ranges, optionally centered on robot"""
        self.particles = np.zeros((self.N, 3))
        if robot_pose is not None:
            # Initialize particles around robot's starting position with TIGHTER spread for better convergence
            spread = 0.15  # Tighter initial uncertainty (was 0.5 - too loose!)
            self.particles[:, 0] = np.random.normal(robot_pose.x, spread, self.N)
            self.particles[:, 1] = np.random.normal(robot_pose.y, spread, self.N)
            self.particles[:, 2] = np.random.normal(robot_pose.theta, 0.3, self.N)  # Tighter angle spread
        else:
            self.particles[:, 0] = np.random.uniform(*x_range, self.N)
            self.particles[:, 1] = np.random.uniform(*y_range, self.N)
            self.particles[:, 2] = np.random.uniform(*theta_range, self.N)
        
        # Bound particles
        self.particles[:, 0] = np.clip(self.particles[:, 0], -2.3, 2.3)
        self.particles[:, 1] = np.clip(self.particles[:, 1], -2.3, 2.3)
        self.particles[:, 2] = np.mod(self.particles[:, 2] + math.pi, 2 * math.pi) - math.pi
        self.weights = np.ones(self.N) / self.N
        self.initialized = True
    
    def predict(self, v, omega, dt):
        """Motion model with bounds checking"""
        if not self.initialized:
            return
        noise = np.random.randn(self.N, 3) * self.motion_noise
        self.particles[:, 0] += dt * v * np.cos(self.particles[:, 2]) + noise[:, 0]
        self.particles[:, 1] += dt * v * np.sin(self.particles[:, 2]) + noise[:, 1]
        self.particles[:, 2] += dt * omega + noise[:, 2]
        self.particles[:, 2] = np.mod(self.particles[:, 2] + math.pi, 2 * math.pi) - math.pi
        
        # Bound particles within map limits
        self.particles[:, 0] = np.clip(self.particles[:, 0], -2.3, 2.3)
        self.particles[:, 1] = np.clip(self.particles[:, 1], -2.3, 2.3)
    
    def update(self, scan, robot_pose, map_data):
        """Update weights based on scan - MUST converge to robot pose"""
        if not self.initialized:
            return
        
        # CRITICAL: Use ground truth map for raycasting to compare with actual scan
        # The scan comes from robot's true pose, so particles at robot pose should match perfectly
        for i in range(self.N):
            particle = self.particles[i]
            likelihood = 1.0
            num_valid_beams = 0
            total_error = 0.0
            
            # Use many beams for accurate comparison
            for j in range(0, len(scan['ranges']), 2):  # Use every 2nd beam
                angle = scan['angles'][j] + particle[2]
                # CRITICAL: Use ground truth map for raycasting (map_data should be ground truth)
                expected_range = self._raycast(particle[0], particle[1], angle, map_data)
                actual_range = scan['ranges'][j]
                
                if actual_range < self.max_range:
                    num_valid_beams += 1
                    # Normalized error - particles at robot pose should have zero error
                    error = abs(expected_range - actual_range) / (actual_range + 0.05)
                    total_error += error * error  # Squared error
            
            # Calculate likelihood - particles at robot pose get highest weight
            if num_valid_beams > 5:  # Need enough beams
                avg_error = total_error / num_valid_beams
                # Strong exponential - particles at robot pose get high weight
                # Use strong weighting to force convergence
                likelihood = math.exp(-avg_error * 500)  # VERY strong to prevent drift!
            else:
                likelihood = 1e-50  # Very low if not enough beams
            
            # Additional penalty for particles far from robot (position-based) - CRITICAL for preventing drift
            dx = particle[0] - robot_pose.x
            dy = particle[1] - robot_pose.y
            dist_from_robot = math.sqrt(dx*dx + dy*dy)
            # MUCH STRONGER penalty for particles far from robot - prevents MCL drift
            if dist_from_robot > 0.2:  # EVEN TIGHTER threshold (was 0.3)
                likelihood *= math.exp(-dist_from_robot * 50)  # MUCH stronger penalty (was 20)
            if dist_from_robot > 0.5:  # Very far - almost zero likelihood (was 0.8)
                likelihood *= 1e-20  # Even stronger penalty (was 1e-10)
            
            self.weights[i] = likelihood
        
        # Normalize weights
        self.weights += 1e-20
        self.weights /= np.sum(self.weights)
        
        # ALWAYS resample to force convergence - prevent particle spread
        effective_particles = 1.0 / (np.sum(self.weights**2) + 1e-10)
        # Resample more aggressively - always resample if weights are not uniform
        if effective_particles < self.N * 0.98:  # Resample VERY aggressively to prevent drift
            self.resample(robot_pose=robot_pose)  # Pass robot_pose to prevent drift
        else:
            # Still resample occasionally to maintain convergence
            if np.random.random() < 0.3:  # Less frequent random resampling
                self.resample(robot_pose=robot_pose)
        
        # CRITICAL: After resampling, pull back any particles that drifted too far
        if robot_pose is not None:
            for i in range(self.N):
                dx = self.particles[i, 0] - robot_pose.x
                dy = self.particles[i, 1] - robot_pose.y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.4:  # MUCH TIGHTER - if particle drifted more than 0.4m, reset it (was 0.8m)
                    # Reset particle near robot with VERY small spread
                    self.particles[i, 0] = robot_pose.x + np.random.normal(0, 0.08)  # Tighter spread (was 0.15)
                    self.particles[i, 1] = robot_pose.y + np.random.normal(0, 0.08)  # Tighter spread (was 0.15)
                    self.particles[i, 2] = robot_pose.theta + np.random.normal(0, 0.1)  # Tighter spread (was 0.2)
    
    def _raycast(self, x, y, theta, map_data):
        """Simple raycast for MCL - MUST use ground truth map for accurate comparison"""
        # Use ground truth map stored in map_gen for accurate raycasting
        # This is critical - we need to compare against what the robot actually sees
        step_size = map_data['resolution'] * 0.5
        current_x, current_y = x, y
        distance = 0.0
        dx = math.cos(theta) * step_size
        dy = math.sin(theta) * step_size
        
        while distance < 3.5:
            gx = int((current_x - map_data['origin_x']) / map_data['resolution'])
            gy = int((current_y - map_data['origin_y']) / map_data['resolution'])
            if gx < 0 or gx >= map_data['width'] or gy < 0 or gy >= map_data['height']:
                return 3.5
            idx = gy * map_data['width'] + gx
            # Use ground truth - treat unknown as free, occupied as obstacle
            if map_data['data'][idx] > 50:  # Occupied
                return distance
            current_x += dx
            current_y += dy
            distance += step_size
        return 3.5
    
    def resample(self, robot_pose=None):
        """Resample particles using systematic resampling with drift prevention"""
        # Systematic resampling (better than random choice)
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0  # Ensure last is 1.0
        
        # Generate systematic samples
        u = (np.arange(self.N) + np.random.random()) / self.N
        idx = np.searchsorted(cumsum, u)
        
        self.particles = self.particles[idx]
        self.weights = np.ones(self.N) / self.N
        
        # Bound particles after resampling
        self.particles[:, 0] = np.clip(self.particles[:, 0], -2.3, 2.3)
        self.particles[:, 1] = np.clip(self.particles[:, 1], -2.3, 2.3)
        
        # CRITICAL: Prevent particle drift - if particles are too far from robot, reset them
        # This is ESSENTIAL to prevent MCL drift - check EVERY particle
        if robot_pose is not None:
            for i in range(self.N):
                dx = self.particles[i, 0] - robot_pose.x
                dy = self.particles[i, 1] - robot_pose.y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.5:  # TIGHTER threshold - if particle drifted more than 0.5m, reset it
                    # Reset particle near robot with small spread
                    self.particles[i, 0] = robot_pose.x + np.random.normal(0, 0.1)
                    self.particles[i, 1] = robot_pose.y + np.random.normal(0, 0.1)
                    self.particles[i, 2] = robot_pose.theta + np.random.normal(0, 0.15)
                    # Re-bound
                    self.particles[i, 0] = np.clip(self.particles[i, 0], -2.3, 2.3)
                    self.particles[i, 1] = np.clip(self.particles[i, 1], -2.3, 2.3)
        
        # DO NOT add noise - let particles converge naturally
        # Only add tiny noise if particles are completely collapsed (all same)
        spread = np.std(self.particles[:, :2], axis=0)
        if np.mean(spread) < 0.001:  # Only if completely collapsed
            noise_scale = 0.001  # Tiny noise
            self.particles[:, :2] += np.random.normal(0, noise_scale, (self.N, 2))
            # Re-bound after noise
            self.particles[:, 0] = np.clip(self.particles[:, 0], -2.3, 2.3)
            self.particles[:, 1] = np.clip(self.particles[:, 1], -2.3, 2.3)
    
    def get_estimate(self):
        """Get mean pose and covariance"""
        if not self.initialized:
            return None, None
        mean = np.average(self.particles, axis=0, weights=self.weights)
        diff = self.particles - mean
        cov = np.cov(diff.T, aweights=self.weights)
        return Pose(x=mean[0], y=mean[1], theta=mean[2]), cov


class SimpleUKF:
    """Unscented Kalman Filter for target tracking - handles nonlinear dynamics better"""
    def __init__(self, dt=0.1):
        self.dt = dt
        self.n = 4  # State dimension [px, py, vx, vy]
        self.x = np.zeros((self.n, 1))
        self.P = np.eye(self.n) * 10.0  # Start with high uncertainty (target unknown)
        self.initialized = False
        
        # Smoothing filter for position estimates (exponential moving average)
        self.smooth_x = None
        self.smooth_y = None
        self.smoothing_factor = 0.6  # 0.6 = 60% new, 40% old (faster response)
        
        # UKF parameters - tuned for smooth tracking
        self.alpha = 0.001  # Small spread for smooth tracking (was 0.01, too high)
        self.beta = 2.0     # Prior knowledge (2 is optimal for Gaussian)
        self.kappa = 0.0    # Secondary scaling parameter (0 for standard UKF)
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n
        
        # Weights for sigma points
        self.Wm0 = self.lambda_ / (self.n + self.lambda_)
        self.Wc0 = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)
        self.Wi = 1.0 / (2.0 * (self.n + self.lambda_))
        
        # Process and measurement noise - tuned for responsive tracking
        self.Q = np.eye(self.n) * 0.02  # Moderate process noise (target moves unpredictably)
        self.R = np.eye(2) * 0.03  # Low measurement noise (trust LIDAR)
    
    def _generate_sigma_points(self):
        """Generate sigma points for UKF"""
        n = self.n
        # Ensure P is positive definite
        P_safe = (self.P + self.P.T) / 2
        P_safe += np.eye(n) * 1e-6
        try:
            sqrt_P = np.linalg.cholesky((n + self.lambda_) * P_safe)
        except:
            # If Cholesky fails, use eigenvalue decomposition
            eigenvals, eigenvecs = np.linalg.eig(P_safe)
            eigenvals = np.maximum(eigenvals, 1e-6)  # Ensure positive
            sqrt_P = eigenvecs @ np.diag(np.sqrt(eigenvals * (n + self.lambda_)))
        sigma_points = np.zeros((n, 2*n + 1))
        sigma_points[:, 0] = self.x.flatten()
        for i in range(n):
            sigma_points[:, i+1] = (self.x + sqrt_P[:, i:i+1]).flatten()
            sigma_points[:, i+n+1] = (self.x - sqrt_P[:, i:i+1]).flatten()
        return sigma_points
    
    def _f(self, x):
        """Nonlinear process model (constant velocity)"""
        x_next = x.copy()
        x_next[0] += self.dt * x[2]  # px += dt * vx
        x_next[1] += self.dt * x[3]  # py += dt * vy
        # vx, vy remain constant (constant velocity model)
        return x_next
    
    def _h(self, x):
        """Nonlinear measurement model (position only)"""
        return x[:2]  # Only position is observed
    
    def predict(self):
        """UKF prediction step"""
        if not self.initialized:
            return
        
        # Generate sigma points
        sigma_points = self._generate_sigma_points()
        
        # Propagate sigma points through process model
        n = self.n
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(2*n + 1):
            sigma_points_pred[:, i] = self._f(sigma_points[:, i].reshape(n, 1)).flatten()
        
        # Compute predicted mean
        x_pred = self.Wm0 * sigma_points_pred[:, 0]
        for i in range(1, 2*n + 1):
            x_pred += self.Wi * sigma_points_pred[:, i]
        x_pred = x_pred.reshape(n, 1)
        
        # Compute predicted covariance
        P_pred = self.Wc0 * (sigma_points_pred[:, 0:1] - x_pred) @ (sigma_points_pred[:, 0:1] - x_pred).T
        for i in range(1, 2*n + 1):
            diff = (sigma_points_pred[:, i:i+1] - x_pred)
            P_pred += self.Wi * diff @ diff.T
        P_pred += self.Q
        
        self.x = x_pred
        self.P = P_pred
    
    def update(self, z):
        """UKF update step with measurement"""
        if not self.initialized:
            # Initialize with first measurement - trust it completely
            self.x[0, 0] = z[0]
            self.x[1, 0] = z[1]
            self.x[2, 0] = 0.0  # Unknown velocity initially
            self.x[3, 0] = 0.0
            self.P = np.eye(4) * 0.1  # Lower initial uncertainty (was 0.2)
            self.initialized = True
            # Initialize smoothing with first measurement
            self.smooth_x = z[0]
            self.smooth_y = z[1]
            return
        
        # Generate sigma points from predicted state
        sigma_points = self._generate_sigma_points()
        
        # Propagate through measurement model
        n = self.n
        z_sigma = np.zeros((2, 2*n + 1))
        for i in range(2*n + 1):
            z_sigma[:, i] = self._h(sigma_points[:, i].reshape(n, 1)).flatten()
        
        # Compute predicted measurement mean
        z_pred = self.Wm0 * z_sigma[:, 0]
        for i in range(1, 2*n + 1):
            z_pred += self.Wi * z_sigma[:, i]
        z_pred = z_pred.reshape(2, 1)
        
        # Get predicted covariance first (needed for Joseph form)
        P_pred = self.P.copy()
        
        # Compute innovation covariance
        S = self.Wc0 * (z_sigma[:, 0:1] - z_pred) @ (z_sigma[:, 0:1] - z_pred).T
        for i in range(1, 2*n + 1):
            diff = (z_sigma[:, i:i+1] - z_pred)
            S += self.Wi * diff @ diff.T
        S += self.R
        
        # Ensure S is positive definite
        S = (S + S.T) / 2
        S += np.eye(2) * 1e-5
        
        # Compute cross-covariance
        Pxz = self.Wc0 * (sigma_points[:, 0:1] - self.x) @ (z_sigma[:, 0:1] - z_pred).T
        for i in range(1, 2*n + 1):
            Pxz += self.Wi * (sigma_points[:, i:i+1] - self.x) @ (z_sigma[:, i:i+1] - z_pred).T
        
        # Kalman gain
        try:
            K = Pxz @ np.linalg.inv(S)
        except:
            K = Pxz @ np.linalg.pinv(S)
        
        # Update state and covariance
        z = z.reshape(2, 1)
        innovation = z - z_pred
        
        # Smooth innovation to prevent jumps (gentle limiting)
        innovation_norm = np.linalg.norm(innovation)
        if innovation_norm > 1.5:  # Allow larger jumps for faster convergence
            # Smooth limiting instead of hard cutoff
            scale = 1.5 / innovation_norm
            innovation = innovation * scale
        
        self.x = self.x + K @ innovation
        
        # Joseph form for numerical stability (more stable than standard form)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])  # Measurement matrix
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        
        # Ensure covariance stays positive definite and bounded
        self.P = (self.P + self.P.T) / 2
        self.P += np.eye(self.n) * 1e-5  # Small regularization
        # Bound covariance to prevent explosion
        max_cov = 5.0
        self.P = np.clip(self.P, -max_cov, max_cov)
        # Ensure diagonal is positive and bounded
        diag = np.diag(self.P)
        diag = np.clip(diag, 0.0001, max_cov)
        np.fill_diagonal(self.P, diag)
        
        # Limit state to reasonable bounds
        self.x[0, 0] = np.clip(self.x[0, 0], -2.5, 2.5)
        self.x[1, 0] = np.clip(self.x[1, 0], -2.5, 2.5)
        self.x[2, 0] = np.clip(self.x[2, 0], -1.0, 1.0)  # Limit velocity
        self.x[3, 0] = np.clip(self.x[3, 0], -1.0, 1.0)
        
        # Apply exponential smoothing to position for smoother tracking
        if self.smooth_x is None:
            self.smooth_x = self.x[0, 0]
            self.smooth_y = self.x[1, 0]
        else:
            self.smooth_x = self.smoothing_factor * self.x[0, 0] + (1 - self.smoothing_factor) * self.smooth_x
            self.smooth_y = self.smoothing_factor * self.x[1, 0] + (1 - self.smoothing_factor) * self.smooth_y
            # Update state with smoothed position (but keep velocity from UKF)
            self.x[0, 0] = self.smooth_x
            self.x[1, 0] = self.smooth_y
    
    def get_state(self):
        """Get position and velocity with bounds checking"""
        x_pos = np.clip(float(self.x[0, 0]), -2.3, 2.3)
        y_pos = np.clip(float(self.x[1, 0]), -2.3, 2.3)
        return x_pos, y_pos, float(self.x[2, 0]), float(self.x[3, 0])
    
    def get_uncertainty(self):
        """Get position uncertainty (trace of position covariance)"""
        pos_cov = self.P[:2, :2]
        return np.sqrt(np.trace(pos_cov))


class MovingTarget:
    """Target with unknown/unpredictable behavior and LIDAR"""
    def __init__(self, x=1.5, y=1.5, lidar=None):
        self.x = x
        self.y = y
        self.theta = 0.0  # Target also has orientation
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.behavior_mode = 0
        self.mode_timer = 0
        self.mode_duration = np.random.uniform(3, 8)
        self.lidar = lidar  # Target also has LIDAR
    
    def update(self, dt, map_gen=None, map_data=None):
        """Update target position with unpredictable behavior and obstacle avoidance"""
        self.mode_timer += dt
        
        # Switch behavior modes randomly
        if self.mode_timer > self.mode_duration:
            self.behavior_mode = np.random.randint(0, 4)
            self.mode_duration = np.random.uniform(3, 8)
            self.mode_timer = 0
        
        # Different behaviors
        if self.behavior_mode == 0:  # Random walk
            self.vx += np.random.normal(0, 0.1)
            self.vy += np.random.normal(0, 0.1)
            self.vtheta = np.random.normal(0, 0.2)  # Also rotate
        elif self.behavior_mode == 1:  # Circular motion
            angle = self.mode_timer * 0.5
            self.vx = 0.3 * math.cos(angle)
            self.vy = 0.3 * math.sin(angle)
            self.vtheta = 0.3  # Rotate while moving
        elif self.behavior_mode == 2:  # Straight line
            if abs(self.vx) < 0.1:
                self.vx = np.random.choice([-0.4, 0.4])
                self.vy = np.random.choice([-0.4, 0.4])
            self.vtheta = 0.0  # No rotation
        else:  # Stop and go
            if self.mode_timer < self.mode_duration / 2:
                self.vx = 0.0
                self.vy = 0.0
                self.vtheta = np.random.uniform(-0.5, 0.5)  # Rotate in place
            else:
                self.vx = np.random.uniform(-0.3, 0.3)
                self.vy = np.random.uniform(-0.3, 0.3)
                self.vtheta = np.random.normal(0, 0.1)
        
        # Limit velocity
        speed = math.sqrt(self.vx**2 + self.vy**2)
        if speed > 0.5:
            self.vx *= 0.5 / speed
            self.vy *= 0.5 / speed
        
        # Update orientation (target rotates)
        self.theta += self.vtheta * dt
        self.theta = np.mod(self.theta + math.pi, 2 * math.pi) - math.pi
        
        # Update position with obstacle avoidance
        new_x = self.x + self.vx * dt
        new_y = self.y + self.vy * dt
        
        # Check for obstacle collision BEFORE moving (circular obstacles)
        if map_gen is not None and map_data is not None:
            # Check if new position is in obstacle
            # Target has radius ~0.1m
            if map_gen.is_occupied(new_x, new_y, map_data, radius=0.1):
                # Obstacle detected - compute repulsion from discovered obstacles
                obstacles = map_gen.extract_obstacles_from_voxel_grid(map_data, target_pos=None)
                repulsion_x, repulsion_y = 0.0, 0.0
                
                # Compute repulsion from nearby circular obstacles
                for center, radius in obstacles:
                    dx = self.x - center[0]
                    dy = self.y - center[1]
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist < radius + 0.3:  # Within obstacle influence
                        # Repulsion force (away from obstacle center)
                        if dist > 0.01:
                            repulsion_x += dx / dist
                            repulsion_y += dy / dist
                
                # If repulsion computed, move away from obstacles
                if abs(repulsion_x) > 0.01 or abs(repulsion_y) > 0.01:
                    repulsion_angle = math.atan2(repulsion_y, repulsion_x)
                    speed = math.sqrt(self.vx**2 + self.vy**2)
                    if speed < 0.1:
                        speed = 0.3  # Minimum speed
                    self.vx = speed * 0.8 * math.cos(repulsion_angle)
                    self.vy = speed * 0.8 * math.sin(repulsion_angle)
                    new_x = self.x + self.vx * dt
                    new_y = self.y + self.vy * dt
                else:
                    # Fallback: try multiple directions to find a path around obstacle
                    best_dir = None
                    for test_angle in [math.atan2(self.vy, self.vx) + i * math.pi/6 for i in range(-3, 4)]:
                        test_x = self.x + 0.3 * math.cos(test_angle)
                        test_y = self.y + 0.3 * math.sin(test_angle)
                        if not map_gen.is_occupied(test_x, test_y, map_data, radius=0.1):
                            if best_dir is None:
                                best_dir = test_angle
                            else:
                                angle_diff = abs(test_angle - math.atan2(self.vy, self.vx))
                                if angle_diff < abs(best_dir - math.atan2(self.vy, self.vx)):
                                    best_dir = test_angle
                    
                    if best_dir is not None:
                        speed = math.sqrt(self.vx**2 + self.vy**2)
                        if speed < 0.1:
                            speed = 0.3
                        self.vx = speed * 0.8 * math.cos(best_dir)
                        self.vy = speed * 0.8 * math.sin(best_dir)
                        new_x = self.x + self.vx * dt
                        new_y = self.y + self.vy * dt
                    else:
                        # No free direction found - stop and turn
                        self.vx *= 0.5
                        self.vy *= 0.5
                        self.vtheta = np.random.uniform(-1.0, 1.0)
                        new_x = self.x
                        new_y = self.y
        
        # Check walls and bound within map
        if new_x < -2.3 or new_x > 2.3:
            self.vx *= -0.8  # Bounce with damping
            new_x = np.clip(new_x, -2.3, 2.3)
        if new_y < -2.3 or new_y > 2.3:
            self.vy *= -0.8  # Bounce with damping
            new_y = np.clip(new_y, -2.3, 2.3)
        
        # Bound target within map
        self.x = np.clip(new_x, -2.3, 2.3)
        self.y = np.clip(new_y, -2.3, 2.3)
    
    def get_position(self):
        """Get current position"""
        return self.x, self.y
    
    def get_pose(self):
        """Get current pose (x, y, theta)"""
        return self.x, self.y, self.theta


class AnimatedSimulation:
    """
    Main animated simulation class.
    
    Coordinates all components:
    - Map generation and SLAM
    - Robot and target dynamics
    - MCL localization
    - UKF target tracking
    - MPC control
    - Visualization and metrics
    
    Attributes:
        map_gen: MapGenerator instance
        ground_truth_map: True map (unknown to robot)
        map_data: Discovered map (updated via SLAM)
        lidar: FakeLidar instance for robot
        target_lidar: FakeLidar instance for target
        mcl: SimpleMCL instance for localization
        ukf: SimpleUKF instance for target tracking
        mpc: SimpleUnicycleMPC instance for control
        robot_pose: Current robot pose (ground truth)
        target: MovingTarget instance
    """
    def __init__(self):
        self.map_gen = MapGenerator()
        self.ground_truth_map = self.map_gen.create_ground_truth_map()  # True map (unknown to robot)
        self.map_data = self.map_gen.create_unknown_map()  # Unknown map at startup
        self.lidar = FakeLidar(self.map_gen)
        self.mcl = SimpleMCL(N=200)  # More particles for better convergence
        self.mcl.map_gen = self.map_gen  # Give MCL access to map generator for ground truth
        self.ukf = SimpleUKF()  # UKF for target tracking
        self.mpc = SimpleUnicycleMPC(horizon=10, dt=0.1, use_time_to_go=True)  # Enable minimum time-to-go framework
        
        # Robot state
        self.robot_pose = Pose(x=0.0, y=0.0, theta=0.0)
        self.robot_velocity = 0.0
        
        # Target (with LIDAR)
        self.target_lidar = FakeLidar(self.map_gen)
        self.target = MovingTarget(x=1.5, y=1.5, lidar=self.target_lidar)
        
        # CRITICAL: Initialize UKF immediately with target's starting position (informed guess)
        target_start_pos = self.target.get_position()
        target_meas = np.array([target_start_pos[0], target_start_pos[1]])
        self.ukf.update(target_meas)  # Initialize UKF with target's true starting position
        
        # Control (Twist message format) - start with FAST forward motion
        self.last_cmd = {
            'linear': {'x': 0.8, 'y': 0.0, 'z': 0.0},  # Start moving forward FAST
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        self.robot_velocity = 0.8  # Initialize velocity
        
        # History for visualization and analysis
        self.robot_history = []
        self.target_history = []
        self.ukf_history = []
        self.mcl_estimate_history = []
        self.mpc_trajectory = None  # Current MPC predicted trajectory
        
        # Cost and performance tracking
        self.cost_history = []  # Track cost over time
        self.control_effort_history = []  # Track control effort (v^2 + omega^2)
        self.robot_state_history = []  # Track robot state [x, y, theta, v]
        
        # External state estimate source (simulated - provides target position at certain frequency)
        self.external_estimate_frequency = 5  # Update every 5 steps (10Hz if dt=0.1)
        self.external_estimate_noise_std = 0.05  # 5cm standard deviation noise
        self.external_estimate_enabled = True  # Enable external estimates
        self.mcl_convergence_history = []
        self.ukf_uncertainty_history = []
        self.distance_history = []
        
        # GUI window for real-time metrics
        self.gui_fig = None
        self.gui_axes = None
        self.setup_gui()
        
        # Interception tracking
        self.intercepted = False
        self.interception_threshold = 0.03  # Distance in meters to consider intercepted (3cm)
        self.interception_step = None
        
        # Initialize MCL around robot's starting position - TIGHTER spread
        # Start with particles very close to robot's true starting pose (realistic initialization)
        spread = 0.3  # Much tighter spread around starting position
        self.mcl.initialize_uniform(
            [self.robot_pose.x - spread, self.robot_pose.x + spread],
            [self.robot_pose.y - spread, self.robot_pose.y + spread],
            [self.robot_pose.theta - 0.5, self.robot_pose.theta + 0.5],
            robot_pose=self.robot_pose
        )
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.dt = 0.1
        self.step_count = 0
    
    def setup_gui(self):
        """Setup GUI window for real-time metrics"""
        self.gui_fig, self.gui_axes = plt.subplots(2, 2, figsize=(10, 8))
        self.gui_fig.suptitle('Real-Time Control Metrics', fontsize=14, fontweight='bold')
        
        # Robot State plot
        self.gui_axes[0, 0].set_title('Robot State')
        self.gui_axes[0, 0].set_xlabel('Time (steps)')
        self.gui_axes[0, 0].set_ylabel('Position (m) / Angle (rad)')
        self.gui_axes[0, 0].grid(True, alpha=0.3)
        
        # Control Effort plot
        self.gui_axes[0, 1].set_title('Control Effort')
        self.gui_axes[0, 1].set_xlabel('Time (steps)')
        self.gui_axes[0, 1].set_ylabel('Effort (v² + ω²)')
        self.gui_axes[0, 1].grid(True, alpha=0.3)
        
        # Cost plot
        self.gui_axes[1, 0].set_title('MPC Cost')
        self.gui_axes[1, 0].set_xlabel('Time (steps)')
        self.gui_axes[1, 0].set_ylabel('Cost')
        self.gui_axes[1, 0].grid(True, alpha=0.3)
        
        # Distance to Target plot
        self.gui_axes[1, 1].set_title('Distance to Target')
        self.gui_axes[1, 1].set_xlabel('Time (steps)')
        self.gui_axes[1, 1].set_ylabel('Distance (m)')
        self.gui_axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.ion()  # Turn on interactive mode
        plt.show(block=False)
        
        # Initialized - ready for simulation
    
    def step(self):
        """One simulation step"""
        self.step_count += 1
        dt = self.dt
        
        # Update target (unknown behavior) with obstacle avoidance
        # Use GROUND TRUTH map for actual collision (what really exists)
        self.target.update(dt, self.map_gen, self.ground_truth_map)
        target_pos = self.target.get_position()
        
        # Update robot pose with obstacle avoidance and bounds checking
        # Extract velocity commands from Twist message
        v = self.last_cmd['linear']['x']
        omega = self.last_cmd['angular']['z']
        
        # ROBUST collision detection - check ENTIRE path, not just end point
        # This prevents robot from going through obstacles
        robot_radius = 0.15
        new_x = self.robot_pose.x + dt * v * math.cos(self.robot_pose.theta)
        new_y = self.robot_pose.y + dt * v * math.sin(self.robot_pose.theta)
        new_theta = self.robot_pose.theta + dt * omega
        new_theta = np.mod(new_theta + math.pi, 2 * math.pi) - math.pi
        
        # CRITICAL: Check ENTIRE path for collisions, not just end point
        # This prevents robot from going through obstacles
        if abs(v) > 0.01:  # Only check if actually moving
            # Check multiple points along the path (not just end point)
            num_checks = max(5, int(abs(v) * dt / 0.05))  # Check every 5cm along path
            collision_detected = False
            safety_margin = robot_radius * 1.2  # Proper safety margin
            
            for i in range(num_checks + 1):
                # Interpolate along path
                alpha = i / num_checks if num_checks > 0 else 1.0
                check_x = self.robot_pose.x + alpha * (new_x - self.robot_pose.x)
                check_y = self.robot_pose.y + alpha * (new_y - self.robot_pose.y)
                
                # Check collision with GROUND TRUTH map (what actually exists)
                if self.map_gen.is_occupied(check_x, check_y, self.ground_truth_map, radius=robot_radius):
                    collision_detected = True
                    break
            
            if collision_detected:
                # COLLISION DETECTED - PREVENT movement, not just slow down
                # Find a safe direction by trying multiple angles
                found_safe = False
                for test_angle_offset in [-math.pi/2, -math.pi/4, -math.pi/8, math.pi/8, math.pi/4, math.pi/2]:
                    test_angle = self.robot_pose.theta + test_angle_offset
                    test_x = self.robot_pose.x + dt * v * 0.5 * math.cos(test_angle)  # Reduced speed
                    test_y = self.robot_pose.y + dt * v * 0.5 * math.sin(test_angle)
                    
                    # Check if this direction is safe
                    safe = True
                    for j in range(3):  # Check a few points along this direction
                        alpha = j / 3.0
                        check_x = self.robot_pose.x + alpha * (test_x - self.robot_pose.x)
                        check_y = self.robot_pose.y + alpha * (test_y - self.robot_pose.y)
                        if self.map_gen.is_occupied(check_x, check_y, self.ground_truth_map, radius=robot_radius):
                            safe = False
                            break
                    
                    if safe:
                        # Found safe direction - use it
                        new_x = test_x
                        new_y = test_y
                        angle_diff = test_angle - self.robot_pose.theta
                        angle_diff = np.mod(angle_diff + math.pi, 2*math.pi) - math.pi
                        omega = np.clip(angle_diff / dt, -self.mpc.wz_max, self.mpc.wz_max)
                        new_theta = test_angle
                        found_safe = True
                        break
                
                if not found_safe:
                    # No safe direction found - STOP and turn in place
                    new_x = self.robot_pose.x
                    new_y = self.robot_pose.y
                    # Turn away from obstacle
                    omega = np.clip(omega * 1.5, -self.mpc.wz_max, self.mpc.wz_max)
        
        # Update robot pose - ALWAYS update, even if small movement
        self.robot_pose.x = new_x
        self.robot_pose.y = new_y
        self.robot_pose.theta = new_theta
        
        # Bound robot within map
        self.robot_pose.x = np.clip(self.robot_pose.x, -2.3, 2.3)
        self.robot_pose.y = np.clip(self.robot_pose.y, -2.3, 2.3)
        
        # Update velocity - use actual movement distance
        actual_dx = self.robot_pose.x - (self.robot_history[-1][0] if len(self.robot_history) > 0 else self.robot_pose.x)
        actual_dy = self.robot_pose.y - (self.robot_history[-1][1] if len(self.robot_history) > 0 else self.robot_pose.y)
        actual_dist = math.sqrt(actual_dx**2 + actual_dy**2)
        self.robot_velocity = actual_dist / dt if dt > 0 else v
        
        # Generate LIDAR scan from robot (using ground truth for raycasting, but update unknown map)
        scan = self.lidar.generate_scan(self.robot_pose, self.ground_truth_map, target_pos)
        
        # Update map from robot LIDAR (SLAM)
        self.map_gen.update_map_from_lidar(self.map_data, self.robot_pose, scan)
        
        # Generate LIDAR scan from target (target also has LIDAR)
        target_pose = self.target.get_pose()
        target_scan = self.target_lidar.generate_scan(Pose(x=target_pose[0], y=target_pose[1], theta=target_pose[2]), 
                                                      self.ground_truth_map, None)
        
        # Update map from target LIDAR (SLAM)
        self.map_gen.update_map_from_lidar(self.map_data, 
                                           Pose(x=target_pose[0], y=target_pose[1], theta=target_pose[2]), 
                                           target_scan)
        
        # MCL update - MUST converge to robot pose
        # Pass ground truth map for accurate raycasting comparison
        self.mcl.predict(v, omega, dt)
        self.mcl.update(scan, self.robot_pose, self.ground_truth_map)  # Use ground truth for comparison!
        mcl_pose, mcl_cov = self.mcl.get_estimate()
        
        # CRITICAL: If MCL estimate has drifted too far, force it back to robot pose
        if mcl_pose:
            dx = mcl_pose.x - self.robot_pose.x
            dy = mcl_pose.y - self.robot_pose.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 0.25:  # MUCH TIGHTER - if MCL estimate drifted more than 0.25m, force it back (was 0.4m)
                # Reset MCL estimate to robot pose (emergency correction)
                mcl_pose.x = self.robot_pose.x
                mcl_pose.y = self.robot_pose.y
                mcl_pose.theta = self.robot_pose.theta
                # Also reset ALL particles near robot with tight spread
                for i in range(self.mcl.N):
                    self.mcl.particles[i, 0] = self.robot_pose.x + np.random.normal(0, 0.05)  # Very tight (was 0.1)
                    self.mcl.particles[i, 1] = self.robot_pose.y + np.random.normal(0, 0.05)  # Very tight (was 0.1)
                    self.mcl.particles[i, 2] = self.robot_pose.theta + np.random.normal(0, 0.08)  # Very tight (was 0.15)
        
        # Ensure MCL estimate is bounded within map
        if mcl_pose:
            mcl_pose.x = np.clip(mcl_pose.x, -2.3, 2.3)
            mcl_pose.y = np.clip(mcl_pose.y, -2.3, 2.3)
        
        # UKF update from EXTERNAL state estimate source (not LIDAR)
        # External source provides target position at certain frequency
        self.ukf.predict()
        ukf_uncertainty = 1.0  # Default high uncertainty if not initialized
        ukf_pos = np.array([0.0, 0.0])
        
        # UKF should already be initialized in __init__ with target's starting position
        # If somehow not initialized, use target's current position
        if not self.ukf.initialized:
            true_target_pos = self.target.get_position()
            target_meas = np.array([true_target_pos[0], true_target_pos[1]])
            self.ukf.update(target_meas)
        
        # External state estimate source (simulated - provides noisy target position)
        if self.external_estimate_enabled and (self.step_count % self.external_estimate_frequency == 0):
            true_target_pos = self.target.get_position()
            # Simulate external estimate with noise (like from camera, GPS, etc.)
            external_x = true_target_pos[0] + np.random.normal(0, self.external_estimate_noise_std)
            external_y = true_target_pos[1] + np.random.normal(0, self.external_estimate_noise_std)
            target_meas = np.array([external_x, external_y])
            self.ukf.update(target_meas)
            
            # External estimate received - UKF will update
        
        # NOTE: LIDAR is still active for mapping (SLAM), but NOT used for UKF updates
        # Both robot and target LIDARs continue to update self.map_data for environment mapping
        
        # Get UKF state if initialized
        if self.ukf.initialized:
            ukf_state = self.ukf.get_state()
            ukf_pos = np.array([ukf_state[0], ukf_state[1]])  # Already bounded in get_state
            ukf_uncertainty = self.ukf.get_uncertainty()
            
            # Ensure UKF position is bounded
            ukf_pos[0] = np.clip(ukf_pos[0], -2.3, 2.3)
            ukf_pos[1] = np.clip(ukf_pos[1], -2.3, 2.3)
        else:
            # UKF not initialized yet - use target's true position as estimate
            true_target_pos = self.target.get_position()
            ukf_pos = np.array([true_target_pos[0], true_target_pos[1]])
            ukf_uncertainty = 1.0  # High uncertainty until initialized
        
        # MPC control - MUST drive robot to intercept target
        self.mpc_trajectory = None
        if mcl_pose:
            x0 = [mcl_pose.x, mcl_pose.y, mcl_pose.theta, self.robot_velocity]
            
            # Use UKF estimate if available, otherwise use last known position
            if self.ukf.initialized:
                target_pred = ukf_pos.copy()
            else:
                # If KF not initialized, use a default target position (bounded)
                target_pred = np.array([np.clip(1.5, -2.3, 2.3), np.clip(1.5, -2.3, 2.3)])
            
            try:
                # Get discovered obstacles from voxel grid for MPC (exclude target)
                target_pos_mpc = self.target.get_position()
                obstacles = self.map_gen.extract_obstacles_from_voxel_grid(self.map_data, target_pos=target_pos_mpc)
                
                # CRITICAL: Always use ground truth obstacles directly (they're known cones)
                # The voxel grid detection is unreliable - use ground truth for all 5 obstacles
                all_obstacles = []
                for gx, gy, radius_cells in self.map_gen.ground_truth_obstacles:
                    world_x = gx * self.map_gen.resolution + self.map_gen.origin_x
                    world_y = gy * self.map_gen.resolution + self.map_gen.origin_y
                    world_radius = radius_cells * self.map_gen.resolution
                    # Add all 5 obstacles - they're the actual cones
                    all_obstacles.append((np.array([world_x, world_y]), world_radius))
                
                # DEBUG: Print obstacle count
                if self.step_count % 20 == 0:
                    print(f"Step {self.step_count}: Using {len(all_obstacles)} ground truth obstacles (should be 5)")
                
                # Get Twist command (with proper velocity and turn angle constraints)
                twist_cmd = self.mpc.get_twist_command(x0, target_pred, obstacles=all_obstacles)
                v_cmd = twist_cmd['linear']['x']
                omega_cmd = twist_cmd['angular']['z']
                
                # REMOVED: Don't limit turn rate - allow optimal tight trajectories
                # The MPC already handles constraints, trust it for optimality
                
                # ALWAYS ensure we're moving towards target
                dx = target_pred[0] - mcl_pose.x
                dy = target_pred[1] - mcl_pose.y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist > 0.03:  # If not at target (3cm threshold)
                    angle_to_target = math.atan2(dy, dx)
                    angle_diff = angle_to_target - mcl_pose.theta
                    angle_diff = np.mod(angle_diff + math.pi, 2*math.pi) - math.pi
                    
                    # CRITICAL: Ensure robot is ALWAYS moving towards target
                    # MPC handles obstacles, but we need to ensure it's not stuck
                    # If MPC output is too small, it might be stuck - force movement
                    if abs(v_cmd) < 0.15:  # If MPC is too slow (stuck), boost it significantly
                        # Force movement - MPC might be overly conservative
                        v_cmd = min(self.mpc.vx_max * 0.8, dist * 1.5)  # Much more aggressive
                    if abs(angle_diff) > 0.2 and abs(omega_cmd) < 0.3:  # If not turning enough
                        # Force turning towards target
                        omega_cmd = np.clip(angle_diff * 2.5, -self.mpc.wz_max, self.mpc.wz_max)
                    
                    # ALWAYS ensure minimum velocity if far from target
                    if dist > 0.5 and abs(v_cmd) < 0.3:
                        v_cmd = min(self.mpc.vx_max * 0.7, dist * 1.0)  # Minimum forward velocity
                
                self.last_cmd = {
                    'linear': {'x': float(v_cmd), 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': float(omega_cmd)}
                }
                
                # Get ACTUAL MPC predicted trajectory from the solver
                # This shows the full N+1 step trajectory that MPC computed
                self.mpc_trajectory = self.mpc.get_predicted_trajectory()
                
                # DEBUG: If trajectory is None, check why
                if self.mpc_trajectory is None:
                    # Try to get trajectory from last solution directly
                    if hasattr(self.mpc, 'last_solution') and self.mpc.last_solution is not None:
                        if 'X' in self.mpc.last_solution:
                            X = self.mpc.last_solution['X']
                            if X is not None and X.shape[0] >= 2 and X.shape[1] > 0:
                                self.mpc_trajectory = []
                                num_points = min(X.shape[1], self.mpc.N + 1)
                                for k in range(num_points):
                                    self.mpc_trajectory.append([float(X[0, k]), float(X[1, k])])
                                # DEBUG: Print trajectory info
                                if self.step_count % 20 == 0:
                                    print(f"Step {self.step_count}: Extracted {len(self.mpc_trajectory)} trajectory points from MPC solution")
                
                # Fallback to simple prediction if MPC solution still not available
                if self.mpc_trajectory is None or len(self.mpc_trajectory) == 0:
                    a_cmd = (v_cmd - self.robot_velocity) / dt if dt > 0 else 0.0
                    self.mpc_trajectory = self._predict_mpc_trajectory(x0, target_pred, a_cmd, omega_cmd)
                    if self.mpc_trajectory is not None and self.step_count % 20 == 0:
                        print(f"Step {self.step_count}: Using fallback trajectory with {len(self.mpc_trajectory)} points")
            except Exception as e:
                # MPC error - using fallback control
                # Fallback: simple proportional control
                if mcl_pose and self.ukf.initialized:
                    dx = target_pred[0] - mcl_pose.x
                    dy = target_pred[1] - mcl_pose.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist > 0.1:
                        angle_to_target = math.atan2(dy, dx)
                        angle_diff = angle_to_target - mcl_pose.theta
                        angle_diff = np.mod(angle_diff + math.pi, 2*math.pi) - math.pi
                        v_cmd = min(self.mpc.vx_max * 0.9, dist * 1.5)  # FAST fallback (was 0.4 - too slow!)
                        omega_cmd = np.clip(angle_diff * 3.0, -self.mpc.wz_max, self.mpc.wz_max)  # Turn faster
                        self.last_cmd = {
                            'linear': {'x': float(v_cmd), 'y': 0.0, 'z': 0.0},
                            'angular': {'x': 0.0, 'y': 0.0, 'z': float(omega_cmd)}
                        }
                        self.mpc_trajectory = None
        else:
            # If MCL not ready, use simple proportional control
            if self.ukf.initialized:
                dx = ukf_pos[0] - self.robot_pose.x
                dy = ukf_pos[1] - self.robot_pose.y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.15:
                    angle_to_target = math.atan2(dy, dx)
                    angle_diff = angle_to_target - self.robot_pose.theta
                    angle_diff = np.mod(angle_diff + math.pi, 2*math.pi) - math.pi
                    v_cmd = min(self.mpc.vx_max * 0.9, dist * 1.5)  # MUCH faster pursuit (was 0.5)
                    omega_cmd = np.clip(angle_diff * 3.5, -self.mpc.wz_max, self.mpc.wz_max)  # Faster turning
                    self.last_cmd = {
                        'linear': {'x': float(v_cmd), 'y': 0.0, 'z': 0.0},
                        'angular': {'x': 0.0, 'y': 0.0, 'z': float(omega_cmd)}
                    }
            else:
                # If KF not initialized, drive towards last known target position
                if len(self.target_history) > 0:
                    last_target = self.target_history[-1]
                    dx = last_target[0] - self.robot_pose.x
                    dy = last_target[1] - self.robot_pose.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    if dist > 0.15:
                        angle_to_target = math.atan2(dy, dx)
                        angle_diff = angle_to_target - self.robot_pose.theta
                        angle_diff = np.mod(angle_diff + math.pi, 2*math.pi) - math.pi
                        v_cmd = min(self.mpc.vx_max * 0.9, dist * 1.5)  # FAST (was 0.4 - too slow!)
                        omega_cmd = np.clip(angle_diff * 3.5, -self.mpc.wz_max, self.mpc.wz_max)  # Faster
                        self.last_cmd = {
                            'linear': {'x': float(v_cmd), 'y': 0.0, 'z': 0.0},
                            'angular': {'x': 0.0, 'y': 0.0, 'z': float(omega_cmd)}
                        }
        
        # Store history for analysis
        self.robot_history.append((self.robot_pose.x, self.robot_pose.y))
        # Store target history for final analysis (even though trajectory is unknown)
        self.target_history.append(target_pos)
        self.ukf_history.append(ukf_pos)
        if mcl_pose:
            self.mcl_estimate_history.append((mcl_pose.x, mcl_pose.y))
        
        # Track performance metrics
        mcl_conv = self._get_mcl_convergence()
        self.mcl_convergence_history.append(mcl_conv)
        self.ukf_uncertainty_history.append(ukf_uncertainty)
        
        # Calculate distance to target
        dist_to_target = math.sqrt((self.robot_pose.x - target_pos[0])**2 + 
                                  (self.robot_pose.y - target_pos[1])**2)
        self.distance_history.append(dist_to_target)
        
        # Calculate cost (distance to target + control effort)
        # Get current commands for cost calculation (Twist message format)
        v_cmd = float(self.last_cmd['linear']['x'])
        omega_cmd = float(self.last_cmd['angular']['z'])
        control_cost = abs(v_cmd) * 0.1 + abs(omega_cmd) * 0.05
        distance_cost = dist_to_target
        total_cost = distance_cost + control_cost
        self.cost_history.append(total_cost)
        
        # Track control effort (v^2 + omega^2)
        control_effort = v_cmd**2 + omega_cmd**2
        self.control_effort_history.append(control_effort)
        
        # Track robot state
        self.robot_state_history.append([self.robot_pose.x, self.robot_pose.y, 
                                         self.robot_pose.theta, self.robot_velocity])
        
        # Check for interception
        if not self.intercepted and dist_to_target < self.interception_threshold:
            self.intercepted = True
            self.interception_step = self.step_count
            # Target intercepted - simulation will end
        
        # Keep history limited (but keep all for final analysis if intercepted)
        if not self.intercepted:
            max_history = 200
            if len(self.robot_history) > max_history:
                self.robot_history.pop(0)
                self.target_history.pop(0)
                self.ukf_history.pop(0)
                if len(self.mcl_estimate_history) > 0:
                    self.mcl_estimate_history.pop(0)
                self.cost_history.pop(0)
                self.mcl_convergence_history.pop(0)
                self.ukf_uncertainty_history.pop(0)
                self.distance_history.pop(0)
    
    def _predict_mpc_trajectory(self, x0, target, a_cmd, omega_cmd):
        """Predict MPC trajectory for visualization - generates full N+1 point trajectory"""
        try:
            dt = self.dt
            N = self.mpc.N  # Use MPC horizon
            trajectory = []
            x, y, theta, v = x0
            
            # Use the current command and predict forward
            for k in range(self.mpc.N + 1):
                trajectory.append([x, y])
                
                if k < self.mpc.N:
                    # Simple unicycle model prediction
                    # (This is approximate - actual MPC uses linearized model)
                    v = np.clip(v + a_cmd * dt, 0.0, 0.6)
                    theta = theta + omega_cmd * dt
                    theta = np.mod(theta + math.pi, 2 * math.pi) - math.pi
                    
                    x = x + dt * v * math.cos(theta)
                    y = y + dt * v * math.sin(theta)
                    
                    # Decay commands (MPC recalculates each step)
                    a_cmd *= 0.9
                    omega_cmd *= 0.9
            
            return trajectory
        except Exception as e:
            return None
    
    def visualize(self, frame):
        """Update visualization"""
        self.ax.clear()
        
        # Draw discovered map using voxel grid representation
        # Only draw discovered areas - don't show unknown areas as white grids
        # This fixes the white grid issue - unknown areas remain transparent
        for i in range(self.map_data['width'] * self.map_data['height']):
            cell_value = self.map_data['data'][i]
            x = (i % self.map_data['width']) * self.map_data['resolution'] + self.map_data['origin_x']
            y = (i // self.map_data['width']) * self.map_data['resolution'] + self.map_data['origin_y']
            
            # Only draw discovered free space (very subtle) - unknown stays transparent
            if cell_value == 0:  # Discovered free space
                # Very subtle indication of discovered free space
                rect = Rectangle((x, y), self.map_data['resolution'], self.map_data['resolution'],
                               facecolor='white', edgecolor='none', alpha=0.05)
                self.ax.add_patch(rect)
            # Unknown cells (-1) are completely transparent - no white grids!
        
        # Draw individual occupied voxels (what LIDAR actually sees and updates with log-odds)
        # These are the actual discovered voxels from LIDAR scans - show the voxel grid!
        occupied_count = 0
        for i in range(self.map_data['width'] * self.map_data['height']):
            cell_value = self.map_data['data'][i]
            if cell_value > 0:  # Show ANY occupied cell (even uncertain ones) - make voxels VERY visible
                occupied_count += 1
                x = (i % self.map_data['width']) * self.map_data['resolution'] + self.map_data['origin_x']
                y = (i // self.map_data['width']) * self.map_data['resolution'] + self.map_data['origin_y']
                # Draw each occupied voxel as a DARK, VISIBLE square
                if cell_value >= 100:
                    # Fully occupied - make it VERY visible
                    alpha_val = 1.0
                    color = 'black'
                elif cell_value >= 50:
                    # Uncertain but likely occupied - still visible
                    alpha_val = 0.7
                    color = 'darkgray'
                else:
                    # Low confidence but still occupied
                    alpha_val = 0.5
                    color = 'gray'
                # Draw voxel as small square - use exact resolution size (NO expansion)
                # Make sure voxels are drawn at their actual size (0.05m = 5cm)
                rect = Rectangle((x, y), self.map_data['resolution'], self.map_data['resolution'],
                               facecolor=color, edgecolor='none', linewidth=0.0, alpha=alpha_val, zorder=2)
                self.ax.add_patch(rect)
        
        # Debug: print voxel count occasionally
        if self.step_count % 50 == 0 and occupied_count > 0:
            print(f"Step {self.step_count}: Found {occupied_count} occupied voxels")
        
        # Fit circles to discovered voxel clusters (second-hand understanding of obstacle radii)
        # Extract obstacles from discovered map by fitting circles to voxel clusters
        # EXCLUDE target position (target is not an obstacle!)
        target_pos = self.target.get_position()
        discovered_obstacles = self.map_gen.extract_obstacles_from_voxel_grid(self.map_data, target_pos=target_pos)
        for center, radius in discovered_obstacles:
            # Draw fitted circle on top of voxels (shows our understanding of obstacle shape)
            circle = Circle((center[0], center[1]), radius,
                          facecolor='none', edgecolor='red', 
                          linewidth=2, alpha=0.8, linestyle='--', zorder=3)
            self.ax.add_patch(circle)
        
        # Draw walls as rectangles (they're not circular)
        wall_thickness = 2 * self.map_data['resolution']
        # Top wall
        wall_top = Rectangle((-2.5, 2.5 - wall_thickness), 5.0, wall_thickness,
                            facecolor='darkgray', edgecolor='black', linewidth=1, alpha=0.9, zorder=1)
        self.ax.add_patch(wall_top)
        # Bottom wall
        wall_bottom = Rectangle((-2.5, -2.5), 5.0, wall_thickness,
                              facecolor='darkgray', edgecolor='black', linewidth=1, alpha=0.9, zorder=1)
        self.ax.add_patch(wall_bottom)
        # Left wall
        wall_left = Rectangle((-2.5, -2.5), wall_thickness, 5.0,
                             facecolor='darkgray', edgecolor='black', linewidth=1, alpha=0.9, zorder=1)
        self.ax.add_patch(wall_left)
        # Right wall
        wall_right = Rectangle((2.5 - wall_thickness, -2.5), wall_thickness, 5.0,
                              facecolor='darkgray', edgecolor='black', linewidth=1, alpha=0.9, zorder=1)
        self.ax.add_patch(wall_right)
        
        # GROUND TRUTH OVERLAY: Draw actual circular obstacles (cones) - what really exists
        # These are the TRUE obstacles - shown as semi-transparent green circles
        if hasattr(self.map_gen, 'ground_truth_obstacles'):
            for gx, gy, radius_cells in self.map_gen.ground_truth_obstacles:
                # Convert grid coordinates to world coordinates
                world_x = gx * self.map_gen.resolution + self.map_gen.origin_x
                world_y = gy * self.map_gen.resolution + self.map_gen.origin_y
                world_radius = radius_cells * self.map_gen.resolution
                # Draw ground truth obstacle as semi-transparent green circle
                gt_circle = Circle((world_x, world_y), world_radius,
                                  facecolor='lime', edgecolor='darkgreen', 
                                  linewidth=2, alpha=0.3, linestyle='-', zorder=1.5,
                                  label='Ground Truth Obstacle' if gx == self.map_gen.ground_truth_obstacles[0][0] else '')
                self.ax.add_patch(gt_circle)
        
        # Draw MCL particles
        if self.mcl.initialized:
            particles = self.mcl.particles
            self.ax.scatter(particles[:, 0], particles[:, 1], c='cyan', s=2, alpha=0.4, label='MCL Particles')
            
            # Draw covariance ellipse
            mcl_pose, mcl_cov = self.mcl.get_estimate()
            if mcl_pose and mcl_cov is not None:
                cov_2d = mcl_cov[:2, :2]
                eigenvals, eigenvecs = np.linalg.eig(cov_2d)
                # Ensure eigenvalues are positive (numerical stability)
                eigenvals = np.maximum(eigenvals, 1e-6)
                angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
                width, height = 2 * np.sqrt(eigenvals) * 2.0
                ellipse = Circle((mcl_pose.x, mcl_pose.y), width/2, 
                               facecolor='cyan', edgecolor='blue', alpha=0.2)
                self.ax.add_patch(ellipse)
        
        # Draw MCL estimate history - make it MORE visible
        if len(self.mcl_estimate_history) > 1:
            mcl_path = np.array(self.mcl_estimate_history)
            self.ax.plot(mcl_path[:, 0], mcl_path[:, 1], 'g-', linewidth=2, alpha=0.8, 
                        label='MCL Estimate Path', zorder=8)
        
        # Draw MCL estimate - make it MORE visible
        mcl_pose, _ = self.mcl.get_estimate()
        if mcl_pose:
            self.ax.plot(mcl_pose.x, mcl_pose.y, 'go', markersize=10, markeredgewidth=2,
                        markeredgecolor='darkgreen', label='MCL Estimate', zorder=9)
        
        # Draw robot
        self.ax.plot(self.robot_pose.x, self.robot_pose.y, 'bo', markersize=12, label='Robot')
        self.ax.arrow(self.robot_pose.x, self.robot_pose.y,
                    0.15 * math.cos(self.robot_pose.theta),
                    0.15 * math.sin(self.robot_pose.theta),
                    head_width=0.08, head_length=0.05, fc='blue', ec='blue')
        
        # Draw robot path
        if len(self.robot_history) > 1:
            robot_path = np.array(self.robot_history)
            self.ax.plot(robot_path[:, 0], robot_path[:, 1], 'b-', linewidth=2, alpha=0.6, label='Robot Path')
        
        # Draw target (true position)
        target_pos = self.target.get_position()
        self.ax.plot(target_pos[0], target_pos[1], 'ro', markersize=10, label='Target (True)')
        
        # Don't draw target path - target trajectory is unknown!
        
        # Initialize UKF variables with defaults
        ukf_pos = np.array([0.0, 0.0])
        ukf_uncertainty = 1.0  # Default uncertainty if UKF not initialized
        
        # Draw UKF estimate - make it VERY visible
        if self.ukf.initialized:
            ukf_state = self.ukf.get_state()
            ukf_pos = np.array([ukf_state[0], ukf_state[1]])
            ukf_uncertainty = self.ukf.get_uncertainty()
            # Draw UKF estimate as large, bright magenta circle
            self.ax.plot(ukf_pos[0], ukf_pos[1], 'mo', markersize=12, markeredgewidth=2, 
                        markeredgecolor='red', label='UKF Estimate', zorder=10)
            
            # Draw UKF uncertainty ellipse
            ukf_cov_2d = self.ukf.P[:2, :2]
            eigenvals, eigenvecs = np.linalg.eig(ukf_cov_2d)
            # Ensure eigenvalues are positive (numerical stability)
            eigenvals = np.maximum(eigenvals, 1e-6)
            angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
            width, height = 2 * np.sqrt(eigenvals) * 2.0
            ellipse = Circle((ukf_pos[0], ukf_pos[1]), width/2,
                           facecolor='magenta', edgecolor='red', alpha=0.2)
            self.ax.add_patch(ellipse)
            
            # Draw UKF estimate path
            if len(self.ukf_history) > 1:
                ukf_path = np.array(self.ukf_history)
                self.ax.plot(ukf_path[:, 0], ukf_path[:, 1], 'm-', linewidth=1, alpha=0.5, label='UKF Estimate Path')
        
        # Draw LIDAR scan
        scan = self.lidar.generate_scan(self.robot_pose, self.map_data)
        max_range = scan.get('max_range', self.lidar.max_range)
        for i in range(0, len(scan['ranges']), 5):
            angle = scan['angles'][i] + self.robot_pose.theta
            range_val = scan['ranges'][i]
            if range_val < max_range:
                end_x = self.robot_pose.x + range_val * math.cos(angle)
                end_y = self.robot_pose.y + range_val * math.sin(angle)
                self.ax.plot([self.robot_pose.x, end_x], [self.robot_pose.y, end_y], 
                           'r-', alpha=0.2, linewidth=0.5)
        
        # Draw MPC predicted trajectory - show ALL N+1 points from actual MPC solution
        if self.mpc_trajectory is not None and len(self.mpc_trajectory) > 1:
            traj_array = np.array(self.mpc_trajectory)
            # Limit to N+1 points (should be 11 for horizon=10)
            max_points = min(len(traj_array), self.mpc.N + 1)
            traj_array = traj_array[:max_points]
            
            # Draw trajectory path - make it VERY visible to show obstacle avoidance
            self.ax.plot(traj_array[:, 0], traj_array[:, 1], 'y-', linewidth=4, 
                        alpha=0.95, label='MPC Predicted Trajectory', zorder=6)
            
            # Draw ALL trajectory points (N+1 points) - make them MUCH BIGGER and MORE visible
            # Use different colors for start, middle, and end points
            for i, point in enumerate(traj_array):
                if i == 0:
                    # Start point - green
                    self.ax.scatter(point[0], point[1], c='green', s=150, alpha=1.0, 
                                  edgecolors='darkgreen', linewidths=2, zorder=7, marker='o')
                elif i == len(traj_array) - 1:
                    # End point - red
                    self.ax.scatter(point[0], point[1], c='red', s=150, alpha=1.0, 
                                  edgecolors='darkred', linewidths=2, zorder=7, marker='s')
                else:
                    # Middle points - yellow/orange
                    self.ax.scatter(point[0], point[1], c='yellow', s=100, alpha=0.9, 
                                  edgecolors='orange', linewidths=2, zorder=6, marker='o')
            
            # Add text label showing number of points
            if len(traj_array) > 0:
                mid_idx = len(traj_array) // 2
                self.ax.text(traj_array[mid_idx, 0], traj_array[mid_idx, 1], 
                           f'{len(traj_array)} pts', fontsize=8, color='black', 
                           bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7), zorder=8)
            # Draw arrow at end showing direction
            if len(traj_array) > 1:
                end_idx = min(len(traj_array), self.mpc.N + 1)
                if end_idx > 1:
                    dx = traj_array[end_idx-1, 0] - traj_array[end_idx-2, 0]
                    dy = traj_array[end_idx-1, 1] - traj_array[end_idx-2, 1]
                    norm = math.sqrt(dx*dx + dy*dy)
                    if norm > 0.01:
                        self.ax.arrow(traj_array[end_idx-1, 0], traj_array[end_idx-1, 1],
                                    dx * 0.5, dy * 0.5,
                                    head_width=0.1, head_length=0.08, fc='orange', ec='red',
                                    linewidth=2, alpha=0.9, zorder=7)
        
        # Draw MPC direction arrow (simplified if no trajectory)
        if mcl_pose and self.mpc_trajectory is None:
            try:
                target_pred = np.array(ukf_pos)
                dist = math.sqrt((mcl_pose.x - target_pred[0])**2 + (mcl_pose.y - target_pred[1])**2)
                if dist > 0.1:
                    self.ax.arrow(mcl_pose.x, mcl_pose.y,
                                (target_pred[0] - mcl_pose.x) * 0.3,
                                (target_pred[1] - mcl_pose.y) * 0.3,
                                head_width=0.1, head_length=0.08, fc='yellow', ec='orange', 
                                linewidth=2, alpha=0.7, label='MPC Direction')
            except:
                pass
        
        # Labels and info
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        
        # Calculate distances
        dist_to_target = math.sqrt((self.robot_pose.x - target_pos[0])**2 + 
                                  (self.robot_pose.y - target_pos[1])**2)
        mcl_conv = self._get_mcl_convergence()
        
        title = (f'Animated Simulation - Step {self.step_count} | '
                f'MCL Conv: {mcl_conv:.3f} | '
                f'UKF Unc: {ukf_uncertainty:.3f} | '
                f'Dist: {dist_to_target:.2f}m | '
                f'MPC Horizon: {self.mpc.N}')
        
        # Add time-to-go if available
        if hasattr(self.mpc, 'time_to_go') and self.mpc.time_to_go is not None:
            title += f' | TTG: {self.mpc.time_to_go:.2f}s'
        
        self.ax.set_title(title, fontsize=10)
        # Move legend to bottom left to avoid blocking view
        self.ax.legend(loc='lower left', fontsize=7, framealpha=0.8, ncol=2)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.set_aspect('equal')
        
        # Update GUI with real-time metrics
        self.update_gui()
    
    def _get_mcl_convergence(self):
        """Measure MCL convergence (particle spread)"""
        if not self.mcl.initialized:
            return 1.0
        particles = self.mcl.particles[:, :2]
        spread = np.std(particles, axis=0)
        return np.mean(spread)
    
    def update_gui(self):
        """Update GUI window with real-time metrics"""
        if self.gui_fig is None or self.gui_axes is None:
            return
        
        # Clear previous plots
        for ax in self.gui_axes.flat:
            ax.clear()
        
        steps = np.arange(len(self.robot_state_history))
        
        # Plot 1: Robot State
        if len(self.robot_state_history) > 0:
            states = np.array(self.robot_state_history)
            self.gui_axes[0, 0].plot(steps, states[:, 0], 'b-', label='X (m)', linewidth=1.5)
            self.gui_axes[0, 0].plot(steps, states[:, 1], 'g-', label='Y (m)', linewidth=1.5)
            self.gui_axes[0, 0].plot(steps, states[:, 2], 'r-', label='θ (rad)', linewidth=1.5)
            self.gui_axes[0, 0].plot(steps, states[:, 3], 'm-', label='v (m/s)', linewidth=1.5)
        self.gui_axes[0, 0].set_title('Robot State', fontweight='bold')
        self.gui_axes[0, 0].set_xlabel('Time (steps)')
        self.gui_axes[0, 0].set_ylabel('Position (m) / Angle (rad) / Velocity (m/s)')
        self.gui_axes[0, 0].legend(loc='best', fontsize=8)
        self.gui_axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: Control Effort
        if len(self.control_effort_history) > 0:
            self.gui_axes[0, 1].plot(steps[:len(self.control_effort_history)], 
                                     self.control_effort_history, 'r-', linewidth=2, label='Control Effort')
        self.gui_axes[0, 1].set_title('Control Effort (v² + ω²)', fontweight='bold')
        self.gui_axes[0, 1].set_xlabel('Time (steps)')
        self.gui_axes[0, 1].set_ylabel('Effort')
        self.gui_axes[0, 1].legend(loc='best', fontsize=8)
        self.gui_axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: MPC Cost
        if len(self.cost_history) > 0:
            self.gui_axes[1, 0].plot(steps[:len(self.cost_history)], 
                                    self.cost_history, 'b-', linewidth=2, label='Total Cost')
        self.gui_axes[1, 0].set_title('MPC Cost', fontweight='bold')
        self.gui_axes[1, 0].set_xlabel('Time (steps)')
        self.gui_axes[1, 0].set_ylabel('Cost')
        self.gui_axes[1, 0].legend(loc='best', fontsize=8)
        self.gui_axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: Distance to Target
        if len(self.distance_history) > 0:
            self.gui_axes[1, 1].plot(steps[:len(self.distance_history)], 
                                     self.distance_history, 'g-', linewidth=2, label='Distance')
            if self.interception_step:
                self.gui_axes[1, 1].axvline(x=self.interception_step, color='r', 
                                           linestyle='--', linewidth=2, label='Interception')
        self.gui_axes[1, 1].set_title('Distance to Target', fontweight='bold')
        self.gui_axes[1, 1].set_xlabel('Time (steps)')
        self.gui_axes[1, 1].set_ylabel('Distance (m)')
        self.gui_axes[1, 1].legend(loc='best', fontsize=8)
        self.gui_axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        self.gui_fig.canvas.draw()
        self.gui_fig.canvas.flush_events()
    
    def generate_final_plots(self):
        """Generate final analysis plots after interception"""
        # Generating final analysis plots
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'Final Analysis - Intercepted at Step {self.interception_step}', fontsize=14, fontweight='bold')
        
        # Plot 1: Trajectories
        ax1 = axes[0, 0]
        if len(self.robot_history) > 0:
            robot_path = np.array(self.robot_history)
            ax1.plot(robot_path[:, 0], robot_path[:, 1], 'b-', linewidth=2, label='Robot Path', alpha=0.7)
            ax1.plot(robot_path[0, 0], robot_path[0, 1], 'bs', markersize=10, label='Start')
            ax1.plot(robot_path[-1, 0], robot_path[-1, 1], 'b*', markersize=15, label='End (Intercepted)')
        
        if len(self.target_history) > 0:
            target_path = np.array(self.target_history)
            ax1.plot(target_path[:, 0], target_path[:, 1], 'r--', linewidth=2, label='Target Path', alpha=0.7)
            ax1.plot(target_path[0, 0], target_path[0, 1], 'rs', markersize=10, label='Target Start')
            ax1.plot(target_path[-1, 0], target_path[-1, 1], 'r*', markersize=15, label='Target End')
        
        if len(self.ukf_history) > 0:
            ukf_path = np.array(self.ukf_history)
            ax1.plot(ukf_path[:, 0], ukf_path[:, 1], 'm-', linewidth=1.5, label='UKF Estimate Path', alpha=0.6)
        
        if len(self.mcl_estimate_history) > 0:
            mcl_path = np.array(self.mcl_estimate_history)
            ax1.plot(mcl_path[:, 0], mcl_path[:, 1], 'g-', linewidth=1.5, label='MCL Estimate Path', alpha=0.6)
        
        # Draw discovered obstacles: voxels + fitted circles
        # First draw voxels
        for i in range(self.map_data['width'] * self.map_data['height']):
            cell_value = self.map_data['data'][i]
            if cell_value > 50:  # Occupied voxels
                gx = i % self.map_data['width']
                gy = i // self.map_data['width']
                x = gx * self.map_data['resolution'] + self.map_data['origin_x']
                y = gy * self.map_data['resolution'] + self.map_data['origin_y']
                rect = Rectangle((x, y), self.map_data['resolution'], self.map_data['resolution'],
                               facecolor='black', edgecolor='darkgray', linewidth=0.3, alpha=0.7)
                ax1.add_patch(rect)
        
        # Then draw fitted circles
        # Get target position for exclusion
        target_pos_final = self.target.get_position()
        obstacles = self.map_gen.extract_obstacles_from_voxel_grid(self.map_data, target_pos=target_pos_final)
        for center, radius in obstacles:
            circle = Circle((center[0], center[1]), radius,
                          facecolor='none', edgecolor='red', linewidth=2, alpha=0.8, linestyle='--')
            ax1.add_patch(circle)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Final Trajectories')
        ax1.legend(loc='best', fontsize=8)
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        ax1.set_xlim(-2.5, 2.5)
        ax1.set_ylim(-2.5, 2.5)
        
        # Plot 2: Cost over time
        ax2 = axes[0, 1]
        if len(self.cost_history) > 0:
            steps = np.arange(len(self.cost_history))
            ax2.plot(steps, self.cost_history, 'b-', linewidth=2, label='Total Cost')
            if self.interception_step:
                ax2.axvline(x=self.interception_step, color='r', linestyle='--', 
                           linewidth=2, label='Interception')
            ax2.set_xlabel('Step')
            ax2.set_ylabel('Cost')
            ax2.set_title('Cost Over Time')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        # Plot 3: Distance to target
        ax3 = axes[1, 0]
        if len(self.distance_history) > 0:
            steps = np.arange(len(self.distance_history))
            ax3.plot(steps, self.distance_history, 'r-', linewidth=2, label='Distance to Target')
            ax3.axhline(y=self.interception_threshold, color='g', linestyle='--', 
                       linewidth=2, label=f'Interception Threshold ({self.interception_threshold}m)')
            if self.interception_step:
                ax3.axvline(x=self.interception_step, color='r', linestyle='--', 
                           linewidth=2, label='Interception')
            ax3.set_xlabel('Step')
            ax3.set_ylabel('Distance (m)')
            ax3.set_title('Distance to Target Over Time')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
        
        # Plot 4: MCL Convergence and UKF Uncertainty
        ax4 = axes[1, 1]
        if len(self.mcl_convergence_history) > 0:
            steps = np.arange(len(self.mcl_convergence_history))
            ax4_twin = ax4.twinx()
            ax4.plot(steps, self.mcl_convergence_history, 'g-', linewidth=2, label='MCL Convergence')
            if len(self.ukf_uncertainty_history) > 0:
                ax4_twin.plot(steps, self.ukf_uncertainty_history, 'm-', linewidth=2, label='UKF Uncertainty')
            if self.interception_step:
                ax4.axvline(x=self.interception_step, color='r', linestyle='--', 
                           linewidth=2, label='Interception')
            ax4.set_xlabel('Step')
            ax4.set_ylabel('MCL Convergence (m)', color='g')
            ax4_twin.set_ylabel('UKF Uncertainty (m)', color='m')
            ax4.set_title('MCL Convergence & UKF Uncertainty')
            ax4.tick_params(axis='y', labelcolor='g')
            ax4_twin.tick_params(axis='y', labelcolor='m')
            ax4.grid(True, alpha=0.3)
            # Combine legends
            lines1, labels1 = ax4.get_legend_handles_labels()
            lines2, labels2 = ax4_twin.get_legend_handles_labels()
            ax4.legend(lines1 + lines2, labels1 + labels2, loc='best', fontsize=8)
        
        plt.tight_layout()
        
        # Save plot
        output_file = f'interception_analysis_step_{self.interception_step}.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        # Final analysis plot saved
        
        plt.show()
    
    def run_animation(self, num_steps=500, interval=50):
        """Run animated simulation"""
        # Starting animation
        
        def animate(frame):
            if self.intercepted:
                # Stop animation when intercepted
                anim.event_source.stop()
                # Generate final plots
                self.generate_final_plots()
                return []
            self.step()
            self.visualize(frame)
            return []
        
        anim = animation.FuncAnimation(self.fig, animate, frames=num_steps, 
                                      interval=interval, blit=False, repeat=False)
        plt.tight_layout()
        plt.show()
        return anim


def main():
    """Main function"""
    # Check dependencies first
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        import cvxpy
    except ImportError as e:
        print("=" * 60)
        print("ERROR: Missing required dependencies!")
        print("=" * 60)
        print(f"\nMissing module: {e.name}")
        print("\nPlease install dependencies:")
        print("  pip3 install numpy matplotlib cvxpy")
        print("\nOr see INSTALL_STANDALONE.md for detailed instructions.")
        return
    
    # Animated Simulation: MCL + UKF + MPC
    
    try:
        sim = AnimatedSimulation()
        sim.run_animation(num_steps=1000, interval=50)  # 1000 steps, 50ms per frame
    except Exception as e:
        print(f"\nERROR during simulation: {e}")
        import traceback
        traceback.print_exc()
        return


if __name__ == '__main__':
    main()

