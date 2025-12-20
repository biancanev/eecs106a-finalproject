#!/usr/bin/env python3
"""
Safety utilities for MPC navigation
Helper functions for obstacle avoidance and trajectory safety checking
"""
import numpy as np


def compute_min_obstacle_distance(x0, obstacles):
    """Compute minimum distance to any obstacle."""
    if not obstacles or len(obstacles) == 0:
        return float('inf')
    
    robot_pos = x0[:2]
    min_dist = float('inf')
    
    for center, radius in obstacles:
        dist_to_center = np.linalg.norm(center - robot_pos)
        dist_to_surface = dist_to_center - radius
        min_dist = min(min_dist, dist_to_surface)
    
    return max(0.0, min_dist)


def line_of_sight_clear(start, goal, obstacles, safety_margin=0.2):
    """Check if straight path from start to goal is free of obstacles."""
    if goal is None or start is None:
        return False
    if np.allclose(start, goal):
        return True
    for center, radius in obstacles:
        dist = point_to_segment_distance(center, start, goal)
        if dist < (radius + safety_margin):
            return False
    return True


def point_to_segment_distance(point, a, b):
    """Compute distance from point to line segment ab."""
    a = np.array(a)
    b = np.array(b)
    p = np.array(point)
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-9:
        return np.linalg.norm(p - a)
    t = np.clip(np.dot(p - a, ab) / denom, 0.0, 1.0)
    proj = a + t * ab
    return np.linalg.norm(p - proj)


def compute_velocity_scale(min_obs_dist):
    """Adaptive velocity scaling based on obstacle proximity."""
    if min_obs_dist >= 0.8:
        return 1.0
    elif min_obs_dist >= 0.5:
        return 0.9 + 0.1 * (min_obs_dist - 0.5) / 0.3
    elif min_obs_dist >= 0.3:
        return 0.75 + 0.15 * (min_obs_dist - 0.3) / 0.2
    elif min_obs_dist >= 0.15:
        return 0.6 + 0.15 * (min_obs_dist - 0.15) / 0.15
    else:
        return 0.5


def verify_full_trajectory_safety(x0, v, omega, obstacles, horizon_steps=15, dt=0.1):
    """Verify safety of entire predicted trajectory."""
    if not obstacles or len(obstacles) == 0:
        return True, float('inf')
    
    x, y, theta, v_curr = x0[0], x0[1], x0[2], x0[3]
    min_clearance = float('inf')
    robot_radius = 0.105
    safety_margin = 0.12
    
    for step in range(horizon_steps):
        x += v_curr * np.cos(theta) * dt
        y += v_curr * np.sin(theta) * dt
        theta += omega * dt
        v_curr = v
        
        robot_pos = np.array([x, y])
        for obs_center, obs_radius in obstacles:
            dist_to_center = np.linalg.norm(obs_center - robot_pos)
            clearance = dist_to_center - obs_radius - robot_radius - safety_margin
            min_clearance = min(min_clearance, clearance)
            
            if clearance < 0.0:
                return False, clearance
    
    return True, min_clearance


def is_command_safe(latest_scan, seeker_state, v_cmd, omega_cmd, lidar_angle_offset=0.0):
    """Check if executing this command would cause collision."""
    if latest_scan is None or seeker_state is None:
        return True
    
    dt = 0.1
    x = seeker_state[0]
    y = seeker_state[1]
    theta = seeker_state[2]
    
    new_theta = theta + omega_cmd * dt
    new_x = x + v_cmd * np.cos(new_theta) * dt
    new_y = y + v_cmd * np.sin(new_theta) * dt
    
    ranges = latest_scan.ranges
    angle_min = latest_scan.angle_min
    angle_increment = latest_scan.angle_increment
    
    safety_dist = 0.25
    
    move_direction = np.arctan2(new_y - y, new_x - x) - theta
    move_direction = np.arctan2(np.sin(move_direction), np.cos(move_direction))
    
    for i, r in enumerate(ranges):
        if not np.isfinite(r) or r > latest_scan.range_max:
            continue
        
        angle = angle_min + i * angle_increment + lidar_angle_offset
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        
        if abs(angle - move_direction) < np.pi / 4:
            if r < safety_dist:
                return False
    
    return True

