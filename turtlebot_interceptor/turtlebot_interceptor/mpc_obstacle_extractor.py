#!/usr/bin/env python3
"""
Obstacle extraction utilities for MPC navigation
Extracts obstacles from various sensor sources (maps, LIDAR, camera, etc.)
"""
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def improved_obstacle_clustering(occupied_points, eps=0.15, min_samples=3):
    """DBSCAN-like clustering to group nearby occupied cells into obstacles."""
    if len(occupied_points) == 0:
        return []
    
    points = np.array(occupied_points)
    n_points = len(points)
    labels = -np.ones(n_points, dtype=int)
    cluster_id = 0
    
    for i in range(n_points):
        if labels[i] != -1:
            continue
        
        dists = np.linalg.norm(points - points[i], axis=1)
        neighbors = np.where(dists <= eps)[0]
        
        if len(neighbors) < min_samples:
            labels[i] = -1
            continue
        
        labels[i] = cluster_id
        seed_set = list(neighbors)
        
        while seed_set:
            current_point = seed_set.pop(0)
            if labels[current_point] == -1:
                labels[current_point] = cluster_id
            elif labels[current_point] != -1:
                continue
            labels[current_point] = cluster_id
            
            dists = np.linalg.norm(points - points[current_point], axis=1)
            new_neighbors = np.where(dists <= eps)[0]
            if len(new_neighbors) >= min_samples:
                seed_set.extend(new_neighbors)
        
        cluster_id += 1
    
    obstacles = []
    for cid in range(cluster_id):
        cluster_points = points[labels == cid]
        if len(cluster_points) < 2:
            continue
        
        center = np.mean(cluster_points, axis=0)
        distances = np.linalg.norm(cluster_points - center, axis=1)
        radius = np.max(distances) + 0.05 + 0.14
        radius = max(radius, 0.14)
        obstacles.append((center, radius))
    
    return obstacles


def extract_cones_from_map(map_msg, seeker_state, obstacle_radius_param=0.08):
    """Extract circular obstacles (cones) from occupancy grid."""
    obstacles = []
    if map_msg is None or seeker_state is None:
        return obstacles
    
    if map_msg.header.frame_id != 'map':
        obstacles.append(("frame_warning", map_msg.header.frame_id))
    
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    
    occupied_cells = []
    wall_margin = 0.35
    occupancy_threshold = 30
    
    for i in range(width * height):
        if map_msg.data[i] > occupancy_threshold:
            gx = i % width
            gy = i // width
            world_x = gx * resolution + origin_x
            world_y = gy * resolution + origin_y
            
            if (world_x < origin_x + wall_margin or 
                world_x > origin_x + width * resolution - wall_margin or
                world_y < origin_y + wall_margin or 
                world_y > origin_y + height * resolution - wall_margin):
                continue
            
            occupied_cells.append((world_x, world_y))
    
    if len(occupied_cells) == 0:
        return obstacles
    
    cluster_distance = 0.15
    clusters = []
    for x, y in occupied_cells:
        assigned = False
        for cluster in clusters:
            cluster_points = np.array(cluster)
            cluster_center = np.mean(cluster_points, axis=0)
            dist = np.sqrt((x - cluster_center[0])**2 + (y - cluster_center[1])**2)
            if dist < cluster_distance:
                cluster.append((x, y))
                assigned = True
                break
        if not assigned:
            clusters.append([(x, y)])
    
    for cluster in clusters:
        if len(cluster) < 2:
            continue
        
        points = np.array(cluster)
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
        width_cluster = max_x - min_x
        height_cluster = max_y - min_y
        
        if width_cluster < 0.005 or height_cluster < 0.005:
            continue
        
        if width_cluster > 0.01 and height_cluster > 0.01:
            aspect_ratio = max(width_cluster, height_cluster) / min(width_cluster, height_cluster)
            if aspect_ratio > 3.0:
                continue
        
        center_x = np.mean(points[:, 0])
        center_y = np.mean(points[:, 1])
        distances_from_center = np.sqrt((points[:, 0] - center_x)**2 + (points[:, 1] - center_y)**2)
        max_dist = np.max(distances_from_center)
        radius = max_dist + resolution * 3
        radius_bbox = max(width_cluster, height_cluster) / 2 + resolution * 2
        radius = max(radius, radius_bbox) * 1.5
        
        if 0.05 < radius < 0.6:
            current_x = seeker_state[0]
            current_y = seeker_state[1]
            dx = center_x - current_x
            dy = center_y - current_y
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > 0.01:
                ux = dx / dist
                uy = dy / dist
                corrected_x = center_x + ux * radius
                corrected_y = center_y + uy * radius
                obstacles.append((np.array([corrected_x, corrected_y]), radius))
            else:
                obstacles.append((np.array([center_x, center_y]), radius))
    
    return obstacles[:10]


def extract_lidar_obstacles(latest_scan, seeker_state, obstacle_radius_param, lidar_angle_offset=0.0, obstacle_timeout=5.0, persistent_obstacles=None, current_time=None):
    """Convert LIDAR scan to obstacles."""
    if latest_scan is None or seeker_state is None:
        return [], persistent_obstacles if persistent_obstacles is not None else []
    
    obstacles = []
    robot_x = seeker_state[0]
    robot_y = seeker_state[1]
    robot_theta = seeker_state[2]
    if current_time is None:
        current_time = 0.0
    
    angle_min = latest_scan.angle_min
    angle_increment = latest_scan.angle_increment
    ranges = latest_scan.ranges
    range_max = latest_scan.range_max
    
    safety_inflation = 0.15
    
    for i, r in enumerate(ranges):
        if r < 0.1 or r > range_max or not np.isfinite(r):
            continue
        if r > 2.0:
            continue
        
        ray_angle = angle_min + i * angle_increment
        world_angle = robot_theta + ray_angle + lidar_angle_offset
        obstacle_x = robot_x + (r + obstacle_radius_param + safety_inflation) * np.cos(world_angle)
        obstacle_y = robot_y + (r + obstacle_radius_param + safety_inflation) * np.sin(world_angle)
        
        obstacle_pos = np.array([obstacle_x, obstacle_y])
        inflated_radius = obstacle_radius_param + safety_inflation
        obstacles.append((obstacle_pos, inflated_radius))
        
        if persistent_obstacles is not None:
            persistent_obstacles.append((obstacle_pos, obstacle_radius_param, current_time))
            persistent_obstacles = [
                (pos, radius, t) for pos, radius, t in persistent_obstacles
                if current_time - t < obstacle_timeout
            ]
    
    return obstacles, persistent_obstacles if persistent_obstacles is not None else []


def extract_scan_matched_obstacles(matched_points, seeker_state, obstacle_radius_param):
    """Extract obstacles from Cartographer's scan-matched point cloud."""
    if matched_points is None or seeker_state is None:
        return []
    
    obstacles = []
    robot_x = seeker_state[0]
    robot_y = seeker_state[1]
    
    try:
        for point in point_cloud2.read_points(matched_points, field_names=("x", "y", "z"), skip_nans=True):
            px, py, pz = point
            dx = px - robot_x
            dy = py - robot_y
            dist = np.sqrt(dx*dx + dy*dy)
            
            if 0.1 < dist < 2.0:
                ux = dx / dist
                uy = dy / dist
                safety_inflation = 0.15
                corrected_x = px + ux * (obstacle_radius_param + safety_inflation)
                corrected_y = py + uy * (obstacle_radius_param + safety_inflation)
                inflated_radius = obstacle_radius_param + safety_inflation
                obstacles.append((np.array([corrected_x, corrected_y]), inflated_radius))
    except Exception:
        pass
    
    return obstacles


def extract_map_obstacles_from_grid(grid_map, seeker_state, obstacle_radius_param=0.08):
    """Extract obstacles from fast local grid."""
    obstacles = []
    if grid_map is None or seeker_state is None:
        return obstacles
    
    width = grid_map.info.width
    height = grid_map.info.height
    resolution = grid_map.info.resolution
    origin_x = grid_map.info.origin.position.x
    origin_y = grid_map.info.origin.position.y
    
    occupancy_threshold = 30
    
    occupied_cells = []
    for i in range(width * height):
        if grid_map.data[i] > occupancy_threshold:
            gx = i % width
            gy = i // width
            world_x = gx * resolution + origin_x + resolution / 2
            world_y = gy * resolution + origin_y + resolution / 2
            occupied_cells.append((world_x, world_y))
    
    if len(occupied_cells) == 0:
        return obstacles
    
    clusters = improved_obstacle_clustering(occupied_cells, eps=0.15, min_samples=2)
    
    robot_pos = seeker_state[:2]
    for center, radius in clusters:
        dist = np.linalg.norm(center - robot_pos)
        if 0.05 < radius < 0.6 and dist < 2.0:
            obstacles.append((center, radius))
    
    return obstacles


def merge_obstacles(obstacles, seeker_state, max_obstacles=100):
    """Remove duplicate obstacles and limit count."""
    if len(obstacles) == 0:
        return []
    
    robot_x = seeker_state[0]
    robot_y = seeker_state[1]
    
    obstacles.sort(key=lambda obs: np.sqrt((obs[0][0]-robot_x)**2 + (obs[0][1]-robot_y)**2))
    
    unique_obstacles = []
    for obs in obstacles:
        is_duplicate = False
        obs_center = obs[0]
        obs_radius = obs[1]
        
        for i, existing in enumerate(unique_obstacles):
            existing_center = existing[0]
            existing_radius = existing[1]
            dist = np.sqrt((obs_center[0]-existing_center[0])**2 + (obs_center[1]-existing_center[1])**2)
            if dist < 0.20:
                if obs_radius > existing_radius:
                    unique_obstacles[i] = obs
                is_duplicate = True
                break
        
        if not is_duplicate:
            unique_obstacles.append(obs)
    
    if len(unique_obstacles) > max_obstacles:
        unique_obstacles = unique_obstacles[:max_obstacles]
    
    return unique_obstacles

