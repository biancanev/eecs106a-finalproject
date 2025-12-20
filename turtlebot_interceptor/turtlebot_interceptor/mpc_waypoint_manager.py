#!/usr/bin/env python3
"""
Waypoint generation for MPC navigation
Handles waypoint generation and visualization when obstacles block the path
"""
import numpy as np


def generate_waypoint_if_blocked(robot_pos, goal_pos, obstacles, logger=None):
    """
    Check if there's an obstacle directly ahead and generate waypoint.
    Returns: waypoint position [x, y] if blocked, None otherwise
    """
    if not obstacles or len(obstacles) == 0:
        return None
    
    robot_xy = robot_pos[:2]
    robot_theta = robot_pos[2]
    forward_dir = np.array([np.cos(robot_theta), np.sin(robot_theta)])
    
    blocking_obstacles = []
    for center, radius in obstacles:
        to_obstacle = center - robot_xy
        dist_to_obstacle = np.linalg.norm(to_obstacle)
        
        if dist_to_obstacle > 0.25:
            continue
        
        if dist_to_obstacle > 0:
            to_obstacle_norm = to_obstacle / dist_to_obstacle
            forward_alignment = np.dot(to_obstacle_norm, forward_dir)
            if forward_alignment > 0.866:  # Within ±30 degrees
                blocking_obstacles.append((center, radius, dist_to_obstacle))
    
    if not blocking_obstacles:
        return None
    
    if logger:
        logger.warn(f"🚨 Obstacle within 0.20m ahead! Generating waypoint for {len(blocking_obstacles)} obstacles...")
    
    blocking_obstacles.sort(key=lambda x: x[2])
    closest_obstacle = blocking_obstacles[0]
    key_center, key_radius, key_dist = closest_obstacle
    
    goal_dir = goal_pos - robot_xy
    goal_dist = np.linalg.norm(goal_dir)
    if goal_dist > 0:
        goal_dir = goal_dir / goal_dist
    else:
        goal_dir = forward_dir
    
    perpendicular = np.array([-goal_dir[1], goal_dir[0]])
    progress_distance = min(0.25, goal_dist * 0.25)
    waypoint_base = robot_xy + goal_dir * progress_distance
    
    min_clearance = key_radius + 0.15 + 0.03
    offset_candidates = [min_clearance, min_clearance * 1.05, min_clearance * 1.1, 0.18]
    
    best_waypoint = None
    best_clearance = -999.0
    best_side = "LEFT"
    
    for offset in offset_candidates:
        for side_mult, side_name in [(1.0, "LEFT"), (-1.0, "RIGHT")]:
            candidate = waypoint_base + perpendicular * (offset * side_mult)
            
            clearances = [np.linalg.norm(obs[0] - candidate) - obs[1] for obs in obstacles]
            min_clearance_val = min(clearances) if clearances else 999.0
            
            path_clear = True
            for obs_center, obs_radius in obstacles:
                to_candidate = candidate - robot_xy
                dist_to_candidate = np.linalg.norm(to_candidate)
                if dist_to_candidate > 0:
                    to_candidate_norm = to_candidate / dist_to_candidate
                    proj = np.dot(obs_center - robot_xy, to_candidate_norm)
                    if 0 < proj < dist_to_candidate:
                        closest_pt = robot_xy + to_candidate_norm * proj
                        perp_dist = np.linalg.norm(obs_center - closest_pt)
                        if perp_dist < (obs_radius + 0.25):
                            path_clear = False
                            break
            
            if path_clear and min_clearance_val > best_clearance:
                best_clearance = min_clearance_val
                best_waypoint = candidate
                best_side = side_name
        
        if best_waypoint is not None and best_clearance > 0.1:
            break
    
    if best_waypoint is None:
        best_waypoint = waypoint_base + perpendicular * 0.8
        best_side = "LEFT"
    
    if logger:
        logger.info(f"📍 Tight waypoint at ({best_waypoint[0]:.2f}, {best_waypoint[1]:.2f}) - routing {best_side}, clearance={best_clearance:.2f}m")
    
    return best_waypoint


def publish_waypoint_marker(waypoint_pub, waypoint, clock):
    """Publish waypoint marker for visualization in RViz."""
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = clock.now().to_msg()
    marker.ns = 'waypoint'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 0.8
    marker.pose.position.x = float(waypoint[0])
    marker.pose.position.y = float(waypoint[1])
    marker.pose.position.z = 0.2
    waypoint_pub.publish(marker)

