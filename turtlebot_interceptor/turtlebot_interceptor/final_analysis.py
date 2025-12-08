#!/usr/bin/env python3
"""
Final Trajectory Analysis
Generates comprehensive post-processing analysis of the complete navigation run.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import os

class FinalTrajectoryAnalysis:
    """Generate comprehensive analysis of complete navigation trajectory"""
    
    def __init__(self):
        pass
    
    def analyze_and_visualize(self, trajectory_history, obstacles, map_data, goal_pos, save_dir="/tmp"):
        """
        Generate comprehensive final analysis visualization.
        
        Args:
            trajectory_history: List of dicts with 'time', 'pose', 'command', 'goal'
            obstacles: List of (center, radius) tuples
            map_data: Optional dict with 'map' (OccupancyGrid), 'origin', 'resolution'
            goal_pos: [x, y] goal position
            save_dir: Directory to save analysis plots
        """
        if not trajectory_history:
            print("⚠️ No trajectory history to analyze")
            return
        
        os.makedirs(save_dir, exist_ok=True)
        
        # Extract trajectory data
        times = [entry['time'] for entry in trajectory_history]
        poses = [entry['pose'] for entry in trajectory_history]
        commands = [entry['command'] for entry in trajectory_history]
        
        traj_x = [pose[0] for pose in poses]
        traj_y = [pose[1] for pose in poses]
        traj_theta = [pose[2] for pose in poses]
        traj_v = [pose[3] for pose in poses]
        
        velocities = [cmd['v'] for cmd in commands]
        omegas = [cmd['omega'] for cmd in commands]
        
        # Calculate metrics
        total_distance = self._calculate_total_distance(traj_x, traj_y)
        total_time = times[-1] - times[0] if len(times) > 1 else 0
        avg_velocity = np.mean(velocities) if velocities else 0
        max_velocity = np.max(velocities) if velocities else 0
        avg_angular_velocity = np.mean(np.abs(omegas)) if omegas else 0
        
        # Create comprehensive figure with subplots
        fig = plt.figure(figsize=(20, 16))
        gs = fig.add_gridspec(4, 3, hspace=0.3, wspace=0.3)
        
        # Main trajectory plot (large, top-left spanning 2x2)
        ax_main = fig.add_subplot(gs[0:2, 0:2])
        self._plot_main_trajectory(ax_main, traj_x, traj_y, traj_theta, obstacles, 
                                   map_data, goal_pos, poses[0] if poses else None)
        
        # Velocity profile
        ax_vel = fig.add_subplot(gs[0, 2])
        self._plot_velocity_profile(ax_vel, times, velocities, traj_v)
        
        # Angular velocity profile
        ax_omega = fig.add_subplot(gs[1, 2])
        self._plot_angular_velocity(ax_omega, times, omegas)
        
        # Distance to goal over time
        ax_dist = fig.add_subplot(gs[2, 0])
        self._plot_distance_to_goal(ax_dist, times, traj_x, traj_y, goal_pos)
        
        # Path curvature
        ax_curv = fig.add_subplot(gs[2, 1])
        self._plot_path_curvature(ax_curv, traj_x, traj_y)
        
        # Statistics panel
        ax_stats = fig.add_subplot(gs[2, 2])
        self._plot_statistics(ax_stats, total_distance, total_time, avg_velocity, 
                             max_velocity, avg_angular_velocity, len(trajectory_history))
        
        # Command history (velocity vs angular velocity)
        ax_cmd = fig.add_subplot(gs[3, 0:2])
        self._plot_command_space(ax_cmd, velocities, omegas)
        
        # Final summary text
        ax_summary = fig.add_subplot(gs[3, 2])
        self._plot_summary(ax_summary, total_distance, total_time, avg_velocity, 
                          max_velocity, len(obstacles), len(trajectory_history))
        
        # Save comprehensive analysis
        save_path = os.path.join(save_dir, "final_trajectory_analysis.png")
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        print(f"✅ Final analysis saved to: {save_path}")
        
        # Show interactively
        print("📊 Displaying final trajectory analysis... Close the window to continue.")
        plt.show(block=True)
        print("✅ Analysis window closed.")
    
    def _calculate_total_distance(self, traj_x, traj_y):
        """Calculate total distance traveled"""
        total = 0.0
        for i in range(1, len(traj_x)):
            dx = traj_x[i] - traj_x[i-1]
            dy = traj_y[i] - traj_y[i-1]
            total += np.sqrt(dx*dx + dy*dy)
        return total
    
    def _plot_main_trajectory(self, ax, traj_x, traj_y, traj_theta, obstacles, 
                              map_data, goal_pos, start_pose):
        """Plot main trajectory with full environment"""
        # Plot map if available
        if map_data and map_data.get('map'):
            map_grid = map_data['map']
            origin_x = map_grid.info.origin.position.x
            origin_y = map_grid.info.origin.position.y
            width = map_grid.info.width
            height = map_grid.info.height
            res = map_grid.info.resolution
            
            map_array = np.array(map_grid.data).reshape((height, width))
            map_display = np.zeros_like(map_array, dtype=float)
            map_display[map_array == -1] = 0.5
            map_display[map_array == 0] = 1.0
            map_display[map_array > 0] = 0.0
            
            x_min = origin_x
            x_max = origin_x + width * res
            y_min = origin_y
            y_max = origin_y + height * res
            
            ax.imshow(map_display, extent=[x_min, x_max, y_min, y_max], 
                     origin='lower', cmap='gray', alpha=0.6, interpolation='nearest')
        
        # Plot obstacles
        for i, (center, radius) in enumerate(obstacles):
            circle = Circle((center[0], center[1]), radius, color='red', 
                          alpha=0.6, linewidth=2, fill=True, edgecolor='darkred')
            ax.add_patch(circle)
        
        # Plot trajectory with color gradient
        colors = plt.cm.viridis(np.linspace(0, 1, len(traj_x)))
        ax.scatter(traj_x, traj_y, c=colors, s=10, alpha=0.6, zorder=5)
        ax.plot(traj_x, traj_y, 'b-', linewidth=2, alpha=0.8, zorder=4, label='Actual Path')
        
        # Plot start
        if start_pose:
            ax.scatter(start_pose[0], start_pose[1], c='green', s=200, marker='s', 
                      zorder=8, edgecolors='darkgreen', linewidths=2, label='Start')
        
        # Plot goal
        ax.scatter(goal_pos[0], goal_pos[1], c='red', s=300, marker='*', 
                  zorder=8, edgecolors='darkred', linewidths=2, label='Goal')
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Complete Navigation Trajectory', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
    
    def _plot_velocity_profile(self, ax, times, velocities, actual_velocities):
        """Plot velocity over time"""
        if len(times) > 1:
            ax.plot(times, velocities, 'g-', linewidth=2, label='Commanded', alpha=0.7)
            ax.plot(times, actual_velocities, 'b--', linewidth=2, label='Actual', alpha=0.7)
        ax.set_xlabel('Time (s)', fontsize=10)
        ax.set_ylabel('Velocity (m/s)', fontsize=10)
        ax.set_title('Velocity Profile', fontsize=12, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def _plot_angular_velocity(self, ax, times, omegas):
        """Plot angular velocity over time"""
        if len(times) > 1:
            ax.plot(times, omegas, 'r-', linewidth=2, alpha=0.7)
        ax.set_xlabel('Time (s)', fontsize=10)
        ax.set_ylabel('Angular Vel (rad/s)', fontsize=10)
        ax.set_title('Angular Velocity', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    def _plot_distance_to_goal(self, ax, times, traj_x, traj_y, goal_pos):
        """Plot distance to goal over time"""
        distances = [np.sqrt((x - goal_pos[0])**2 + (y - goal_pos[1])**2) 
                     for x, y in zip(traj_x, traj_y)]
        if len(times) > 1:
            ax.plot(times, distances, 'purple', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=10)
        ax.set_ylabel('Distance to Goal (m)', fontsize=10)
        ax.set_title('Distance to Goal', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0.08, color='r', linestyle='--', alpha=0.5, label='Goal Threshold')
        ax.legend()
    
    def _plot_path_curvature(self, ax, traj_x, traj_y):
        """Plot path curvature"""
        if len(traj_x) < 3:
            ax.text(0.5, 0.5, 'Insufficient data', ha='center', va='center', transform=ax.transAxes)
            return
        
        curvatures = []
        indices = []
        for i in range(1, len(traj_x) - 1):
            p1 = np.array([traj_x[i-1], traj_y[i-1]])
            p2 = np.array([traj_x[i], traj_y[i]])
            p3 = np.array([traj_x[i+1], traj_y[i+1]])
            
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                # Approximate curvature
                cross = np.cross(v1, v2)
                curvature = abs(cross) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
                curvatures.append(curvature)
                indices.append(i)
        
        if curvatures:
            ax.plot(indices, curvatures, 'orange', linewidth=2)
        ax.set_xlabel('Path Index', fontsize=10)
        ax.set_ylabel('Curvature', fontsize=10)
        ax.set_title('Path Curvature', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
    
    def _plot_statistics(self, ax, total_dist, total_time, avg_vel, max_vel, avg_omega, num_steps):
        """Plot statistics as text"""
        ax.axis('off')
        stats_text = f"""
📊 NAVIGATION STATISTICS

Total Distance: {total_dist:.2f} m
Total Time: {total_time:.1f} s
Average Speed: {avg_vel:.3f} m/s
Max Speed: {max_vel:.3f} m/s
Avg Angular Vel: {avg_omega:.3f} rad/s
Total Steps: {num_steps}

Efficiency:
  Avg Speed: {avg_vel:.3f} m/s
  Path Length: {total_dist:.2f} m
"""
        ax.text(0.1, 0.5, stats_text, transform=ax.transAxes, 
               fontsize=11, family='monospace', verticalalignment='center',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def _plot_command_space(self, ax, velocities, omegas):
        """Plot command space (velocity vs angular velocity)"""
        ax.scatter(velocities, omegas, c=range(len(velocities)), cmap='viridis', 
                  s=20, alpha=0.6)
        ax.set_xlabel('Linear Velocity (m/s)', fontsize=10)
        ax.set_ylabel('Angular Velocity (rad/s)', fontsize=10)
        ax.set_title('Command Space', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    
    def _plot_summary(self, ax, total_dist, total_time, avg_vel, max_vel, num_obstacles, num_steps):
        """Plot final summary"""
        ax.axis('off')
        summary_text = f"""
🎯 MISSION SUMMARY

✅ Goal Reached Successfully!

Distance Traveled: {total_dist:.2f} m
Time Elapsed: {total_time:.1f} s
Obstacles Encountered: {num_obstacles}
Navigation Steps: {num_steps}

Performance:
  Average Velocity: {avg_vel:.3f} m/s
  Maximum Velocity: {max_vel:.3f} m/s
"""
        ax.text(0.1, 0.5, summary_text, transform=ax.transAxes, 
               fontsize=11, family='monospace', verticalalignment='center',
               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

