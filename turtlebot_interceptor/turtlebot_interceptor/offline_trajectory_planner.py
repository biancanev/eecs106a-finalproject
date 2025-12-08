#!/usr/bin/env python3
"""
Offline Trajectory Planner
Computes full trajectory from start to goal using MPC with known environment.
Visualizes the complete path for debugging and validation.
"""
import numpy as np
import matplotlib.pyplot as plt
from turtlebot_interceptor.MPC_test import SimpleUnicycleMPC

class OfflineTrajectoryPlanner:
    """Plan full trajectory offline using MPC with known obstacles"""
    
    def __init__(self, dt=0.1, N=15, v_max=0.35, omega_max=2.84):
        self.dt = dt
        self.N = N
        # MPC uses 'horizon' not 'N', and doesn't accept v_max/omega_max in constructor
        self.mpc = SimpleUnicycleMPC(horizon=N, dt=dt)
        # Set velocity limits after initialization
        self.mpc.v_max = v_max
        self.mpc.vx_max = v_max  # Also set vx_max
        self.mpc.omega_max = omega_max
        self.mpc.wz_max = omega_max  # Also set wz_max
        
    def plan_trajectory(self, start_pose, goal_pos, obstacles, max_steps=200):
        """
        Plan full trajectory from start to goal.
        
        Args:
            start_pose: [x, y, theta, v] initial state
            goal_pos: [x, y] goal position
            obstacles: List of (center, radius) tuples
            max_steps: Maximum planning steps
        
        Returns:
            trajectory: List of [x, y, theta, v] states
            commands: List of [v, omega] commands
        """
        trajectory = [start_pose.copy()]
        commands = []
        current_state = start_pose.copy()
        
        for step in range(max_steps):
            # Check if we've reached the goal
            dist_to_goal = np.sqrt((current_state[0] - goal_pos[0])**2 + 
                                  (current_state[1] - goal_pos[1])**2)
            if dist_to_goal < 0.15:  # 15cm threshold
                break
            
            # Create constant target sequence (goal position)
            target_seq = np.zeros((2, self.N + 1))
            target_seq[0, :] = goal_pos[0]
            target_seq[1, :] = goal_pos[1]
            
            # Solve MPC with obstacles
            try:
                twist_cmd = self.mpc.get_twist_command(current_state, target_seq, obstacles)
                v_cmd = twist_cmd['linear']['x']
                omega_cmd = twist_cmd['angular']['z']
                
                # Validate commands
                if np.isnan(v_cmd) or np.isnan(omega_cmd) or \
                   np.isinf(v_cmd) or np.isinf(omega_cmd):
                    break
                
                commands.append([v_cmd, omega_cmd])
                
                # Simulate forward one step
                x, y, theta, v = current_state
                v = np.clip(v + (v_cmd - v) * 0.5, 0.0, self.mpc.v_max)  # Smooth velocity
                theta = theta + omega_cmd * self.dt
                theta = np.mod(theta + np.pi, 2*np.pi) - np.pi
                x = x + self.dt * v * np.cos(theta)
                y = y + self.dt * v * np.sin(theta)
                
                current_state = np.array([x, y, theta, v])
                trajectory.append(current_state.copy())
                
            except Exception as e:
                print(f"MPC solve failed at step {step}: {e}")
                break
        
        return trajectory, commands
    
    def visualize_trajectory(self, trajectory, obstacles, goal_pos, map_data=None, save_path=None):
        """
        Visualize the planned trajectory with full environment.
        
        Args:
            trajectory: List of [x, y, theta, v] states
            obstacles: List of (center, radius) tuples
            goal_pos: [x, y] goal position
            map_data: Optional dict with 'map' (OccupancyGrid), 'origin', 'resolution'
            save_path: Optional path to save figure
        """
        fig, ax = plt.subplots(figsize=(16, 12))
        
        # Plot occupancy grid map if available
        if map_data is not None:
            map_grid = map_data.get('map')
            origin = map_data.get('origin', [0, 0])
            resolution = map_data.get('resolution', 0.05)
            
            if map_grid is not None:
                # Extract map dimensions
                width = map_grid.info.width
                height = map_grid.info.height
                origin_x = map_grid.info.origin.position.x
                origin_y = map_grid.info.origin.position.y
                res = map_grid.info.resolution
                
                # Create map image
                map_array = np.array(map_grid.data).reshape((height, width))
                
                # Create extent for imshow
                x_min = origin_x
                x_max = origin_x + width * res
                y_min = origin_y
                y_max = origin_y + height * res
                
                # Plot map: unknown=gray, free=white, occupied=black
                map_display = np.zeros_like(map_array, dtype=float)
                map_display[map_array == -1] = 0.5  # Unknown = gray
                map_display[map_array == 0] = 1.0   # Free = white
                map_display[map_array > 0] = 0.0    # Occupied = black
                
                ax.imshow(map_display, extent=[x_min, x_max, y_min, y_max], 
                         origin='lower', cmap='gray', alpha=0.7, interpolation='nearest')
                
                # Plot occupied cells as red dots for visibility
                occupied_x = []
                occupied_y = []
                for i, val in enumerate(map_grid.data):
                    if val > 30:  # Occupied threshold
                        gx = i % width
                        gy = i // width
                        wx = origin_x + gx * res + res / 2
                        wy = origin_y + gy * res + res / 2
                        occupied_x.append(wx)
                        occupied_y.append(wy)
                
                if occupied_x:
                    ax.scatter(occupied_x, occupied_y, c='darkred', s=1, alpha=0.3, 
                             label='Occupied Cells', zorder=1)
        
        # Plot detected obstacles as circles
        for i, (center, radius) in enumerate(obstacles):
            circle = plt.Circle((center[0], center[1]), radius, 
                              color='red', alpha=0.6, linewidth=2, 
                              fill=True, edgecolor='darkred',
                              label='Detected Obstacles' if i == 0 else '')
            ax.add_patch(circle)
            # Add obstacle ID
            ax.text(center[0], center[1], f'{i+1}', ha='center', va='center', 
                   fontsize=8, color='white', weight='bold', zorder=10)
        
        # Plot trajectory
        traj_x = [state[0] for state in trajectory]
        traj_y = [state[1] for state in trajectory]
        
        # Plot trajectory line
        ax.plot(traj_x, traj_y, 'b-', linewidth=3, label='Planned Trajectory', 
               alpha=0.8, zorder=5)
        
        # Plot trajectory points with color gradient (start=green, end=red)
        colors = plt.cm.viridis(np.linspace(0, 1, len(traj_x[::2])))
        ax.scatter(traj_x[::2], traj_y[::2], c=colors, s=30, alpha=0.6, 
                  marker='o', edgecolors='blue', linewidths=0.5, zorder=6)
        
        # Plot start
        ax.scatter(traj_x[0], traj_y[0], c='green', s=200, marker='s', 
                  label='Start', zorder=8, edgecolors='darkgreen', linewidths=2)
        ax.text(traj_x[0], traj_y[0] + 0.1, 'START', ha='center', 
               fontsize=10, weight='bold', color='green', zorder=9)
        
        # Plot goal
        ax.scatter(goal_pos[0], goal_pos[1], c='red', s=300, marker='*', 
                  label='Goal', zorder=8, edgecolors='darkred', linewidths=2)
        ax.text(goal_pos[0], goal_pos[1] + 0.15, 'GOAL', ha='center', 
               fontsize=10, weight='bold', color='red', zorder=9)
        
        # Plot robot orientation at key points
        key_indices = [0, len(trajectory)//4, len(trajectory)//2, 
                      3*len(trajectory)//4, len(trajectory)-1]
        for i in key_indices:
            if i < len(trajectory):
                state = trajectory[i]
                dx = 0.15 * np.cos(state[2])
                dy = 0.15 * np.sin(state[2])
                ax.arrow(state[0], state[1], dx, dy, 
                        head_width=0.08, head_length=0.08, fc='blue', ec='blue', 
                        alpha=0.7, zorder=7, width=0.02)
        
        # Add velocity profile visualization
        velocities = [state[3] for state in trajectory]
        if velocities:
            # Create secondary axis for velocity
            ax2 = ax.twinx()
            ax2.plot(traj_x, velocities, 'g--', linewidth=2, alpha=0.5, label='Velocity')
            ax2.set_ylabel('Velocity (m/s)', color='green', fontsize=10)
            ax2.tick_params(axis='y', labelcolor='green')
            ax2.set_ylim([0, max(velocities) * 1.2])
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Offline MPC Trajectory Planning - Full Environment', 
                    fontsize=16, fontweight='bold', pad=20)
        ax.legend(loc='upper left', fontsize=10, framealpha=0.9)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')
        
        # Add comprehensive info text
        total_dist = 0.0
        for i in range(1, len(trajectory)):
            dx = traj_x[i] - traj_x[i-1]
            dy = traj_y[i] - traj_y[i-1]
            total_dist += np.sqrt(dx*dx + dy*dy)
        
        info_text = f'📊 TRAJECTORY INFO:\n'
        info_text += f'Steps: {len(trajectory)}\n'
        info_text += f'Total Distance: {total_dist:.2f}m\n'
        info_text += f'Start: ({traj_x[0]:.2f}, {traj_y[0]:.2f})\n'
        info_text += f'Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})\n'
        info_text += f'Final Distance to Goal: {np.sqrt((traj_x[-1]-goal_pos[0])**2 + (traj_y[-1]-goal_pos[1])**2):.2f}m\n'
        info_text += f'Max Velocity: {max(velocities):.2f} m/s\n'
        info_text += f'Avg Velocity: {np.mean(velocities):.2f} m/s\n'
        info_text += f'\n🚧 OBSTACLES:\n'
        info_text += f'Detected: {len(obstacles)}\n'
        if map_data and map_data.get('map'):
            occupied_count = sum(1 for val in map_data['map'].data if val > 30)
            info_text += f'Map Occupied Cells: {occupied_count}'
        
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
               verticalalignment='top', fontsize=9, family='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black'))
        
        plt.tight_layout()
        
        # Always save the plot
        if save_path:
            plt.savefig(save_path, dpi=200, bbox_inches='tight')
            print(f"✅ Saved comprehensive trajectory plot to {save_path}")
        
        # Show plot interactively and wait for user to close
        print("📊 Displaying trajectory plot... Close the window to continue.")
        plt.show(block=True)  # Block until window is closed
        print("✅ Plot window closed, continuing...")

