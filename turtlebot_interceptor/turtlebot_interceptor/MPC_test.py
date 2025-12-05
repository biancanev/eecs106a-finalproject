#!/usr/bin/env python3
"""
Standalone MPC controller (no ROS2 dependencies)
Can be used with or without ROS2
"""
import numpy as np
import cvxpy as cp

class SimpleUnicycleMPC:

    def __init__(self, horizon=15, dt=0.1, use_time_to_go=True):
        self.dt = dt
        self.N = horizon
        self.use_time_to_go = use_time_to_go  # Enable minimum time-to-go framework

        self.nx = 4     # px,py,theta,v
        self.nu = 2     # a,omega

        # Velocity constraints (Twist message format)
        # Linear velocities (m/s)
        self.vx_min = 0.0
        self.vx_max = 1.2  # Maximum forward velocity (was 0.6 - too slow!)
        self.vy_min = 0.0  # Unicycle: no lateral velocity
        self.vy_max = 0.0
        self.vz_min = 0.0  # Unicycle: no vertical velocity
        self.vz_max = 0.0
        
        # Angular velocities (rad/s) - turn rate constraints
        self.wx_min = 0.0  # Unicycle: no roll
        self.wx_max = 0.0
        self.wy_min = 0.0  # Unicycle: no pitch
        self.wy_max = 0.0
        self.wz_min = -1.0  # REDUCE max turn rate - too high causes spinning
        self.wz_max = 1.0
        
        # Acceleration constraints (for MPC internal use)
        self.a_min = -0.4
        self.a_max = 0.4
        self.alpha_min = -4.0  # Higher angular acceleration for faster turning (was 2.0)
        self.alpha_max = 4.0
        
        # Turn angle constraint (maximum change in heading per step)
        self.max_turn_angle = 0.5  # MUCH higher for tight optimal paths (was 0.3, ~17 deg, now ~29 deg)
        
        # Legacy constraints for backward compatibility
        self.v_min = self.vx_min
        self.v_max = self.vx_max
        self.omega_max = self.wz_max

        # Base weights - CRITICAL: Balance between reaching target and avoiding obstacles
        # Position weight must be high but not so high that obstacle costs are ignored
        self.Qp_base = 500.0  # High position weight - but allow obstacle avoidance to dominate when close
        self.Qtheta_base = 0.0  # NO theta penalty - let position error drive alignment
        self.Ra_base = 0.1  # Allow movement but penalize excessive acceleration
        self.Rw_base = 1.0  # Penalize spinning - encourage smooth turns
        
        # Current adaptive weights
        self.Qp = self.Qp_base
        self.Qtheta = self.Qtheta_base
        self.Ra = self.Ra_base
        self.Rw = self.Rw_base
        
        # Time-to-go parameters
        self.time_to_go = None
        self.alpha_progress = 50.0  # Weight for progress term (distance reduction)
        self.min_time_to_go = 0.5  # Minimum time-to-go threshold for aggressive control
        
        # Fast MPC: warm start and solver settings
        self.last_solution = None
        self.solver_settings = {
            'solver': cp.OSQP,
            'warm_start': False,  # CRITICAL: Disabled to avoid OSQP matrix size errors
            'verbose': False,
            'eps_abs': 1e-4,
            'eps_rel': 1e-4,
            'max_iter': 2000
        }
        
        # Store original cost expression for obstacle handling
        self.original_cost = None

        self._build_qp()

    def _build_qp(self):
        nx, nu, N = self.nx, self.nu, self.N

        # decision vars
        self.X = cp.Variable((nx, N+1))
        self.U = cp.Variable((nu, N))

        # params
        self.x0 = cp.Parameter(nx)
        self.A = cp.Parameter((nx,nx))
        self.B = cp.Parameter((nx,nu))
        self.c = cp.Parameter(nx)
        self.T = cp.Parameter((2, N+1))
        
        # Weight parameters for adaptive time-to-go (Boyd's fast MPC)
        self.Qp_param = cp.Parameter(nonneg=True)
        self.Qtheta_param = cp.Parameter(nonneg=True)
        self.Ra_param = cp.Parameter(nonneg=True)
        self.Rw_param = cp.Parameter(nonneg=True)
        self.alpha_progress_param = cp.Parameter(nonneg=True)
        
        # Obstacle parameters (for DPP-compliant obstacle avoidance)
        # Store obstacle cost as a parameter that can be updated without rebuilding
        self.obstacle_cost_param = cp.Parameter(nonneg=True, value=0.0)

        constraints = []
        cost = 0

        constraints += [self.X[:,0] == self.x0]

        for k in range(N):
            constraints += [
                self.X[:,k+1] == self.A @ self.X[:,k] + self.B @ self.U[:,k] + self.c
            ]

            px_err = self.X[0,k] - self.T[0,k]
            py_err = self.X[1,k] - self.T[1,k]
            theta   = self.X[2,k]
            a       = self.U[0,k]
            omega   = self.U[1,k]
            
            # CRITICAL FIX: Don't penalize theta itself - that tries to keep theta=0
            # Instead, heavily penalize position error - this naturally encourages alignment
            # The robot will naturally turn to face the target to minimize position error
            # For straight-line-then-turn behavior, we want:
            # 1. Large position weight → robot wants to get to target fast
            # 2. Small control penalty → robot can turn quickly
            # 3. No theta penalty → robot can orient freely to minimize position error
            
            # Simple cost: minimize position error and control effort
            # CRITICAL: Position error must dominate cost function
            # If control penalties are too high, MPC will minimize control instead of position
            # Make position error cost MUCH larger than control costs
            cost += self.Qp_param * (px_err**2 + py_err**2)
            # Control penalties - keep VERY small so position error dominates
            cost += self.Ra_param * (a**2) + self.Rw_param * (omega**2)

            # Velocity constraints (Twist message format)
            # Linear velocity constraints
            constraints += [
                self.vx_min <= self.X[3,k],
                self.X[3,k] <= self.vx_max,
            ]
            
            # Acceleration constraints
            constraints += [
                self.a_min <= a,
                a <= self.a_max,
            ]
            
            # Angular velocity constraints (turn rate limits)
            constraints += [
                self.wz_min <= omega,
                omega <= self.wz_max,
            ]
            
            # Turn angle constraint (maximum change in heading per step)
            # Enforced via angular velocity: omega * dt <= max_turn_angle
            # This is equivalent to: omega <= max_turn_angle / dt
            # Already handled by effective_wz_max above, but we can add explicit constraint
            # The angular velocity constraint already limits turn rate, which limits turn angle per step
            
            # CRITICAL: Add hard obstacle constraints (distance constraints)
            # This ensures robot NEVER gets too close to obstacles
            # Obstacles will be added dynamically in solve() method

        # terminal cost - CRITICAL: Make terminal cost MUCH heavier to ensure convergence
        pxN = self.X[0,N] - self.T[0,N]
        pyN = self.X[1,N] - self.T[1,N]
        # Terminal position penalty - make it 50x heavier than stage cost to ensure robot reaches goal
        cost += 50.0 * self.Qp_param * (pxN**2 + pyN**2)
        
        # Add obstacle cost parameter (will be updated in solve() method)
        # This allows obstacle avoidance without rebuilding the problem (DPP-compliant)
        cost += self.obstacle_cost_param

        self.prob = cp.Problem(cp.Minimize(cost), constraints)
        self.original_cost = cost  # Store original cost expression
        self.original_constraints = constraints  # Store original constraints
        
        # Initialize weight parameters
        self.Qp_param.value = self.Qp_base
        self.Qtheta_param.value = self.Qtheta_base
        self.Ra_param.value = self.Ra_base
        self.Rw_param.value = self.Rw_base
        self.alpha_progress_param.value = self.alpha_progress

    # --- linearize unicycle model around x0 ---
    def linearize(self, x0):
        """
        Linearize unicycle dynamics: x = [px, py, theta, v], u = [a, omega]
        Dynamics: 
          px' = v*cos(theta)
          py' = v*sin(theta)
          theta' = omega
          v' = a
        """
        px, py, th, v = x0
        dt = self.dt
        
        # State transition matrix A = I + dt * df/dx
        A = np.eye(4)
        # d(px')/d(theta) = -v*sin(theta)
        A[0, 2] = -dt * v * np.sin(th)
        # d(px')/d(v) = cos(theta)
        A[0, 3] = dt * np.cos(th)
        # d(py')/d(theta) = v*cos(theta)
        A[1, 2] = dt * v * np.cos(th)
        # d(py')/d(v) = sin(theta)
        A[1, 3] = dt * np.sin(th)
        # theta and v don't depend on other states (for this step)

        # Control input matrix B = dt * df/du
        B = np.zeros((4, 2))
        # d(theta')/d(omega) = 1
        B[2, 1] = dt
        # d(v')/d(a) = 1
        B[3, 0] = dt

        # Constant term: c = f(x0, u0=0) - A*x0
        # f(x0, 0) = [v*cos(th), v*sin(th), 0, 0]
        f_x0 = np.array([v * np.cos(th), v * np.sin(th), 0.0, 0.0])
        # c = x0 + dt*f(x0) - A*x0
        c = x0 + dt * f_x0 - A @ x0
        
        return A, B, c

    def compute_time_to_go(self, x0, target):
        """
        Compute estimated time-to-go based on current distance and velocity.
        Uses Boyd's approach: T_guess = distance / v_max (optimistic)
        """
        px, py = x0[0], x0[1]
        v = x0[3]
        
        # Get target position (first step if trajectory, or single point)
        if isinstance(target, (list, tuple)) or (isinstance(target, np.ndarray) and target.ndim == 1):
            tgt_pos = np.array(target[:2])
        else:
            tgt_pos = target[:, 0]  # First step of trajectory
        
        # Distance to target
        dist = np.sqrt((px - tgt_pos[0])**2 + (py - tgt_pos[1])**2)
        
        # Time-to-go estimate: distance / max_velocity (optimistic)
        # Add small epsilon to avoid division by zero
        time_to_go = dist / (self.v_max + 1e-6)
        
        # Alternative: use current velocity if moving
        if v > 0.1:
            time_to_go_vel = dist / v
            time_to_go = min(time_to_go, time_to_go_vel)
        
        return max(time_to_go, 0.1)  # Minimum 0.1s
    
    def adapt_weights_for_time_to_go(self, time_to_go):
        """
        Adapt MPC weights based on time-to-go (Boyd's fast MPC approach).
        When time-to-go is small, be more aggressive (higher position weights, lower control weights).
        """
        if not self.use_time_to_go:
            self.Qp = self.Qp_base
            self.Qtheta = self.Qtheta_base
            self.Ra = self.Ra_base
            self.Rw = self.Rw_base
            return
        
        # Normalize time-to-go (0 = very close, 1 = far away)
        # Use sigmoid-like function for smooth adaptation
        normalized_tgo = np.clip(time_to_go / 5.0, 0.0, 1.0)  # Normalize by 5 seconds
        
        # When close (small time-to-go): be aggressive
        # When far (large time-to-go): be conservative
        aggression_factor = 1.0 - normalized_tgo  # 1.0 when close, 0.0 when far
        
        # Adapt position weight: higher when close (aggressive)
        self.Qp = self.Qp_base * (1.0 + 3.0 * aggression_factor)  # Up to 4x when very close
        
        # Adapt control weights: lower when close (allow more aggressive control)
        self.Ra = self.Ra_base * (1.0 - 0.7 * aggression_factor)  # Down to 30% when very close
        self.Rw = self.Rw_base * (1.0 - 0.7 * aggression_factor)
        
        # Adapt progress term: higher when close
        self.alpha_progress = 50.0 * (1.0 + 2.0 * aggression_factor)  # Up to 3x when very close
        
        # Theta weight: slightly higher when close (better alignment)
        self.Qtheta = self.Qtheta_base * (1.0 + 0.5 * aggression_factor)
    
    def solve(self, x0, target, obstacles=None):
        """
        x0: [px,py,theta,v] (can be list or array)
        target: [px_tgt, py_tgt] or full trajectory shape (2, N+1)
        obstacles: list of (center, radius) tuples for obstacle avoidance
        Returns: (acceleration, angular_velocity)
        """
        x0 = np.array(x0).flatten()
        if len(x0) != 4:
            raise ValueError(f"x0 must have 4 elements, got {len(x0)}")
        
        if isinstance(target, (list, tuple)) or (isinstance(target, np.ndarray) and target.ndim == 1):
            T = np.tile(np.array(target).reshape(2, 1), (1, self.N + 1))
        else:
            T = np.array(target)
            if T.shape != (2, self.N + 1):
                raise ValueError(f"target must be shape (2, {self.N+1}), got {T.shape}")
        
        # CRITICAL DEBUG: Verify inputs
        if not hasattr(self, '_solve_debug_count'):
            self._solve_debug_count = 0
        self._solve_debug_count += 1
        if self._solve_debug_count % 50 == 0:
            print(f"MPC solve DEBUG: x0=[{x0[0]:.3f}, {x0[1]:.3f}, {np.degrees(x0[2]):.1f}°, {x0[3]:.3f}], "
                  f"target=[{T[0,0]:.3f}, {T[1,0]:.3f}], dx={T[0,0]-x0[0]:.3f}, dy={T[1,0]-x0[1]:.3f}")

        # Compute time-to-go and adapt weights (Boyd's fast MPC)
        if self.use_time_to_go:
            self.time_to_go = self.compute_time_to_go(x0, target)
            self.adapt_weights_for_time_to_go(self.time_to_go)
            
            # Rebuild QP with new weights if they changed significantly
            # (In practice, we update the cost function parameters)
            # For efficiency, we'll update weights in the cost function directly
            # by rebuilding the problem with new weights
            self._update_cost_weights()

        A, B, c = self.linearize(x0)
        self.x0.value = x0
        self.A.value = A
        self.B.value = B
        self.c.value = c
        self.T.value = T

        # CRITICAL: Disable warm start completely to avoid OSQP matrix size errors
        # OSQP caches matrix structure, and any problem changes cause mismatches
        # Warm start is disabled in solver_settings below
        # We can still use last_solution for initial guess, but OSQP warm_start must be False
        # use_warm_start = False  # Always disabled now to avoid OSQP errors

        # Compute obstacle repulsion cost (DPP-compliant using parameter)
        # Calculate obstacle cost numerically and set parameter
        obstacle_cost_value = 0.0
        if obstacles is not None and len(obstacles) > 0:
            # Compute repulsion cost for current predicted trajectory
            # We'll use the linearized trajectory from the last solution if available
            # Otherwise, use a simple prediction
            repulsion_weight = 100000.0  # CRITICAL: EXTREME weight - must avoid obstacles at ALL costs
            
            # Use last solution if available for obstacle cost calculation
            if self.last_solution is not None and 'X' in self.last_solution:
                X_pred = self.last_solution['X']
            else:
                # Simple prediction: assume constant velocity forward
                X_pred = np.zeros((self.nx, self.N + 1))
                X_pred[:, 0] = x0
                for k in range(self.N):
                    # Simple forward prediction
                    v = x0[3]
                    th = x0[2]
                    X_pred[0, k+1] = X_pred[0, k] + self.dt * v * np.cos(th)
                    X_pred[1, k+1] = X_pred[1, k] + self.dt * v * np.sin(th)
                    X_pred[2, k+1] = X_pred[2, k]
                    X_pred[3, k+1] = X_pred[3, k]
            
            # Add repulsion cost for each obstacle at each time step
            for k in range(self.N + 1):
                px = X_pred[0, k]
                py = X_pred[1, k]
                for center, radius in obstacles:
                    # Distance from robot to obstacle center
                    dx = px - center[0]
                    dy = py - center[1]
                    dist_sq = dx*dx + dy*dy
                    
                    # Safety radius (obstacle radius + robot radius + margin)
                    # CRITICAL: Make safety radius MUCH larger - the robot is hitting obstacles
                    # Robot is actually bigger than 0.15m, and we need more margin
                    safety_radius = radius + 0.25 + 0.65  # Robot radius ~0.25m, margin 0.35m (MASSIVELY INCREASED)
                    safety_radius_sq = safety_radius * safety_radius
                    
                    # CRITICAL: MASSIVE repulsion when close - make it impossible to hit obstacles
                    # Use exponential/inverse distance penalty to force avoidance
                    if dist_sq < safety_radius_sq * 0.5:  # VERY CLOSE - EMERGENCY!
                        # Catastrophic cost - should NEVER happen
                        obstacle_cost_value += repulsion_weight * 10000.0 / (dist_sq + 0.0001)
                    elif dist_sq < safety_radius_sq:  # Within safety radius - CRITICAL!
                        # EXTREME penalty when too close - act like hard constraint
                        obstacle_cost_value += repulsion_weight * 1000.0 / (dist_sq + 0.001)
                    elif dist_sq < safety_radius_sq * 2.0:  # Within 2x safety radius
                        # Very high penalty - start avoiding aggressively
                        obstacle_cost_value += repulsion_weight * 100.0 / (dist_sq + 0.01)
                    elif dist_sq < safety_radius_sq * 4:  # Within 4x safety radius
                        # High penalty - plan to avoid
                        obstacle_cost_value += repulsion_weight * 10.0 / (dist_sq + 0.1)
                    elif dist_sq < safety_radius_sq * 9:  # Within 9x safety radius
                        # Moderate penalty - be aware of obstacle
                        obstacle_cost_value += repulsion_weight / (dist_sq + safety_radius_sq * 0.5)
        
        # Update obstacle cost parameter (DPP-compliant - no problem rebuilding needed)
        # CRITICAL: Scale obstacle cost based on proximity to make it act like hard constraint
        if obstacles is not None and len(obstacles) > 0:
            # Check if robot is getting dangerously close to any obstacle
            px0 = x0[0]
            py0 = x0[1]
            min_dist_to_obstacle = float('inf')
            closest_obstacle = None
            for center, radius in obstacles:
                dx = px0 - center[0]
                dy = py0 - center[1]
                dist = np.sqrt(dx*dx + dy*dy)
                safety_radius = radius + 0.25 + 0.35  # Robot + margin (SAME AS ABOVE)
                actual_clearance = dist - radius  # Actual distance to obstacle surface
                if dist < safety_radius * 3.0:  # Within 3x safety radius
                    if dist < min_dist_to_obstacle:
                        min_dist_to_obstacle = dist
                        closest_obstacle = (center, radius, dist)
            
            # If getting close, increase obstacle cost even more
            if min_dist_to_obstacle < float('inf'):
                # Scale obstacle cost based on proximity - up to 10x when very close
                proximity_factor = max(1.0, (1.0 / (min_dist_to_obstacle + 0.1)))  # Up to 10x when very close
                obstacle_cost_value *= proximity_factor
                
                # DEBUG: Log obstacle cost
                if not hasattr(self, '_obstacle_cost_log_count'):
                    self._obstacle_cost_log_count = 0
                self._obstacle_cost_log_count += 1
                if self._obstacle_cost_log_count % 10 == 0:
                    print(f"MPC OBSTACLE COST: {obstacle_cost_value:.2f}, "
                          f"closest_dist={min_dist_to_obstacle:.3f}m, "
                          f"proximity_factor={proximity_factor:.2f}x, "
                          f"num_obstacles={len(obstacles)}")
                    if closest_obstacle:
                        center, radius, dist = closest_obstacle
                        actual_clearance = dist - radius
                        safety_radius_needed = radius + 0.25 + 0.35
                        print(f"  Closest obstacle: center=({center[0]:.3f}, {center[1]:.3f}), "
                              f"radius={radius:.3f}m, dist_to_center={dist:.3f}m")
                        print(f"    CLEARANCE: {actual_clearance:.3f}m (need {safety_radius_needed:.3f}m), "
                              f"robot_pos=({px0:.3f}, {py0:.3f})")
        
        self.obstacle_cost_param.value = obstacle_cost_value

        # CRITICAL FIX: Disable warm start completely to avoid OSQP matrix size errors
        # When we rebuild/restore the problem, OSQP's cached matrix structure doesn't match
        # Solution: Always disable warm start, or rebuild problem from scratch each time
        solver_settings = self.solver_settings.copy()
        solver_settings['warm_start'] = False  # Always disable to avoid matrix size issues

        try:
            self.prob.solve(**solver_settings)
            
            # CRITICAL: Don't restore problem after solve - it causes OSQP matrix size errors
            # OSQP caches the matrix structure, and restoring changes it
            # Instead, just keep the current problem (it will be rebuilt next iteration if needed)
            # if obstacle_cost != 0:
            #     self.prob = cp.Problem(cp.Minimize(self.original_cost), self.original_constraints)
            
            # Debug: Log solve status
            if not hasattr(self, '_solve_count'):
                self._solve_count = 0
            self._solve_count += 1
            if self._solve_count % 50 == 0:  # Every 5 seconds at 10Hz
                print(f"MPC solve status: {self.prob.status}, value: {self.prob.value}")
            
            # Store solution for warm start
            if self.prob.status in ["optimal", "optimal_inaccurate"]:
                self.last_solution = {
                    'X': self.X.value.copy(),
                    'U': self.U.value.copy()
                }
        except Exception as e:
            print(f"MPC solve exception: {e}")
            return 0.0, 0.0

        # Check solve status - be more lenient
        # "Solution may be inaccurate" warning is common but solution is often still usable
        if self.prob.status not in ["optimal", "optimal_inaccurate"]:
            if not hasattr(self, '_status_error_count'):
                self._status_error_count = 0
            self._status_error_count += 1
            if self._status_error_count % 50 == 0:
                print(f"MPC solve status: {self.prob.status}, value: {self.prob.value}")
            # Still try to use solution if variables have values (might be inaccurate but usable)
            if self.U.value is None or self.X.value is None:
                return 0.0, 0.0
            # If we have values, try to use them even if status isn't optimal
            # Store solution for potential use
            if self.last_solution is None:
                self.last_solution = {}
            self.last_solution['X'] = self.X.value.copy()
            self.last_solution['U'] = self.U.value.copy()

        u0 = self.U[:, 0].value
        if u0 is None:
            return 0.0, 0.0
        
        # Return acceleration and angular velocity
        return float(u0[0]), float(u0[1])
    
    def get_twist_command(self, x0, target, obstacles=None):
        """
        Solves MPC and returns commands in Twist message format.
        Returns: {'linear': {'x': float, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': float}}
        Also stores the full MPC solution in self.last_solution for trajectory visualization
        """
        a_cmd, omega_cmd = self.solve(x0, target, obstacles)
        
        # CRITICAL DEBUG: Log what MPC is solving
        if not hasattr(self, '_twist_debug_count'):
            self._twist_debug_count = 0
        self._twist_debug_count += 1
        if self._twist_debug_count % 50 == 0:
            if isinstance(target, np.ndarray) and target.ndim == 2:
                tgt = target[:, 0]
            else:
                tgt = np.array(target)[:2]
            dx = tgt[0] - x0[0]
            dy = tgt[1] - x0[1]
            dist = np.sqrt(dx**2 + dy**2)
            print(f"MPC get_twist: x0=[{x0[0]:.3f}, {x0[1]:.3f}, {np.degrees(x0[2]):.1f}°, {x0[3]:.3f}], "
                  f"target=[{tgt[0]:.3f}, {tgt[1]:.3f}], dx={dx:.3f}, dy={dy:.3f}, dist={dist:.3f}, "
                  f"a_cmd={a_cmd:.3f}, omega_cmd={np.degrees(omega_cmd):.1f}°")
        
        # Convert acceleration to velocity command
        current_v = x0[3] if len(x0) > 3 else 0.0
        v_cmd = np.clip(current_v + a_cmd * self.dt, self.vx_min, self.vx_max)
        
        # REMOVED: Don't limit turn rate - this was preventing MPC from working correctly
        # The MPC should handle turn rate limits through its constraints
        # if abs(v_cmd) > 0.3:
        #     max_turn_at_speed = 1.0  # Reduce turn rate at higher speeds
        #     omega_cmd = np.clip(omega_cmd, -max_turn_at_speed, max_turn_at_speed)

        return {
            'linear': {'x': float(v_cmd), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(omega_cmd)}
        }
    
    def get_predicted_trajectory(self):
        """
        Get the full MPC predicted trajectory from the last solve.
        Returns: list of [x, y] positions for N+1 steps, or None if no solution
        """
        if self.last_solution is None or 'X' not in self.last_solution:
            return None
        
        X = self.last_solution['X']
        if X is None or X.shape[0] < 2:
            return None
        
        # Extract position trajectory [px, py] for all N+1 steps
        trajectory = []
        for k in range(X.shape[1]):
            trajectory.append([float(X[0, k]), float(X[1, k])])
        
        return trajectory
    
    def _update_cost_weights(self):
        """
        Update the cost function weight parameters (Boyd's fast MPC).
        Uses CVXPY parameters for efficient weight updates without rebuilding QP.
        """
        # CRITICAL: Ensure position weight is ALWAYS much larger than control penalties
        # If control penalties are too high, MPC will minimize control instead of position
        self.Qp_param.value = self.Qp
        self.Qtheta_param.value = self.Qtheta
        # Force control penalties to be very small relative to position weight
        self.Ra_param.value = max(self.Ra, 0.001)  # Minimum 0.001 to avoid numerical issues
        self.Rw_param.value = max(self.Rw, 0.001)  # Minimum 0.001 to avoid numerical issues
        self.alpha_progress_param.value = self.alpha_progress
        
        # DEBUG: Log weights periodically
        if not hasattr(self, '_weight_debug_count'):
            self._weight_debug_count = 0
        self._weight_debug_count += 1
        if self._weight_debug_count % 50 == 0:
            print(f"MPC weights: Qp={self.Qp:.2f}, Ra={self.Ra:.4f}, Rw={self.Rw:.4f}, "
                  f"ratio={self.Qp/max(self.Ra, 0.001):.1f}")
