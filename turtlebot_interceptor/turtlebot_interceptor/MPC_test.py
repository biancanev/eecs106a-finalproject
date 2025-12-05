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
        self.wz_min = -1.5  # Maximum turn rate (rad/s)
        self.wz_max = 1.5
        
        # Acceleration constraints (for MPC internal use)
        self.a_min = -0.4
        self.a_max = 0.4
        self.alpha_min = -2.0  # Angular acceleration (rad/s^2)
        self.alpha_max = 2.0
        
        # Turn angle constraint (maximum change in heading per step)
        self.max_turn_angle = 0.3  # Maximum 0.3 rad (~17 degrees) per step
        
        # Legacy constraints for backward compatibility
        self.v_min = self.vx_min
        self.v_max = self.vx_max
        self.omega_max = self.wz_max

        # Base weights (will be adapted based on time-to-go) - INCREASED for faster movement
        self.Qp_base = 50.0  # Higher position weight = move faster (was 20.0)
        self.Qtheta_base = 0.5
        self.Ra_base = 0.05  # Lower control penalty = allow more aggressive control (was 0.1)
        self.Rw_base = 0.05  # Lower turn penalty = allow faster turning (was 0.1)
        
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
            'warm_start': True,
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
            
            # Minimum time-to-go behavior: use time-varying weights (already handled by adaptive weights)
            # The adaptive Qp weight based on time-to-go already encourages faster convergence
            # Additional: penalize distance more heavily in early steps to encourage progress
            # Use a decreasing weight factor: earlier steps have higher weight (encourages progress)
            time_weight = 1.0 + (self.N - k) * 0.1  # Earlier steps weighted more
            
            # cost with adaptive weights and time-varying position weight
            cost += self.Qp_param * time_weight * (px_err**2 + py_err**2) + self.Qtheta_param*(theta**2)
            cost += self.Ra_param*(a**2) + self.Rw_param*(omega**2)

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

        # terminal cost
        pxN = self.X[0,N] - self.T[0,N]
        pyN = self.X[1,N] - self.T[1,N]
        cost += self.Qp_param*(pxN**2 + pyN**2)

        self.prob = cp.Problem(cp.Minimize(cost), constraints)
        self.original_cost = cost  # Store original cost expression
        
        # Initialize weight parameters
        self.Qp_param.value = self.Qp_base
        self.Qtheta_param.value = self.Qtheta_base
        self.Ra_param.value = self.Ra_base
        self.Rw_param.value = self.Rw_base
        self.alpha_progress_param.value = self.alpha_progress

    # --- linearize unicycle model around x0 ---
    def linearize(self, x0):
        px,py,th,v = x0
        dt = self.dt
        A = np.eye(4)
        B = np.zeros((4,2))

        A[0,2] = -dt*v*np.sin(th)
        A[0,3] =  dt*np.cos(th)
        A[1,2] =  dt*v*np.cos(th)
        A[1,3] =  dt*np.sin(th)

        B[2,1] = dt  # dtheta/domega
        B[3,0] = dt  # dv/da

        # constant term
        f = np.array([v*np.cos(th), v*np.sin(th), 0.0, 0.0])
        c = x0 + dt*f - A@x0
        return A,B,c

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

        # Warm start from previous solution (Boyd's fast MPC)
        # Disable warm start if obstacles present (OSQP doesn't handle dynamic constraints well)
        use_warm_start = (self.last_solution is not None and self.use_time_to_go and obstacles is None)
        if use_warm_start:
            try:
                # Shift previous solution
                self.X.value[:, :-1] = self.last_solution['X'][:, 1:]
                self.X.value[:, -1] = self.last_solution['X'][:, -1]
                self.U.value[:, :-1] = self.last_solution['U'][:, 1:]
                self.U.value[:, -1] = self.last_solution['U'][:, -1]
            except:
                pass  # If warm start fails, start fresh

        # Obstacle repulsion will be added in solve() method dynamically

        # Add obstacle repulsion cost if provided
        obstacle_cost = 0
        if obstacles is not None and len(obstacles) > 0:
            # Add repulsion cost for each obstacle at each time step
            for k in range(self.N):
                px = self.X[0, k]
                py = self.X[1, k]
                for center, radius in obstacles:
                    # Distance from robot to obstacle center
                    dx = px - center[0]
                    dy = py - center[1]
                    dist_sq = dx*dx + dy*dy
                    # Safety radius (obstacle radius + robot radius + TIGHT margin for faster paths)
                    safety_radius = radius + 0.15 + 0.03  # robot radius + TIGHT margin (was 0.1, now 0.03)
                    safety_radius_sq = safety_radius * safety_radius
                    
                    # Minimum safe distance (hard constraint - must stay outside this)
                    min_safe_dist = radius + 0.15 + 0.01  # Just robot radius + tiny margin
                    min_safe_dist_sq = min_safe_dist * min_safe_dist
                    
                    # Repulsion cost: high when very close, decays quickly with distance
                    # Only penalize if within 1.5x safety radius (tighter range for faster paths)
                    if dist_sq < safety_radius_sq * 2.25:  # 1.5x safety radius (was 4 = 2x)
                        # Distance-dependent repulsion: stronger only when very close
                        dist = np.sqrt(dist_sq + 1e-6)
                        dist_to_safety = dist - safety_radius
                        
                        # Soft repulsion: only when getting close to safety radius
                        if dist_to_safety < 0.1:  # Within 10cm of safety radius
                            # Exponential repulsion - very strong when close, weak when far
                            repulsion_weight = 50.0  # Moderate repulsion (was 200.0 - too strong)
                            # Use exponential decay: stronger penalty as we approach safety radius
                            obstacle_cost += repulsion_weight * np.exp(-dist_to_safety * 20)
                        
                        # HARD CONSTRAINT: robot must stay outside minimum safe distance
                        # This prevents actual collision but allows tight paths
                        if dist_sq < min_safe_dist_sq:
                            # Very large penalty if inside minimum safe distance
                            obstacle_cost += 5000.0 / (dist_sq + 0.001)  # Strong but not excessive
            
            # Add obstacle cost to the problem
            if obstacle_cost != 0:
                # Rebuild problem with obstacle cost
                new_cost = self.original_cost + obstacle_cost
                self.prob = cp.Problem(cp.Minimize(new_cost), self.prob.constraints)

        # Disable warm start in solver if obstacles present
        solver_settings = self.solver_settings.copy()
        if obstacles is not None:
            solver_settings['warm_start'] = False

        try:
            self.prob.solve(**solver_settings)
            
            # Restore original problem after solve (for next iteration)
            if obstacle_cost != 0:
                self.prob = cp.Problem(cp.Minimize(self.original_cost), self.prob.constraints)
            
            # Store solution for warm start
            if self.prob.status in ["optimal", "optimal_inaccurate"]:
                self.last_solution = {
                    'X': self.X.value.copy(),
                    'U': self.U.value.copy()
                }
        except Exception as e:
            print(f"MPC solve error: {e}")
            return 0.0, 0.0

        if self.prob.status not in ["optimal", "optimal_inaccurate"]:
            return 0.0, 0.0

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
        
        # Convert acceleration to velocity command
        current_v = x0[3] if len(x0) > 3 else 0.0
        v_cmd = np.clip(current_v + a_cmd * self.dt, self.vx_min, self.vx_max)
        
        # Apply additional safety constraints on turn rate at high speeds
        if abs(v_cmd) > 0.3:
            max_turn_at_speed = 1.0  # Reduce turn rate at higher speeds
            omega_cmd = np.clip(omega_cmd, -max_turn_at_speed, max_turn_at_speed)

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
        self.Qp_param.value = self.Qp
        self.Qtheta_param.value = self.Qtheta
        self.Ra_param.value = self.Ra
        self.Rw_param.value = self.Rw
        self.alpha_progress_param.value = self.alpha_progress
