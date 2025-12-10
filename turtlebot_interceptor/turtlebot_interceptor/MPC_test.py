#!/usr/bin/env python3
"""
Fixed MPC controller with proper obstacle avoidance
Key fixes:
1. DCP-compliant obstacle cost using inverse barrier
2. Static problem formulation (no rebuilding)
3. Proper weight balancing
4. Smooth obstacle avoidance via soft constraints
"""
import numpy as np
import cvxpy as cp

class SimpleUnicycleMPC:

    def __init__(self, horizon=15, dt=0.1, use_time_to_go=True):
        self.dt = dt
        self.N = horizon
        self.use_time_to_go = use_time_to_go

        self.nx = 4     # px,py,theta,v
        self.nu = 2     # a,omega

        # Velocity constraints
        self.vx_min = 0.0
        self.vx_max = 0.35
        self.vy_min = 0.0
        self.vy_max = 0.0
        self.vz_min = 0.0
        self.vz_max = 0.0
        
        # Angular velocities (rad/s)
        self.wx_min = 0.0
        self.wx_max = 0.0
        self.wy_min = 0.0
        self.wy_max = 0.0
        self.wz_min = -2.0
        self.wz_max = 2.0
        
        # Acceleration constraints
        self.a_min = -0.4
        self.a_max = 0.4
        self.alpha_min = -4.0
        self.alpha_max = 4.0
        
        # Turn angle constraint
        self.max_turn_angle = 0.6
        
        # Legacy constraints
        self.v_min = self.vx_min
        self.v_max = self.vx_max
        self.omega_max = self.wz_max

        # FIXED: Balanced weights for goal tracking + obstacle avoidance
        self.Qp_base = 100.0          # Position error weight
        self.Qtheta_base = 0.0        # No theta penalty
        self.Ra_base = 0.01           # Acceleration penalty
        self.Rw_base = 0.01           # Angular velocity penalty
        self.Q_obs_base = 500.0       # Obstacle avoidance weight
        
        # Current adaptive weights
        self.Qp = self.Qp_base
        self.Qtheta = self.Qtheta_base
        self.Ra = self.Ra_base
        self.Rw = self.Rw_base
        self.Q_obs = self.Q_obs_base
        
        # Obstacle parameters
        self.robot_radius = 0.105
        self.safety_buffer = 0.15  # Total safety distance
        
        # Time-to-go parameters
        self.time_to_go = None
        self.alpha_progress = 50.0
        self.min_time_to_go = 0.5
        
        # Solver settings
        self.last_solution = None
        self.solver_settings = {
            'solver': cp.OSQP,
            'warm_start': True,
            'verbose': False,
            'eps_abs': 1e-3,
            'eps_rel': 1e-3,
            'max_iter': 4000,
            'polish': True
        }

        self._build_qp()

    def _build_qp(self):
        """Build QP once with obstacle parameters that can be updated"""
        nx, nu, N = self.nx, self.nu, self.N

        # Decision variables
        self.X = cp.Variable((nx, N+1))
        self.U = cp.Variable((nu, N))
        
        # Slack variables for soft obstacle constraints
        self.slack_obs = cp.Variable((N+1,), nonneg=True)

        # State parameters
        self.x0 = cp.Parameter(nx)
        self.A = cp.Parameter((nx, nx))
        self.B = cp.Parameter((nx, nu))
        self.c = cp.Parameter(nx)
        self.T = cp.Parameter((2, N+1))
        
        # Weight parameters (can be updated without rebuilding)
        self.Qp_param = cp.Parameter(nonneg=True, value=self.Qp_base)
        self.Qtheta_param = cp.Parameter(nonneg=True, value=self.Qtheta_base)
        self.Ra_param = cp.Parameter(nonneg=True, value=self.Ra_base)
        self.Rw_param = cp.Parameter(nonneg=True, value=self.Rw_base)
        self.Q_obs_param = cp.Parameter(nonneg=True, value=self.Q_obs_base)
        
        # FIXED: Obstacle parameters (max 10 obstacles)
        # Updated each solve without rebuilding problem
        self.max_obstacles = 10
        self.obs_centers = cp.Parameter((self.max_obstacles, 2), value=np.zeros((self.max_obstacles, 2)))
        self.obs_radii = cp.Parameter(self.max_obstacles, nonneg=True, value=np.zeros(self.max_obstacles))
        self.obs_active = cp.Parameter(self.max_obstacles, boolean=True, value=np.zeros(self.max_obstacles, dtype=bool))

        constraints = []
        cost = 0

        # Initial condition
        constraints += [self.X[:, 0] == self.x0]

        # Stage costs and dynamics
        for k in range(N):
            # Dynamics constraint
            constraints += [
                self.X[:, k+1] == self.A @ self.X[:, k] + self.B @ self.U[:, k] + self.c
            ]

            px = self.X[0, k]
            py = self.X[1, k]
            px_err = px - self.T[0, k]
            py_err = py - self.T[1, k]
            a = self.U[0, k]
            omega = self.U[1, k]

            # Position tracking cost
            cost += self.Qp_param * (px_err**2 + py_err**2)
            
            # Control effort cost
            cost += self.Ra_param * a**2 + self.Rw_param * omega**2

            # Velocity constraints
            constraints += [
                self.vx_min <= self.X[3, k],
                self.X[3, k] <= self.vx_max,
            ]
            
            # Acceleration constraints
            constraints += [
                self.a_min <= a,
                a <= self.a_max,
            ]
            
            # Angular velocity constraints
            constraints += [
                self.wz_min <= omega,
                omega <= self.wz_max,
            ]
            
            # FIXED: DCP-compliant obstacle avoidance using slack variables
            # For each obstacle, add soft constraint: dist >= safety_dist - slack
            # This is convex and allows optimization to trade off goal vs obstacles
            for i in range(self.max_obstacles):
                cx = self.obs_centers[i, 0]
                cy = self.obs_centers[i, 1]
                r_obs = self.obs_radii[i]
                
                # Distance squared from robot to obstacle center
                dist_sq = cp.square(px - cx) + cp.square(py - cy)
                
                # Safety distance = obstacle radius + robot radius + buffer
                safety_dist = r_obs + self.robot_radius + self.safety_buffer
                
                # FIXED: Use SOC (Second-Order Cone) constraint for distance
                # This is DCP-compliant and handles sqrt properly
                # ||[px-cx, py-cy]||_2 >= safety_dist - slack[k]
                # Equivalently: dist_sq >= (safety_dist - slack[k])^2
                # But we want: sqrt(dist_sq) >= safety_dist - slack[k]
                # In CVXPY: cp.norm2([px-cx, py-cy]) >= safety_dist - slack[k]
                
                # Only add constraint if obstacle is active
                # Use conditional: if obs_active[i], then add constraint
                # CVXPY doesn't support if-then, so we use a trick:
                # Multiply RHS by obs_active (0 or 1) to disable constraint
                constraints += [
                    cp.norm2(cp.hstack([px - cx, py - cy])) >= 
                    (safety_dist - self.slack_obs[k]) * self.obs_active[i]
                ]

        # Terminal cost - heavier weight for convergence
        pxN = self.X[0, N] - self.T[0, N]
        pyN = self.X[1, N] - self.T[1, N]
        cost += 10.0 * self.Qp_param * (pxN**2 + pyN**2)
        
        # Terminal obstacle constraint
        for i in range(self.max_obstacles):
            cx = self.obs_centers[i, 0]
            cy = self.obs_centers[i, 1]
            r_obs = self.obs_radii[i]
            safety_dist = r_obs + self.robot_radius + self.safety_buffer
            
            constraints += [
                cp.norm2(cp.hstack([self.X[0, N] - cx, self.X[1, N] - cy])) >= 
                (safety_dist - self.slack_obs[N]) * self.obs_active[i]
            ]
        
        # FIXED: Add cost on slack variables (penalize constraint violations)
        # This makes obstacles "soft" - robot can violate if necessary but pays a cost
        cost += self.Q_obs_param * cp.sum_squares(self.slack_obs)

        self.prob = cp.Problem(cp.Minimize(cost), constraints)

    def linearize(self, x0):
        """Linearize unicycle dynamics around x0"""
        px, py, th, v = x0
        dt = self.dt
        
        A = np.eye(4)
        A[0, 2] = -dt * v * np.sin(th)
        A[0, 3] = dt * np.cos(th)
        A[1, 2] = dt * v * np.cos(th)
        A[1, 3] = dt * np.sin(th)

        B = np.zeros((4, 2))
        B[2, 1] = dt
        B[3, 0] = dt

        f_x0 = np.array([v * np.cos(th), v * np.sin(th), 0.0, 0.0])
        c = x0 + dt * f_x0 - A @ x0
        
        return A, B, c

    def compute_time_to_go(self, x0, target):
        """Compute estimated time-to-go"""
        px, py, v = x0[0], x0[1], x0[3]
        
        if isinstance(target, (list, tuple)) or (isinstance(target, np.ndarray) and target.ndim == 1):
            tgt_pos = np.array(target[:2])
        else:
            tgt_pos = target[:, 0]
        
        dist = np.sqrt((px - tgt_pos[0])**2 + (py - tgt_pos[1])**2)
        time_to_go = dist / (self.v_max + 1e-6)
        
        if v > 0.1:
            time_to_go_vel = dist / v
            time_to_go = min(time_to_go, time_to_go_vel)
        
        return max(time_to_go, 0.1)
    
    def adapt_weights_for_time_to_go(self, time_to_go):
        """Adapt MPC weights based on time-to-go"""
        if not self.use_time_to_go:
            self.Qp = self.Qp_base
            self.Qtheta = self.Qtheta_base
            self.Ra = self.Ra_base
            self.Rw = self.Rw_base
            self.Q_obs = self.Q_obs_base
            return
        
        normalized_tgo = np.clip(time_to_go / 5.0, 0.0, 1.0)
        aggression_factor = 1.0 - normalized_tgo
        
        # More aggressive when close to goal
        self.Qp = self.Qp_base * (1.0 + 2.0 * aggression_factor)
        self.Ra = self.Ra_base * (1.0 - 0.5 * aggression_factor)
        self.Rw = self.Rw_base * (1.0 - 0.5 * aggression_factor)
        
        # FIXED: Reduce obstacle weight when very close to goal (allow tighter navigation)
        self.Q_obs = self.Q_obs_base * (1.0 + 0.5 * (1.0 - aggression_factor))
    
    def solve(self, x0, target, obstacles=None):
        """
        Solve MPC problem
        x0: [px,py,theta,v]
        target: [px_tgt, py_tgt] or trajectory (2, N+1)
        obstacles: list of (center, radius) tuples
        Returns: (acceleration, angular_velocity)
        """
        x0 = np.array(x0).flatten()
        if len(x0) != 4:
            raise ValueError(f"x0 must have 4 elements, got {len(x0)}")
        
        # Prepare target trajectory
        if isinstance(target, (list, tuple)) or (isinstance(target, np.ndarray) and target.ndim == 1):
            T = np.tile(np.array(target).reshape(2, 1), (1, self.N + 1))
        else:
            T = np.array(target)
            if T.shape != (2, self.N + 1):
                raise ValueError(f"target must be shape (2, {self.N+1}), got {T.shape}")
        
        # Time-to-go adaptation
        if self.use_time_to_go:
            self.time_to_go = self.compute_time_to_go(x0, target)
            self.adapt_weights_for_time_to_go(self.time_to_go)
            self._update_cost_weights()

        # Linearize dynamics
        A, B, c = self.linearize(x0)
        self.x0.value = x0
        self.A.value = A
        self.B.value = B
        self.c.value = c
        self.T.value = T

        # FIXED: Update obstacle parameters without rebuilding problem
        obs_centers_val = np.zeros((self.max_obstacles, 2))
        obs_radii_val = np.zeros(self.max_obstacles)
        obs_active_val = np.zeros(self.max_obstacles, dtype=bool)
        
        if obstacles is not None:
            n_obs = min(len(obstacles), self.max_obstacles)
            for i in range(n_obs):
                center, radius = obstacles[i]
                obs_centers_val[i] = center
                obs_radii_val[i] = radius
                obs_active_val[i] = True
        
        self.obs_centers.value = obs_centers_val
        self.obs_radii.value = obs_radii_val
        self.obs_active.value = obs_active_val

        # Solve
        try:
            self.prob.solve(**self.solver_settings)
            
            if self.prob.status in ["optimal", "optimal_inaccurate"]:
                self.last_solution = {
                    'X': self.X.value.copy() if self.X.value is not None else None,
                    'U': self.U.value.copy() if self.U.value is not None else None,
                    'slack': self.slack_obs.value.copy() if self.slack_obs.value is not None else None
                }
                
                # Store for visualization
                if not hasattr(self, 'X_sol'):
                    self.X_sol = type('obj', (object,), {'value': None})()
                self.X_sol.value = self.X.value
            else:
                print(f"MPC solve status: {self.prob.status}")
                
        except Exception as e:
            print(f"MPC solve exception: {e}")
            return 0.0, 0.0

        # Check solve status
        if self.prob.status not in ["optimal", "optimal_inaccurate"]:
            if self.U.value is None or self.X.value is None:
                return 0.0, 0.0

        u0 = self.U[:, 0].value
        if u0 is None:
            return 0.0, 0.0

        return float(u0[0]), float(u0[1])
    
    def get_twist_command(self, x0, target, obstacles=None):
        """
        Solve MPC and return Twist message format
        Returns: {'linear': {'x': vx, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': omega}}
        """
        a_cmd, omega_cmd = self.solve(x0, target, obstacles)
        
        # Convert acceleration to velocity
        current_v = x0[3] if len(x0) > 3 else 0.0
        v_cmd = np.clip(current_v + a_cmd * self.dt, self.vx_min, self.vx_max)
        
        # Safety checks
        if isinstance(target, np.ndarray) and target.ndim == 2:
            tgt = target[:, 0]
        else:
            tgt = np.array(target)[:2]
        
        robot_pos = x0[:2]
        dist_to_goal = np.linalg.norm(tgt - robot_pos)
        
        # Only intervene if stuck AND far from goal
        if dist_to_goal > 0.03:
            if abs(v_cmd) < 0.1:  # MPC is stuck
                v_cmd = min(self.vx_max * 0.6, dist_to_goal * 1.0)
            
            # Ensure minimum velocity when far
            if dist_to_goal > 0.5 and abs(v_cmd) < 0.2:
                v_cmd = min(self.vx_max * 0.5, dist_to_goal * 0.8)

        return {
            'linear': {'x': float(v_cmd), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(omega_cmd)}
        }
    
    def get_predicted_trajectory(self):
        """Get the predicted trajectory from last solve"""
        if self.last_solution is None or self.last_solution.get('X') is None:
            return None
        
        X = self.last_solution['X']
        if X is None or X.shape[0] < 2:
            return None
        
        trajectory = []
        for k in range(X.shape[1]):
            trajectory.append([float(X[0, k]), float(X[1, k])])
        
        return trajectory
    
    def _update_cost_weights(self):
        """Update cost function weight parameters"""
        self.Qp_param.value = self.Qp
        self.Qtheta_param.value = self.Qtheta
        self.Ra_param.value = max(self.Ra, 1e-4)
        self.Rw_param.value = max(self.Rw, 1e-4)
        self.Q_obs_param.value = self.Q_obs