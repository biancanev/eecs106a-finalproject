#!/usr/bin/env python3
"""
SimpleUnicycleMPC – Clean, fully DCP-compliant MPC with curved obstacle avoidance.
Obstacle model uses convex hinge penalties to create smooth avoidance behavior.
"""

import numpy as np
import cvxpy as cp

class SimpleUnicycleMPC:

    def __init__(self, horizon=15, dt=0.1, use_time_to_go=True):
        self.dt = dt
        self.N = horizon
        self.use_time_to_go = use_time_to_go

        self.nx = 4   # px, py, theta, v
        self.nu = 2   # a, omega

        # Limits
        self.v_min = 0.0
        self.v_max = 0.35
        self.a_min = -0.4
        self.a_max = 0.4
        self.w_min = -2.0
        self.w_max =  2.0

        # Robot geometry
        self.robot_radius = 0.105
        self.safety_buffer = 0.18  # slight extra clearance to avoid clipping

        # Weights
        self.Qp  = 140.0
        self.Qtheta = 10.0  # stronger heading alignment
        self.Ra = 0.01
        self.Rw = 0.01
        self.Q_obs = 900.0  # stronger avoidance while still allowing progress

        # Keep baseline weights for any future adaptation logic
        self.Qp_base = self.Qp
        self.Qtheta_base = self.Qtheta
        self.Ra_base = self.Ra
        self.Rw_base = self.Rw
        self.Q_obs_base = self.Q_obs

        # Can update without rebuilding
        self.max_obstacles = 20
        self.obs_centers = cp.Parameter((self.max_obstacles, 2))
        self.obs_radii   = cp.Parameter(self.max_obstacles)
        # Linearized obstacle normals and safety distances (per obstacle)
        self.obs_normals = cp.Parameter((self.max_obstacles, 2))
        self.safety_dists = cp.Parameter(self.max_obstacles)

        # Track last solver output for visualization
        self.last_solution = None

        # Convenience velocity bounds (used by get_twist_command)
        self.vx_min = self.v_min
        self.vx_max = self.v_max

        self._build_problem()

    # --------------------------------------------------------------------------
    # Build convex MPC once
    # --------------------------------------------------------------------------
    def _build_problem(self):
        N = self.N

        # Decision variables
        self.X = cp.Variable((self.nx, N+1))
        self.U = cp.Variable((self.nu, N))
        # Soft slacks for linearized keep-out half-spaces
        self.S = cp.Variable((self.max_obstacles, N))

        # Parameters for dynamics
        self.x0 = cp.Parameter(self.nx)
        self.A  = cp.Parameter((self.nx, self.nx))
        self.B  = cp.Parameter((self.nx, self.nu))
        self.c  = cp.Parameter(self.nx)

        # Target trajectory (2xN+1)
        self.T = cp.Parameter((2, N+1))
        # Desired heading along the horizon
        self.theta_ref = cp.Parameter(N+1)

        cost = 0
        constraints = []

        # Initial condition
        constraints += [self.X[:,0] == self.x0]

        # Main loop
        for k in range(N):

            # Dynamics
            constraints += [
                self.X[:,k+1] == self.A @ self.X[:,k] + self.B @ self.U[:,k] + self.c
            ]

            px = self.X[0,k]
            py = self.X[1,k]
            a = self.U[0,k]
            w = self.U[1,k]

            # Tracking cost
            cost += self.Qp * cp.sum_squares(self.X[0:2,k] - self.T[:,k])
            cost += self.Qtheta * cp.square(self.X[2, k] - self.theta_ref[k])

            # Control effort cost
            cost += self.Ra * cp.square(a) + self.Rw * cp.square(w)

            # Limits
            constraints += [
                self.v_min <= self.X[3,k], self.X[3,k] <= self.v_max,
                self.a_min <= a, a <= self.a_max,
                self.w_min <= w, w <= self.w_max
            ]

            # ---- Linearized obstacle avoidance with soft slack ----
            for i in range(self.max_obstacles):
                n = self.obs_normals[i, :]
                safety = self.safety_dists[i]
                cx = self.obs_centers[i, 0]
                cy = self.obs_centers[i, 1]

                # Support half-space: n·(p - (c + safety n)) >= -slack
                p_vec = cp.hstack([px, py]) - cp.hstack([cx, cy]) - safety * n
                constraints += [n @ p_vec >= -self.S[i, k]]

                # Slack is non-negative
                constraints += [self.S[i, k] >= 0]

                # Penalize slack usage
                cost += self.Q_obs * cp.square(self.S[i, k])

        # Terminal cost: position + heading alignment
        cost += 10 * self.Qp * cp.sum_squares(self.X[0:2,N] - self.T[:,N])
        cost += 5 * self.Qtheta * cp.square(self.X[2, N] - self.theta_ref[N])
        # Build problem
        self.prob = cp.Problem(cp.Minimize(cost), constraints)

    # --------------------------------------------------------------------------
    # Dynamics linearization
    # --------------------------------------------------------------------------
    def linearize(self, x0):
        px, py, th, v = x0
        dt = self.dt

        A = np.eye(4)
        A[0,2] = -dt * v * np.sin(th)
        A[0,3] =  dt * np.cos(th)
        A[1,2] =  dt * v * np.cos(th)
        A[1,3] =  dt * np.sin(th)

        B = np.zeros((4,2))
        B[2,1] = dt
        B[3,0] = dt

        f = np.array([v*np.cos(th), v*np.sin(th), 0.0, 0.0])
        c = x0 + dt*f - A@x0

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
            self.Qp = self.Qp_base  # encoura
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
        
        # Maintain strong obstacle avoidance always
        self.Q_obs = self.Q_obs_base
    
    # --------------------------------------------------------------------------
    # Solve MPC
    # --------------------------------------------------------------------------
    def solve(self, x0, target, obstacles):

        # Format target
        if target.ndim == 1:
            T = np.tile(target.reshape(2,1), (1, self.N+1))
        else:
            T = target

        # Linearize dynamics
        A, B, c = self.linearize(x0)

        # Assign parameters
        self.x0.value = x0
        self.A.value = A
        self.B.value = B
        self.c.value = c
        self.T.value = T
        # Heading reference: point toward final target position (wrapped)
        goal_dx = T[0, -1] - x0[0]
        goal_dy = T[1, -1] - x0[1]
        desired_theta = np.arctan2(goal_dy, goal_dx)
        # Wrap desired heading to be close to current heading to avoid 2π flips
        angle_err = np.arctan2(np.sin(desired_theta - x0[2]), np.cos(desired_theta - x0[2]))
        desired_theta_wrapped = x0[2] + angle_err
        theta_vec = np.full(self.N + 1, desired_theta_wrapped)
        self.theta_ref.value = theta_vec

        # Adapt weights based on time-to-go to encourage minimum-time behavior
        t_go = self.compute_time_to_go(x0, T[:, 0])
        self.adapt_weights_for_time_to_go(t_go)

        # Obstacle parameters
        centers = np.zeros((self.max_obstacles,2))
        radii   = np.zeros(self.max_obstacles)
        normals = np.zeros((self.max_obstacles, 2))
        safety_d = np.zeros(self.max_obstacles)

        robot_xy = x0[:2]
        for i,(center,r) in enumerate(obstacles[:self.max_obstacles]):
            centers[i] = center
            radii[i] = r

            # Linearize keep-out boundary around current robot position
            vec = robot_xy - center
            norm = np.linalg.norm(vec) + 1e-6
            normals[i] = vec / norm
            safety_d[i] = r + self.robot_radius + self.safety_buffer

        self.obs_centers.value = centers
        self.obs_radii.value   = radii
        self.obs_normals.value = normals
        self.safety_dists.value = safety_d

        # Solve
        # Solve with OSQP QP solver (fast/stable)
        self.prob.solve(solver=cp.OSQP, warm_start=False, ignore_dpp=True)
        status = self.prob.status
        if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
            raise RuntimeError(f"MPC solve status: {status}")

        # Cache solution for visualization/debugging
        self.last_solution = {
            'X': self.X.value,
            'U': self.U.value,
            'S': self.S.value,
            'status': self.prob.status
        }

        # Controls
        if self.U.value is None:
            raise RuntimeError("MPC solve returned no control")
        return float(self.U[0,0].value), float(self.U[1,0].value)
    
    def get_twist_command(self, x0, target, obstacles=None):
        """
        Solve MPC and return Twist message format
        Returns: {'linear': {'x': vx, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': omega}}
        """
        if obstacles is None:
            obstacles = []

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

        # Slow down as we approach the goal to avoid overshoot/looping
        if dist_to_goal < 0.4:
            v_cmd = min(v_cmd, max(0.05, dist_to_goal * 0.8))

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
