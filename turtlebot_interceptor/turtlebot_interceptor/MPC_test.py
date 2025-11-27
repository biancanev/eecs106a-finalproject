import numpy as np
import cvxpy as cp

class SimpleUnicycleMPC(Node):

    def __init__(self, horizon=15, dt=0.1):
        super().__init__('mpc_server')
        self.dt = dt
        self.N = horizon

        self.nx = 4     # px,py,theta,v
        self.nu = 2     # a,omega

        # constraints
        self.v_min = 0.0
        self.v_max = 0.6
        self.a_min = -0.4
        self.a_max =  0.4
        self.omega_max = 1.5

        # weights
        self.Qp = 20.0
        self.Qtheta = 0.5
        self.Ra = 0.1
        self.Rw = 0.1

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

            # cost
            cost += self.Qp*(px_err**2 + py_err**2) + self.Qtheta*(theta**2)
            cost += self.Ra*(a**2) + self.Rw*(omega**2)

            # constraints
            constraints += [
                self.v_min <= self.X[3,k],
                self.X[3,k] <= self.v_max,
                self.a_min <= a,
                a <= self.a_max,
                cp.abs(omega) <= self.omega_max
            ]

        # terminal cost
        pxN = self.X[0,N] - self.T[0,N]
        pyN = self.X[1,N] - self.T[1,N]
        cost += self.Qp*(pxN**2 + pyN**2)

        self.prob = cp.Problem(cp.Minimize(cost), constraints)

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

    def solve(self, x0, target):
        """
        x0: [px,py,theta,v]
        target: [px_tgt, py_tgt]  or full trajectory shape (2, N+1)
        """
        if len(target.shape)==1:
            T = np.tile(target.reshape(2,1),(1,self.N+1))
        else:
            T = target

        A,B,c = self.linearize(x0)
        self.x0.value = x0
        self.A.value  = A
        self.B.value  = B
        self.c.value  = c
        self.T.value  = T

        self.prob.solve(solver=cp.OSQP, warm_start=True)

        if self.prob.status not in ["optimal", "optimal_inaccurate"]:
            return 0.0, 0.0

        u0 = self.U[:,0].value
        return float(u0[0]), float(u0[1])
