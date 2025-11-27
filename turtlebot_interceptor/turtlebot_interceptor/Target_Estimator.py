import numpy as np
from geometry_msgs.msgs import Twist

class TargetKF(Node):
    """
    KF for target with state [px, py, vx, vy]
    """

    def __init__(self, dt=0.1):
        super().__init__('kf_server')

        self._srv = self.create_service(Twist, f'KF', self.predict)
        self.create_subscription(Twist, f'turtlebot2/position???', self.udate)

        self.dt = dt
        self.x = np.zeros(4)
        self.P = np.eye(4)

        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])
        self.Q = np.eye(4) * 0.01
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        self.R = np.eye(2) * 0.05

    def predict(self, msg):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.x

    def update(self, msg):
        measurement = msg
        z = np.array(measurement)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def get_state_and_cov(self):
        return self.x.copy(), self.P.copy()
