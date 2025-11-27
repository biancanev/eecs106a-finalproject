import numpy as np
import rclpy
from nav_msgs.msgs import OccupancyGrid

class MCL(Node):
    """
    Minimal Monte Carlo Localization implementation.
    State = [x, y, theta]
    """

    def __init__(self, N=300, motion_noise=[0.02, 0.02, 0.01]):
        super().__init__('mcl_server')

        self.N = N
        self.motion_noise = np.array(motion_noise)
        self.particles = None
        self.weights = None
        self.likelihood_fn = None

        self._srv = self.create_service(Twist, f'map', self.mcl_callback) #TODO
        self.create_subscription(OccupancyGrid, 'lidar???', self.update)

    def initialize_uniform(self, x_range, y_range, theta_range):
        self.particles = np.zeros((self.N, 3))
        self.particles[:, 0] = np.random.uniform(*x_range, self.N)
        self.particles[:, 1] = np.random.uniform(*y_range, self.N)
        self.particles[:, 2] = np.random.uniform(*theta_range, self.N)
        self.weights = np.ones(self.N) / self.N

    def predict(self, u, dt):
        """
        u = [v, omega]
        """
        v, w = u
        noise = np.random.randn(self.N, 3) * self.motion_noise

        self.particles[:, 0] += dt * v * np.cos(self.particles[:, 2]) + noise[:,0]
        self.particles[:, 1] += dt * v * np.sin(self.particles[:, 2]) + noise[:,1]
        self.particles[:, 2] += dt * w + noise[:,2]

    def update(self, msg):
        """
        msg: OccupancyGrid raw lidar array
        """
        for i in range(self.N):
            self.weights[i] = self.likelihood_fn(self.particles[i], lidar_scan)

        self.weights += 1e-12
        self.weights /= np.sum(self.weights)
        self.resample()

    def resample(self):
        idx = np.random.choice(self.N, self.N, p=self.weights)
        self.particles = self.particles[idx]
        self.weights = np.ones(self.N)/self.N

    def mean_and_covariance(self):
        mean = np.average(self.particles, axis=0, weights=self.weights)
        cov = np.cov(self.particles.T, aweights=self.weights)
        return mean, cov
