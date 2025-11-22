#!/usr/bin/env python3

#!/usr/bin/env python3
import math
import rclpy
import tf2_ros
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped

class NoiseModel:
    def __init__(self):
        self.imu_bias = np.array([0.02, 0.01, 0.05])
        self.imu_noise_std = np.array([0.01, 0.01, 0.03])
        self.lidar_drift_rate = 0.05
        self.lidar_noise_std = 0.02
    
    def get_pose_covariance(self, distance_from_start):
        drift_factor = 1.0 + self.lidar_drift_rate * distance_from_start
        
        cov = np.diag([
            (self.imu_noise_std[0] * drift_factor)**2,
            (self.imu_noise_std[1] * drift_factor)**2,
            (self.imu_noise_std[2] * drift_factor)**2
        ])
        return cov
    
    def sample_measurement(self, true_pose, covariance):
        noise = np.random.multivariate_normal(np.zeros(3), covariance)
        noisy_pose = true_pose + self.imu_bias + noise
        return noisy_pose

def plot_trajectory_with_uncertainty(waypoints_with_cov, sampled_trajectories=None):
    fig, ax = plt.subplots(figsize=(12, 8))
    
    nominal_x = [w[0][0] for w in waypoints_with_cov]
    nominal_y = [w[0][1] for w in waypoints_with_cov]
    
    ax.plot(nominal_x, nominal_y, 'b-', linewidth=2, label='Nominal Trajectory', zorder=3)
    ax.scatter(nominal_x[0], nominal_y[0], color='green', s=150, zorder=5, label='Start')
    ax.scatter(nominal_x[-1], nominal_y[-1], color='red', s=150, zorder=5, label='Goal')
    
    for pose, cov in waypoints_with_cov:
        x, y, theta = pose
        
        eigenvalues, eigenvectors = np.linalg.eig(cov[:2, :2])
        angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
        width, height = 2 * np.sqrt(eigenvalues) * 2.0
        
        ellipse = Ellipse((x, y), width, height, angle=np.degrees(angle),
                         facecolor='cyan', edgecolor='blue', alpha=0.3, zorder=1)
        ax.add_patch(ellipse)
        
        ax.arrow(x, y, 0.05*np.cos(theta), 0.05*np.sin(theta),
                head_width=0.015, head_length=0.008, fc='blue', ec='blue', zorder=4)
    
    if sampled_trajectories is not None:
        for traj in sampled_trajectories:
            x_vals = [p[0] for p in traj]
            y_vals = [p[1] for p in traj]
            ax.plot(x_vals, y_vals, 'r-', alpha=0.2, linewidth=0.5, zorder=2)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Probabilistic Trajectory with Uncertainty Ellipses')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.axis('equal')
    plt.tight_layout()
    plt.show()

def bezier_curve(p0, p1, p2, p3, t):
    return (1 - t)**3 * p0 + 3*(1 - t)**2*t*p1 + 3*(1 - t)*t**2*p2 + t**3*p3

def generate_bezier_waypoints(x1, y1, theta1, x2, y2, theta2, 
                                           offset=1.0, num_points=10):
    d1 = np.array([np.cos(theta1), np.sin(theta1)])
    d2 = np.array([-np.cos(theta2), -np.sin(theta2)])
    c1 = np.array([x1, y1]) + offset * d1
    c2 = np.array([x2, y2]) + offset * d2
    
    t_vals = np.linspace(0, 1, num_points)
    pts = [bezier_curve(np.array([x1, y1]), c1, c2, np.array([x2, y2]), t) for t in t_vals]
    
    thetas = []
    for i in range(len(pts) - 1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        thetas.append(np.arctan2(dy, dx))
    thetas.append(thetas[-1])
    
    noise_model = NoiseModel()
    waypoints_with_cov = []
    cumulative_distance = 0.0
    
    for i in range(len(pts)):
        pose = np.array([pts[i][0], pts[i][1], thetas[i]])
        
        if i > 0:
            dx = pts[i][0] - pts[i-1][0]
            dy = pts[i][1] - pts[i-1][1]
            cumulative_distance += np.sqrt(dx**2 + dy**2)
        
        cov = noise_model.get_pose_covariance(cumulative_distance)
        waypoints_with_cov.append((pose, cov))
    
    return waypoints_with_cov

def sample_noisy_trajectories(waypoints_with_cov, num_samples=20):
    noise_model = NoiseModel()
    sampled_trajectories = []
    
    for _ in range(num_samples):
        noisy_traj = []
        for pose, cov in waypoints_with_cov:
            noisy_pose = noise_model.sample_measurement(pose, cov)
            noisy_traj.append(noisy_pose)
        sampled_trajectories.append(noisy_traj)
    
    return sampled_trajectories

def plan_probabilistic_trajectory(target_position):
    node = rclpy.create_node('probabilistic_planner')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    
    while rclpy.ok():
        try:
            trans = tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            node.get_logger().warn('TF lookup failed, retrying...')
            rclpy.spin_once(node, timeout_sec=0.1)
    
    x1 = trans.transform.translation.x
    y1 = trans.transform.translation.y
    q = trans.transform.rotation
    roll, pitch, yaw = euler.quat2euler([q.w, q.x, q.y, q.z])
    
    x2 = x1 + target_position[0]
    y2 = y1 + target_position[1]
    
    waypoints_with_cov = generate_bezier_waypoints(
        x1, y1, yaw, x2, y2, yaw, offset=0.2, num_points=15
    )
    
    sampled_trajectories = sample_noisy_trajectories(waypoints_with_cov, num_samples=30)
    plot_trajectory_with_uncertainty(waypoints_with_cov, sampled_trajectories)
    
    node.destroy_node()
    return waypoints_with_cov, sampled_trajectories

def main(args=None):
    rclpy.init(args=args)
    
    waypoints_with_cov = generate_bezier_waypoints(
        0.0, 0.0, np.pi/4,
        1.0, 1.0, np.pi/4,
        offset=0.3, num_points=20
    )
    
    sampled_trajectories = sample_noisy_trajectories(waypoints_with_cov, num_samples=50)
    plot_trajectory_with_uncertainty(waypoints_with_cov, sampled_trajectories)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()