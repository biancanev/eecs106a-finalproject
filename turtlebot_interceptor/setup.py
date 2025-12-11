from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_interceptor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'numpy<2.0',  # cv_bridge requires numpy 1.x
        'opencv-python',  # For cv2
    ],
    zip_safe=True,
    maintainer='biancano',
    maintainer_email='biancano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'mpc_node = turtlebot_interceptor.mpc_node:main',
                'mcl_node = turtlebot_interceptor.mcl_node:main',
                'ukf_node = turtlebot_interceptor.ukf_node:main',
                'target_kf_node = turtlebot_interceptor.target_kf_node:main',  # Legacy - uses UKF
                'target_measurement_node = turtlebot_interceptor.target_measurement_node:main',  # Artificial + terminal homing
                'slam_node = turtlebot_interceptor.slam_node:main',
                'lidar_processor_node = turtlebot_interceptor.lidar_processor_node:main',
                'simple_pose_publisher = turtlebot_interceptor.simple_pose_publisher:main',  # For LIDAR validation
                'ekf_pose_estimator = turtlebot_interceptor.ekf_pose_estimator:main',  # EKF sensor fusion (IMU + Mag + Encoders)
                'cartographer_pose_bridge = turtlebot_interceptor.cartographer_pose_bridge:main',  # Bridge Cartographer pose to /amcl_pose
                'fast_local_grid = turtlebot_interceptor.fast_local_grid:main',  # Fast local occupancy grid (5-10 Hz)
                'fast_local_grid_target = turtlebot_interceptor.fast_local_grid_target:main',  # Fast local occupancy grid (5-10 Hz)
                'navigation_visualizer = turtlebot_interceptor.navigation_visualizer_node:main',  # RViz visualization
                'animated_sim = turtlebot_interceptor.animated_sim:main',  # Standalone simulation
                'target_est_node = turtlebot_interceptor.target_est_node:main',
                'camera_cone_detector = turtlebot_interceptor.camera_cone_detector:main',  # Camera-based yellow cone detection
                'test_camera_cones = turtlebot_interceptor.test_camera_cones:main',  # Test camera cone detection
        ],
    },
)
