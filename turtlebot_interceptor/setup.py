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
    ],
    install_requires=['setuptools'],
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
                'animated_sim = turtlebot_interceptor.animated_sim:main',  # Standalone simulation
            ],
        },
)
