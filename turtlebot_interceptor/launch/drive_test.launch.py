#!/usr/bin/env python3
"""
Drive test launch: runs EKF, visualizer, and drive_tester to compare /amcl_pose vs /odom_ekf
without target interception or MPC. Useful for estimator debugging.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='1.5',
            description='Goal X position (for visualization)'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
            description='Goal Y position (for visualization)'
        ),

        # EKF pose estimator (IMU + mag + encoders)
        Node(
            package='turtlebot_interceptor',
            executable='ekf_pose_estimator',
            name='ekf_pose_estimator',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'
        ),

        # Drive tester publishes cmd_vel patterns and logs EKF vs AMCL error
        Node(
            package='turtlebot_interceptor',
            executable='drive_tester',
            name='drive_tester',
            output='screen'
        ),

        # Navigation visualizer (shows poses, path, covariances)
        Node(
            package='turtlebot_interceptor',
            executable='navigation_visualizer',
            name='navigation_visualizer',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'robot_radius': 0.15,
                'path_history_length': 200,
            }],
            output='screen'
        ),

        # RViz for visualization (optional; adjust config in RViz)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
