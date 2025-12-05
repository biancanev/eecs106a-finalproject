#!/usr/bin/env python3
"""
Launch file for single robot navigation (no target tracking)
Simplified setup: 1 TurtleBot with LIDAR navigating to a goal point through cones

Based on lab4, lab6, and lab8 patterns from:
https://github.com/KushMahajan/EECS106a-Labs
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='1.5',
            description='Goal X position (meters)'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='1.5',
            description='Goal Y position (meters)'
        ),
        
        # LIDAR processor node (processes raw LIDAR data)
        # Based on lab4 patterns
        Node(
            package='turtlebot_interceptor',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # SLAM node (log-odds occupancy grid mapping)
        # Based on lab6/mapping patterns
        Node(
            package='turtlebot_interceptor',
            executable='slam_node',
            name='slam_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_width': 100,
                'map_height': 100,
                'resolution': 0.05,  # 5cm resolution (standard for TurtleBot)
                'origin_x': -2.5,
                'origin_y': -2.5,
                'log_odds_free': -0.6,
                'log_odds_occupied': 0.8,
                'log_odds_min': -3.0,
                'log_odds_max': 3.0,
                'occupancy_threshold': 0.25,
            }],
            output='screen'
        ),
        
        # MCL node (localization using map and LIDAR)
        # Based on lab4 patterns
        Node(
            package='turtlebot_interceptor',
            executable='mcl_node',
            name='mcl_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'num_particles': 300,
                'motion_noise_x': 0.02,
                'motion_noise_y': 0.02,
                'motion_noise_theta': 0.01,
                'max_range': 3.5,
                'min_range': 0.25,
                'resample_threshold': 0.98,
            }],
            output='screen'
        ),
        
        # MPC node (control to goal point)
        # Based on lab8 patterns - simplified for single goal point
        Node(
            package='turtlebot_interceptor',
            executable='mpc_node',
            name='mpc_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mpc_horizon': 15,
                'dt': 0.1,
                'v_max_base': 0.6,  # TurtleBot safe max speed
                'v_min': 0.0,
                'omega_max': 2.5,  # Higher for tight turns around cones
                'Kp_v': 2.0,  # Fallback control gains (lab8 pattern)
                'Kp_w': 0.8,
                'Kd_w': 0.5,
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }],
            output='screen'
        ),
    ])

