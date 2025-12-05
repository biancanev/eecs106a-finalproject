#!/usr/bin/env python3
"""
Launch file for hardware mode
Runs all nodes with real sensors
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
        
        # LIDAR processor node (processes raw LIDAR data)
        Node(
            package='turtlebot_interceptor',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # SLAM node (log-odds occupancy grid mapping)
        Node(
            package='turtlebot_interceptor',
            executable='slam_node',
            name='slam_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_width': 100,
                'map_height': 100,
                'resolution': 0.05,
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
        
        # Target measurement node (artificial + terminal homing)
        Node(
            package='turtlebot_interceptor',
            executable='target_measurement_node',
            name='target_measurement_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'artificial_rate': 10.0,  # Hz - rate for artificial measurements
                'lidar_detection_range': 2.0,  # m - switch to terminal homing
                'target_radius': 0.15,  # m - target physical radius
                'use_external_source': True,  # Use external target if available
                'external_source_topic': '/target_estimate',  # Optional external source
            }],
            output='screen'
        ),
        
        # UKF node for target tracking
        Node(
            package='turtlebot_interceptor',
            executable='ukf_node',
            name='ukf_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'dt': 0.1,
                'process_noise': 0.02,
                'measurement_noise': 0.03,
            }],
            output='screen'
        ),
        
        # MPC node (control)
        Node(
            package='turtlebot_interceptor',
            executable='mpc_node',
            name='mpc_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mpc_horizon': 15,
                'dt': 0.1,
                'v_max_base': 0.6,
                'v_min': 0.0,
                'omega_max': 1.5,
                'Kp_v': 2.0,  # Fallback control gains (lab8 pattern)
                'Kp_w': 0.8,
                'Kd_w': 0.5,
            }],
            output='screen'
        ),
    ])

