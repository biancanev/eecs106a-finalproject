#!/usr/bin/env python3
"""
Launch file for LIDAR validation and map visualization
Step 1: Validate LIDAR obstacle detection and voxel grid generation

This launches only:
- LIDAR processor (processes raw LIDAR)
- SLAM node (creates voxel grid map)
- RViz for visualization

No control nodes - just mapping and visualization
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
        
        # Static TF publisher (map to base_scan for RViz)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_scan_tf',
            arguments=[
                '0', '0', '0',  # x, y, z translation
                '0', '0', '0', '1',  # quaternion (no rotation)
                'map',  # parent frame
                'base_scan'  # child frame
            ],
            output='screen'
        ),
        
        # Simple pose publisher (for SLAM node - uses odometry or static pose)
        Node(
            package='turtlebot_interceptor',
            executable='simple_pose_publisher',
            name='simple_pose_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_odom': True,  # Use /odom if available, else static pose
                'static_pose': False,
            }],
            output='screen'
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
        # This creates the voxel grid map from LIDAR scans
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
        
        # RViz for visualization
        # Note: User should configure RViz manually (see LIDAR_VALIDATION_GUIDE.md)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])

