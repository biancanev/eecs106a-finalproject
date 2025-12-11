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
            'max_obstacles',
            default_value='5',
            description='Maximum number of obstacles to track'
        ),
        DeclareLaunchArgument(
            'obstacle_radius',
            default_value='0.20',
            description='Obstacle radius in meters (smaller = less conservative)'
        ),
        
        # Static TF publisher (map to base_scan for RViz)
        # CRITICAL: This allows RViz to display the occupancy grid map
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

        # Simple pose publisher (republishes /odom as /amcl_pose)
        # Uses raw wheel odometry - no sensor fusion for now
        Node(
            package='turtlebot_interceptor',
            executable='simple_pose_publisher',
            name='simple_pose_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_odom': True,
                'static_pose': False,
            }],
            output='screen'
        ),
        
        # ============================================================
        # IMPORTANT: Launch Cartographer SEPARATELY before this!
        # ============================================================
        # Run this command FIRST (in a separate terminal):
        #   ros2 launch turtlebot3_cartographer cartographer.launch.py
        #
        # Then run this launch file:
        #   ros2 launch turtlebot_interceptor single_robot_navigation.launch.py
        #
        # Cartographer provides global /map (slow, accurate)
        # Fast Local Grid provides /local_map (fast, dynamic)
        # ============================================================
        
        # Fast Local Grid (HIGH-SPEED local mapping for dynamic navigation)
        # Updates at LIDAR rate (5-10 Hz) with simple ray-casting
        # Perfect for maze navigation - instant environment awareness!
        Node(
            package='turtlebot_interceptor',
            executable='fast_local_grid',
            name='fast_local_grid',
            output='screen'
        ),

        # Fast Local Grid (HIGH-SPEED local mapping for dynamic navigation)
        # Updates at LIDAR rate (5-10 Hz) with simple ray-casting
        # Perfect for maze navigation - instant environment awareness!
        Node(
            package='turtlebot_interceptor',
            executable='fast_local_grid_target',
            name='fast_local_grid_target',
            output='screen'
        ),
        
        # Target estimator node (align target map to seeker map)
        Node(
            package='turtlebot_interceptor',
            executable='target_est_node',
            name='target_estimator',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])

