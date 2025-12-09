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
            default_value='0.0',
            description='Goal Y position (meters)'
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
        
        # DISABLED: EKF has drift issues - using raw odometry instead
        # The EKF integrates velocities and causes massive drift when stationary
        # For now, use simple_pose_publisher which just republishes /odom
        # TODO: Fix EKF drift before re-enabling
        # Node(
        #     package='turtlebot_interceptor',
        #     executable='ekf_pose_estimator',
        #     name='ekf_pose_estimator',
        #     parameters=[{
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #     }],
        #     output='screen'
        # ),
        
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
        
        # Camera Cone Detector (yellow cone detection with 15cm diameter)
        # Provides close-range obstacle detection for MPC
        Node(
            package='turtlebot_interceptor',
            executable='camera_cone_detector',
            name='camera_cone_detector',
            parameters=[{
                'image_topic': '/image_raw',
                'camera_info_topic': '/camera_info',
                'pose_topic': '/amcl_pose',
            }],
            output='screen'
        ),
        
        # MCL node (localization using map and LIDAR)
        # OPTIONAL: Cartographer already provides pose tracking
        # Can disable this and use Cartographer's pose directly from /tracked_pose
        # For now, keep it for compatibility with existing MPC code that expects /amcl_pose
        # Node(
        #     package='turtlebot_interceptor',
        #     executable='mcl_node',
        #     name='mcl_node',
        #     parameters=[{
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #         'num_particles': 300,
        #         'motion_noise_x': 0.02,
        #         'motion_noise_y': 0.02,
        #         'motion_noise_theta': 0.01,
        #         'max_range': 3.5,
        #         'min_range': 0.25,
        #         'resample_threshold': 0.98,
        #     }],
        #     output='screen'
        # ),
        
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
                'max_obstacles': LaunchConfiguration('max_obstacles'),
                'obstacle_radius': LaunchConfiguration('obstacle_radius'),
            }],
            output='screen'
        ),
        
        # Navigation visualizer (RViz markers for progress tracking)
        Node(
            package='turtlebot_interceptor',
            executable='navigation_visualizer',
            name='navigation_visualizer',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'robot_radius': 0.15,
                'path_history_length': 200,  # Keep more history for better visualization
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

