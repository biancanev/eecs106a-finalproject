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
            default_value='0.0',
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
            default_value='0.08',
            description='Obstacle radius in meters (smaller = less conservative)'
        ),
        DeclareLaunchArgument(
            'warm_start_x',
            default_value='0.0',
            description='Initial guess for target X'
        ),
        DeclareLaunchArgument(
            'warm_start_y',
            default_value='0.0',
            description='Initial guess for target Y'
        ),
        DeclareLaunchArgument(
            'warm_start_yaw',
            default_value='0.0',
            description='Initial guess for target yaw (rad)'
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
        # Target-side fast local grid (for target estimator alignment)
        Node(
            package='turtlebot_interceptor',
            executable='fast_local_grid_target',
            name='fast_local_grid_target',
            output='screen'
        ),

        # Target EKF (fuses pose measurements into smooth target estimate)
        Node(
            package='turtlebot_interceptor',
            executable='target_ekf_node',
            name='target_ekf_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
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
            arguments=['--ros-args', '--log-level', 'camera_cone_detector:=warn'],
            output='log'
        ),

        # Target estimator (vision + warm start)
        Node(
            package='turtlebot_interceptor',
            executable='target_est_node',
            name='target_estimator_vision',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'warm_start_x': LaunchConfiguration('warm_start_x'),
                'warm_start_y': LaunchConfiguration('warm_start_y'),
                'warm_start_yaw': LaunchConfiguration('warm_start_yaw'),
            }],
            remappings=[
                ('/target_est', '/target_estimate'),
            ],
            output='screen'
        ),

        # Target motion driver
        Node(
            package='turtlebot_interceptor',
            executable='move_target',
            name='move_target',
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
                'mpc_horizon': 12,
                'dt': 0.08,
                'v_max_base': 0.35,  # Matches MPC v_max for smooth curves (was 0.6)
                'v_min': 0.0,
                'omega_max': 2.0,  # Matches MPC wz_max for smooth curves (was 2.5)
                'Kp_v': 2.0,  # Fallback control gains (lab8 pattern)
                'Kp_w': 0.8,
                'Kd_w': 0.5,
                # Use warm-start as navigation goal (keeps seeker chasing target guess)
                'goal_x': LaunchConfiguration('warm_start_x'),
                'goal_y': LaunchConfiguration('warm_start_y'),
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

        # Drive tester (optional) - publishes cmd_vel patterns and logs EKF vs AMCL error
        # Disable if you don't want automated motions during testing
        # Node(
        #     package='turtlebot_interceptor',
        #     executable='drive_tester',
        #     name='drive_tester',
        #     output='screen'
        # ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
