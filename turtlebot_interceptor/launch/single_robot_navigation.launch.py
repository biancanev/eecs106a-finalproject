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
        
        # EKF Pose Estimator (HIGH-PRECISION STATE ESTIMATION)
        # Fuses IMU + Magnetometer + Wheel Encoders for true robot dynamics
        # This is the SOURCE OF TRUTH for robot pose/velocity
        # Cartographer will use this pose for mapping
        Node(
            package='turtlebot_interceptor',
            executable='ekf_pose_estimator',
            name='ekf_pose_estimator',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'
        ),
        
        # Cartographer SLAM (ENVIRONMENT MAPPING)
        # Uses EKF pose + LIDAR to build accurate map
        # EKF provides precise robot state, Cartographer builds the environment model
        # This is MAPPING ONLY - pose comes from EKF above
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=[
                '-configuration_directory', '/opt/ros/humble/share/turtlebot3_cartographer/config',
                '-configuration_basename', 'turtlebot3_lds_2d.lua'
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Tell Cartographer to trust the EKF odometry more
                'tracking_frame': 'base_footprint',
                'published_frame': 'map',
            }],
            remappings=[
                # CRITICAL: Cartographer uses EKF's refined odometry (not raw wheel encoders)
                # EKF fuses IMU + Mag + Encoders → publishes /odom_ekf
                # This gives Cartographer the TRUE robot state for accurate mapping
                ('/odom', '/odom_ekf'),
            ],
            output='screen'
        ),
        
        # Cartographer occupancy grid node (converts Cartographer's map to OccupancyGrid)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            arguments=['-resolution', '0.02'],  # 2cm resolution for small obstacle detection
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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

