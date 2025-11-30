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
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # MCL node (localization using map and LIDAR)
        Node(
            package='turtlebot_interceptor',
            executable='mcl_node',
            name='mcl_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # UKF node for target tracking
        Node(
            package='turtlebot_interceptor',
            executable='ukf_node',
            name='ukf_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # MPC node (control)
        Node(
            package='turtlebot_interceptor',
            executable='mpc_node',
            name='mpc_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])

