#!/usr/bin/env python3
"""
Launch file for simulation mode
Runs all nodes with fake lidar and voxel grid map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Simulation nodes
        Node(
            package='turtlebot_interceptor',
            executable='simulation_node',
            name='simulation_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--map-only'],
            output='screen'
        ),
        
        Node(
            package='turtlebot_interceptor',
            executable='simulation_node',
            name='fake_lidar_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--lidar-only'],
            output='screen'
        ),
        
        # MCL node
        Node(
            package='turtlebot_interceptor',
            executable='mcl_node',
            name='mcl_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['--sim'],
            output='screen'
        ),
        
        # Target KF node
        Node(
            package='turtlebot_interceptor',
            executable='target_kf_node',
            name='target_kf_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # MPC node
        Node(
            package='turtlebot_interceptor',
            executable='mpc_node',
            name='mpc_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])

