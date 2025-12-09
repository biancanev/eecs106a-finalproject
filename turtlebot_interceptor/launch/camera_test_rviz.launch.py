#!/usr/bin/env python3
"""
Launch file for camera cone detection testing with RViz
Brings up camera, detector, and RViz with all necessary topics configured
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Launch camera node (v4l2)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path'
        ),
        
        # Camera node (v4l2)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'device': LaunchConfiguration('camera_device'),
                'image_size': [640, 480],
                'pixel_format': 'YUY2',
            }],
            condition=IfCondition(LaunchConfiguration('use_camera')),
            output='screen'
        ),
        
        # Camera cone detector
        Node(
            package='turtlebot_interceptor',
            executable='camera_cone_detector',
            name='camera_cone_detector',
            output='screen'
        ),
        
        # Test visualization node
        Node(
            package='turtlebot_interceptor',
            executable='test_camera_cones',
            name='test_camera_cones',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
        
        # RViz with pre-configured topics
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('turtlebot_interceptor'),
                'config',
                'camera_test.rviz'
            )],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        ),
    ])

