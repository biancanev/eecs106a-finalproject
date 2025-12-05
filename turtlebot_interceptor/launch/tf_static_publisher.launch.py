#!/usr/bin/env python3
"""
Static TF Publisher for Map Frame
Publishes a static transform from map to base_scan so RViz can display the map
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Static transform from map to base_scan
        # This allows RViz to display the map (in 'map' frame) when fixed frame is 'base_scan'
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
    ])

