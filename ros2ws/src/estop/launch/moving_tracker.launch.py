#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Run your moving object tracker script
        Node(
            package='estop',  # <-- REPLACE with your actual ROS 2 package name
            executable='moving_tracker',  # <-- REPLACE with your setup.py entry point/executable name
            name='moving_object_tracker',
            output='screen',
            parameters=[{
                'cluster_tolerance': 0.3,
                'min_cluster_size': 3,
                'max_cluster_size': 30,
                'movement_threshold': 0.15,
            }],
            remappings=[
                ('/scan', '/scan')  # Left side is what your code expects, right side is the actual robot topic
            ]
        ),

        # 2. Automatically launch RViz2 pre-configured or empty
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])