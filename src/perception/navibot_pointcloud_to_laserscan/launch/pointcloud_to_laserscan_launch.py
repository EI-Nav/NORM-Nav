#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file for point cloud to laser scan conversion.

This launch file starts the pointcloud_to_laserscan node to convert
3D point cloud data to 2D laser scan for navigation algorithms.

Usage:
    ros2 launch navibot_pointcloud_to_laserscan pointcloud_to_laserscan_launch.py

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from typing import List


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for point cloud to laser scan conversion.
    
    Returns:
        LaunchDescription: Launch configuration for the pointcloud_to_laserscan node
    """
    # Default parameter values for NaviBot navigation system
    DEFAULT_TARGET_FRAME = 'livox_frame'
    DEFAULT_TRANSFORM_TOLERANCE = 0.01
    DEFAULT_MIN_HEIGHT = -1.0
    DEFAULT_MAX_HEIGHT = 0.1
    DEFAULT_ANGLE_MIN = -3.14159  # -π
    DEFAULT_ANGLE_MAX = 3.14159   # π
    DEFAULT_ANGLE_INCREMENT = 0.0043  # π/360.0 (1 degree)
    DEFAULT_SCAN_TIME = 0.3333  # 30 Hz
    DEFAULT_RANGE_MIN = 0.45
    DEFAULT_RANGE_MAX = 10.0
    DEFAULT_USE_INF = True
    DEFAULT_INF_EPSILON = 1.0
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', 
            default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='navibot_pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/segmentation/obstacle'),  # Input: segmented obstacle point cloud
                ('scan', '/scan')                       # Output: 2D laser scan
            ],
            parameters=[{
                # Coordinate transformation parameters
                'target_frame': DEFAULT_TARGET_FRAME,
                'transform_tolerance': DEFAULT_TRANSFORM_TOLERANCE,
                
                # Height filtering parameters (meters)
                'min_height': DEFAULT_MIN_HEIGHT,      # Ground level filtering
                'max_height': DEFAULT_MAX_HEIGHT,      # Obstacle height limit
                
                # Laser scan geometry parameters (radians)
                'angle_min': DEFAULT_ANGLE_MIN,        # Full 360-degree scan
                'angle_max': DEFAULT_ANGLE_MAX,
                'angle_increment': DEFAULT_ANGLE_INCREMENT,  # 1 degree resolution
                'scan_time': DEFAULT_SCAN_TIME,        # 30 Hz update rate
                
                # Range filtering parameters (meters)
                'range_min': DEFAULT_RANGE_MIN,        # Minimum detection range
                'range_max': DEFAULT_RANGE_MAX,        # Maximum detection range
                
                # Infinity representation parameters
                'use_inf': DEFAULT_USE_INF,            # Use infinity for unobstructed rays
                'inf_epsilon': DEFAULT_INF_EPSILON     # Epsilon for infinity representation
            }],
            name='pointcloud_to_laserscan',
            output='screen'
        )
    ])