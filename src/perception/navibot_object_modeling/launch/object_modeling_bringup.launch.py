#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Object Modeling Launch File.

Launches the object modeling node that synchronizes point cloud
and GroundedSam messages to model objects in 3D space.

Usage:
    # Launch with default configuration
    ros2 launch navibot_object_modeling object_modeling.launch.py

    # Launch with custom configuration file
    ros2 launch navibot_object_modeling object_modeling.launch.py \
        config_file:=/path/to/custom_config.yaml

    # Launch with simulation time (default for simulation environment)
    ros2 launch navibot_object_modeling object_modeling.launch.py \
        use_sim_time:=true

    # Launch for real robot (disable simulation time)
    ros2 launch navibot_object_modeling object_modeling.launch.py \
        use_sim_time:=false

Features:
    - Point cloud segmentation by object
    - Cross-frame object tracking with OBB modeling
    - Real-time object visualization with markers
    - Configurable parameters via YAML files
    - Support for both simulation and real robot environments

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for object modeling.
    
    Returns:
        LaunchDescription containing the modeling node and configuration
    """
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navibot_object_modeling'),
            'config',
            'modeling_params.yaml'
        ]),
        description='Path to the modeling configuration YAML file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true (default for simulation environment)'
    )
    
    # Create modeling node
    modeling_node = Node(
        package='navibot_object_modeling',
        executable='object_modeling',
        name='object_modeling',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # Log launch information
    launch_info = LogInfo(
        msg=[
            'Starting Object Modeling Node:\n',
            '  - Config File: ', LaunchConfiguration('config_file'), '\n',
            '  - Use Sim Time: ', LaunchConfiguration('use_sim_time'), '\n',
            '  - Node Name: object_modeling'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_config_file,
        declare_use_sim_time,
        
        # Log launch info
        launch_info,
        
        # Nodes
        modeling_node,
    ])