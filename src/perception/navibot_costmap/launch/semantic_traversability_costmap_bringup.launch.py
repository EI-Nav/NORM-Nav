#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Semantic Traversability Costmap Launch File.

Converts semantic segmentation results to traversability costmap for Nav2 navigation.
This launch file provides a simplified interface with only essential parameters.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for semantic traversability costmap node.
    
    This launch file starts the semantic traversability costmap node with configurable
    parameters. Topic configurations are managed through the YAML parameter file.
    
    Args:
        costmap_params_file: Path to the costmap parameter YAML file
        use_sim_time: Whether to use simulation time (True/False)
    
    Returns:
        LaunchDescription: Launch description with semantic costmap generation node
    """
    # Get package directory and set default parameter file
    pkg_dir = get_package_share_directory('navibot_costmap')
    default_params_file = os.path.join(pkg_dir, 'config', 'semantic_traversability_costmap_params.yaml')
    
    # Launch arguments with enhanced descriptions
    declare_params_file_cmd = DeclareLaunchArgument(
        'costmap_params_file',
        default_value=default_params_file,
        description='Path to costmap parameter YAML file. '
                   'Use semantic_traversability_costmap_params.yaml for default settings.')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to use simulation time. Set to True for simulation, False for real robot.')
    
    
    # Semantic traversability costmap node with simplified configuration
    semantic_traversability_costmap_node = Node(
        package='navibot_costmap',
        executable='semantic_traversability_costmap_node',
        name='semantic_traversability_costmap_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('costmap_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ])
    
    # Create and populate launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add node
    ld.add_action(semantic_traversability_costmap_node)
    
    return ld


