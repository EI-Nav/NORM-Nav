#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Velocity Constraint Launch File.

Generates velocity constraints from behavioral constraints.
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
    Generate launch description for velocity constraint node.
    
    This launch file starts the velocity constraint node with configurable
    parameters. The node generates velocity limits from behavioral constraints
    based on robot position relative to constraint regions.
    
    Args:
        velocity_params_file: Path to the velocity constraint parameter YAML file
        use_sim_time: Whether to use simulation time (True/False)
    
    Returns:
        LaunchDescription: Launch description with velocity constraint node
    """
    # Get package directory and set default parameter file
    pkg_dir = get_package_share_directory('navibot_costmap')
    default_params_file = os.path.join(pkg_dir, 'config', 'velocity_constraint_params.yaml')
    
    # Launch arguments with enhanced descriptions
    declare_params_file_cmd = DeclareLaunchArgument(
        'velocity_params_file',
        default_value=default_params_file,
        description='Path to velocity constraint parameter YAML file. '
                   'Use velocity_constraint_params.yaml for default settings.')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to use simulation time. Set to True for simulation, False for real robot.')
    
    
    
    # Velocity constraint node with simplified configuration
    velocity_constraint_node = Node(
        package='navibot_costmap',
        executable='velocity_constraint_node',
        name='velocity_constraint_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('velocity_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        )
    
    # Create and populate launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add node
    ld.add_action(velocity_constraint_node)
    
    return ld
