#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Directional Constraint Costmap Launch File.

Generates directional constraint costmap from enhanced behavioral constraints.
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
    Generate launch description for directional constraint costmap node.
    
    This launch file starts the directional constraint costmap node with configurable
    parameters. The node generates directional constraint costmap from enhanced
    behavioral constraints containing AABB information and direction constraints.
    
    Args:
        costmap_params_file: Path to the costmap parameter YAML file
        use_sim_time: Whether to use simulation time (True/False)
    
    Returns:
        LaunchDescription: Launch description with directional constraint costmap node
    """
    # Get package directory and set default parameter file
    pkg_dir = get_package_share_directory('navibot_costmap')
    default_params_file = os.path.join(pkg_dir, 'config', 'directional_constraint_costmap_params.yaml')
    
    # Launch arguments with enhanced descriptions
    declare_params_file_cmd = DeclareLaunchArgument(
        'costmap_params_file',
        default_value=default_params_file,
        description='Path to directional constraint costmap parameter YAML file. '
                   'Use directional_constraint_costmap_params.yaml for default settings.')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to use simulation time. Set to True for simulation, False for real robot.')
    
    
    # Directional constraint costmap node with simplified configuration
    directional_constraint_costmap_node = Node(
        package='navibot_costmap',
        executable='directional_constraint_costmap_node',
        name='directional_constraint_costmap_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('costmap_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            # Allow remapping of input/output topics if needed
            ('/compensated_behavioral_constraints', LaunchConfiguration('constraint_topic', default='/compensated_behavioral_constraints')),
            ('/costmap/directional', LaunchConfiguration('output_topic', default='/costmap/directional')),
        ])
    
    # Create and populate launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add node
    ld.add_action(directional_constraint_costmap_node)
    
    return ld
