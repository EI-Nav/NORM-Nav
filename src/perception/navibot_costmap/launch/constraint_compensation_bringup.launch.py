#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Constraint Compensation Launch File.

Compensates behavioral constraints based on geometric traversability costmap.
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
    Generate launch description for constraint compensation node.
    
    This launch file starts the constraint compensation node with configurable
    parameters. The node compensates behavioral constraints based on geometric
    traversability costmap, calculating average geometric cost for AABB regions
    and updating traversability constraints for null values.
    
    Args:
        costmap_params_file: Path to the constraint compensation parameter YAML file
        use_sim_time: Whether to use simulation time (True/False)
    
    Returns:
        LaunchDescription: Launch description with constraint compensation node
    """
    # Get package directory and set default parameter file
    pkg_dir = get_package_share_directory('navibot_costmap')
    default_params_file = os.path.join(pkg_dir, 'config', 'constraint_compensation_params.yaml')
    
    # Launch arguments with enhanced descriptions
    declare_params_file_cmd = DeclareLaunchArgument(
        'costmap_params_file',
        default_value=default_params_file,
        description='Path to constraint compensation parameter YAML file. '
                   'Use constraint_compensation_params.yaml for default settings.')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to use simulation time. Set to True for simulation, False for real robot.')
    
    
    # Constraint compensation node with simplified configuration
    constraint_compensation_node = Node(
        package='navibot_costmap',
        executable='constraint_compensation_node',
        name='constraint_compensation_node',
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
            ('/behavioral_constraints', LaunchConfiguration('constraint_topic', default='/behavioral_constraints')),
            ('/costmap/geometric', LaunchConfiguration('geometric_costmap_topic', default='/costmap/geometric')),
            ('/object_modeling/object_obb_info', LaunchConfiguration('obb_info_topic', default='/object_modeling/object_obb_info')),
            ('/compensated_behavioral_constraints', LaunchConfiguration('output_topic', default='/compensated_behavioral_constraints')),
        ])
    
    # Create and populate launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add node
    ld.add_action(constraint_compensation_node)
    
    return ld
