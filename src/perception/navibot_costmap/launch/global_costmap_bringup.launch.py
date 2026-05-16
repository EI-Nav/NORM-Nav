#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Layer Costmap Bringup Launch File.

Launches all six nodes for the multi-layer costmap system:
1. Geometric Traversability Costmap Node
2. Semantic Traversability Costmap Node  
3. Directional Constraint Costmap Node
4. Velocity Constraint Node
5. Costmap Fusion Node
6. Constraint Compensation Node

This launch file provides a unified interface for starting the complete
multi-layer costmap system with configurable parameters for each component.

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
    Generate launch description for multi-layer costmap system.
    
    This launch file starts all six costmap nodes with configurable parameters.
    Each node can be configured through its respective YAML parameter file.
    
    Args:
        use_sim_time: Whether to use simulation time (True/False)
        geometric_config: Path to geometric costmap parameter file
        semantic_config: Path to semantic costmap parameter file
        directional_config: Path to directional costmap parameter file
        velocity_config: Path to velocity constraint parameter file
        fusion_config: Path to costmap fusion parameter file
        constraint_config: Path to constraint compensation parameter file
    
    Returns:
        LaunchDescription with all six costmap nodes
    """
    # Get package directory and set default parameter files
    pkg_dir = get_package_share_directory('navibot_costmap')
    
    # Launch arguments with enhanced descriptions
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Whether to use simulation time. Set to True for simulation, False for real robot.'
    )
    
    declare_geometric_params_file_cmd = DeclareLaunchArgument(
        'geometric_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'geometric_traversability_costmap_params.yaml'),
        description='Path to geometric costmap configuration file. '
                   'Use geometric_traversability_costmap_params.yaml for default settings.'
    )
    
    declare_semantic_params_file_cmd = DeclareLaunchArgument(
        'semantic_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'semantic_traversability_costmap_params.yaml'),
        description='Path to semantic costmap configuration file. '
                   'Use semantic_traversability_costmap_params.yaml for default settings.'
    )
    
    declare_directional_params_file_cmd = DeclareLaunchArgument(
        'directional_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'directional_constraint_costmap_params.yaml'),
        description='Path to directional costmap configuration file. '
                   'Use directional_constraint_costmap_params.yaml for default settings.'
    )
    
    declare_fusion_params_file_cmd = DeclareLaunchArgument(
        'fusion_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'costmap_fusion_params.yaml'),
        description='Path to costmap fusion configuration file. '
                   'Use costmap_fusion_params.yaml for default settings.'
    )
    
    declare_constraint_params_file_cmd = DeclareLaunchArgument(
        'constraint_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'constraint_compensation_params.yaml'),
        description='Path to constraint compensation configuration file. '
                   'Use constraint_compensation_params.yaml for default settings.'
    )
    
    declare_velocity_params_file_cmd = DeclareLaunchArgument(
        'velocity_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'velocity_constraint_params.yaml'),
        description='Path to velocity constraint configuration file. '
                   'Use velocity_constraint_params.yaml for default settings.'
    )
    
    
    # Geometric traversability costmap node with simplified configuration
    geometric_costmap_node = Node(
        package='navibot_costmap',
        executable='geometric_traversability_costmap_node',
        name='geometric_traversability_costmap_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('geometric_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ])
    
    # Semantic traversability costmap node with simplified configuration
    semantic_costmap_node = Node(
        package='navibot_costmap',
        executable='semantic_traversability_costmap_node',
        name='semantic_traversability_costmap_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('semantic_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ])
    
    # Directional constraint costmap node with simplified configuration
    directional_costmap_node = Node(
        package='navibot_costmap',
        executable='directional_constraint_costmap_node',
        name='directional_constraint_costmap_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('directional_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ])
    
    # Costmap fusion node with simplified configuration
    costmap_fusion_node = Node(
        package='navibot_costmap',
        executable='costmap_fusion_node',
        name='costmap_fusion_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('fusion_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ])
    
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
        ])
    
    # Constraint compensation node with simplified configuration
    constraint_compensation_node = Node(
        package='navibot_costmap',
        executable='constraint_compensation_node',
        name='constraint_compensation_node',
        output='screen',
        arguments=[],
        parameters=[
            LaunchConfiguration('constraint_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ])
    
    # Create and populate launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_geometric_params_file_cmd)
    ld.add_action(declare_semantic_params_file_cmd)
    ld.add_action(declare_directional_params_file_cmd)
    ld.add_action(declare_fusion_params_file_cmd)
    ld.add_action(declare_velocity_params_file_cmd)
    ld.add_action(declare_constraint_params_file_cmd)
    
    # Add nodes
    ld.add_action(geometric_costmap_node)
    ld.add_action(semantic_costmap_node)
    ld.add_action(directional_costmap_node)
    ld.add_action(velocity_constraint_node)
    ld.add_action(costmap_fusion_node)
    ld.add_action(constraint_compensation_node)
    
    return ld
