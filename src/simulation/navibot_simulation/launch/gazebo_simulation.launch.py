#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo simulation environment launch file for NaviBot.

Launches Gazebo with configurable world environments and robot spawning.
Supports multiple world sizes and robot configurations for navigation testing.

Usage:
    ros2 launch navibot_simulation gazebo_simulation.launch.py world:=LARGE_OSM

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import os
from enum import Enum
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            GroupAction, IncludeLaunchDescription)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


class WorldType(str, Enum):
    """Enumeration of available world types."""
    LARGE_OSM = 'LARGE_OSM'
    MEDIUM_OSM = 'MEDIUM_OSM'
    SMALL_OSM = 'SMALL_OSM'


def get_world_config(world_type: str) -> Optional[Dict[str, str]]:
    """
    Get world configuration parameters for a given world type.
    
    Args:
        world_type: World type identifier (from WorldType enum)
        
    Returns:
        Dictionary with spawn position and world path, or None if invalid type
    """
    world_configs: Dict[str, Dict[str, str]] = {
        WorldType.LARGE_OSM.value: {
            'x': '0.0',
            'y': '0.0',
            'z': '0.1',
            'yaw': '0.0',
            'world_path': 'large_osm/large_osm.world'
        },
        WorldType.MEDIUM_OSM.value: {
            'x': '0.0',
            'y': '0.0',
            'z': '0.1',
            'yaw': '0.0',
            'world_path': 'medium_osm/medium_osm.world'
        },
        WorldType.SMALL_OSM.value: {
            'x': '0.0',
            'y': '0.0',
            'z': '0.1',
            'yaw': '0.0',
            'world_path': 'small_osm/small_osm.world'
        }
    }
    return world_configs.get(world_type, None)


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for Gazebo simulation.
    
    Returns:
        Complete launch description with Gazebo and robot spawning
    """
    # Get the launch directory
    bringup_dir = get_package_share_directory('navibot_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    default_robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('navibot_simulation'), 'urdf', 'simulation_waking_robot.xacro')])

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')
    robot_description = LaunchConfiguration('robot_description')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.LARGE_OSM.value,
        description='Choose world type'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description, value_type=str)
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description, value_type=str)
        }],
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    def create_gazebo_launch_group(world_type: str) -> Optional[GroupAction]:
        """
        Create a launch group for a specific world type.
        
        Args:
            world_type: World type identifier
            
        Returns:
            GroupAction for launching the world, or None if config not found
        """
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw']
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, 'world', world_config['world_path'])}.items(),
                )
            ]
        )

    bringup_LARGE_OSM_cmd_group = create_gazebo_launch_group(WorldType.LARGE_OSM.value)
    bringup_MEDIUM_OSM_cmd_group = create_gazebo_launch_group(WorldType.MEDIUM_OSM.value)
    bringup_SMALL_OSM_cmd_group = create_gazebo_launch_group(WorldType.SMALL_OSM.value)

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_LARGE_OSM_cmd_group)
    ld.add_action(bringup_MEDIUM_OSM_cmd_group)
    ld.add_action(bringup_SMALL_OSM_cmd_group)

    # Uncomment this line if you want to start RViz
    ld.add_action(start_rviz_cmd)

    return ld
