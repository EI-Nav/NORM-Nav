#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Classic navigation bringup launch file for Nav2.

Launches the Nav2 navigation stack with configurable parameters including:
- Navigation lifecycle nodes (controller, planner, behavior servers)
- RViz visualization
- Composed or standalone node execution

Usage:
    ros2 launch navibot_navigation bringup_classic_navigation.py

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for classic navigation.
    
    Returns:
        Complete launch description with Nav2 navigation stack
    """
    # Package directories
    bringup_dir = get_package_share_directory('navibot_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # Topic remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Parameter substitutions and rewriting
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # RViz configuration selection based on use_sim_time
    rviz_config_file = LaunchConfiguration('rviz_config', default=PythonExpression([
        "'", os.path.join(bringup_dir, 'rviz', 'nav2_sim.rviz'), "' if '", use_sim_time, "' == 'True' else '",
        os.path.join(bringup_dir, 'rviz', 'nav2_real.rviz'), "'"
    ]))

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='True',
        description='Whether to respawn if a node crashes (when composition is disabled)')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    # Navigation bringup group
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_mt',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'rviz_config': rviz_config_file
            }.items(),
            condition=IfCondition(use_nav_rviz)),
    ])

    # Build launch description
    ld = LaunchDescription()

    # Add environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_nav_rviz_cmd)

    # Add navigation bringup group
    ld.add_action(bringup_cmd_group)

    return ld
