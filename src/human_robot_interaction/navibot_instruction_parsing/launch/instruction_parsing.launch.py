#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch file for behavioral instruction parsing node.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for instruction parsing node."""
    pkg_dir = get_package_share_directory('navibot_instruction_parsing')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'instruction_parsing_params.yaml'),
        description='Path to configuration file'
    )
    
    instruction_parsing_node = Node(
        package='navibot_instruction_parsing',
        executable='instruction_parsing_node',
        name='instruction_parsing_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_file_arg,
        instruction_parsing_node,
    ])
