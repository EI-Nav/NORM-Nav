#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bringup launch file for NaviBot simulation environment.

This launch file starts the complete navigation stack including:
- Gazebo simulation environment
- Point cloud to laser scan conversion
- FAST-LIO localization
- Geometric traversability costmap generation from LIO point cloud
- Nav2 navigation stack
- Static TF transforms (map->odom, odom->camera_init)

Usage:
    ros2 launch navibot_bringup bringup_sim.launch.py world:=MEDIUM_OSM lio:=fastlio lio_rviz:=False nav_rviz:=True use_sim_time:=True

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import math
import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap

# Constants for point cloud to laser scan conversion
PI: float = math.pi


@dataclass(frozen=True)
class PointCloudToLaserScanConfig:
    """Configuration for point cloud to laser scan conversion."""
    SCAN_ANGLE_INCREMENT: float = PI / 720.0  # Approximately 0.0043 radians
    MIN_HEIGHT_SIM: float = -0.2
    MAX_HEIGHT_SIM: float = 0.1
    TRANSFORM_TOLERANCE: float = 0.01
    SCAN_TIME: float = 0.3333
    MIN_RANGE: float = 0.2
    MAX_RANGE: float = 10.0
    INF_EPSILON: float = 1.0


@dataclass(frozen=True)
class StaticTFConfig:
    """Static transform configurations for simulation."""
    ODOM_TO_CAMERA_INIT_X: float = 0.0
    ODOM_TO_CAMERA_INIT_Y: float = 0.0
    ODOM_TO_CAMERA_INIT_Z: float = 0.0
    MAP_TO_ODOM_X: float = 0.0
    MAP_TO_ODOM_Y: float = 0.0
    MAP_TO_ODOM_Z: float = 0.0


# Create configuration instances
pc_to_scan_cfg = PointCloudToLaserScanConfig()
static_tf_cfg = StaticTFConfig()


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for simulation bringup.
    
    Returns:
        Complete launch description with all nodes and configurations
    """
    # Package directories
    navibot_bringup_dir = get_package_share_directory('navibot_bringup')
    navibot_simulation_launch_dir = os.path.join(get_package_share_directory('navibot_simulation'), 'launch')
    navibot_navigation_launch_dir = os.path.join(get_package_share_directory('navibot_navigation'), 'launch')
    navibot_costmap_launch_dir = os.path.join(get_package_share_directory('navibot_costmap'), 'launch')

    # Launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # Load robot description
    robot_description_content = Command([
        'xacro ', os.path.join(navibot_bringup_dir, 'urdf', 'NaviBot_sim.xacro')
    ])

    # Configuration file paths
    fastlio_mid360_params = os.path.join(navibot_bringup_dir, 'config', 'simulation', 'fastlio_mid360_sim.yaml')
    fastlio_rviz_cfg_dir = os.path.join(navibot_bringup_dir, 'rviz', 'fastlio.rviz')
    nav2_params_file_dir = os.path.join(navibot_bringup_dir, 'config', 'simulation', 'nav2_params_sim.yaml')
    costmap_params_file = os.path.join(get_package_share_directory('navibot_costmap'), 'config', 'geometric_traversability_costmap_params.yaml')

    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_lio_rviz_cmd = DeclareLaunchArgument(
        'lio_rviz',
        default_value='False',
        description='Launch FAST-LIO RViz visualization')

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='LARGE_OSM',
        description='Select gazebo world and PCD file name')

    # Gazebo simulation
    start_gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navibot_simulation_launch_dir, 'gazebo_simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_description': robot_description_content,
            'rviz': 'False'}.items()
    )

    bringup_pointcloud_to_laserscan_node = Node(
        package='navibot_pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in',  ['/livox/lidar/pointcloud']),
                    ('scan',  ['/scan'])],
        parameters=[{
            'target_frame': 'lidar_link',
            'transform_tolerance': pc_to_scan_cfg.TRANSFORM_TOLERANCE,
            'min_height': pc_to_scan_cfg.MIN_HEIGHT_SIM,
            'max_height': pc_to_scan_cfg.MAX_HEIGHT_SIM,
            'angle_min': -PI,
            'angle_max': PI,
            'angle_increment': pc_to_scan_cfg.SCAN_ANGLE_INCREMENT,
            'scan_time': pc_to_scan_cfg.SCAN_TIME,
            'range_min': pc_to_scan_cfg.MIN_RANGE,
            'range_max': pc_to_scan_cfg.MAX_RANGE,
            'use_inf': True,
            'inf_epsilon': pc_to_scan_cfg.INF_EPSILON
        }],
        name='pointcloud_to_laserscan',
        on_exit=Shutdown()
    )

    # FAST-LIO (LiDAR-Inertial Odometry) group
    bringup_LIO_group = GroupAction([
        # Static TF: map -> odom
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                '--x', str(static_tf_cfg.MAP_TO_ODOM_X),
                '--y', str(static_tf_cfg.MAP_TO_ODOM_Y),
                '--z', str(static_tf_cfg.MAP_TO_ODOM_Z),
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'map', '--child-frame-id', 'odom'
            ],
            on_exit=Shutdown()
        ),

        # Static TF: odom -> camera_init (camera_init is LIO's world frame)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                '--x', str(static_tf_cfg.ODOM_TO_CAMERA_INIT_X),
                '--y', str(static_tf_cfg.ODOM_TO_CAMERA_INIT_Y),
                '--z', str(static_tf_cfg.ODOM_TO_CAMERA_INIT_Z),
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'odom', '--child-frame-id', 'camera_init'
            ],
            on_exit=Shutdown()
        ),

        # FAST-LIO mapping node
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[fastlio_mid360_params, {use_sim_time: use_sim_time}],
            output='screen',
            on_exit=Shutdown()
        ),

        # FAST-LIO RViz visualization (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', fastlio_rviz_cfg_dir],
            condition=IfCondition(use_lio_rviz),
            on_exit=Shutdown()
        ),

        # LIO interface for TF publishing
        Node(
            package='navibot_lio_interface',
            executable='lioInterface',
            name='lioInterface',
            output='screen',
            parameters=[{
                'stateEstimationTopic': '/Odometry',
                'registeredScanTopic': '/cloud_registered_body',
                'flipStateEstimation': False,
                'flipRegisteredScan': False,
                'sendTF': True,
                'reverseTF': False,
                'use_sim_time': use_sim_time
            }],
            on_exit=Shutdown()
        )
    ])

    # Geometric traversability costmap generation from FAST-LIO point cloud
    # Publishes to /map topic, replacing traditional map_server
    start_geometric_costmap_node = GroupAction([
        SetRemap(src='/costmap/geometric', dst='/map'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navibot_costmap_launch_dir, 'geometric_traversability_costmap_bringup.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'costmap_params_file': costmap_params_file
            }.items()
        )
    ])

    # Navigation2 stack
    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navibot_navigation_launch_dir, 'bringup_classic_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file_dir,
            'nav_rviz': use_nav_rviz
        }.items()
    )

    # Build launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)

    # Add nodes and groups
    ld.add_action(start_gazebo_simulation)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(bringup_LIO_group)
    
    # Geometric traversability costmap from FAST-LIO (replaces map_server)
    ld.add_action(start_geometric_costmap_node)
    
    # Navigation2 stack
    ld.add_action(start_navigation2)

    return ld
