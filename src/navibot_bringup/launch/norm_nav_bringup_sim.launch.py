#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NORM-Nav bringup launch file for NaviBot simulation environment.

This enhanced launch file starts the complete navigation stack including:
- Gazebo simulation environment
- Point cloud to laser scan conversion
- FAST-LIO localization
- GroundedSAM2 object detection and tracking
- Object modeling for 3D object representation
- Multi-layer costmap system (geometric, semantic, directional, fusion)
- Nav2 navigation stack
- Static TF transforms (map->odom, odom->camera_init)

Usage:
    ros2 launch navibot_bringup norm_nav_bringup_sim.launch.py world:=MEDIUM_OSM lio_rviz:=False nav_rviz:=True use_sim_time:=True

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
from launch_ros.actions import Node

# Constants for point cloud to laser scan conversion
PI: float = math.pi


@dataclass(frozen=True)
class TopicNames:
    """ROS topic names used throughout the system."""
    LIVOX_POINTCLOUD: str = '/livox/lidar/pointcloud'
    SCAN: str = '/scan'
    ODOMETRY: str = '/Odometry'
    CLOUD_REGISTERED: str = '/cloud'


@dataclass(frozen=True)
class FrameNames:
    """TF frame names used throughout the system."""
    MAP: str = 'map'
    ODOM: str = 'odom'
    CAMERA_INIT: str = 'camera_init'
    CLOUD_LINK: str = 'imu_link'
    BASE_LINK: str = 'base_link'


@dataclass
class SimulationConfigPaths:
    """Configuration file paths for simulation environment."""
    def __init__(self, navibot_bringup_dir: str):
        self.base_dir = navibot_bringup_dir
        self.config_dir = os.path.join(navibot_bringup_dir, 'config', 'simulation')
        self.costmap_config_dir = os.path.join(self.config_dir, 'costmap')
        
        # Core configuration files
        self.fastlio_params = os.path.join(self.config_dir, 'fastlio_mid360_sim.yaml')
        self.fastlio_rviz = os.path.join(navibot_bringup_dir, 'rviz', 'fastlio.rviz')
        self.nav2_params = os.path.join(self.config_dir, 'nav2_params_sim.yaml')
        
        # Perception configuration files
        self.tracker_params = os.path.join(self.config_dir, 'tracker_params_sim.yaml')
        self.modeling_params = os.path.join(self.config_dir, 'modeling_params_sim.yaml')
        
        # Costmap configuration files
        self.geometric_costmap = os.path.join(self.costmap_config_dir, 'geometric_traversability_costmap_params_sim.yaml')
        self.semantic_costmap = os.path.join(self.costmap_config_dir, 'semantic_traversability_costmap_params_sim.yaml')
        self.directional_costmap = os.path.join(self.costmap_config_dir, 'directional_constraint_costmap_params_sim.yaml')
        self.fusion_costmap = os.path.join(self.costmap_config_dir, 'costmap_fusion_params_sim.yaml')
        self.constraint_compensation = os.path.join(self.costmap_config_dir, 'constraint_compensation_params_sim.yaml')
        self.velocity_constraint = os.path.join(self.costmap_config_dir, 'velocity_constraint_params_sim.yaml')


@dataclass(frozen=True)
class PointCloudToLaserScanConfig:
    """Configuration for point cloud to laser scan conversion."""
    SCAN_ANGLE_INCREMENT: float = PI / 720.0  # Approximately 0.0043 radians
    MIN_HEIGHT_SIM: float = -0.1
    MAX_HEIGHT_SIM: float = 0.5
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


def create_static_tf_publisher(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
    frame_id: str, child_frame_id: str,
    name: str = None
) -> Node:
    """
    Create a static transform publisher node.
    
    Args:
        x, y, z: Translation components
        roll, pitch, yaw: Rotation components (in radians)
        frame_id: Parent frame ID
        child_frame_id: Child frame ID
        name: Optional node name
        
    Returns:
        Node: Static transform publisher node
    """
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name or f"static_tf_{frame_id}_to_{child_frame_id}",
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', frame_id, '--child-frame-id', child_frame_id
        ],
        on_exit=Shutdown()
    )


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for NORM-Nav simulation bringup.
    
    Returns:
        Complete launch description with all nodes and configurations
    """
    # Package directories
    navibot_bringup_dir = get_package_share_directory('navibot_bringup')
    navibot_simulation_launch_dir = os.path.join(get_package_share_directory('navibot_simulation'), 'launch')
    navibot_navigation_launch_dir = os.path.join(get_package_share_directory('navibot_navigation'), 'launch')
    navibot_costmap_launch_dir = os.path.join(get_package_share_directory('navibot_costmap'), 'launch')
    navibot_grounded_sam2_launch_dir = os.path.join(get_package_share_directory('navibot_grounded_sam2'), 'launch')
    navibot_object_modeling_launch_dir = os.path.join(get_package_share_directory('navibot_object_modeling'), 'launch')

    # Initialize configuration paths
    config_paths = SimulationConfigPaths(navibot_bringup_dir)
    topic_names = TopicNames()
    frame_names = FrameNames()

    # Launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_lio_rviz = LaunchConfiguration('lio_rviz')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    # Load robot description
    robot_description_content = Command([
        'xacro ', os.path.join(navibot_bringup_dir, 'urdf', 'NaviBot_sim.xacro')
    ])

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

    # ============================================================================
    # Point Cloud to Laser Scan Conversion
    # ============================================================================
    bringup_pointcloud_to_laserscan_node = Node(
        package='navibot_pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', topic_names.LIVOX_POINTCLOUD),
            ('scan', topic_names.SCAN)
        ],
        parameters=[{
            'target_frame': frame_names.CLOUD_LINK,
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
            'inf_epsilon': pc_to_scan_cfg.INF_EPSILON,
            'use_sim_time': use_sim_time
        }],
        name='pointcloud_to_laserscan',
        on_exit=Shutdown()
    )

    # ============================================================================
    # FAST-LIO (LiDAR-Inertial Odometry) System
    # ============================================================================
    bringup_LIO_group = GroupAction([
        # Static TF: map -> odom
        create_static_tf_publisher(
            x=static_tf_cfg.MAP_TO_ODOM_X,
            y=static_tf_cfg.MAP_TO_ODOM_Y,
            z=static_tf_cfg.MAP_TO_ODOM_Z,
            roll=0.0, pitch=0.0, yaw=0.0,
            frame_id=frame_names.MAP,
            child_frame_id=frame_names.ODOM,
            name='static_tf_map_to_odom'
        ),

        # Static TF: odom -> camera_init (camera_init is LIO's world frame)
        create_static_tf_publisher(
            x=static_tf_cfg.ODOM_TO_CAMERA_INIT_X,
            y=static_tf_cfg.ODOM_TO_CAMERA_INIT_Y,
            z=static_tf_cfg.ODOM_TO_CAMERA_INIT_Z,
            roll=0.0, pitch=0.0, yaw=0.0,
            frame_id=frame_names.ODOM,
            child_frame_id=frame_names.CAMERA_INIT,
            name='static_tf_odom_to_camera_init'
        ),

        # FAST-LIO mapping node
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=[
                config_paths.fastlio_params,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            on_exit=Shutdown()
        ),

        # FAST-LIO RViz visualization (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', config_paths.fastlio_rviz],
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
                'stateEstimationTopic': topic_names.ODOMETRY,
                'registeredScanTopic': topic_names.CLOUD_REGISTERED,
                'flipStateEstimation': False,
                'flipRegisteredScan': False,
                'sendTF': True,
                'reverseTF': False,
                'use_sim_time': use_sim_time
            }],
            on_exit=Shutdown()
        )
    ])

    # ============================================================================
    # Visual Perception Pipeline
    # ============================================================================
    
    # GroundedSAM2 Tracker for object detection and tracking
    start_grounded_sam2_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navibot_grounded_sam2_launch_dir, 'grounded_sam2_tracker_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': config_paths.tracker_params
        }.items()
    )

    # Object Modeling for 3D object representation
    start_object_modeling = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navibot_object_modeling_launch_dir, 'object_modeling_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': config_paths.modeling_params
        }.items()
    )

    # ============================================================================
    # Global Costmap System
    # ============================================================================
    start_global_costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navibot_costmap_launch_dir, 'global_costmap_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'geometric_params_file': config_paths.geometric_costmap,
            'semantic_params_file': config_paths.semantic_costmap,
            'directional_params_file': config_paths.directional_costmap,
            'fusion_params_file': config_paths.fusion_costmap,
            'constraint_params_file': config_paths.constraint_compensation,
            'velocity_params_file': config_paths.velocity_constraint
        }.items()
    )

    # ============================================================================
    # Navigation2 Stack
    # ============================================================================
    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navibot_navigation_launch_dir, 'bringup_classic_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': config_paths.nav2_params,
            'nav_rviz': use_nav_rviz
        }.items()
    )

    # ============================================================================
    # Build Launch Description
    # ============================================================================
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lio_rviz_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_world_cmd)

    # Add nodes and groups in logical order
    # 1. Simulation environment
    ld.add_action(start_gazebo_simulation)
    
    # 2. Sensor data processing
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    
    # 3. Localization system
    ld.add_action(bringup_LIO_group)
    
    # 4. Global costmap system
    ld.add_action(start_global_costmap)
    
    # 5. Visual perception pipeline
    ld.add_action(start_grounded_sam2_tracker)
    ld.add_action(start_object_modeling)
    
    # 6. Navigation system
    ld.add_action(start_navigation2)

    return ld
