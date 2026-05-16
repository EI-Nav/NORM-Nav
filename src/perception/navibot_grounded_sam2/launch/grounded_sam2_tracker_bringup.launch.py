#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file for GroundedSAM2 object tracker node.

This launch file starts the real-time object tracking node with configurable
parameters for GroundingDINO detection and SAM2 segmentation.
All detection and tracking parameters are configured in tracker_params.yaml.

Usage Example:
    # Launch with default configuration
    ros2 launch navibot_grounded_sam2 grounded_sam2_tracker.launch.py
    
    # Launch with custom configuration file
    ros2 launch navibot_grounded_sam2 grounded_sam2_tracker.launch.py \
        config_file:=/path/to/custom_config.yaml
    
    # Launch with simulation time
    ros2 launch navibot_grounded_sam2 grounded_sam2_tracker.launch.py use_sim_time:=true

Note:
    To modify detection parameters (prompt, thresholds, topics, etc.), 
    edit config/tracker_params.yaml or provide a custom config file.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_conda_base():
    """Find conda installation base directory.
    
    Returns:
        str: Path to conda base directory
        
    Raises:
        RuntimeError: If conda installation cannot be found
    """
    # Try to get conda base from CONDA_EXE environment variable
    conda_exe = os.environ.get('CONDA_EXE')
    if conda_exe:
        # Extract conda base from CONDA_EXE (e.g., /home/user/miniconda3/bin/conda -> /home/user/miniconda3)
        return os.path.dirname(os.path.dirname(conda_exe))
    
    # Try common conda installation paths
    home = os.path.expanduser('~')
    for possible_conda in [
        os.path.join(home, 'miniconda3'),
        os.path.join(home, 'anaconda3'),
        '/opt/conda',
        '/usr/local/conda',
    ]:
        if os.path.exists(possible_conda):
            return possible_conda
    
    raise RuntimeError(
        "Cannot find conda installation. Please set CONDA_EXE environment variable or "
        "install conda in a standard location (~/miniconda3, ~/anaconda3, /opt/conda)"
    )


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for GroundedSAM2 tracker.
    
    Returns:
        LaunchDescription object containing all launch actions
    """
    # Get package directory
    pkg_dir = get_package_share_directory('navibot_grounded_sam2')
    
    # Default config file path
    default_config = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')

    # Resolve gsam2 conda env paths
    conda_base = find_conda_base()
    python_path = os.path.join(conda_base, 'envs', 'gsam2', 'bin', 'python')
    conda_env_site_packages = os.path.join(
        conda_base, 'envs', 'gsam2', 'lib', 'python3.10', 'site-packages'
    )

    # Setup Python path for conda environment to find custom modules.
    # Prepend the gsam2 conda env site-packages so its torch/torchvision
    # shadow the incompatible torch in ~/.local/site-packages. We deliberately
    # do NOT set PYTHONNOUSERSITE=1, because ~/.local is still needed as a
    # fallback for packages the gsam2 env does not install (e.g. 'packaging'
    # required by matplotlib).
    package_install_path = pkg_dir.replace('/share/navibot_grounded_sam2', '')
    custom_path = os.path.join(
        package_install_path,
        'lib/python3.10/site-packages/navibot_grounded_sam2',
    )

    current_pythonpath = os.environ.get('PYTHONPATH', '')
    os.environ['PYTHONPATH'] = (
        f"{custom_path}:{conda_env_site_packages}:{current_pythonpath}"
    )

    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to tracker configuration YAML file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Set Hugging Face mirror endpoint for faster model downloads in China
    os.environ['HF_ENDPOINT'] = 'https://hf-mirror.com'
    
    # Create tracker node
    tracker_node = Node(
        package='navibot_grounded_sam2',
        executable='grounded_sam2_tracker',
        name='grounded_sam2_tracker',
        output='screen',
        prefix=[python_path],
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_use_sim_time,
        tracker_node,
    ])

