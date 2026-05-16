#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualization utilities for object modeling.

This module contains functions for creating markers, point clouds,
and other visualization elements.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from ..core.data_structures import ObjectState
import numpy as np
import numpy.typing as npt
from builtin_interfaces.msg import Duration as BuiltinDuration
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from ..core.geometry import yaw_to_quat
from .pointcloud_processing import get_object_color


def create_xyzrgb_pointcloud(
    points: npt.NDArray[np.float32], 
    colors: npt.NDArray[np.uint8], 
    header: Header
) -> PointCloud2:
    """
    Create XYZRGB PointCloud2 message.

    Args:
        points: 3D points
        colors: RGB colors
        header: Message header

    Returns:
        PointCloud2 message
    """
    if len(points) == 0:
        # Return empty point cloud
        return point_cloud2.create_cloud_xyz32(header, np.empty((0, 3), dtype=np.float32))

    # Pack RGB into single uint32 value
    rgb_packed = (
        (colors[:, 0].astype(np.uint32) << 16)
        | (colors[:, 1].astype(np.uint32) << 8)
        | colors[:, 2].astype(np.uint32)
    )

    # Create structured array for XYZRGB
    cloud_data = np.zeros(
        len(points), dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.uint32)]
    )
    cloud_data["x"] = points[:, 0]
    cloud_data["y"] = points[:, 1]
    cloud_data["z"] = points[:, 2]
    cloud_data["rgb"] = rgb_packed

    # Use create_cloud to simplify creation
    return point_cloud2.create_cloud(
        header,
        [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ],
        cloud_data,
    )


def create_obb_marker(
    marker_id: int,
    object_id: int,
    center_2d: npt.NDArray[np.float32],
    size_2d: npt.NDArray[np.float32],
    yaw: float,
    header: Header,
    pointcloud_frame: str,
    obb_marker_scale: float,
    obb_marker_height: float,
    obb_marker_lifetime: float,
    object_color_map: Dict[int, Tuple[int, int, int]]
) -> Marker:
    """
    Create OBB marker for visualization with 2D data.

    Args:
        marker_id: Marker ID
        object_id: Object ID
        center_2d: 2D center coordinates
        size_2d: 2D size
        yaw: Yaw angle
        header: Message header
        pointcloud_frame: Point cloud frame
        obb_marker_scale: Marker scale
        obb_marker_height: Marker height
        obb_marker_lifetime: Marker lifetime
        object_color_map: Object color mapping

    Returns:
        Marker message
    """
    marker = Marker()
    marker.header = header
    # Ensure marker frame is set to pointcloud_frame since OBB data is in pointcloud_frame
    marker.header.frame_id = pointcloud_frame
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Set pose with 2D center and fixed height
    marker.pose.position.x = float(center_2d[0])
    marker.pose.position.y = float(center_2d[1])
    marker.pose.position.z = obb_marker_height / 2.0  # Center at half height

    # Set orientation from yaw angle
    quat = yaw_to_quat(yaw)
    marker.pose.orientation.x = float(quat[0])
    marker.pose.orientation.y = float(quat[1])
    marker.pose.orientation.z = float(quat[2])
    marker.pose.orientation.w = float(quat[3])

    # Set scale with 2D size and fixed height
    marker.scale.x = float(size_2d[0] * obb_marker_scale)
    marker.scale.y = float(size_2d[1] * obb_marker_scale)
    marker.scale.z = float(obb_marker_height * obb_marker_scale)

    # Set color based on object ID
    color = get_object_color(object_id, object_color_map)
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0
    marker.color.a = 0.6  # Semi-transparent

    # Set lifetime
    marker.lifetime = BuiltinDuration(
        sec=int(obb_marker_lifetime),
        nanosec=int((obb_marker_lifetime - int(obb_marker_lifetime)) * 1e9),
    )

    return marker


def model_objects_with_accumulation(
    modeled_objects: Dict[int, 'ObjectState'], 
    header: Header,
    pointcloud_frame: str,
    min_obb_points: int,
    obb_height_min: float,
    obb_height_max: float,
    base_link_frame: str,
    obb_marker_scale: float,
    obb_marker_height: float,
    obb_marker_lifetime: float,
    object_color_map: Dict[int, Tuple[int, int, int]],
    tf_buffer,
    logger,
    fit_obb_2d_func
) -> MarkerArray:
    """
    Model objects using accumulated point clouds.

    Args:
        modeled_objects: Dictionary of modeled objects
        header: Message header
        pointcloud_frame: Point cloud frame
        min_obb_points: Minimum points for OBB
        obb_height_min: Minimum height for filtering
        obb_height_max: Maximum height for filtering
        base_link_frame: Base link frame
        obb_marker_scale: Marker scale
        obb_marker_height: Marker height
        obb_marker_lifetime: Marker lifetime
        object_color_map: Object color mapping
        tf_buffer: TF buffer
        logger: Logger instance
        fit_obb_2d_func: Function to fit OBB

    Returns:
        MarkerArray message
    """
    # Update OBB information for all objects using latest available transform
    update_object_obbs(modeled_objects, header.frame_id, min_obb_points, obb_height_min, obb_height_max, base_link_frame, tf_buffer, logger, fit_obb_2d_func)

    marker_array = MarkerArray()
    marker_id = 0

    # Create markers for all objects (both active and inactive)
    for obj_id, obj_state in modeled_objects.items():
        # Skip objects that are not warmed up
        if not obj_state.is_warmed_up:
            continue
            
        # Skip objects without valid OBB
        if (
            obj_state.last_obb_center is None
            or obj_state.last_obb_size is None
            or obj_state.last_obb_rotation is None
        ):
            continue

        # Create OBB marker using the stored OBB data
        marker = create_obb_marker(
            marker_id,
            obj_id,
            obj_state.last_obb_center,
            obj_state.last_obb_size,
            obj_state.last_obb_rotation,
            header,
            pointcloud_frame,
            obb_marker_scale,
            obb_marker_height,
            obb_marker_lifetime,
            object_color_map
        )

        # Set color based on object activity
        if not obj_state.is_active:
            # White color for inactive objects
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.3  # More transparent for inactive objects

        marker_array.markers.append(marker)
        marker_id += 1

    return marker_array


def update_object_obbs(
    modeled_objects: Dict[int, 'ObjectState'], 
    frame_id: str,
    min_obb_points: int,
    obb_height_min: float,
    obb_height_max: float,
    base_link_frame: str,
    tf_buffer,
    logger,
    fit_obb_2d_func
) -> None:
    """Update OBB information for all modeled objects using accumulated points."""
    for _, obj_state in modeled_objects.items():
        if len(obj_state.accumulated_points) < min_obb_points:
            # Skip fitting if not enough points - preserve existing OBB
            continue

        # Fit 2D OBB using accumulated points
        obb_center, obb_size, obb_rotation = fit_obb_2d_func(
            obj_state.accumulated_points, 
            frame_id,
            base_link_frame,
            frame_id,  # pointcloud_frame
            min_obb_points,
            obb_height_min,
            obb_height_max,
            tf_buffer,
            logger
        )

        if obb_center is not None and obb_size is not None and obb_rotation is not None:
            # Store OBB information for use in both markers and constraints
            obj_state.last_obb_center = obb_center.copy()
            obj_state.last_obb_size = obb_size.copy()
            obj_state.last_obb_rotation = obb_rotation.copy()
        # If fitting failed, preserve existing OBB (no action needed)
