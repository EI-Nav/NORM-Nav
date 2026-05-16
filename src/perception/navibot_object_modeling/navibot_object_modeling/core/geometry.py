#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Geometric computation utilities for object modeling.

This module contains functions for geometric calculations including
OBB fitting, IOU computation, and coordinate transformations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import Optional, Tuple, Any
import numpy as np
import numpy.typing as npt
import rclpy
from tf2_ros import Buffer


def calculate_iou(
    center1: npt.NDArray[np.float32],
    size1: npt.NDArray[np.float32],
    center2: npt.NDArray[np.float32],
    size2: npt.NDArray[np.float32],
) -> float:
    """
    Calculate partial IOU between two 2D OBBs in map frame.
    Partial IOU = intersection / min(area1, area2)

    Args:
        center1: Center of first OBB [x, y] (map frame)
        size1: Size of first OBB [length, width]
        center2: Center of second OBB [x, y] (map frame)
        size2: Size of second OBB [length, width]

    Returns:
        Partial IOU value between 0.0 and 1.0
    """
    # Calculate 2D bounding boxes
    min1, max1 = center1 - size1 / 2.0, center1 + size1 / 2.0
    min2, max2 = center2 - size2 / 2.0, center2 + size2 / 2.0

    # Calculate intersection using np.clip for cleaner code
    intersection_min = np.maximum(min1, min2)
    intersection_max = np.minimum(max1, max2)
    intersection_size = np.clip(intersection_max - intersection_min, 0, None)
    intersection_area = np.prod(intersection_size)

    # Calculate areas and return IOU
    areas = np.prod(size1), np.prod(size2)
    min_area = min(areas)

    return intersection_area / min_area if min_area > 0 else 0.0


def fit_obb_2d(
    points: npt.NDArray[np.float32],
    source_frame: str,
    base_link_frame: str,
    _pointcloud_frame: str,  # Renamed to indicate it's unused
    min_obb_points: int,
    obb_height_min: float,
    obb_height_max: float,
    tf_buffer: Buffer,
    logger,
) -> Tuple[Optional[npt.NDArray[np.float32]], Optional[npt.NDArray[np.float32]], Optional[float]]:
    """
    Fit 2D OBB to points in base_link frame, then transform to pointcloud_frame.

    Args:
        points: Input points
        source_frame: Source frame for points
        base_link_frame: Base link frame
        pointcloud_frame: Point cloud frame
        min_obb_points: Minimum points required for OBB fitting
        obb_height_min: Minimum height for filtering
        obb_height_max: Maximum height for filtering
        tf_buffer: TF buffer for transformations
        logger: Logger instance

    Returns:
        Tuple of (center, size, rotation) or (None, None, None) if fitting fails
    """
    if len(points) < min_obb_points:
        return None, None, None

    # Filter points by height range
    height_mask = (points[:, 2] >= obb_height_min) & (points[:, 2] <= obb_height_max)
    filtered_points = points[height_mask]

    if len(filtered_points) < min_obb_points:
        return None, None, None

    # Get transform from source_frame to base_link
    transform_to_base = get_transform(source_frame, base_link_frame, tf_buffer, logger)
    if transform_to_base is None:
        logger.warn(f"Failed to get transform from {source_frame} to {base_link_frame}")
        return None, None, None

    # Transform points to base_link frame
    points_base = transform_points_manual(filtered_points, transform_to_base)

    # Compute 2D AABB in base_link frame (only x, y coordinates)
    xy_points = points_base[:, :2]  # Only x, y coordinates
    min_coords = np.min(xy_points, axis=0)
    max_coords = np.max(xy_points, axis=0)

    center_2d_base = (min_coords + max_coords) / 2.0
    size_2d = max_coords - min_coords

    # Ensure minimum size to avoid zero-size boxes
    min_size = 0.1  # 10cm minimum size
    size_2d = np.maximum(size_2d, min_size)

    # Calculate inverse transform from base_link to pointcloud_frame using matrix inverse
    transform_base_to_pointcloud = np.linalg.inv(transform_to_base)

    # Transform center from base_link to pointcloud_frame
    center_3d_base = np.array([center_2d_base[0], center_2d_base[1], 0.0], dtype=np.float32)
    center_3d_pointcloud = transform_points_manual(center_3d_base.reshape(1, 3), transform_base_to_pointcloud)
    center_2d_pointcloud = center_3d_pointcloud[0, :2]

    # Extract yaw angle from base_link to pointcloud_frame transform matrix
    rotation_matrix = transform_base_to_pointcloud[:3, :3]
    yaw_base_to_pointcloud = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return center_2d_pointcloud, size_2d, yaw_base_to_pointcloud


def quat_to_matrix(quat: list) -> npt.NDArray[np.float32]:
    """Convert quaternion to 4x4 homogeneous transformation matrix."""
    x, y, z, w = quat
    
    # Normalize quaternion
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm > 0:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    # Convert to rotation matrix
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ], dtype=np.float32)
    
    return R


def yaw_to_quat(yaw: float) -> npt.NDArray[np.float32]:
    """Convert yaw angle to quaternion [x, y, z, w]."""
    half_yaw = yaw / 2.0
    return np.array([0.0, 0.0, np.sin(half_yaw), np.cos(half_yaw)], dtype=np.float32)


def transform_points_manual(
    points: npt.NDArray[np.float32], transform_matrix: npt.NDArray[np.float32]
) -> npt.NDArray[np.float32]:
    """Transform points using 4x4 homogeneous transformation matrix."""
    # Convert points to homogeneous coordinates [x, y, z, 1]
    points_homogeneous = np.column_stack([points, np.ones(points.shape[0], dtype=np.float32)])
    
    # Apply transformation: p_target = T * p_source
    points_transformed_homogeneous = (transform_matrix @ points_homogeneous.T).T
    
    # Convert back to 3D coordinates
    points_transformed = points_transformed_homogeneous[:, :3]
    
    return points_transformed


def get_transform(
    source_frame: str, 
    target_frame: str, 
    tf_buffer: Buffer, 
    logger: Any,
    timeout: float = 0.1
) -> Optional[npt.NDArray[np.float32]]:
    """Get transform from TF2 buffer as 4x4 homogeneous transformation matrix."""
    try:
        from rclpy.duration import Duration
        transform_stamped = tf_buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=timeout)
        )
        
        # Extract translation and rotation
        t = transform_stamped.transform.translation
        r = transform_stamped.transform.rotation
        
        # Convert quaternion to rotation matrix
        quat = [r.x, r.y, r.z, r.w]
        rotation_matrix = quat_to_matrix(quat)
        
        # Create 4x4 homogeneous transformation matrix
        transform_matrix = np.eye(4, dtype=np.float32)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = [t.x, t.y, t.z]
        
        return transform_matrix
        
    except (Exception, RuntimeError) as e:
        logger.warn(f"TF lookup failed ({source_frame} -> {target_frame}): {e}")
        return None


def get_transform_at_time(
    source_frame: str, 
    target_frame: str, 
    timestamp,
    tf_buffer: Buffer, 
    logger: Any,
    timeout: float = 0.1
) -> Optional[npt.NDArray[np.float32]]:
    """Get transform from TF2 buffer at specific timestamp as 4x4 homogeneous transformation matrix."""
    try:
        from rclpy.duration import Duration
        from rclpy.time import Time
        
        # Convert timestamp to ROS2 Time object
        if hasattr(timestamp, 'sec') and hasattr(timestamp, 'nanosec'):
            # It's a builtin_interfaces.msg.Time, convert to seconds
            seconds = float(timestamp.sec) + float(timestamp.nanosec) / 1e9
            ros_time = Time(seconds=seconds)
        else:
            # Assume it's already a ROS2 Time object
            ros_time = timestamp
            
        transform_stamped = tf_buffer.lookup_transform(
            target_frame, source_frame, ros_time, timeout=Duration(seconds=timeout)
        )
        
        # Extract translation and rotation
        t = transform_stamped.transform.translation
        r = transform_stamped.transform.rotation
        
        # Convert quaternion to rotation matrix
        quat = [r.x, r.y, r.z, r.w]
        rotation_matrix = quat_to_matrix(quat)
        
        # Create 4x4 homogeneous transformation matrix
        transform_matrix = np.eye(4, dtype=np.float32)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = [t.x, t.y, t.z]
        
        return transform_matrix
        
    except (Exception, RuntimeError) as e:
        logger.warn(f"TF lookup at time failed ({source_frame} -> {target_frame}): {e}")
        return None
