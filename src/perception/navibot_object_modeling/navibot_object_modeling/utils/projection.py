#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Projection and coordinate transformation utilities for object modeling.

This module contains functions for point cloud projection, coordinate
transformation, and image processing operations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import Optional, Tuple
import cv2
import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.duration import Duration
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer

from ..core.geometry import get_transform


def project_points_to_image(
    points_camera: npt.NDArray[np.float32],
    camera_matrix: npt.NDArray[np.float64],
    dist_coeffs: npt.NDArray[np.float64],
    image_width: int,
    image_height: int,
    min_depth: float,
    max_depth: float
) -> Tuple[npt.NDArray[np.int32], npt.NDArray[np.bool_]]:
    """
    Project 3D points to 2D image plane using camera intrinsics.

    Args:
        points_camera: Points in camera frame
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        image_width: Image width in pixels
        image_height: Image height in pixels
        min_depth: Minimum depth for filtering
        max_depth: Maximum depth for filtering

    Returns:
        Tuple of (pixel_coords, valid_mask)
    """
    # Extract depth (Z coordinate in camera frame)
    depth = points_camera[:, 2]

    # Filter by depth range
    valid_depth = (depth > min_depth) & (depth < max_depth)

    # Project to normalized image plane
    # [u, v, 1]^T = K * [X, Y, Z]^T / Z
    points_homogeneous = points_camera / depth[:, np.newaxis]
    projected = (camera_matrix @ points_homogeneous.T).T

    # Apply distortion correction
    points_2d = projected[:, :2].reshape(-1, 1, 2).astype(np.float32)
    undistorted_points = cv2.undistortPoints(points_2d, camera_matrix, dist_coeffs, P=camera_matrix)
    u = undistorted_points[:, 0, 0].astype(np.int32)
    v = undistorted_points[:, 0, 1].astype(np.int32)

    # Check if within image bounds
    valid_u = (u >= 0) & (u < image_width)
    valid_v = (v >= 0) & (v < image_height)

    # Combine all validity checks
    valid_mask = valid_depth & valid_u & valid_v

    pixel_coords = np.stack([u, v], axis=1)

    return pixel_coords, valid_mask


def pointcloud2_to_array(pc_msg) -> Optional[npt.NDArray[np.float32]]:
    """
    Convert PointCloud2 message to numpy array.

    Args:
        pc_msg: PointCloud2 message

    Returns:
        Numpy array of points or None if conversion fails
    """
    try:
        # Read points directly as numpy array
        points = point_cloud2.read_points_numpy(pc_msg, field_names=("x", "y", "z"), skip_nans=True)

        if len(points) == 0:
            return None

        return points.astype(np.float32)

    except Exception as e:
        return None


def query_object_ids(
    pixel_coords: npt.NDArray[np.int32],
    label_image: npt.NDArray[np.uint16],
    valid_mask: npt.NDArray[np.bool_],
) -> npt.NDArray[np.int32]:
    """
    Query object IDs from label image for each point.

    Args:
        pixel_coords: Pixel coordinates
        label_image: Label image
        valid_mask: Valid pixel mask

    Returns:
        Object IDs for each point
    """
    object_ids = np.zeros(len(pixel_coords), dtype=np.int32)

    # Only query valid pixels using vectorized indexing
    valid_pixels = pixel_coords[valid_mask]
    if len(valid_pixels) > 0:
        # Vectorized query: image indexing is [row, col] = [v, u]
        object_ids[valid_mask] = label_image[valid_pixels[:, 1], valid_pixels[:, 0]].astype(np.int32)

    return object_ids
