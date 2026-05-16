#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point cloud processing utilities for object modeling.

This module contains functions for point cloud filtering, clustering,
and segmentation operations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from ..core.data_structures import ObjectState
import colorsys
import numpy as np
import numpy.typing as npt
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors


def apply_radius_outlier_removal(
    points: npt.NDArray[np.float32], 
    colors: npt.NDArray[np.uint8], 
    object_ids: npt.NDArray[np.int32],
    radius: float,
    min_neighbors: int
) -> Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]:
    """
    Apply radius-based outlier removal to filter noise points for each object.

    Args:
        points: Input points
        colors: Point colors
        object_ids: Object IDs for points
        radius: Search radius for neighbors
        min_neighbors: Minimum neighbors required

    Returns:
        Filtered points, colors, and object IDs
    """
    if len(points) == 0:
        return points, colors, object_ids

    unique_object_ids = get_valid_object_ids(object_ids)
    if len(unique_object_ids) == 0:
        return points, colors, object_ids

    filtered_data = process_objects_with_outlier_removal(points, colors, object_ids, unique_object_ids, radius, min_neighbors)
    return combine_filtered_data(filtered_data)


def get_valid_object_ids(object_ids: npt.NDArray[np.int32]) -> npt.NDArray[np.int32]:
    """Get unique object IDs excluding background."""
    unique_object_ids = np.unique(object_ids)
    return unique_object_ids[unique_object_ids > 0]


def process_objects_with_outlier_removal(
    points: npt.NDArray[np.float32],
    colors: npt.NDArray[np.uint8],
    object_ids: npt.NDArray[np.int32],
    unique_object_ids: npt.NDArray[np.int32],
    radius: float,
    min_neighbors: int
) -> List[Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]]:
    """Process each object with radius-based outlier removal."""
    filtered_data = []

    for obj_id in unique_object_ids:
        obj_points, obj_colors, obj_object_ids = extract_object_data(points, colors, object_ids, obj_id)

        if len(obj_points) < min_neighbors:
            continue

        filtered_obj_data = apply_outlier_removal_to_object(obj_id, obj_points, obj_colors, obj_object_ids, radius, min_neighbors)
        if filtered_obj_data is not None:
            filtered_data.append(filtered_obj_data)

    return filtered_data


def extract_object_data(
    points: npt.NDArray[np.float32],
    colors: npt.NDArray[np.uint8],
    object_ids: npt.NDArray[np.int32],
    obj_id: int,
) -> Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]:
    """Extract data for a specific object."""
    obj_mask = object_ids == obj_id
    return points[obj_mask], colors[obj_mask], object_ids[obj_mask]


def apply_outlier_removal_to_object(
    obj_id: int,
    obj_points: npt.NDArray[np.float32],
    obj_colors: npt.NDArray[np.uint8],
    obj_object_ids: npt.NDArray[np.int32],
    radius: float,
    min_neighbors: int
) -> Optional[Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]]:
    """Apply radius-based outlier removal to a single object."""
    try:
        valid_mask = find_valid_points_with_neighbors(obj_points, radius, min_neighbors)

        if np.any(valid_mask):
            return (obj_points[valid_mask], obj_colors[valid_mask], obj_object_ids[valid_mask])
        return None

    except Exception as e:
        # If outlier removal fails, return original data
        return (obj_points, obj_colors, obj_object_ids)


def find_valid_points_with_neighbors(obj_points: npt.NDArray[np.float32], radius: float, min_neighbors: int) -> npt.NDArray[np.bool_]:
    """Find points with sufficient neighbors within radius."""
    nbrs = NearestNeighbors(radius=radius, algorithm="ball_tree").fit(obj_points)
    _, indices = nbrs.radius_neighbors(obj_points)
    neighbor_counts = np.array([len(neighbors) - 1 for neighbors in indices])
    return neighbor_counts >= min_neighbors


def combine_filtered_data(
    filtered_data: List[Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]]
) -> Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]:
    """Combine filtered data from all objects."""
    if len(filtered_data) > 0:
        filtered_points = np.vstack([data[0] for data in filtered_data])
        filtered_colors = np.vstack([data[1] for data in filtered_data])
        filtered_object_ids = np.concatenate([data[2] for data in filtered_data])
    else:
        filtered_points = np.empty((0, 3), dtype=np.float32)
        filtered_colors = np.empty((0, 3), dtype=np.uint8)
        filtered_object_ids = np.empty(0, dtype=np.int32)
    return filtered_points, filtered_colors, filtered_object_ids


def apply_dbscan_to_accumulated_points(
    modeled_objects: Dict[int, 'ObjectState'], 
    cluster_eps: float, 
    cluster_min_samples: int
) -> None:
    """Apply DBSCAN clustering to accumulated points for each object."""
    for obj_id, obj_state in modeled_objects.items():
        # Only apply DBSCAN to warmed up objects
        if not obj_state.is_warmed_up:
            continue
            
        if len(obj_state.accumulated_points) < cluster_min_samples:
            # Skip objects with insufficient points
            continue

        try:
            # Apply DBSCAN clustering to accumulated points
            clustering = DBSCAN(eps=cluster_eps, min_samples=cluster_min_samples).fit(
                obj_state.accumulated_points
            )

            labels = clustering.labels_

            # Find all valid clusters (excluding noise points with label -1)
            unique_labels = np.unique(labels)
            valid_labels = unique_labels[unique_labels != -1]

            if len(valid_labels) == 0:
                # No valid clusters found, clear accumulated data
                obj_state.accumulated_points = np.empty((0, 3), dtype=np.float32)
                obj_state.accumulated_colors = np.empty((0, 3), dtype=np.uint8)
                obj_state.accumulated_object_ids = np.empty(0, dtype=np.int32)
                continue

            # Keep all valid clusters (non-noise points)
            valid_cluster_mask = np.isin(labels, valid_labels)

            # Update accumulated data with filtered points
            obj_state.accumulated_points = obj_state.accumulated_points[valid_cluster_mask]
            obj_state.accumulated_colors = obj_state.accumulated_colors[valid_cluster_mask]
            obj_state.accumulated_object_ids = obj_state.accumulated_object_ids[valid_cluster_mask]

        except Exception as e:
            # Keep original accumulated data if clustering fails
            pass


def filter_segmented_points(
    points: npt.NDArray[np.float32], 
    object_ids: npt.NDArray[np.int32], 
    detected_ids: list,
    min_object_points: int
) -> Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32]]:
    """Filter points to keep only detected object points."""
    # Filter points belonging to detected objects
    object_mask = object_ids > 0
    unique_ids, counts = np.unique(object_ids[object_mask], return_counts=True)
    valid_objects = unique_ids[counts >= min_object_points]
    valid_object_mask = np.isin(object_ids, valid_objects)

    filtered_points = points[valid_object_mask]
    filtered_object_ids = object_ids[valid_object_mask]
    filtered_colors = assign_colors(filtered_object_ids, {})

    return filtered_points, filtered_colors, filtered_object_ids


def assign_colors(object_ids: npt.NDArray[np.int32], object_color_map: Dict[int, Tuple[int, int, int]]) -> npt.NDArray[np.uint8]:
    """Assign RGB colors to points based on object IDs."""
    colors = np.zeros((len(object_ids), 3), dtype=np.uint8)

    # Get unique object IDs and generate colors for new ones
    unique_ids = np.unique(object_ids)
    for obj_id in unique_ids:
        if obj_id not in object_color_map:
            object_color_map[obj_id] = generate_color(obj_id)

    # Vectorized color assignment
    for obj_id in unique_ids:
        mask = object_ids == obj_id
        colors[mask] = object_color_map[obj_id]

    return colors


def generate_color(object_id: int) -> Tuple[int, int, int]:
    """Generate a unique color for an object ID using HSV color space."""
    # Generate hue using golden angle distribution
    hue = (object_id * 137.508) % 360.0
    h, s, v = hue / 360.0, 0.85, 0.95  # Default saturation and value
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return (int(r * 255), int(g * 255), int(b * 255))


def get_object_color(object_id: int, object_color_map: Dict[int, Tuple[int, int, int]]) -> Tuple[int, int, int]:
    """Get color for object ID."""
    if object_id not in object_color_map:
        object_color_map[object_id] = generate_color(object_id)
    return object_color_map[object_id]
