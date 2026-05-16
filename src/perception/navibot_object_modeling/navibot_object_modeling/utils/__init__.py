#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utility modules for object modeling.

This package contains utility functions for point cloud processing,
projection, and visualization.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from .pointcloud_processing import (
    apply_radius_outlier_removal,
    apply_dbscan_to_accumulated_points,
    filter_segmented_points,
    assign_colors,
    generate_color,
)
from .projection import (
    project_points_to_image,
    get_transform,
    pointcloud2_to_array,
    query_object_ids,
)
from .visualization import (
    create_obb_marker,
    model_objects_with_accumulation,
    create_xyzrgb_pointcloud,
    get_object_color,
)

__all__ = [
    "apply_radius_outlier_removal",
    "apply_dbscan_to_accumulated_points", 
    "filter_segmented_points",
    "assign_colors",
    "generate_color",
    "project_points_to_image",
    "get_transform",
    "pointcloud2_to_array",
    "query_object_ids",
    "create_obb_marker",
    "model_objects_with_accumulation",
    "create_xyzrgb_pointcloud",
    "get_object_color",
]
