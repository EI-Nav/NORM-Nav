#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Core modules for object modeling.

This package contains the core functionality for object modeling including
data structures, tracking algorithms, and geometric computations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from .data_structures import OBBInfo, ObjectState
from .tracker import ObjectModelingTracker
from .geometry import (
    calculate_iou,
    fit_obb_2d,
    quat_to_matrix,
    yaw_to_quat,
    transform_points_manual,
)

__all__ = [
    "OBBInfo",
    "ObjectState", 
    "ObjectModelingTracker",
    "calculate_iou",
    "fit_obb_2d",
    "quat_to_matrix",
    "yaw_to_quat",
    "transform_points_manual",
]
