#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracking utilities for GroundedSAM2 tracker.

This module provides utility functions for point sampling and tracking operations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""
# Third-party libraries
import numpy as np


def sample_points_from_masks(masks: np.ndarray, num_points: int) -> np.ndarray:
    """Sample points from masks and return absolute coordinates.

    Args:
        masks: Array with shape (n, h, w) containing mask data.
        num_points: Number of points to sample from each mask.

    Returns:
        Array with shape (n, points, 2) containing sampled point coordinates.
    """
    n, _, _ = masks.shape
    points = []

    for i in range(n):
        # find the valid mask points
        indices = np.argwhere(masks[i] == 1)  
        # the output format of np.argwhere is (y, x) and the shape is (num_points, 2)
        # we should convert it to (x, y)
        indices = indices[:, ::-1]  # (num_points, [y x]) to (num_points, [x y])
        
        # import pdb; pdb.set_trace()
        if len(indices) == 0:
            # if there are no valid points, append an empty array
            points.append(np.array([]))
            continue
        
        # resampling if there's not enough points
        if len(indices) < num_points:
            sampled_indices = np.random.choice(len(indices), num_points, replace=True)
        else:
            sampled_indices = np.random.choice(len(indices), num_points, replace=False)
        
        sampled_points = indices[sampled_indices]
        points.append(sampled_points)

    # convert to np.array
    points = np.array(points, dtype=np.float32)
    return points
