#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Data structures for object modeling.

This module contains the core data classes used for object modeling,
including OBB information and object state tracking.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np
import numpy.typing as npt


@dataclass
class OBBInfo:
    """OBB information for object."""

    center: npt.NDArray[np.float32]  # OBB center [x, y] (map frame)
    size: npt.NDArray[np.float32]  # OBB size [length, width]
    rotation: float  # yaw angle in radians
    object_id: int  # Object ID
    class_name: str  # Object class name


@dataclass
class ObjectState:
    """Object state for cross-frame tracking."""

    object_id: int
    accumulated_points: npt.NDArray[np.float32]
    accumulated_colors: npt.NDArray[np.uint8]
    accumulated_object_ids: npt.NDArray[np.int32]
    frame_count: int
    age: int
    is_active: bool
    last_obb_center: Optional[npt.NDArray[np.float32]] = None  # OBB center [x, y] in map frame
    last_obb_size: Optional[npt.NDArray[np.float32]] = None  # OBB size [length, width]
    last_obb_rotation: Optional[float] = None  # yaw angle in radians
    object_name: str = ""  # Object name for fusion compatibility
    warmup_frames: int = 0  # Number of consecutive frames with successful association
    is_warmed_up: bool = False  # Whether object has completed warmup phase


@dataclass
class ModelingParams:
    """Centralized parameters for object modeling node.

    This dataclass mirrors ROS parameters declared in the node and
    enables passing a single immutable config object across modules.
    """

    # Camera parameters
    camera_matrix: npt.NDArray[np.float64]
    dist_coeffs: npt.NDArray[np.float64]
    image_width: int
    image_height: int

    # Topic parameters
    pointcloud_topic: str
    grounded_sam_topic: str
    output_topic: str
    sync_queue_size: int
    sync_slop: float

    # Frame parameters
    pointcloud_frame: str
    camera_frame: str
    base_link_frame: str
    tf_timeout: float

    # Segmentation parameters
    min_depth: float
    max_depth: float
    min_object_points: int
    color_saturation: float
    color_value: float
    enable_debug_logging: bool
    enable_cluster_filtering: bool
    cluster_eps: float
    cluster_min_samples: int
    radius_outlier_removal_radius: float
    radius_outlier_removal_min_neighbors: int

    # Modeling parameters
    min_obb_points: int
    obb_marker_scale: float
    publish_obb_markers: bool
    obb_marker_topic: str
    obb_marker_lifetime: float
    enable_cross_frame_association: bool
    max_accumulation_frames: int
    iou_threshold: float
    max_object_age: int
    enable_object_fusion: bool
    fusion_iou_threshold: float
    accumulation_iou_threshold: float
    warmup_frames_threshold: int
    publish_rate: float
    publish_obb_info: bool
    obb_info_topic: str
    obb_height_min: float
    obb_height_max: float
    obb_marker_height: float

    @staticmethod
    def from_node(node) -> "ModelingParams":
        """Create params object by reading from a ROS2 node."""
        # Camera
        camera_matrix = np.array(node.get_parameter("camera_matrix").value).reshape(3, 3)
        dist_coeffs = np.array(node.get_parameter("dist_coeffs").value)
        image_width = int(node.get_parameter("image_width").value)
        image_height = int(node.get_parameter("image_height").value)

        # Topics
        pointcloud_topic = str(node.get_parameter("pointcloud_topic").value)
        grounded_sam_topic = str(node.get_parameter("grounded_sam_topic").value)
        output_topic = str(node.get_parameter("output_topic").value)
        sync_queue_size = int(node.get_parameter("sync_queue_size").value)
        sync_slop = float(node.get_parameter("sync_slop").value)

        # Frames
        pointcloud_frame = str(node.get_parameter("pointcloud_frame").value)
        camera_frame = str(node.get_parameter("camera_frame").value)
        base_link_frame = str(node.get_parameter("base_link_frame").value)
        tf_timeout = float(node.get_parameter("tf_timeout").value)

        # Segmentation
        min_depth = float(node.get_parameter("min_depth").value)
        max_depth = float(node.get_parameter("max_depth").value)
        min_object_points = int(node.get_parameter("min_object_points").value)
        color_saturation = float(node.get_parameter("color_saturation").value)
        color_value = float(node.get_parameter("color_value").value)
        enable_debug_logging = bool(node.get_parameter("enable_debug_logging").value)
        enable_cluster_filtering = bool(node.get_parameter("enable_cluster_filtering").value)
        cluster_eps = float(node.get_parameter("cluster_eps").value)
        cluster_min_samples = int(node.get_parameter("cluster_min_samples").value)
        radius_outlier_removal_radius = float(node.get_parameter("radius_outlier_removal_radius").value)
        radius_outlier_removal_min_neighbors = int(node.get_parameter("radius_outlier_removal_min_neighbors").value)

        # Modeling
        min_obb_points = int(node.get_parameter("min_obb_points").value)
        obb_marker_scale = float(node.get_parameter("obb_marker_scale").value)
        publish_obb_markers = bool(node.get_parameter("publish_obb_markers").value)
        obb_marker_topic = str(node.get_parameter("obb_marker_topic").value)
        obb_marker_lifetime = float(node.get_parameter("obb_marker_lifetime").value)
        enable_cross_frame_association = bool(node.get_parameter("enable_cross_frame_association").value)
        max_accumulation_frames = int(node.get_parameter("max_accumulation_frames").value)
        iou_threshold = float(node.get_parameter("iou_threshold").value)
        max_object_age = int(node.get_parameter("max_object_age").value)
        enable_object_fusion = bool(node.get_parameter("enable_object_fusion").value)
        fusion_iou_threshold = float(node.get_parameter("fusion_iou_threshold").value)
        accumulation_iou_threshold = float(node.get_parameter("accumulation_iou_threshold").value)
        warmup_frames_threshold = int(node.get_parameter("warmup_frames_threshold").value)
        publish_rate = float(node.get_parameter("publish_rate").value)
        publish_obb_info = bool(node.get_parameter("publish_obb_info").value)
        obb_info_topic = str(node.get_parameter("obb_info_topic").value)
        obb_height_min = float(node.get_parameter("obb_height_min").value)
        obb_height_max = float(node.get_parameter("obb_height_max").value)
        obb_marker_height = float(node.get_parameter("obb_marker_height").value)

        return ModelingParams(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_width=image_width,
            image_height=image_height,
            pointcloud_topic=pointcloud_topic,
            grounded_sam_topic=grounded_sam_topic,
            output_topic=output_topic,
            sync_queue_size=sync_queue_size,
            sync_slop=sync_slop,
            pointcloud_frame=pointcloud_frame,
            camera_frame=camera_frame,
            base_link_frame=base_link_frame,
            tf_timeout=tf_timeout,
            min_depth=min_depth,
            max_depth=max_depth,
            min_object_points=min_object_points,
            color_saturation=color_saturation,
            color_value=color_value,
            enable_debug_logging=enable_debug_logging,
            enable_cluster_filtering=enable_cluster_filtering,
            cluster_eps=cluster_eps,
            cluster_min_samples=cluster_min_samples,
            radius_outlier_removal_radius=radius_outlier_removal_radius,
            radius_outlier_removal_min_neighbors=radius_outlier_removal_min_neighbors,
            min_obb_points=min_obb_points,
            obb_marker_scale=obb_marker_scale,
            publish_obb_markers=publish_obb_markers,
            obb_marker_topic=obb_marker_topic,
            obb_marker_lifetime=obb_marker_lifetime,
            enable_cross_frame_association=enable_cross_frame_association,
            max_accumulation_frames=max_accumulation_frames,
            iou_threshold=iou_threshold,
            max_object_age=max_object_age,
            enable_object_fusion=enable_object_fusion,
            fusion_iou_threshold=fusion_iou_threshold,
            accumulation_iou_threshold=accumulation_iou_threshold,
            warmup_frames_threshold=warmup_frames_threshold,
            publish_rate=publish_rate,
            publish_obb_info=publish_obb_info,
            obb_info_topic=obb_info_topic,
            obb_height_min=obb_height_min,
            obb_height_max=obb_height_max,
            obb_marker_height=obb_marker_height,
        )
