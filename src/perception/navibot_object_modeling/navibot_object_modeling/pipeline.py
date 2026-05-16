#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Processing pipeline for object modeling node.

This module extracts the per-frame processing into a cohesive pipeline
that orchestrates transform, projection, segmentation, grouping, OBB
estimation, fusion/modeling, and message building.
"""

from typing import Optional, Tuple, Dict, Any
import numpy as np
import numpy.typing as npt

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from .core import ObjectState, fit_obb_2d, transform_points_manual
from .utils import (
    project_points_to_image,
    pointcloud2_to_array,
    query_object_ids,
    filter_segmented_points,
    apply_radius_outlier_removal,
    apply_dbscan_to_accumulated_points,
    model_objects_with_accumulation,
    create_xyzrgb_pointcloud,
)


class Pipeline:
    """Per-frame processing pipeline that reuses node helpers for behavior parity."""

    def __init__(self, node: Any) -> None:
        self.node = node

    def run(self, pc_msg: PointCloud2, gsam_msg: Any) -> Tuple[Optional[PointCloud2], Optional[Any], Optional[Any]]:
        """Run one processing step; returns modeled_pc, marker_array, obb_info_array."""
        # Extract and validate inputs
        label_image = self.node._extract_label_image(gsam_msg)
        transform = self.node._get_cached_transform(pc_msg.header.stamp)

        if transform is None:
            self.node.get_logger().warn(
                f"Failed to get transform, skipping frame {self.node.frame_count}"
            )
            return None, None, None

        points = pointcloud2_to_array(pc_msg)
        if points is None or len(points) == 0:
            self.node.get_logger().warn(
                f"Empty point cloud, skipping frame {self.node.frame_count}"
            )
            return None, None, None

        # Transform input point cloud to pointcloud_frame if needed
        if pc_msg.header.frame_id != self.node.pointcloud_frame:
            transform_to_pointcloud_frame = self.node._get_transform_to_pointcloud_frame(
                pc_msg.header.frame_id, pc_msg.header.stamp
            )
            if transform_to_pointcloud_frame is None:
                self.node.get_logger().warn(
                    f"Failed to get transform from {pc_msg.header.frame_id} to {self.node.pointcloud_frame}, skipping frame {self.node.frame_count}"
                )
                return None, None, None
            points = transform_points_manual(points, transform_to_pointcloud_frame)

        # Transform and project points
        points_camera = transform_points_manual(points, transform)
        pixel_coords, valid_mask = project_points_to_image(
            points_camera,
            self.node.camera_matrix,
            self.node.dist_coeffs,
            self.node.image_width,
            self.node.image_height,
            self.node.min_depth,
            self.node.max_depth,
        )

        # Segment and filter points
        object_ids = query_object_ids(pixel_coords, label_image, valid_mask)
        filtered_points, filtered_colors, filtered_object_ids = filter_segmented_points(
            points, object_ids, gsam_msg.ids, self.node.min_object_points
        )

        if self.node.enable_cluster_filtering and len(filtered_points) > 0:
            filtered_points, filtered_colors, filtered_object_ids = apply_radius_outlier_removal(
                filtered_points,
                filtered_colors,
                filtered_object_ids,
                self.node.radius_outlier_removal_radius,
                self.node.radius_outlier_removal_min_neighbors,
            )

        # Group by id with classes
        grouped_objects: Dict[int, Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32], str]] = {}
        unique_ids = np.unique(filtered_object_ids)
        id_to_name = {}
        for i, obj_id in enumerate(gsam_msg.ids):
            if i < len(gsam_msg.class_names):
                id_to_name[obj_id] = gsam_msg.class_names[i]
        for obj_id in unique_ids:
            if obj_id > 0:
                mask = filtered_object_ids == obj_id
                object_name = id_to_name.get(obj_id, "unknown")
                grouped_objects[obj_id] = (
                    filtered_points[mask],
                    filtered_colors[mask],
                    filtered_object_ids[mask],
                    object_name,
                )

        # Current OBBs
        current_obbs: Dict[int, Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32], float]] = {}
        for oid, (obj_points, _, _, _) in grouped_objects.items():
            if len(obj_points) >= self.node.min_obb_points:
                obb_center, obb_size, obb_rotation = fit_obb_2d(
                    obj_points,
                    self.node.pointcloud_frame,
                    self.node.base_link_frame,
                    self.node.pointcloud_frame,
                    self.node.min_obb_points,
                    self.node.obb_height_min,
                    self.node.obb_height_max,
                    self.node.tf_buffer,
                    self.node.get_logger(),
                )
                if obb_center is not None and obb_size is not None and obb_rotation is not None:
                    current_obbs[oid] = (obb_center, obb_size, obb_rotation)

        # Tracking / fusion / modeling
        latest_marker_array = None
        latest_modeled_pc: PointCloud2

        if self.node.enable_cross_frame_association and self.node.object_modeling_tracker is not None:
            self.node.object_modeling_tracker.update_frame(self.node.frame_count)
            modeled_objects = self.node.object_modeling_tracker.associate_objects(grouped_objects, current_obbs)
            if self.node.enable_object_fusion:
                self.node.object_modeling_tracker.fuse_objects(self.node.fusion_iou_threshold)
                modeled_objects = self.node.object_modeling_tracker.modeled_objects
            apply_dbscan_to_accumulated_points(
                modeled_objects, self.node.cluster_eps, self.node.cluster_min_samples
            )
            if self.node.publish_obb_markers:
                latest_marker_array = model_objects_with_accumulation(
                    modeled_objects,
                    pc_msg.header,
                    self.node.pointcloud_frame,
                    self.node.min_obb_points,
                    self.node.obb_height_min,
                    self.node.obb_height_max,
                    self.node.base_link_frame,
                    self.node.obb_marker_scale,
                    self.node.obb_marker_height,
                    self.node.obb_marker_lifetime,
                    self.node.object_color_map,
                    self.node.tf_buffer,
                    self.node.get_logger(),
                    fit_obb_2d,
                )
            # Build modeled pc
            if not modeled_objects:
                latest_modeled_pc = create_xyzrgb_pointcloud(
                    np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8), pc_msg.header
                )
            else:
                all_points = []
                all_colors = []
                for _, obj_state in modeled_objects.items():
                    if len(obj_state.accumulated_points) > 0:
                        all_points.append(obj_state.accumulated_points)
                        if obj_state.is_active:
                            all_colors.append(obj_state.accumulated_colors)
                        else:
                            all_colors.append(
                                np.full((len(obj_state.accumulated_points), 3), 255, dtype=np.uint8)
                            )
                if not all_points:
                    latest_modeled_pc = create_xyzrgb_pointcloud(
                        np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8), pc_msg.header
                    )
                else:
                    latest_modeled_pc = create_xyzrgb_pointcloud(
                        np.vstack(all_points), np.vstack(all_colors), pc_msg.header
                    )
        else:
            # Without cross-frame association
            all_points = []
            all_colors = []
            for obj_points, obj_colors, _, _ in grouped_objects.values():
                all_points.append(obj_points)
                all_colors.append(obj_colors)
            if all_points:
                latest_modeled_pc = create_xyzrgb_pointcloud(
                    np.vstack(all_points), np.vstack(all_colors), pc_msg.header
                )
            else:
                latest_modeled_pc = create_xyzrgb_pointcloud(
                    np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8), pc_msg.header
                )

        # OBB info array
        latest_obb_info_array = None
        if (
            self.node.publish_obb_info
            and self.node.enable_cross_frame_association
            and self.node.object_modeling_tracker is not None
        ):
            latest_obb_info_array = self.node._create_obb_info_array(pc_msg.header)

        return latest_modeled_pc, latest_marker_array, latest_obb_info_array


