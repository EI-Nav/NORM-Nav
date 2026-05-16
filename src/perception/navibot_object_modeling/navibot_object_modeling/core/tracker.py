#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Object modeling tracker for cross-frame object tracking and accumulation.

This module contains the ObjectModelingTracker class that handles
cross-frame object association, accumulation, and fusion.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import Dict, Optional, Tuple, Callable
import numpy as np
import numpy.typing as npt

from .data_structures import ObjectState
from .geometry import calculate_iou


class ObjectModelingTracker:
    """Cross-frame object modeling tracker with accumulation and IOU-based association."""

    def __init__(
        self, 
        max_accumulation_frames: int, 
        iou_threshold: float, 
        max_object_age: int, 
        accumulation_iou_threshold: float, 
        warmup_frames_threshold: int, 
        fit_obb_callback: Optional[Callable] = None
    ) -> None:   
        """
        Initialize object modeling tracker.

        Args:
            max_accumulation_frames: Maximum frames to accumulate points (deprecated, kept for compatibility)
            iou_threshold: IOU threshold for association
            max_object_age: Maximum age for objects (frames)
            accumulation_iou_threshold: IOU threshold for accumulation (only accumulate when IOU < this threshold)
            warmup_frames_threshold: Number of consecutive frames required for object warmup
            fit_obb_callback: Optional callback function for OBB fitting when objects age out
        """
        self.max_accumulation_frames = max_accumulation_frames
        self.iou_threshold = iou_threshold
        self.max_object_age = max_object_age
        self.accumulation_iou_threshold = accumulation_iou_threshold
        self.warmup_frames_threshold = warmup_frames_threshold
        self.fit_obb_callback = fit_obb_callback

        # Track all objects that have ever appeared
        self.modeled_objects: Dict[int, ObjectState] = {}
        self.frame_count = 0

    def update_frame(self, frame_count: int) -> None:
        """Update frame count and age objects."""
        self.frame_count = frame_count

        # Age all objects
        for obj_state in self.modeled_objects.values():
            obj_state.age += 1
            if obj_state.age > self.max_object_age:
                # Object has aged out - update OBB using accumulated points before deactivating
                if (self.fit_obb_callback is not None and 
                    obj_state.is_warmed_up and 
                    len(obj_state.accumulated_points) > 0):
                    # Use callback to fit OBB from accumulated points
                    obb_result = self.fit_obb_callback(obj_state.accumulated_points, self.frame_count)
                    if obb_result is not None:
                        obb_center, obb_size, obb_rotation = obb_result
                        if (obb_center is not None and obb_size is not None and obb_rotation is not None):
                            obj_state.last_obb_center = obb_center.copy()
                            obj_state.last_obb_size = obb_size.copy()
                            obj_state.last_obb_rotation = obb_rotation.copy()
                obj_state.is_active = False

    def associate_objects(
        self,
        current_objects: Dict[int, Tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.int32], str]],
        current_obbs: Dict[int, Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32], npt.NDArray[np.float32]]],
    ) -> Dict[int, ObjectState]:
        """
        Associate current frame objects with tracked objects.

        Args:
            current_objects: Dict mapping object_id to (points, colors, object_ids, object_name)
            current_obbs: Dict mapping object_id to (center, size, rotation)

        Returns:
            Updated tracked objects
        """
        # Update existing objects or create new ones
        for obj_id, (points, colors, object_ids, object_name) in current_objects.items():
            if obj_id in self.modeled_objects:
                # Check IOU with previous OBB
                obj_state = self.modeled_objects[obj_id]
                
                # Case 1: Both previous and current OBBs exist - perform IOU association
                if (
                    obj_state.last_obb_center is not None
                    and obj_state.last_obb_size is not None
                    and obj_state.last_obb_rotation is not None
                    and obj_id in current_obbs
                ):
                    current_center, current_size, _ = current_obbs[obj_id]
                    iou = calculate_iou(
                        obj_state.last_obb_center, obj_state.last_obb_size, current_center, current_size
                    )

                    if iou >= self.iou_threshold:
                        # Good association - increment warmup frames
                        obj_state.warmup_frames += 1
                        
                        # Check if object has completed warmup
                        if obj_state.warmup_frames >= self.warmup_frames_threshold:
                            obj_state.is_warmed_up = True
                        
                        # Always try to accumulate points (logic inside _accumulate_points handles warmup vs normal mode)
                        self._accumulate_points(obj_state, points, colors, object_ids, iou)
                        obj_state.is_active = True
                        obj_state.age = 0
                    else:
                        # Poor association - reset warmup state and accumulation
                        obj_state.warmup_frames = 0
                        obj_state.is_warmed_up = False
                        self._reset_accumulation(obj_state, points, colors, object_ids)
                
                # Case 2: Current frame cannot compute OBB (insufficient points) - keep all states unchanged
                elif obj_id not in current_obbs:
                    # Skip this frame, keep all object states unchanged
                    # Object will still age through update_frame method
                    pass
                
                # Case 3: Object previously had no OBB, current frame has OBB - first successful OBB computation
                elif (
                    obj_state.last_obb_center is None
                    and obj_state.last_obb_size is None
                    and obj_state.last_obb_rotation is None
                    and obj_id in current_obbs
                ):
                    # First successful OBB computation - reset and start fresh
                    obj_state.warmup_frames = 1
                    obj_state.is_warmed_up = False
                    self._reset_accumulation(obj_state, points, colors, object_ids)
                    obj_state.is_active = True
                    obj_state.age = 0
            else:
                # New object - only create if current frame can compute OBB
                if obj_id in current_obbs:
                    # Create new state with warmup initialization
                    self.modeled_objects[obj_id] = ObjectState(
                        object_id=obj_id,
                        accumulated_points=points.copy(),
                        accumulated_colors=colors.copy(),
                        accumulated_object_ids=object_ids.copy(),
                        frame_count=0,
                        age=0,
                        is_active=True,
                        object_name=object_name,
                        warmup_frames=1,  # First frame counts as 1
                        is_warmed_up=False,  # Not warmed up yet
                    )
                # If obj_id not in current_obbs, skip creating this object (insufficient points)

        # Update OBB information for all current objects
        for obj_id, (center, size, rotation) in current_obbs.items():
            if obj_id in self.modeled_objects:
                self.modeled_objects[obj_id].last_obb_center = center.copy()
                self.modeled_objects[obj_id].last_obb_size = size.copy()
                self.modeled_objects[obj_id].last_obb_rotation = rotation.copy()

        return self.modeled_objects

    def _accumulate_points(
        self,
        obj_state: ObjectState,
        new_points: npt.NDArray[np.float32],
        new_colors: npt.NDArray[np.uint8],
        new_object_ids: npt.NDArray[np.int32],
        iou: float,
    ) -> None:
        """
        Accumulate new points to existing object state.

        Args:
            obj_state: Object state to accumulate points into
            new_points: New points to add
            new_colors: Colors for new points
            new_object_ids: Object IDs for new points
            iou: IOU value between current and previous OBB
        """
        # Check if we should continue accumulating
        if not obj_state.is_warmed_up:
            # During warmup phase, always accumulate regardless of IOU
            should_accumulate = True
        else:
            # After warmup, use original logic: IOU is small enough OR not reached max frames
            should_accumulate = (iou < self.accumulation_iou_threshold) or (obj_state.frame_count < self.max_accumulation_frames)
        
        if not should_accumulate:
            # Conditions not met, don't accumulate new points
            return

        # Add new points
        obj_state.accumulated_points = np.vstack([obj_state.accumulated_points, new_points])
        obj_state.accumulated_colors = np.vstack([obj_state.accumulated_colors, new_colors])
        obj_state.accumulated_object_ids = np.concatenate([obj_state.accumulated_object_ids, new_object_ids])
        
        # Increment frame count after successful accumulation
        obj_state.frame_count += 1

    def _reset_accumulation(
        self,
        obj_state: ObjectState,
        new_points: npt.NDArray[np.float32],
        new_colors: npt.NDArray[np.uint8],
        new_object_ids: npt.NDArray[np.int32],
    ) -> None:
        """
        Reset accumulation with new points.

        Args:
            obj_state: Object state to reset
            new_points: New points to set
            new_colors: Colors for new points
            new_object_ids: Object IDs for new points
        """
        obj_state.accumulated_points = new_points.copy()
        obj_state.accumulated_colors = new_colors.copy()
        obj_state.accumulated_object_ids = new_object_ids.copy()
        obj_state.frame_count = 1  # Start with 1 since we have one frame of data

    def get_active_objects(self) -> Dict[int, ObjectState]:
        """
        Get all active tracked objects.

        Returns:
            Dictionary mapping object IDs to active object states
        """
        return {obj_id: obj_state for obj_id, obj_state in self.modeled_objects.items() if obj_state.is_active}

    def fuse_objects(self, fusion_iou_threshold: float) -> None:
        """
        Fuse objects with high IOU overlap.

        Args:
            fusion_iou_threshold: IOU threshold for object fusion
        """
        if len(self.modeled_objects) < 2:
            return

        # Get all objects (both active and inactive)
        all_objects = self.modeled_objects
        if len(all_objects) < 2:
            return

        # Create list of object IDs for iteration
        object_ids = list(all_objects.keys())
        objects_to_remove = set()

        # Check all pairs of objects for fusion
        for i in range(len(object_ids)):
            obj_id1 = object_ids[i]
            if obj_id1 in objects_to_remove:
                continue

            obj_state1 = all_objects[obj_id1]
            if (
                obj_state1.last_obb_center is None
                or obj_state1.last_obb_size is None
                or obj_state1.last_obb_rotation is None
            ):
                continue

            for j in range(i + 1, len(object_ids)):
                obj_id2 = object_ids[j]
                if obj_id2 in objects_to_remove:
                    continue

                obj_state2 = all_objects[obj_id2]
                if (
                    obj_state2.last_obb_center is None
                    or obj_state2.last_obb_size is None
                    or obj_state2.last_obb_rotation is None
                ):
                    continue

                # Check if objects have the same name (prerequisite for fusion)
                if obj_state1.object_name != obj_state2.object_name:
                    continue

                # Calculate IOU between the two objects
                iou = calculate_iou(
                    obj_state1.last_obb_center,
                    obj_state1.last_obb_size,
                    obj_state2.last_obb_center,
                    obj_state2.last_obb_size,
                )

                if iou >= fusion_iou_threshold:
                    # Fuse objects: merge into the one with larger ID (newer object)
                    smaller_id = min(obj_id1, obj_id2)
                    larger_id = max(obj_id1, obj_id2)

                    if larger_id == obj_id1:
                        self._merge_objects(obj_state1, obj_state2)
                    else:
                        self._merge_objects(obj_state2, obj_state1)

                    # Mark the smaller ID object for removal
                    objects_to_remove.add(smaller_id)

        # Remove fused objects
        for obj_id in objects_to_remove:
            if obj_id in self.modeled_objects:
                del self.modeled_objects[obj_id]

    def _merge_objects(self, target_state: ObjectState, source_state: ObjectState) -> None:
        """
        Merge source object into target object.

        Args:
            target_state: Target object state to merge into
            source_state: Source object state to merge from
        """
        # Combine accumulated points
        target_state.accumulated_points = np.vstack([target_state.accumulated_points, source_state.accumulated_points])

        # Combine accumulated colors
        target_state.accumulated_colors = np.vstack([target_state.accumulated_colors, source_state.accumulated_colors])

        # Combine accumulated object IDs
        target_state.accumulated_object_ids = np.concatenate(
            [target_state.accumulated_object_ids, source_state.accumulated_object_ids]
        )

        # Update OBB to encompass both objects
        if (
            source_state.last_obb_center is not None
            and source_state.last_obb_size is not None
            and source_state.last_obb_rotation is not None
        ):

            if (
                target_state.last_obb_center is not None
                and target_state.last_obb_size is not None
                and target_state.last_obb_rotation is not None
            ):
                # Both have OBBs - compute union
                target_min = target_state.last_obb_center - target_state.last_obb_size / 2.0
                target_max = target_state.last_obb_center + target_state.last_obb_size / 2.0
                source_min = source_state.last_obb_center - source_state.last_obb_size / 2.0
                source_max = source_state.last_obb_center + source_state.last_obb_size / 2.0

                union_min = np.minimum(target_min, source_min)
                union_max = np.maximum(target_max, source_max)

                target_state.last_obb_center = (union_min + union_max) / 2.0
                target_state.last_obb_size = union_max - union_min
                # Keep target's rotation (or could average rotations)
                # For simplicity, keep target's rotation
            else:
                # Only source has OBB - use it
                target_state.last_obb_center = source_state.last_obb_center.copy()
                target_state.last_obb_size = source_state.last_obb_size.copy()
                target_state.last_obb_rotation = source_state.last_obb_rotation.copy()

        # Reset frame count for accumulation
        target_state.frame_count = 0

        # If source object is active, make target object active too
        if source_state.is_active:
            target_state.is_active = True
            target_state.age = 0
