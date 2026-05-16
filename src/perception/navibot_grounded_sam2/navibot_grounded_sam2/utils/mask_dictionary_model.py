#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mask dictionary model for GroundedSAM2 tracker.

This module provides data structures and utilities for managing
object masks and annotations in the tracking system.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""
# Standard library
import json
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# Third-party libraries
import numpy as np
import torch


@dataclass
class MaskDictionaryModel:
    """Model for managing mask dictionaries and object annotations.
    
    This dataclass stores mask information including dimensions, labels,
    and provides methods for updating and managing object annotations.
    
    Attributes:
        mask_name: Name identifier for the mask.
        mask_height: Height of the mask in pixels.
        mask_width: Width of the mask in pixels.
        promote_type: Type of promotion for the mask.
        labels: Dictionary containing object annotations.
    """
    mask_name: str = ""
    mask_height: int = 1080
    mask_width: int = 1920
    promote_type: str = "mask"
    labels: Dict[int, 'ObjectInfo'] = field(default_factory=dict)

    def add_new_frame_annotation(
        self, 
        mask_list: torch.Tensor, 
        box_list: List[List[float]], 
        label_list: List[str], 
        score_list: Optional[List[float]] = None, 
        background_value: int = 0
    ) -> None:
        """Add new frame annotations to the mask dictionary.
        
        Args:
            mask_list: Tensor containing mask data.
            box_list: List of bounding box coordinates.
            label_list: List of class labels.
            score_list: Optional list of confidence scores.
            background_value: Background value for mask indexing.
        """
        mask_img = torch.zeros(mask_list.shape[-2:])
        anno_2d = {}
        
        # If score_list is not provided, create a list of 0.0
        if score_list is None:
            score_list = [0.0] * len(mask_list)
        # Convert torch.Tensor to list if needed
        elif isinstance(score_list, torch.Tensor):
            score_list = score_list.cpu().tolist()
        
        for idx, (mask, box, label, score) in enumerate(zip(mask_list, box_list, label_list, score_list)):
            final_index = background_value + idx + 1

            if mask.shape[0] != mask_img.shape[0] or mask.shape[1] != mask_img.shape[1]:
                raise ValueError("The mask shape should be the same as the mask_img shape.")
            # mask = mask
            mask_img[mask == True] = final_index
            # print("label", label)
            name = label
            # box = box # .numpy().tolist()
            new_annotation = ObjectInfo(instance_id = final_index, mask = mask, class_name = name, x1 = box[0], y1 = box[1], x2 = box[2], y2 = box[3], logit = float(score))
            anno_2d[final_index] = new_annotation

        # np.save(os.path.join(output_dir, output_file_name), mask_img.numpy().astype(np.uint16))
        self.mask_height = mask_img.shape[0]
        self.mask_width = mask_img.shape[1]
        self.labels = anno_2d

    def update_masks(
        self, 
        tracking_annotation_dict: 'MaskDictionaryModel', 
        iou_threshold: float = 0.8, 
        objects_count: int = 0
    ) -> int:
        """Update masks based on tracking annotations.
        
        Args:
            tracking_annotation_dict: Dictionary containing tracking annotations.
            iou_threshold: IoU threshold for mask association.
            objects_count: Current count of objects.
            
        Returns:
            Updated object count.
        """
        updated_masks = {}

        for _, seg_mask in self.labels.items():  # tracking_masks
            flag = 0 
            new_mask_copy = ObjectInfo()
            if seg_mask.mask.sum() == 0:
                continue
            
            for _, object_info in tracking_annotation_dict.labels.items():  # grounded_sam masks
                iou = self.calculate_iou(seg_mask.mask, object_info.mask)  # tensor, numpy
                # print("iou", iou)
                if iou > iou_threshold:
                    flag = object_info.instance_id
                    new_mask_copy.mask = seg_mask.mask
                    new_mask_copy.instance_id = object_info.instance_id
                    new_mask_copy.class_name = seg_mask.class_name
                    new_mask_copy.logit = seg_mask.logit  # Preserve confidence score
                    break
                
            if not flag:
                objects_count += 1
                flag = objects_count
                new_mask_copy.instance_id = objects_count
                new_mask_copy.mask = seg_mask.mask
                new_mask_copy.class_name = seg_mask.class_name
                new_mask_copy.logit = seg_mask.logit  # Preserve confidence score
            updated_masks[flag] = new_mask_copy
        self.labels = updated_masks
        return objects_count

    def get_target_class_name(self, instance_id: int) -> str:
        """Get class name for a specific instance ID.
        
        Args:
            instance_id: The instance ID to query.
            
        Returns:
            Class name for the instance.
        """
        return self.labels[instance_id].class_name

    def get_target_logit(self, instance_id: int) -> float:
        """Get logit (confidence score) for a specific instance ID.
        
        Args:
            instance_id: The instance ID to query.
            
        Returns:
            Logit value for the instance.
        """
        return self.labels[instance_id].logit
    
    @staticmethod
    def calculate_iou(mask1: torch.Tensor, mask2: torch.Tensor) -> torch.Tensor:
        """Calculate Intersection over Union (IoU) between two masks.
        
        Args:
            mask1: First mask tensor.
            mask2: Second mask tensor.
            
        Returns:
            IoU value as a tensor.
        """
        # Keep masks as bool to avoid float32 materialization (~4x memory vs float masks).
        # Ensure boolean tensors before bitwise ops.
        if mask1.dtype != torch.bool:
            mask1 = mask1 > 0
        if mask2.dtype != torch.bool:
            mask2 = mask2 > 0
        
        # Bitwise AND/OR for intersection/union (cheaper than multiply-heavy paths).
        intersection = (mask1 & mask2).sum().float()
        union = (mask1 | mask2).sum().float()
        
        # Avoid divide-by-zero when both masks are empty.
        if union == 0:
            return torch.tensor(0.0, device=mask1.device)
        
        # Calculate IoU
        iou = intersection / union
        return iou


    def save_empty_mask_and_json(
        self, 
        mask_data_dir: str, 
        json_data_dir: str, 
        image_name_list: Optional[List[str]] = None
    ) -> None:
        """Save empty mask and JSON files.
        
        Args:
            mask_data_dir: Directory to save mask files.
            json_data_dir: Directory to save JSON files.
            image_name_list: Optional list of image names to process.
        """
        mask_img = torch.zeros((self.mask_height, self.mask_width))
        if image_name_list:
            for image_base_name in image_name_list:
                image_base_name = image_base_name.split(".")[0]+".npy"
                mask_name = "mask_"+image_base_name
                np.save(os.path.join(mask_data_dir, mask_name), mask_img.numpy().astype(np.uint16))

                json_data_path = os.path.join(json_data_dir, mask_name.replace(".npy", ".json"))
                print("save_empty_mask_and_json", json_data_path)
                self.to_json(json_data_path)
        else:
            np.save(os.path.join(mask_data_dir, self.mask_name), mask_img.numpy().astype(np.uint16))
            json_data_path = os.path.join(json_data_dir, self.mask_name.replace(".npy", ".json"))
            print("save_empty_mask_and_json", json_data_path)
            self.to_json(json_data_path)


    def to_dict(self) -> Dict[str, Any]:
        """Convert the model to a dictionary representation.
        
        Returns:
            Dictionary containing all model data.
        """
        return {
            "mask_name": self.mask_name,
            "mask_height": self.mask_height,
            "mask_width": self.mask_width,
            "promote_type": self.promote_type,
            "labels": {k: v.to_dict() for k, v in self.labels.items()}
        }
    
    def to_json(self, json_file: str) -> None:
        """Save the model to a JSON file.
        
        Args:
            json_file: Path to the JSON file to save.
        """
        with open(json_file, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=4)
            
    def from_json(self, json_file: str) -> 'MaskDictionaryModel':
        """Load the model from a JSON file.
        
        Args:
            json_file: Path to the JSON file to load.
            
        Returns:
            Self instance for method chaining.
        """
        with open(json_file, "r", encoding="utf-8") as f:
            data = json.load(f)
            self.mask_name = data["mask_name"]
            self.mask_height = data["mask_height"]
            self.mask_width = data["mask_width"]
            self.promote_type = data["promote_type"]
            self.labels = {int(k): ObjectInfo(**v) for k, v in data["labels"].items()}
        return self


@dataclass
class ObjectInfo:
    """Information container for object annotations.
    
    This dataclass stores information about detected objects including
    instance ID, mask data, class name, bounding box coordinates, and confidence.
    
    Attributes:
        instance_id: Unique identifier for the object instance.
        mask: Tensor containing the object mask.
        class_name: Name of the object class.
        x1: Left coordinate of bounding box.
        y1: Top coordinate of bounding box.
        x2: Right coordinate of bounding box.
        y2: Bottom coordinate of bounding box.
        logit: Confidence score for the detection.
    """
    instance_id: int = 0
    mask: Any = None
    class_name: str = ""
    x1: int = 0
    y1: int = 0
    x2: int = 0
    y2: int = 0
    logit: float = 0.0

    def get_mask(self) -> Any:
        """Get the mask tensor.
        
        Returns:
            The mask tensor for this object.
        """
        return self.mask
    
    def get_id(self) -> int:
        """Get the instance ID.
        
        Returns:
            The instance ID for this object.
        """
        return self.instance_id

    def update_box(self) -> None:
        """Update bounding box coordinates from mask data."""
        # Indices of all nonzero mask pixels.
        nonzero_indices = torch.nonzero(self.mask)
        
        # Empty mask → leave box unset (legacy API returns []).
        if nonzero_indices.size(0) == 0:
            # print("nonzero_indices", nonzero_indices)
            return []
        
        # Axis-aligned min/max in pixel coordinates.
        y_min, x_min = torch.min(nonzero_indices, dim=0)[0]
        y_max, x_max = torch.max(nonzero_indices, dim=0)[0]
        
        # Bounding box [x_min, y_min, x_max, y_max].
        bbox = [x_min.item(), y_min.item(), x_max.item(), y_max.item()]        
        self.x1 = bbox[0]
        self.y1 = bbox[1]
        self.x2 = bbox[2]
        self.y2 = bbox[3]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the object info to a dictionary representation.
        
        Returns:
            Dictionary containing all object information.
        """
        return {
            "instance_id": self.instance_id,
            "class_name": self.class_name,
            "x1": self.x1,
            "y1": self.y1,
            "x2": self.x2,
            "y2": self.y2,
            "logit": self.logit
        }