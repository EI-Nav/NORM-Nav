#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Structured output definitions for behavioral instruction parsing.

This module defines the data structures used to represent parsed behavioral
constraints in a structured format suitable for navigation systems.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import List, Dict, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum
import json
import time


# Direction and velocity constraints are now numeric values
# Direction: -1.0 (left) to 1.0 (right), 0.0 (center), None (no constraint)
# Velocity: 0.0 (slow) to 1.0 (fast), 0.5 (normal), None (no constraint)


class TraversabilityConstraint(Enum):
    """Traversability constraint options."""
    NULL = 0
    TRAVERSABLE = 1
    NON_TRAVERSABLE = 2


@dataclass
class BehavioralConstraint:
    """
    Represents a single behavioral constraint.
    
    This structure maps to the tuple (Oi, di, Vi, ti) where:
    - Oi: Objects being referenced (can be multiple)
    - di: Spatial preference (direction) - numeric value from -1.0 to 1.0, or None
    - Vi: Velocity requirement - numeric value from 0.0 to 1.0, or None
    - ti: Semantic traversability
    """
    object_list: List[str]
    direction_constrain: Optional[float]  # -1.0 (left) to 1.0 (right), 0.0 (center), None (no constraint)
    velocity_constrain: Optional[float]  # 0.0 (slow) to 1.0 (fast), 0.5 (normal), None (no constraint)
    traversability_constrain: int
    
    def __post_init__(self) -> None:
        """Validate constraint values after initialization."""
        self._validate_constraints()
    
    def _validate_constraints(self) -> None:
        """Validate that constraint values are valid."""
        valid_traversability = [t.value for t in TraversabilityConstraint]
        
        # Validate object_list
        if not isinstance(self.object_list, list):
            raise ValueError(f"Object list must be a list: {self.object_list}")
        if not self.object_list:
            raise ValueError("Object list cannot be empty")
        for obj in self.object_list:
            if not isinstance(obj, str) or not obj.strip():
                raise ValueError(f"All objects must be non-empty strings: {obj}")
        
        # Validate direction constraint (numeric or None)
        if self.direction_constrain is not None:
            if not isinstance(self.direction_constrain, (int, float)):
                raise ValueError(f"Direction constraint must be numeric or None, got {type(self.direction_constrain).__name__}: {self.direction_constrain}")
            if not -1.0 <= self.direction_constrain <= 1.0:
                raise ValueError(f"Direction constraint must be between -1.0 and 1.0, got: {self.direction_constrain}")
        
        # Validate velocity constraint (numeric or None)
        if self.velocity_constrain is not None:
            if not isinstance(self.velocity_constrain, (int, float)):
                raise ValueError(f"Velocity constraint must be numeric or None, got {type(self.velocity_constrain).__name__}: {self.velocity_constrain}")
            if not 0.0 <= self.velocity_constrain <= 1.0:
                raise ValueError(f"Velocity constraint must be between 0.0 and 1.0, got: {self.velocity_constrain}")
        
        # Validate traversability constraint (integer)
        if not isinstance(self.traversability_constrain, int):
            raise ValueError(f"Traversability constraint must be an integer, got {type(self.traversability_constrain).__name__}: {self.traversability_constrain}")
        if self.traversability_constrain not in valid_traversability:
            raise ValueError(f"Invalid traversability constraint: {self.traversability_constrain}. Valid values: {valid_traversability}")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert constraint to dictionary."""
        return asdict(self)
    
    def to_json(self) -> str:
        """Convert constraint to JSON string."""
        return json.dumps(self.to_dict(), indent=2)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'BehavioralConstraint':
        """Create constraint from dictionary."""
        # Handle both old string format and new list format for backward compatibility
        object_list = data.get("object_list", [])
        if isinstance(object_list, str):
            object_list = [object_list] if object_list else []
        elif not isinstance(object_list, list):
            object_list = []
        
        # Handle direction constraint (convert from string to numeric if needed)
        direction_constrain = data.get("direction_constrain", None)
        if isinstance(direction_constrain, str):
            if direction_constrain == "null":
                direction_constrain = None
            elif direction_constrain == "left":
                direction_constrain = -1.0
            elif direction_constrain == "right":
                direction_constrain = 1.0
            elif direction_constrain == "center":
                direction_constrain = 0.0
        
        # Handle velocity constraint (convert from string to numeric if needed)
        velocity_constrain = data.get("velocity_constrain", None)
        if isinstance(velocity_constrain, str):
            if velocity_constrain == "null":
                velocity_constrain = None
            elif velocity_constrain == "slow":
                velocity_constrain = 0.0
            elif velocity_constrain == "normal":
                velocity_constrain = 0.5
            elif velocity_constrain == "fast":
                velocity_constrain = 1.0
        
        return cls(
            object_list=object_list,
            direction_constrain=direction_constrain,
            velocity_constrain=velocity_constrain,
            traversability_constrain=data.get("traversability_constrain", 0)
        )
    
    @classmethod
    def from_json(cls, json_str: str) -> 'BehavioralConstraint':
        """Create constraint from JSON string."""
        data = json.loads(json_str)
        return cls.from_dict(data)
    
    def to_ros_msg(self) -> Any:
        """Convert to ROS2 BehavioralConstraint message."""
        try:
            from navibot_interfaces.msg import BehavioralConstraint as BehavioralConstraintMsg
            from std_msgs.msg import Header
            
            # Create header
            header = Header()
            header.stamp.sec = int(time.time())
            header.stamp.nanosec = int((time.time() % 1) * 1e9)
            header.frame_id = "map"
            
            # Create ROS2 message
            msg = BehavioralConstraintMsg()
            msg.header = header
            msg.object_list = self.object_list
            
            # Handle null values for direction and velocity constraints
            msg.direction_constrain = self.direction_constrain if self.direction_constrain is not None else -999.0
            msg.velocity_constrain = self.velocity_constrain if self.velocity_constrain is not None else -999.0
            msg.traversability_constrain = self.traversability_constrain
            
            # OBB list is empty for now (will be populated later from object_modeling node)
            msg.obb_list = []
            
            return msg
        except ImportError as exc:
            raise ImportError("navibot_interfaces package not available. Cannot convert to ROS2 message.") from exc


@dataclass
class StructuredOutput:
    """
    Represents the complete structured output from instruction parsing.
    
    This contains all parsed behavioral constraints that can be used
    by the multi-layer costmap system.
    """
    constraints: List[BehavioralConstraint]
    metadata: Optional[Dict[str, Any]] = None
    
    def __post_init__(self) -> None:
        """Initialize metadata if not provided."""
        if self.metadata is None:
            # Flatten all objects from all constraints
            all_objects = []
            for constraint in self.constraints:
                all_objects.extend(constraint.object_list)
            
            self.metadata = {
                "total_constraints": len(self.constraints),
                "objects": list(set(all_objects)),
                "timestamp": None
            }
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert output to dictionary."""
        return {
            "constraints": [c.to_dict() for c in self.constraints],
            "metadata": self.metadata
        }
    
    def to_json(self) -> str:
        """Convert output to JSON string."""
        return json.dumps(self.to_dict(), indent=2)
    
    def get_constraints_by_object(self, object_name: str) -> List[BehavioralConstraint]:
        """Get all constraints for a specific object."""
        return [c for c in self.constraints if object_name in c.object_list]
    
    def get_constraints_by_direction(self, direction: Optional[float]) -> List[BehavioralConstraint]:
        """Get all constraints with a specific direction."""
        return [c for c in self.constraints if c.direction_constrain == direction]
    
    def get_constraints_by_velocity(self, velocity: Optional[float]) -> List[BehavioralConstraint]:
        """Get all constraints with a specific velocity."""
        return [c for c in self.constraints if c.velocity_constrain == velocity]
    
    def get_constraints_by_traversability(self, traversability: int) -> List[BehavioralConstraint]:
        """Get all constraints with a specific traversability."""
        return [c for c in self.constraints if c.traversability_constrain == traversability]
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'StructuredOutput':
        """Create output from dictionary."""
        constraints = [BehavioralConstraint.from_dict(c) for c in data.get("constraints", [])]
        metadata = data.get("metadata", {})
        return cls(constraints=constraints, metadata=metadata)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'StructuredOutput':
        """Create output from JSON string."""
        data = json.loads(json_str)
        return cls.from_dict(data)
    
    def to_ros_msg_array(self) -> Any:
        """Convert to ROS2 BehavioralConstraintArray message."""
        try:
            from navibot_interfaces.msg import BehavioralConstraintArray
            from std_msgs.msg import Header
            
            # Create header
            header = Header()
            header.stamp.sec = int(time.time())
            header.stamp.nanosec = int((time.time() % 1) * 1e9)
            header.frame_id = "map"
            
            # Create ROS2 message array
            msg = BehavioralConstraintArray()
            msg.header = header
            msg.constraints = [constraint.to_ros_msg() for constraint in self.constraints]
            
            return msg
        except ImportError as exc:
            raise ImportError("navibot_interfaces package not available. Cannot convert to ROS2 message.") from exc
