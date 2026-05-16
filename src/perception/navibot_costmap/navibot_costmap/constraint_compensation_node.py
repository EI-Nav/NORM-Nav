#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Constraint Compensation Node.

Compensates behavioral constraints based on geometric traversability costmap.
Calculates average geometric cost for AABB regions and updates traversability
constraints for null values based on geometric cost threshold.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

import copy
from typing import List, Optional, Tuple

import numpy as np
import numpy.typing as npt
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import rotate

# Import custom message types
from navibot_interfaces.msg import BehavioralConstraintArray, OBBInfo2DArray


# Constants for improved maintainability
class ConstraintCompensationConstants:
    """Constants for constraint compensation calculations."""

    # Rotation and angle tolerances
    ROTATION_TOLERANCE_RADIANS = np.radians(1.0)  # 1 degree tolerance
    NEGLIGIBLE_ROTATION_THRESHOLD = 1e-6

    # Traversability constraint values
    TRAVERSABILITY_NULL = 0
    TRAVERSABILITY_TRAVERSABLE = 1
    TRAVERSABILITY_NON_TRAVERSABLE = 2

    # Default costmap values
    COSTMAP_UNKNOWN = -1
    COSTMAP_FREE = 0


class ConstraintCompensationNode(Node):
    """
    Compensate behavioral constraints based on geometric traversability costmap.

    This node subscribes to enhanced behavioral constraints and geometric costmap,
    calculates average geometric cost for AABB regions, and updates traversability
    constraints for null values based on geometric cost threshold.
    """

    def __init__(self) -> None:
        """Initialize the node with parameters and publishers/subscribers."""
        super().__init__("constraint_compensation_node")

        self._declare_parameters()
        self._load_parameters()
        self._initialize_components()

        self.get_logger().info("Constraint Compensation Node initialized")

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        # Input/Output configuration
        self.declare_parameter("constraint_topic", "/behavioral_constraints")
        self.declare_parameter("geometric_costmap_topic", "/costmap/geometric")
        self.declare_parameter("obb_info_topic", "/object_modeling/object_obb_info")
        self.declare_parameter("output_topic", "/compensated_behavioral_constraints")

        # Geometric cost compensation configuration
        self.declare_parameter("geometric_cost_threshold", 50.0)
        self.declare_parameter("rotation_tolerance_deg", 1.0)
        self.declare_parameter("min_valid_cost_points", 1)

        # Performance configuration
        self.declare_parameter("enable_debug_logging", False)
        self.declare_parameter("publish_rate", 1.0)  # Hz

    def _load_parameters(self) -> None:
        """Load and validate all ROS parameters."""
        # Input/Output configuration
        self.constraint_topic = self.get_parameter("constraint_topic").value
        self.geometric_costmap_topic = self.get_parameter("geometric_costmap_topic").value
        self.obb_info_topic = self.get_parameter("obb_info_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        # Geometric cost compensation configuration
        self.geometric_cost_threshold = self.get_parameter("geometric_cost_threshold").value
        self.rotation_tolerance_deg = self.get_parameter("rotation_tolerance_deg").value
        self.min_valid_cost_points = self.get_parameter("min_valid_cost_points").value

        # Performance configuration
        self.enable_debug_logging = self.get_parameter("enable_debug_logging").value
        self.publish_rate = self.get_parameter("publish_rate").value

        # Convert rotation tolerance to radians for internal use
        self.rotation_tolerance_rad = np.radians(self.rotation_tolerance_deg)

        # Validate parameters
        self._validate_parameters()

        # Set logger level
        if self.enable_debug_logging:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Log key configuration parameters
        self.get_logger().info(f"Constraint topic: {self.constraint_topic}")
        self.get_logger().info(f"Geometric costmap topic: {self.geometric_costmap_topic}")
        self.get_logger().info(f"OBB info topic: {self.obb_info_topic}")
        self.get_logger().info(f"Output topic: {self.output_topic}")
        self.get_logger().info(f"Geometric cost threshold: {self.geometric_cost_threshold}")
        self.get_logger().info(f"Rotation tolerance: {self.rotation_tolerance_deg} degrees")
        self.get_logger().info(f"Min valid cost points: {self.min_valid_cost_points}")
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")

    def _validate_parameters(self) -> None:
        """Validate parameter values and raise errors for invalid configurations."""
        if self.geometric_cost_threshold < 0 or self.geometric_cost_threshold > 100:
            raise ValueError(f"Geometric cost threshold must be between 0 and 100, got {self.geometric_cost_threshold}")

        if self.rotation_tolerance_deg < 0 or self.rotation_tolerance_deg > 180:
            raise ValueError(f"Rotation tolerance must be between 0 and 180 degrees, got {self.rotation_tolerance_deg}")

        if self.min_valid_cost_points < 1:
            raise ValueError(f"Min valid cost points must be >= 1, got {self.min_valid_cost_points}")

        if self.publish_rate <= 0:
            raise ValueError(f"Publish rate must be > 0, got {self.publish_rate}")

    def _initialize_components(self) -> None:
        """Initialize ROS components (publishers, subscribers)."""
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Initialize subscribers
        self.constraint_sub = self.create_subscription(
            BehavioralConstraintArray, self.constraint_topic, self.constraint_callback, 1
        )
        self.geometric_costmap_sub = self.create_subscription(
            OccupancyGrid, self.geometric_costmap_topic, self.geometric_costmap_callback, 1
        )
        self.obb_info_sub = self.create_subscription(OBBInfo2DArray, self.obb_info_topic, self.obb_info_callback, 1)

        # Initialize publisher
        self.compensated_constraint_pub = self.create_publisher(
            BehavioralConstraintArray, self.output_topic, qos_profile
        )

        # Internal state
        self.latest_constraints: Optional[BehavioralConstraintArray] = None
        self.latest_geometric_costmap: Optional[OccupancyGrid] = None
        self.latest_obb_info: Optional[OBBInfo2DArray] = None
        self.processing: bool = False
        
        # Costmap rotation cache for performance optimization
        self.cached_rotated_costmap: Optional[Tuple[npt.NDArray[np.int8], MapMetaData]] = None
        self.cached_rotation_angle: Optional[float] = None
        self.cached_costmap_stamp: Optional[Time] = None
        
        # Reusable objects to reduce allocation overhead
        self._reusable_constraint_array = BehavioralConstraintArray()
        self._reusable_filtered_array = BehavioralConstraintArray()

        # Create timer for periodic processing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("Subscribers and publishers initialized")

    def constraint_callback(self, msg: BehavioralConstraintArray) -> None:
        """
        Callback for receiving behavioral constraints.

        Args:
            msg: BehavioralConstraintArray message containing behavioral constraints
        """
        try:
            self.latest_constraints = msg

            if self.enable_debug_logging:
                self.get_logger().debug(f"Received behavioral constraints: {len(msg.constraints)} constraints")

            self.get_logger().debug(f"Stored behavioral constraints with {len(msg.constraints)} constraints")

        except (ValueError, AttributeError) as e:
            self.get_logger().error(f"Error processing constraints: {e}")

    def geometric_costmap_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback for receiving geometric costmap.

        Args:
            msg: Geometric costmap message
        """
        self.latest_geometric_costmap = msg
        if self.enable_debug_logging:
            self.get_logger().debug("Received geometric costmap")

    def obb_info_callback(self, msg: OBBInfo2DArray) -> None:
        """
        Callback for receiving OBB information.

        Args:
            msg: OBBInfo2DArray message containing object OBB information
        """
        self.latest_obb_info = msg
        if self.enable_debug_logging:
            self.get_logger().debug(f"Received OBB info: {len(msg.obb_array)} objects")

    def timer_callback(self) -> None:
        """Process and publish compensated constraints at configured rate."""
        if self.latest_constraints is None:
            self.get_logger().debug("No constraint data received yet")
            return

        if self.latest_geometric_costmap is None:
            self.get_logger().debug("No geometric costmap data received yet")
            return

        if self.latest_obb_info is None:
            self.get_logger().debug("No OBB info data received yet")
            return

        if self.processing:
            self.get_logger().warn("Still processing, skipping this cycle")
            return

        self.processing = True

        try:
            # Compensate constraints based on geometric costmap
            compensated_constraints = self._compensate_constraints(
                self.latest_constraints, self.latest_geometric_costmap, self.latest_obb_info
            )

            if compensated_constraints:
                # Publish compensated constraints
                self.compensated_constraint_pub.publish(compensated_constraints)
                self.get_logger().debug(f"Published compensated constraints at {self.publish_rate} Hz")

        except (ValueError, AttributeError, RuntimeError) as e:
            self.get_logger().error(f"Error processing constraint compensation: {e}")
        finally:
            self.processing = False

    def _merge_obb_into_constraints(
        self, constraints: BehavioralConstraintArray, obb_info_array: OBBInfo2DArray
    ) -> BehavioralConstraintArray:
        """
        Merge OBB information into behavioral constraints.

        Args:
            constraints: BehavioralConstraintArray message
            obb_info_array: OBBInfo2DArray message containing object OBB information

        Returns:
            BehavioralConstraintArray with merged OBB information
        """
        # Reuse object to avoid allocation overhead
        merged_constraints = self._reusable_constraint_array
        merged_constraints.header = constraints.header
        merged_constraints.constraints.clear()

        for constraint in constraints.constraints:
            # Create a copy of the constraint
            merged_constraint = constraint

            # Clear existing OBB list
            merged_constraint.obb_list = []

            # Convert object_list to set for O(1) lookup performance
            object_set = set(constraint.object_list)

            # Merge OBB information based on object name matching
            for obb in obb_info_array.obb_array:
                if obb.object_name in object_set:
                    merged_constraint.obb_list.append(obb)

            merged_constraints.constraints.append(merged_constraint)

        return merged_constraints

    def _compensate_constraints(
        self, constraints: BehavioralConstraintArray, geometric_costmap: OccupancyGrid, obb_info_array: OBBInfo2DArray
    ) -> Optional[BehavioralConstraintArray]:
        """
        Compensate constraints based on geometric costmap.

        Args:
            constraints: BehavioralConstraintArray message
            geometric_costmap: Geometric traversability costmap
            obb_info_array: OBBInfo2DArray message

        Returns:
            Compensated BehavioralConstraintArray or None if no compensation needed
        """
        try:
            # First merge OBB information into constraints
            merged_constraints = self._merge_obb_into_constraints(constraints, obb_info_array)

            # Filter constraints to only include those with non-empty OBB lists
            filtered_constraints = self._filter_constraints_with_obbs(merged_constraints)
            if filtered_constraints is None:
                return None

            # Prepare costmap data and handle rotation
            costmap_data, costmap_info = self._prepare_costmap_data(geometric_costmap, filtered_constraints)

            # Process each constraint for compensation
            compensated_constraints, compensation_applied = self._process_constraint_compensation(
                filtered_constraints, costmap_data, costmap_info
            )

            # Return appropriate result based on whether compensation was applied
            return self._create_compensation_result(filtered_constraints, compensated_constraints, compensation_applied)

        except (ValueError, AttributeError, RuntimeError) as e:
            self.get_logger().error(f"Error compensating constraints: {e}")
            return None

    def _filter_constraints_with_obbs(self, merged_constraints: BehavioralConstraintArray) -> Optional[BehavioralConstraintArray]:
        """
        Filter constraints to only include those with non-empty OBB lists.
        
        Args:
            merged_constraints: Constraints with merged OBB information
            
        Returns:
            Filtered BehavioralConstraintArray or None if no valid constraints
        """
        # Reuse object to avoid allocation overhead
        filtered_constraints = self._reusable_filtered_array
        filtered_constraints.header = merged_constraints.header
        filtered_constraints.constraints.clear()
        
        for constraint in merged_constraints.constraints:
            if constraint.obb_list:  # Only include constraints with non-empty OBB
                filtered_constraints.constraints.append(constraint)
            else:
                self.get_logger().debug("Skipping constraint with empty obb_list")
        
        if not filtered_constraints.constraints:
            self.get_logger().warn("All constraints have empty obb_list, skipping publication")
            return None
        
        return filtered_constraints

    def _prepare_costmap_data(
        self, geometric_costmap: OccupancyGrid, merged_constraints: BehavioralConstraintArray
    ) -> Tuple[npt.NDArray[np.int8], MapMetaData]:
        """
        Prepare costmap data and handle rotation if needed.

        Args:
            geometric_costmap: Original geometric costmap
            merged_constraints: Constraints with OBB information

        Returns:
            Tuple of (costmap_data, costmap_info)
        """
        # Check if we have OBBs and extract rotation angle
        rotation_angle = self._extract_rotation_angle(merged_constraints)
        
        # Check cache for rotated costmap
        current_stamp = geometric_costmap.header.stamp
        if (self.cached_rotated_costmap is not None and 
            self.cached_rotation_angle is not None and 
            self.cached_costmap_stamp is not None and
            abs(self.cached_rotation_angle - (rotation_angle or 0.0)) < ConstraintCompensationConstants.NEGLIGIBLE_ROTATION_THRESHOLD and
            self.cached_costmap_stamp.sec == current_stamp.sec and
            self.cached_costmap_stamp.nanosec == current_stamp.nanosec):
            # Use cached rotated costmap
            if self.enable_debug_logging:
                self.get_logger().debug("Using cached rotated costmap")
            return self.cached_rotated_costmap

        # Always start with fresh costmap data from the latest message
        costmap_data = np.array(geometric_costmap.data, dtype=np.int8).reshape(
            (geometric_costmap.info.height, geometric_costmap.info.width)
        )

        # Initialize costmap info for potential updates
        current_costmap_info = geometric_costmap.info

        # Rotate costmap if we have a valid rotation angle
        if (
            rotation_angle is not None
            and abs(rotation_angle) > ConstraintCompensationConstants.NEGLIGIBLE_ROTATION_THRESHOLD
        ):
            costmap_data, current_costmap_info = self._rotate_costmap(
                costmap_data, current_costmap_info, rotation_angle
            )
            # Cache the rotated costmap
            self.cached_rotated_costmap = (costmap_data, current_costmap_info)
            self.cached_rotation_angle = rotation_angle
            self.cached_costmap_stamp = current_stamp
            if self.enable_debug_logging:
                self.get_logger().debug(f"Rotated costmap by {np.degrees(rotation_angle):.2f} degrees and cached")
        else:
            # No rotation needed, cache the original costmap
            self.cached_rotated_costmap = (costmap_data, current_costmap_info)
            self.cached_rotation_angle = 0.0
            self.cached_costmap_stamp = current_stamp

        return costmap_data, current_costmap_info

    def _extract_rotation_angle(self, merged_constraints: BehavioralConstraintArray) -> Optional[float]:
        """
        Extract rotation angle from constraints and validate consistency.

        Args:
            merged_constraints: Constraints with OBB information

        Returns:
            Rotation angle in radians, or None if no valid rotation found
        """
        if not merged_constraints.constraints or not merged_constraints.constraints[0].obb_list:
            return None

        # Get rotation angle from first OBB (all OBBs should have same rotation)
        first_obb = merged_constraints.constraints[0].obb_list[0]
        rotation_angle = first_obb.rotation

        # Verify all OBBs have the same rotation (with tolerance)
        for constraint in merged_constraints.constraints:
            for obb in constraint.obb_list:
                angle_diff = abs(obb.rotation - rotation_angle)
                if angle_diff > self.rotation_tolerance_rad:
                    self.get_logger().warn(f"OBB rotation mismatch: {np.degrees(angle_diff):.2f} degrees difference")

        return rotation_angle

    def _process_constraint_compensation(
        self,
        merged_constraints: BehavioralConstraintArray,
        costmap_data: npt.NDArray[np.int8],
        costmap_info: MapMetaData,
    ) -> Tuple[List, bool]:
        """
        Process constraint compensation for all constraints.

        Args:
            merged_constraints: Constraints with OBB information
            costmap_data: Costmap data array
            costmap_info: Costmap metadata

        Returns:
            Tuple of (compensated_constraints, compensation_applied)
        """
        compensated_constraints = []
        compensation_applied = False

        for constraint in merged_constraints.constraints:
            # Create compensated constraint (copy original)
            compensated_constraint = constraint

            # Only compensate constraints with null traversability
            if constraint.traversability_constrain == ConstraintCompensationConstants.TRAVERSABILITY_NULL:
                compensation_result = self._compensate_single_constraint(constraint, costmap_data, costmap_info)

                if compensation_result is not None:
                    compensated_constraint.traversability_constrain = compensation_result
                    compensation_applied = True

            compensated_constraints.append(compensated_constraint)

        return compensated_constraints, compensation_applied

    def _compensate_single_constraint(
        self, constraint, costmap_data: npt.NDArray[np.int8], costmap_info: MapMetaData
    ) -> Optional[int]:
        """
        Compensate a single constraint based on geometric cost.

        Args:
            constraint: Single constraint to compensate
            costmap_data: Costmap data array
            costmap_info: Costmap metadata

        Returns:
            New traversability value or None if no compensation needed
        """
        if not constraint.obb_list:
            return None

        # Calculate average cost for each OBB
        obb_costs = []
        for obb in constraint.obb_list:
            avg_cost = self._calculate_obb_average_cost(obb, costmap_data, costmap_info)
            if avg_cost is not None:
                obb_costs.append(avg_cost)

        if not obb_costs:
            return None

        # If any OBB has average cost > threshold, set to non-traversable
        max_avg_cost = max(obb_costs)
        if max_avg_cost > self.geometric_cost_threshold:
            self.get_logger().debug(f"Compensated constraint to non-traversable (max cost: {max_avg_cost:.2f})")
            return ConstraintCompensationConstants.TRAVERSABILITY_NON_TRAVERSABLE
        else:
            self.get_logger().debug(f"Compensated constraint to traversable (max cost: {max_avg_cost:.2f})")
            return ConstraintCompensationConstants.TRAVERSABILITY_TRAVERSABLE

    def _create_compensation_result(
        self, merged_constraints: BehavioralConstraintArray, compensated_constraints: List, compensation_applied: bool
    ) -> BehavioralConstraintArray:
        """
        Create the final compensation result.

        Args:
            merged_constraints: Original merged constraints
            compensated_constraints: Processed constraints
            compensation_applied: Whether compensation was applied

        Returns:
            Final BehavioralConstraintArray result
        """
        if compensation_applied:
            # Create compensated output using reusable object
            compensated_output = BehavioralConstraintArray()
            compensated_output.header = merged_constraints.header
            compensated_output.constraints = compensated_constraints
            return compensated_output
        else:
            # No compensation needed, return original constraints
            return merged_constraints

    def _calculate_obb_average_cost(
        self, obb, costmap_data: npt.NDArray[np.int8], costmap_info: MapMetaData
    ) -> Optional[float]:
        """
        Calculate average geometric cost for OBB region.

        Args:
            obb: OBBInfo2D message containing center, size, and rotation
            costmap_data: Geometric costmap data as numpy array
            costmap_info: Costmap info for coordinate transformation

        Returns:
            Average geometric cost in OBB region or None if invalid
        """
        try:
            # Extract OBB information
            center = np.array([obb.center[0], obb.center[1]])
            size = np.array([obb.size[0], obb.size[1]])

            # Since costmap is already rotated to OBB orientation, treat OBB as AABB
            # Convert world coordinates to grid coordinates
            min_x, min_y = self._world_to_grid(center[0] - size[0] / 2, center[1] - size[1] / 2, costmap_info)
            max_x, max_y = self._world_to_grid(center[0] + size[0] / 2, center[1] + size[1] / 2, costmap_info)

            # Ensure coordinates are within bounds
            min_x = max(0, min_x)
            min_y = max(0, min_y)
            max_x = min(costmap_data.shape[1] - 1, max_x)
            max_y = min(costmap_data.shape[0] - 1, max_y)

            if min_x >= max_x or min_y >= max_y:
                self.get_logger().warn(f"Invalid OBB bounds: min({min_x}, {min_y}) max({max_x}, {max_y})")
                return None

            # Extract cost values in OBB region and filter valid costs in one operation
            obb_costs = costmap_data[min_y : max_y + 1, min_x : max_x + 1]
            
            # Use boolean indexing to filter valid costs efficiently
            valid_mask = obb_costs > ConstraintCompensationConstants.COSTMAP_UNKNOWN
            valid_costs = obb_costs[valid_mask]
            
            if len(valid_costs) < self.min_valid_cost_points:
                self.get_logger().debug("Insufficient valid costs in OBB region")
                return ConstraintCompensationConstants.COSTMAP_FREE

            # Use np.mean with dtype specification for better performance
            avg_cost = np.mean(valid_costs, dtype=np.float64)

            if self.enable_debug_logging:
                self.get_logger().debug(f"OBB avg cost: {avg_cost:.2f} (region: {min_x},{min_y} to {max_x},{max_y})")

            return float(avg_cost)

        except (ValueError, AttributeError, IndexError) as e:
            self.get_logger().error(f"Error calculating OBB average cost: {e}")
            return None

    def _rotate_costmap(
        self, costmap_data: npt.NDArray[np.int8], costmap_info: MapMetaData, rotation_angle: float
    ) -> Tuple[npt.NDArray[np.int8], MapMetaData]:
        """
        Rotate costmap by the specified angle and update origin accordingly.

        Args:
            costmap_data: Costmap data as numpy array
            costmap_info: Costmap info containing origin and resolution
            rotation_angle: Rotation angle in radians

        Returns:
            Tuple of (rotated_costmap_data, updated_costmap_info)
        """
        try:
            # Convert rotation angle from radians to degrees
            rotation_degrees = np.degrees(rotation_angle)

            # Rotate the costmap data
            rotated_data = rotate(costmap_data, rotation_degrees, order=0, reshape=True, cval=-1)

            # Calculate the new origin after rotation
            # The rotation center is the center of the original costmap
            center_x = costmap_data.shape[1] / 2.0
            center_y = costmap_data.shape[0] / 2.0

            # Convert center to world coordinates
            center_world_x = costmap_info.origin.position.x + center_x * costmap_info.resolution
            center_world_y = costmap_info.origin.position.y + center_y * costmap_info.resolution

            # Calculate the new origin after rotation
            # The rotated costmap will have a different size, so we need to adjust the origin
            new_height, new_width = rotated_data.shape
            new_center_x = new_width / 2.0
            new_center_y = new_height / 2.0

            # Calculate the offset from the new center to the original center
            offset_x = (new_center_x - center_x) * costmap_info.resolution
            offset_y = (new_center_y - center_y) * costmap_info.resolution

            # Apply rotation to the offset
            cos_angle = np.cos(rotation_angle)
            sin_angle = np.sin(rotation_angle)
            rotated_offset_x = offset_x * cos_angle - offset_y * sin_angle
            rotated_offset_y = offset_x * sin_angle + offset_y * cos_angle

            # Update the origin
            new_origin_x = center_world_x + rotated_offset_x - new_center_x * costmap_info.resolution
            new_origin_y = center_world_y + rotated_offset_y - new_center_y * costmap_info.resolution

            # Create updated costmap info (shallow copy to avoid deep copy overhead)
            updated_info = copy.copy(costmap_info)
            updated_info.origin = copy.copy(costmap_info.origin)
            updated_info.origin.position.x = new_origin_x
            updated_info.origin.position.y = new_origin_y
            updated_info.width = new_width
            updated_info.height = new_height

            if self.enable_debug_logging:
                self.get_logger().debug(
                    f"Rotated costmap by {rotation_degrees:.2f} degrees, new size: {new_width}x{new_height}"
                )

            return rotated_data, updated_info

        except (ValueError, AttributeError, RuntimeError) as e:
            self.get_logger().error(f"Error rotating costmap: {e}")
            return costmap_data, costmap_info

    def _world_to_grid(self, world_x: float, world_y: float, costmap_info: MapMetaData) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.

        Args:
            world_x: World X coordinate
            world_y: World Y coordinate
            costmap_info: Costmap info containing origin and resolution

        Returns:
            Grid coordinates (x, y)
        """
        # Calculate grid coordinates
        grid_x = int((world_x - costmap_info.origin.position.x) / costmap_info.resolution)
        grid_y = int((world_y - costmap_info.origin.position.y) / costmap_info.resolution)

        return grid_x, grid_y


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the constraint compensation node.

    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = ConstraintCompensationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
