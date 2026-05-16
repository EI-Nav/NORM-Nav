#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Directional Constraint Costmap Node.

Generates directional constraint costmap based on enhanced behavioral constraints.
Uses AABB information and direction constraints from object modeling to create
directional constraint costmap layer for navigation.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import List, Optional, Tuple

import numpy as np
import numpy.typing as npt
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
try:
    from tf2_ros.transform_listener import ConnectivityException, ExtrapolationException, LookupException
except ImportError:
    # Fallback for different ROS2 versions
    from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

try:
    from navibot_interfaces.msg import BehavioralConstraintArray
except ImportError:
    # Fallback for development/testing
    class BehavioralConstraintArray:
        def __init__(self):
            self.constraints = []


class DirectionalConstraintCostmapNode(Node):
    """
    Generate directional constraint costmap from enhanced behavioral constraints.
    
    This node subscribes to enhanced behavioral constraints containing AABB information
    and direction constraints, then generates a directional constraint costmap layer
    that represents directional navigation constraints.
    """
    
    def __init__(self) -> None:
        """Initialize the node with parameters and publishers/subscribers."""
        super().__init__('directional_constraint_costmap_node')
        
        self._declare_parameters()
        self._load_parameters()
        self._initialize_components()
        
        self.get_logger().info('Directional Constraint Costmap Node initialized')
    
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        # Input/Output configuration
        self.declare_parameter('constraint_topic', '/compensated_behavioral_constraints')
        self.declare_parameter('output_topic', '/costmap/directional')
        
        # Map configuration
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('map_width', 20.0)  # Map width in meters
        self.declare_parameter('map_height', 20.0)  # Map height in meters
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('frame_id', 'map')
        
        # Update configuration
        self.declare_parameter('publish_rate', 0.5)
        self.declare_parameter('enable_debug_logging', True)
        
        # TF configuration
        self.declare_parameter('center_frame_id', 'base_link')
        self.declare_parameter('transform_timeout', 1.0)
        
        # Directional constraint costmap configuration
        self.declare_parameter('free_space_value', 0)
        
        # Interpolation parameters
        self.declare_parameter('interpolation_alpha', 1.0)
        self.declare_parameter('margin_distance', 0.5)
        self.declare_parameter('c1_cost_value', 100)
        self.declare_parameter('c2_cost_value', 0)
        self.declare_parameter('c3_cost_value', 100)
    
    def _load_parameters(self) -> None:
        """Load and validate all ROS parameters."""
        # Input/Output configuration
        self.constraint_topic_ = self.get_parameter('constraint_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        
        # Map configuration
        self.resolution_ = self.get_parameter('resolution').value
        self.map_width_ = self.get_parameter('map_width').value
        self.map_height_ = self.get_parameter('map_height').value
        self.origin_x_ = self.get_parameter('origin_x').value
        self.origin_y_ = self.get_parameter('origin_y').value
        self.frame_id_ = self.get_parameter('frame_id').value
        
        # Calculate pixel dimensions from meter dimensions
        # Ensure at least 1 pixel
        self.width_ = max(1, int(self.map_width_ / self.resolution_))
        self.height_ = max(1, int(self.map_height_ / self.resolution_))
        
        # Update configuration
        self.publish_rate_: float = self.get_parameter('publish_rate').value
        self.enable_debug_logging_: bool = self.get_parameter('enable_debug_logging').value
        
        # TF configuration
        self.center_frame_id_ = self.get_parameter('center_frame_id').value
        self.transform_timeout_ = self.get_parameter('transform_timeout').value
        
        # Directional constraint costmap configuration
        self.free_space_value_ = self.get_parameter('free_space_value').value
        
        # Interpolation parameters
        self.interpolation_alpha_ = self.get_parameter('interpolation_alpha').value
        self.margin_distance_ = self.get_parameter('margin_distance').value
        self.c1_cost_value_ = self.get_parameter('c1_cost_value').value
        self.c2_cost_value_ = self.get_parameter('c2_cost_value').value
        self.c3_cost_value_ = self.get_parameter('c3_cost_value').value
        
        # Set logger level
        if self.enable_debug_logging_:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Validate parameters
        if not self._validate_parameters():
            raise ValueError("Invalid parameter configuration detected")
        
        # Log key configuration parameters
        self.get_logger().info(f"Constraint topic: {self.constraint_topic_}")
        self.get_logger().info(f"Output topic: {self.output_topic_}")
        self.get_logger().info(f"Map size: {self.map_width_}x{self.map_height_}m ({self.width_}x{self.height_} pixels) at {self.resolution_}m/pixel")
        self.get_logger().info(f"Publish rate: {self.publish_rate_} Hz")
    
    def _validate_parameters(self) -> bool:
        """
        Validate all ROS parameters for correctness and reasonable ranges.
        
        Returns:
            bool: True if all parameters are valid, False otherwise
        """
        try:
            # Validate resolution
            if self.resolution_ <= 0:
                self.get_logger().error(f"Invalid resolution: {self.resolution_}. Must be > 0")
                return False
            
            # Validate map dimensions
            if self.map_width_ <= 0 or self.map_height_ <= 0:
                self.get_logger().error(f"Invalid map dimensions: {self.map_width_}x{self.map_height_}. Must be > 0")
                return False
            
            # Validate publish rate
            if self.publish_rate_ <= 0:
                self.get_logger().error(f"Invalid publish rate: {self.publish_rate_}. Must be > 0")
                return False
            
            # Validate cost values
            # Standard cost values (0-100)
            standard_cost_values = [
                (self.free_space_value_, "free_space_value"),
                (self.c1_cost_value_, "c1_cost_value"),
                (self.c2_cost_value_, "c2_cost_value"),
                (self.c3_cost_value_, "c3_cost_value")
            ]
            
            for value, name in standard_cost_values:
                if not (0 <= value <= 100):
                    self.get_logger().error(f"Invalid {name}: {value}. Must be in range [0, 100]")
                    return False
            
            
            # Validate interpolation parameters
            if self.interpolation_alpha_ <= 0:
                self.get_logger().error(f"Invalid interpolation_alpha: {self.interpolation_alpha_}. Must be > 0")
                return False
            
            if self.margin_distance_ < 0:
                self.get_logger().error(f"Invalid margin_distance: {self.margin_distance_}. Must be >= 0")
                return False
            
            # Validate transform timeout
            if self.transform_timeout_ <= 0:
                self.get_logger().error(f"Invalid transform_timeout: {self.transform_timeout_}. Must be > 0")
                return False
            
            self.get_logger().debug("All parameters validated successfully")
            return True
            
        except (ValueError, TypeError, AttributeError) as e:
            self.get_logger().error(f"Parameter validation error: {e}")
            return False
    
    def _calculate_map_boundaries(self, robot_x: float, robot_y: float, 
                                  width: int, height: int, 
                                  resolution: float) -> Tuple[float, float, float, float]:
        """
        Calculate map boundaries centered on robot position.
        
        Args:
            robot_x: Robot X position in map frame
            robot_y: Robot Y position in map frame
            width: Map width in pixels
            height: Map height in pixels
            resolution: Map resolution in meters per pixel
            
        Returns:
            Tuple of (map_min_x, map_max_x, map_min_y, map_max_y)
        """
        map_min_x = robot_x - (width * resolution / 2.0)
        map_max_x = robot_x + (width * resolution / 2.0)
        map_min_y = robot_y - (height * resolution / 2.0)
        map_max_y = robot_y + (height * resolution / 2.0)
        
        return map_min_x, map_max_x, map_min_y, map_max_y
    
    def _process_constraint(self, constraint, costmap_data: npt.NDArray[np.int8],
                           map_min_x: float, map_max_x: float,
                           map_min_y: float, map_max_y: float,
                           resolution: float) -> None:
        """
        Process a single behavioral constraint and apply it to the costmap.
        
        Args:
            constraint: Single behavioral constraint
            costmap_data: Costmap data array to modify
            map_min_x: Map minimum X coordinate
            map_max_x: Map maximum X coordinate
            map_min_y: Map minimum Y coordinate
            map_max_y: Map maximum Y coordinate
            resolution: Map resolution in meters per pixel
        """
        di = float(constraint.direction_constrain)
        # Skip null direction
        if di == -999.0:
            return

        traversability = int(constraint.traversability_constrain)

        for obb in constraint.obb_list:
            self._process_obb(obb, di, traversability, costmap_data, 
                            map_min_x, map_max_x, map_min_y, map_max_y, resolution)
    
    def _process_obb(self, obb, di: float, traversability: int,
                    costmap_data: npt.NDArray[np.int8],
                    map_min_x: float, map_max_x: float,
                    map_min_y: float, map_max_y: float,
                    resolution: float) -> None:
        """
        Process a single OBB and apply directional constraints to the costmap.
        
        Args:
            obb: Oriented bounding box information
            di: Direction constraint value [-1, 1]
            traversability: Traversability constraint value
            costmap_data: Costmap data array to modify
            map_min_x: Map minimum X coordinate
            map_max_x: Map maximum X coordinate
            map_min_y: Map minimum Y coordinate
            map_max_y: Map maximum Y coordinate
            resolution: Map resolution in meters per pixel
        """
        # OBBInfo2D fields: center[2], size[2], rotation
        center = [float(obb.center[0]), float(obb.center[1])]
        size = [float(obb.size[0]), float(obb.size[1])]
        rotation = float(obb.rotation)

        if len(center) < 2 or len(size) < 2:
            self.get_logger().warn(f"Invalid OBB data: center={center}, size={size}")
            return

        # Expand for non-traversable (enum 2)
        if traversability == 2:
            size = [size[0] + 2 * self.margin_distance_, size[1] + 2 * self.margin_distance_]

        # Rasterize oriented OBB to get mask within map window
        mask, min_row, max_row, min_col, max_col = self._rasterize_oriented_obb(
            center, size, rotation, map_min_x, map_max_x, map_min_y, map_max_y,
            resolution, self.height_, self.width_)

        # If empty window, skip
        if min_row > max_row or min_col > max_col:
            return

        # Apply directional interpolation per column, restricted to mask
        self._apply_directional_interpolation_with_mask(
            costmap_data, mask, min_row, max_row, min_col, max_col, di,
            center, size, rotation, map_min_x, map_min_y, resolution)

        if self.enable_debug_logging_:
            self.get_logger().debug(
                f"Applied directional constraint di={di:.2f} to window "
                f"[{min_row}:{max_row+1}, {min_col}:{max_col+1}]")
    
    def _create_occupancy_grid_msg(self, costmap_data: npt.NDArray[np.int8],
                                   map_min_x: float, map_min_y: float,
                                   width: int, height: int,
                                   resolution: float) -> OccupancyGrid:
        """
        Create OccupancyGrid message from costmap data.
        
        Args:
            costmap_data: Costmap data array
            map_min_x: Map minimum X coordinate
            map_min_y: Map minimum Y coordinate
            width: Map width in pixels
            height: Map height in pixels
            resolution: Map resolution in meters per pixel
            
        Returns:
            OccupancyGrid message
        """
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = self.frame_id_
        occupancy_grid.info.resolution = resolution
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        
        # Set origin based on actual robot position
        occupancy_grid.info.origin.position.x = map_min_x
        occupancy_grid.info.origin.position.y = map_min_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        occupancy_grid.data = costmap_data.flatten().tolist()
        
        return occupancy_grid
    
    def _validate_constraints(self, constraints: BehavioralConstraintArray) -> bool:
        """
        Validate constraint data for correctness and completeness.
        
        Args:
            constraints: Behavioral constraint array to validate
            
        Returns:
            bool: True if constraints are valid, False otherwise
        """
        try:
            if not hasattr(constraints, 'constraints') or constraints.constraints is None:
                self.get_logger().warn("Constraints field is missing or None")
                return False
            
            for i, constraint in enumerate(constraints.constraints):
                # Validate direction constraint
                if not hasattr(constraint, 'direction_constrain'):
                    self.get_logger().warn(f"Constraint {i}: missing direction_constrain field")
                    return False
                
                di = float(constraint.direction_constrain)
                if di != -999.0 and not (-1.0 <= di <= 1.0):
                    self.get_logger().warn(f"Constraint {i}: invalid direction constraint {di}")
                    return False
                
                # Validate traversability constraint
                if not hasattr(constraint, 'traversability_constrain'):
                    self.get_logger().warn(f"Constraint {i}: missing traversability_constrain field")
                    return False
                
                traversability = int(constraint.traversability_constrain)
                if not (0 <= traversability <= 2):
                    self.get_logger().warn(f"Constraint {i}: invalid traversability constraint {traversability}")
                    return False
                
                # Validate OBB list
                if not hasattr(constraint, 'obb_list') or constraint.obb_list is None:
                    self.get_logger().warn(f"Constraint {i}: missing obb_list field")
                    return False
                
                for j, obb in enumerate(constraint.obb_list):
                    if not hasattr(obb, 'center') or not hasattr(obb, 'size') or not hasattr(obb, 'rotation'):
                        self.get_logger().warn(f"Constraint {i}, OBB {j}: missing required fields")
                        return False
                    
                    if len(obb.center) < 2 or len(obb.size) < 2:
                        self.get_logger().warn(f"Constraint {i}, OBB {j}: invalid center or size dimensions")
                        return False
            
            return True
            
        except (AttributeError, ValueError, TypeError) as e:
            self.get_logger().error(f"Constraint validation error: {e}")
            return False
    
    def _validate_costmap(self, costmap: OccupancyGrid) -> bool:
        """
        Validate generated costmap for correctness.
        
        Args:
            costmap: OccupancyGrid message to validate
            
        Returns:
            bool: True if costmap is valid, False otherwise
        """
        try:
            # Check basic structure
            if costmap is None:
                return False
            
            if not hasattr(costmap, 'data') or costmap.data is None:
                self.get_logger().warn("Costmap data is missing")
                return False
            
            if not hasattr(costmap, 'info') or costmap.info is None:
                self.get_logger().warn("Costmap info is missing")
                return False
            
            # Check dimensions
            if costmap.info.width <= 0 or costmap.info.height <= 0:
                self.get_logger().warn(f"Invalid costmap dimensions: {costmap.info.width}x{costmap.info.height}")
                return False
            
            if costmap.info.resolution <= 0:
                self.get_logger().warn(f"Invalid costmap resolution: {costmap.info.resolution}")
                return False
            
            # Check data size matches dimensions
            expected_size = costmap.info.width * costmap.info.height
            if len(costmap.data) != expected_size:
                self.get_logger().warn(f"Costmap data size mismatch: expected {expected_size}, got {len(costmap.data)}")
                return False
            
            # Check data values are in valid range
            for value in costmap.data:
                if not (-1 <= value <= 100):
                    self.get_logger().warn(f"Invalid costmap value: {value}")
                    return False
            
            return True
            
        except (AttributeError, ValueError, TypeError) as e:
            self.get_logger().error(f"Costmap validation error: {e}")
            return False
    
    def _initialize_components(self) -> None:
        """Initialize ROS components (publishers, subscribers)."""
        # QoS profile for latched map topics
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Initialize TF2 components
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        # Initialize subscriber for enhanced behavioral constraints
        self.constraint_sub_ = self.create_subscription(
            BehavioralConstraintArray, self.constraint_topic_, self.constraint_callback, 1)
        
        # Initialize publisher for directional constraint costmap
        self.costmap_pub_ = self.create_publisher(
            OccupancyGrid, self.output_topic_, map_qos)
        
        # Internal state
        self.latest_constraints_: Optional[BehavioralConstraintArray] = None
        self.processing_: bool = False
        self._last_tf_error_time_ = self.get_clock().now()
        
        # Create timer for periodic publishing
        period = 1.0 / max(1e-6, float(self.publish_rate_))
        self.timer_ = self.create_timer(period, self.timer_callback)
        
        # Parameter change callback for dynamic updates
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        
        self.get_logger().info("Subscribers and publishers initialized")
    
    def constraint_callback(self, msg: BehavioralConstraintArray) -> None:
        """
        Callback for receiving enhanced behavioral constraints.
        
        Args:
            msg: BehavioralConstraintArray message containing enhanced behavioral constraints
        """
        try:
            # Validate message structure
            if not hasattr(msg, 'constraints') or msg.constraints is None:
                self.get_logger().warn("Received invalid constraint message: missing constraints field")
                return
            
            # Validate constraint count
            if len(msg.constraints) == 0:
                self.get_logger().debug("Received empty constraint list")
                return
            
            # Store structured compensated behavioral constraints directly
            self.latest_constraints_ = msg
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Received compensated constraints: {len(msg.constraints)} constraints")
            self.get_logger().info(f"Stored compensated constraints with {len(msg.constraints)} constraints")
            
        except AttributeError as e:
            self.get_logger().error(f"Invalid constraint message structure: {e}")
        except ValueError as e:
            self.get_logger().error(f"Invalid constraint data values: {e}")
        except TypeError as e:
            self.get_logger().error(f"Constraint processing error: {e}")
    
    def timer_callback(self) -> None:
        """Process and publish directional constraint costmap at configured rate."""
        if self.latest_constraints_ is None:
            self.get_logger().debug('No constraint data received yet')
            return
        
        if self.processing_:
            self.get_logger().warn('Still processing, skipping this cycle')
            return
        
        self.processing_ = True
        
        try:
            # Validate constraints before processing
            if not self._validate_constraints(self.latest_constraints_):
                self.get_logger().warn("Invalid constraints detected, skipping costmap generation")
                return
            
            # Build directional constraint costmap from constraints
            costmap = self._build_directional_constraint_costmap(
                self.latest_constraints_, 
                self.width_, 
                self.height_, 
                self.resolution_, 
                (self.origin_x_, self.origin_y_)
            )
            
            # Validate costmap before publishing
            if costmap is None or not self._validate_costmap(costmap):
                self.get_logger().warn("Generated invalid costmap, skipping publication")
                return
            
            # Publish costmap
            self.costmap_pub_.publish(costmap)
            self.get_logger().debug(f'Published directional constraint costmap at {self.publish_rate_} Hz')
            
        except ValueError as e:
            self.get_logger().error(f'Invalid data in costmap generation: {e}')
        except RuntimeError as e:
            self.get_logger().error(f'Runtime error in costmap generation: {e}')
        except AttributeError as e:
            self.get_logger().error(f'Costmap generation error: {e}')
        finally:
            self.processing_ = False
    
    def _build_directional_constraint_costmap(
        self, 
        constraints: BehavioralConstraintArray, 
        width: int, 
        height: int, 
        resolution: float, 
        origin: Tuple[float, float]
    ) -> OccupancyGrid:
        """
        Build directional constraint costmap from enhanced constraints.
        
        Args:
            constraints: Enhanced behavioral constraints data
            width: Map width in pixels
            height: Map height in pixels
            resolution: Map resolution in meters per pixel
            origin: Map origin (x, y) in meters
            
        Returns:
            OccupancyGrid message with directional constraint information
        """
        # Get robot position in map frame
        robot_pose = self._get_robot_pose_in_map()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot pose, using fallback origin")
            robot_x, robot_y = origin[0], origin[1]
        else:
            robot_x, robot_y = robot_pose
            # Update origin based on robot position
            origin = (robot_x, robot_y)
        
        # Calculate map boundaries centered on robot
        map_min_x, map_max_x, map_min_y, map_max_y = self._calculate_map_boundaries(
            robot_x, robot_y, width, height, resolution)
        
        if self.enable_debug_logging_:
            self.get_logger().debug(f"Map boundaries: X=[{map_min_x:.2f}, {map_max_x:.2f}], Y=[{map_min_y:.2f}, {map_max_y:.2f}]")
        
        # Initialize costmap with free space values
        costmap_data = np.full((height, width), self.free_space_value_, dtype=np.int8)
        
        # Process structured constraints
        for constraint in constraints.constraints:
            self._process_constraint(constraint, costmap_data, map_min_x, map_max_x, 
                                   map_min_y, map_max_y, resolution)
        
        if self.enable_debug_logging_:
            unique_values, counts = np.unique(costmap_data, return_counts=True)
            self.get_logger().debug(f"Costmap statistics: {dict(zip(unique_values, counts))}")
        
        # Build OccupancyGrid message
        return self._create_occupancy_grid_msg(costmap_data, map_min_x, map_min_y, 
                                             width, height, resolution)
    

    def _apply_directional_interpolation_with_mask(
        self,
        costmap_data: npt.NDArray[np.int8],
        mask: np.ndarray,
        min_row: int,
        max_row: int,
        min_col: int,
        max_col: int,
        di: float,
        obb_center: List[float],
        obb_size: List[float],
        obb_rotation: float,
        map_min_x: float,
        map_min_y: float,
        resolution: float
    ) -> None:
        """
        Apply directional interpolation along the minor axis in OBB local coordinate system,
        only assigning values to pixels where mask is True.
        
        Uses the minor axis (size[1] corresponding axis) of OBB local coordinate system for interpolation.
        di=-1 indicates left direction, di=+1 indicates right direction.
        """
        cx, cy = obb_center[0], obb_center[1]
        half_h = obb_size[1] / 2.0
        
        # Minor axis range in local coordinate system
        ly1, ly3 = -half_h, half_h
        ly2 = ly1 + (1.0 - di) / 2.0 * (ly3 - ly1)
        
        # Pre-compute trigonometric values
        cos_neg = np.cos(-obb_rotation)
        sin_neg = np.sin(-obb_rotation)
        
        # Create coordinate grids for vectorized operations
        rows = np.arange(min_row, max_row + 1)
        cols = np.arange(min_col, max_col + 1)
        grid_rows, grid_cols = np.meshgrid(rows, cols, indexing='ij')
        
        # Extract only masked pixels for processing
        masked_rows = grid_rows[mask[min_row:max_row + 1, min_col:max_col + 1]]
        masked_cols = grid_cols[mask[min_row:max_row + 1, min_col:max_col + 1]]
        
        if len(masked_rows) == 0:
            return
        
        # Vectorized coordinate transformation
        px = map_min_x + (masked_cols + 0.5) * resolution
        py = map_min_y + (masked_rows + 0.5) * resolution
        
        # Transform to OBB local coordinates
        dx = px - cx
        dy = py - cy
        ly = sin_neg * dx + cos_neg * dy
        
        # Vectorized piecewise interpolation
        cost = np.zeros_like(ly)
        
        # First segment: ly <= ly2
        mask1 = ly <= ly2
        if np.any(mask1):
            if ly2 > ly1:
                t = (ly[mask1] - ly1) / (ly2 - ly1)
                cost[mask1] = (self.c1_cost_value_ + 
                             (self.c2_cost_value_ - self.c1_cost_value_) * 
                             (t ** self.interpolation_alpha_))
            else:
                cost[mask1] = self.c1_cost_value_
        
        # Second segment: ly > ly2
        mask2 = ly > ly2
        if np.any(mask2):
            if ly3 > ly2:
                t = (ly[mask2] - ly2) / (ly3 - ly2)
                cost[mask2] = (self.c2_cost_value_ + 
                             (self.c3_cost_value_ - self.c2_cost_value_) * 
                             (t ** self.interpolation_alpha_))
            else:
                cost[mask2] = self.c2_cost_value_
        
        # Clamp cost values and convert to int8
        cost = np.clip(cost, 0, 100)
        cost_int = np.round(cost).astype(np.int8)
        
        # Assign costs to masked pixels
        costmap_data[masked_rows, masked_cols] = cost_int
    
    def _get_robot_pose_in_map(self) -> Optional[Tuple[float, float]]:
        """
        Get robot position in map frame using TF2.
        
        Returns:
            Tuple of (x, y) position in map frame, or None if transform fails
        """
        try:
            timeout = Duration(seconds=self.transform_timeout_)
            transform = self.tf_buffer_.lookup_transform(
                'map', self.center_frame_id_, rclpy.time.Time(), timeout)
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Robot pose in map: ({x:.3f}, {y:.3f})")
            
            return (x, y)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            now = self.get_clock().now()
            if (now - self._last_tf_error_time_).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"Failed to get robot pose: {e}")
                self._last_tf_error_time_ = now
            return None
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"Robot pose calculation error: {e}")
            return None

    def _on_parameters_changed(self, params):
        """Dynamically handle runtime parameter changes (e.g., publish_rate)."""
        from rcl_interfaces.msg import SetParametersResult
        try:
            for p in params:
                if p.name == 'publish_rate' and p.type_ in (p.Type.DOUBLE, p.Type.INTEGER):
                    new_rate = float(p.value)
                    if new_rate > 0:
                        self.publish_rate_ = new_rate
                        try:
                            self.timer_.cancel()
                        except (AttributeError, RuntimeError):
                            # Timer may not exist or already cancelled
                            pass
                        self.timer_ = self.create_timer(1.0 / self.publish_rate_, self.timer_callback)
            return SetParametersResult(successful=True)
        except (ValueError, TypeError, AttributeError) as e:
            self.get_logger().warn(f'Failed to apply parameter changes: {e}')
            return SetParametersResult(successful=False)
    

    def _rasterize_oriented_obb(
        self,
        obb_center: List[float],
        obb_size: List[float],
        obb_rotation: float,
        map_min_x: float,
        map_max_x: float,
        map_min_y: float,
        map_max_y: float,
        resolution: float,
        costmap_height: int,
        costmap_width: int
    ) -> Tuple[np.ndarray, int, int, int, int]:
        """
        Rasterize a rotated 2D OBB into a boolean mask and return the row/column range of the mask window.
        Returns: (mask, min_row, max_row, min_col, max_col)
        """
        cx, cy = obb_center[0], obb_center[1]
        w, h = obb_size[0], obb_size[1]

        half_w, half_h = w / 2.0, h / 2.0
        
        # Pre-compute rotation matrix
        cos_r, sin_r = np.cos(obb_rotation), np.sin(obb_rotation)
        rotation_matrix = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
        
        # Calculate OBB corners in local coordinates
        corners_local = np.array([
            [-half_w, -half_h],
            [half_w, -half_h],
            [half_w, half_h],
            [-half_w, half_h]
        ])

        # Transform corners to map coordinates
        corners_rotated = corners_local @ rotation_matrix.T
        corners_map = corners_rotated + np.array([cx, cy])

        # Calculate bounding box
        min_x, max_x = np.min(corners_map[:, 0]), np.max(corners_map[:, 0])
        min_y, max_y = np.min(corners_map[:, 1]), np.max(corners_map[:, 1])

        # Clamp to map boundaries
        min_x = max(min_x, map_min_x)
        max_x = min(max_x, map_max_x)
        min_y = max(min_y, map_min_y)
        max_y = min(max_y, map_max_y)

        mask = np.zeros((costmap_height, costmap_width), dtype=bool)
        if min_x >= max_x or min_y >= max_y:
            return mask, 1, 0, 1, 0  # empty window

        # Calculate grid indices
        min_col = max(0, int((min_x - map_min_x) / resolution))
        max_col = min(costmap_width - 1, int((max_x - map_min_x) / resolution))
        min_row = max(0, int((min_y - map_min_y) / resolution))
        max_row = min(costmap_height - 1, int((max_y - map_min_y) / resolution))

        if min_row > max_row or min_col > max_col:
            return mask, 1, 0, 1, 0

        # Create coordinate grids
        rows = np.arange(min_row, max_row + 1)
        cols = np.arange(min_col, max_col + 1)
        grid_cols, grid_rows = np.meshgrid(cols, rows)

        # Calculate pixel coordinates
        px = map_min_x + (grid_cols + 0.5) * resolution
        py = map_min_y + (grid_rows + 0.5) * resolution

        # Transform to OBB local coordinates
        dx = px - cx
        dy = py - cy
        
        # Pre-compute inverse rotation
        cos_neg_r = np.cos(-obb_rotation)
        sin_neg_r = np.sin(-obb_rotation)
        lx = cos_neg_r * dx - sin_neg_r * dy
        ly = sin_neg_r * dx + cos_neg_r * dy

        # Check if points are inside OBB
        inside = (np.abs(lx) <= half_w) & (np.abs(ly) <= half_h)
        mask[min_row:max_row + 1, min_col:max_col + 1] = inside
        
        return mask, min_row, max_row, min_col, max_col


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the directional constraint costmap node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = DirectionalConstraintCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
