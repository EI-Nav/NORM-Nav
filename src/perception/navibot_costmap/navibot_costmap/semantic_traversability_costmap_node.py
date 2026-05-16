#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Semantic Traversability Costmap Node.

Generates semantic traversability costmap based on enhanced behavioral constraints.
Uses AABB information and traversability constraints from object modeling to create
semantic costmap layer for navigation.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from navibot_interfaces.msg import BehavioralConstraintArray
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)


class SemanticTraversabilityCostmapNode(Node):
    """
    Generate semantic traversability costmap from enhanced behavioral constraints.
    
    This node subscribes to enhanced behavioral constraints containing AABB information
    and traversability constraints, then generates a semantic costmap layer that
    represents semantic traversability information for navigation.
    """
    
    def __init__(self) -> None:
        """Initialize the node with parameters and publishers/subscribers."""
        super().__init__('semantic_traversability_costmap_node')
        
        self._declare_parameters()
        self._load_parameters()
        self._initialize_components()
        
        self.get_logger().info('Semantic Traversability Costmap Node initialized')
    
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        # Input/Output configuration
        self.declare_parameter('constraint_topic', '/compensated_behavioral_constraints')
        self.declare_parameter('output_topic', '/costmap/semantic')
        
        # Map configuration
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('map_width', 20.0)  # Map width in meters
        self.declare_parameter('map_height', 20.0)  # Map height in meters
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('frame_id', 'map')
        
        # Update configuration
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('enable_debug_logging', False)
        
        # TF configuration
        self.declare_parameter('center_frame_id', 'base_link')
        self.declare_parameter('transform_timeout', 1.0)
        
        # Semantic costmap configuration
        self.declare_parameter('free_space_value', 0)
        self.declare_parameter('traversable_value', 1)
        self.declare_parameter('non_traversable_value', 2)
        self.declare_parameter('null_value', -1)
    
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

        self.publish_rate_ = self.get_parameter('publish_rate').value
        
        # Validate parameters
        if self.resolution_ <= 0:
            raise ValueError('resolution must be > 0')
        if self.map_width_ <= 0 or self.map_height_ <= 0:
            raise ValueError('map_width and map_height must be > 0')
        
        # Validate publish rate
        if self.publish_rate_ <= 0 or self.publish_rate_ > 100:
            raise ValueError('publish_rate must be > 0 and <= 100')
        
        # Validate transform timeout
        transform_timeout = self.get_parameter('transform_timeout').value
        if transform_timeout <= 0:
            raise ValueError('transform_timeout must be > 0')
        
        # Calculate pixel dimensions from meter dimensions (at least 1 px)
        self.width_ = max(1, int(self.map_width_ / self.resolution_))
        self.height_ = max(1, int(self.map_height_ / self.resolution_))
        
        # Update configuration
        self.publish_rate_ = float(self.get_parameter('publish_rate').value)
        self.enable_debug_logging_ = self.get_parameter('enable_debug_logging').value
        
        # TF configuration
        self.center_frame_id_ = self.get_parameter('center_frame_id').value
        self.transform_timeout_ = self.get_parameter('transform_timeout').value
        
        # Semantic costmap configuration
        self.free_space_value_ = int(self.get_parameter('free_space_value').value)
        self.traversable_value_ = int(self.get_parameter('traversable_value').value)
        self.non_traversable_value_ = int(self.get_parameter('non_traversable_value').value)
        self.null_value_ = int(self.get_parameter('null_value').value)
        
        # Set logger level
        if self.enable_debug_logging_:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Log key configuration parameters
        self.get_logger().info(f"Constraint topic: {self.constraint_topic_}")
        self.get_logger().info(f"Output topic: {self.output_topic_}")
        self.get_logger().info(f"Map size: {self.map_width_}x{self.map_height_}m ({self.width_}x{self.height_} pixels) at {self.resolution_}m/pixel")
        self.get_logger().info(f"Publish rate: {self.publish_rate_} Hz")
    
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
        
        # Initialize subscriber for compensated behavioral constraints
        self.constraint_sub_ = self.create_subscription(
            BehavioralConstraintArray, self.constraint_topic_, self.constraint_callback, 1)
        
        # Initialize publisher for semantic costmap
        self.costmap_pub_ = self.create_publisher(
            OccupancyGrid, self.output_topic_, map_qos)
        
        # Internal state
        self.latest_constraints_: Optional[BehavioralConstraintArray] = None
        self.processing_: bool = False
        self._last_tf_error_time_ = self.get_clock().now()  # for throttled TF errors
        
        # TF cache for performance optimization
        self._tf_cache_: Dict[str, Tuple[Tuple[float, float], rclpy.time.Time]] = {}
        self._tf_cache_timeout_: float = 0.1  # 100ms cache timeout
        
        # Create timer for periodic publishing
        period = 1.0 / max(1e-6, self.publish_rate_)
        self.timer_ = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info("Subscribers and publishers initialized")
        
        # Parameter change callback for dynamic updates
        self.add_on_set_parameters_callback(self._on_parameters_changed)
    
    def constraint_callback(self, msg: BehavioralConstraintArray) -> None:
        """
        Callback for receiving compensated behavioral constraints.
        
        Args:
            msg: BehavioralConstraintArray message containing compensated behavioral constraints
        """
        try:
            # Store compensated behavioral constraints directly
            self.latest_constraints_ = msg
            
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Received compensated constraints: {len(msg.constraints)} constraints")
            
            self.get_logger().debug(f"Stored compensated constraints with {len(msg.constraints)} constraints")
            
        except (AttributeError, TypeError) as e:
            self.get_logger().error(f"Data structure error processing constraints: {e}")
        except ValueError as e:
            self.get_logger().error(f"Data validation error processing constraints: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error processing constraints: {e}")
    
    def timer_callback(self) -> None:
        """Process and publish semantic costmap at configured rate."""
        if self.latest_constraints_ is None:
            self.get_logger().debug('No constraint data received yet')
            return
        
        if self.processing_:
            self.get_logger().warn('Still processing, skipping this cycle')
            return
        
        self.processing_ = True
        
        try:
            # Build semantic costmap from constraints
            costmap = self._build_semantic_traversability_costmap(
                self.latest_constraints_, 
                self.width_, 
                self.height_, 
                self.resolution_
            )
            
            # Always publish at configured rate
            self.costmap_pub_.publish(costmap)
            self.get_logger().debug(f'Published semantic costmap at ~{self.publish_rate_} Hz')
            
        except (ValueError, TypeError) as e:
            self.get_logger().error(f'Data processing error in semantic costmap: {e}')
        except RuntimeError as e:
            self.get_logger().error(f'Runtime error in semantic costmap: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error publishing semantic costmap: {e}')
        finally:
            self.processing_ = False
    
    def _build_semantic_traversability_costmap(
        self, 
        constraints: Dict, 
        width: int, 
        height: int, 
        resolution: float
    ) -> OccupancyGrid:
        """
        Build semantic traversability costmap from enhanced constraints.
        
        Args:
            constraints: Enhanced behavioral constraints data
            width: Map width in pixels
            height: Map height in pixels
            resolution: Map resolution in meters per pixel
            
        Returns:
            OccupancyGrid message with semantic traversability information
        """
        # Get robot position in map frame
        robot_pose = self._get_robot_pose_in_map()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot pose, using fallback origin")
            robot_x, robot_y = self.origin_x_, self.origin_y_
        else:
            robot_x, robot_y = robot_pose
        
        # Calculate map boundaries centered on robot
        map_min_x = robot_x - (width * resolution / 2.0)
        map_max_x = robot_x + (width * resolution / 2.0)
        map_min_y = robot_y - (height * resolution / 2.0)
        map_max_y = robot_y + (height * resolution / 2.0)
        
        if self.enable_debug_logging_:
            self.get_logger().debug(f"Map boundaries: X=[{map_min_x:.2f}, {map_max_x:.2f}], Y=[{map_min_y:.2f}, {map_max_y:.2f}]")
        
        # Initialize costmap with free space values
        costmap_data = np.full((height, width), self.free_space_value_, dtype=np.int8)
        
        # Group constraints by traversability for batch processing
        constraint_groups = {}
        for constraint in constraints.constraints:
            traversability = constraint.traversability_constrain
            
            # Map traversability constraint to cost value (parameterized)
            if traversability == 0:  # null
                cost_value = self.null_value_
            elif traversability == 1:  # traversable
                cost_value = self.traversable_value_
            elif traversability == 2:  # non-traversable
                cost_value = self.non_traversable_value_
            else:
                self.get_logger().warn(f"Unknown traversability constraint: {traversability}")
                continue
            
            if cost_value not in constraint_groups:
                constraint_groups[cost_value] = []
            
            # Collect valid OBBs for this constraint
            valid_obbs = []
            for obb in constraint.obb_list:
                center = [obb.center[0], obb.center[1]]  # OBBInfo2D only has 2D data
                size = [obb.size[0], obb.size[1]]       # OBBInfo2D only has 2D data
                rotation = obb.rotation
                
                if len(center) < 2 or len(size) < 2:
                    self.get_logger().warn(f"Invalid OBB data: center={center}, size={size}")
                    continue
                
                # Quick AABB check for early filtering
                if self._is_obb_outside_map_bounds(center, size, map_min_x, map_max_x, map_min_y, map_max_y):
                    continue
                
                valid_obbs.append((center, size, rotation))
            
            constraint_groups[cost_value].extend(valid_obbs)
        
        # Process each group of constraints with same cost value
        for cost_value, obb_list in constraint_groups.items():
            if not obb_list:
                continue
                
            # Batch process OBBs with same cost value
            combined_mask = np.zeros((self.height_, self.width_), dtype=bool)
            
            for center, size, rotation in obb_list:
                # Rasterize oriented OBB to grid
                obb_mask = self._rasterize_oriented_obb(
                    center, size, rotation, map_min_x, map_max_x, map_min_y, map_max_y, 
                    resolution, self.height_, self.width_)
                
                # Combine masks using OR operation
                combined_mask |= obb_mask
                
                if self.enable_debug_logging_:
                    mask_count = np.sum(obb_mask)
                    self.get_logger().debug(
                        f"Applied traversability constraint to {mask_count} pixels "
                        f"with cost {cost_value}")
            
            # Apply cost values to all masked regions at once
            costmap_data[combined_mask] = cost_value
        
        # Log costmap statistics
        unique_values, counts = np.unique(costmap_data, return_counts=True)
        self.get_logger().debug(f"Costmap statistics: {dict(zip(unique_values, counts))}")
        
        if self.enable_debug_logging_:
            self.get_logger().debug(f"Map origin: ({map_min_x:.3f}, {map_min_y:.3f})")
            self.get_logger().debug(f"Robot position: ({robot_x:.3f}, {robot_y:.3f})")
        
        # Build OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = self.frame_id_
        occupancy_grid.info.resolution = resolution
        occupancy_grid.info.width = self.width_
        occupancy_grid.info.height = self.height_
        
        # Set origin to map bottom-left corner (robot-centered local map in map frame)
        occupancy_grid.info.origin.position.x = map_min_x
        occupancy_grid.info.origin.position.y = map_min_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        # Use frombuffer for better performance, avoiding data copy
        occupancy_grid.data = costmap_data.flatten().astype(np.int8).tolist()
        
        return occupancy_grid

    def _is_obb_outside_map_bounds(
        self, 
        center: List[float], 
        size: List[float], 
        map_min_x: float, 
        map_max_x: float, 
        map_min_y: float, 
        map_max_y: float
    ) -> bool:
        """
        Quick AABB check to determine if OBB is completely outside map bounds.
        
        Args:
            center: OBB center [x, y]
            size: OBB size [width, height]
            map_min_x: Map minimum X coordinate
            map_max_x: Map maximum X coordinate
            map_min_y: Map minimum Y coordinate
            map_max_y: Map maximum Y coordinate
            
        Returns:
            True if OBB is completely outside map bounds
        """
        # Calculate AABB bounds
        half_w, half_h = size[0] / 2.0, size[1] / 2.0
        aabb_min_x = center[0] - half_w
        aabb_max_x = center[0] + half_w
        aabb_min_y = center[1] - half_h
        aabb_max_y = center[1] + half_h
        
        # Check if completely outside map bounds
        return (aabb_max_x < map_min_x or aabb_min_x > map_max_x or 
                aabb_max_y < map_min_y or aabb_min_y > map_max_y)

    def _on_parameters_changed(self, params):
        """Dynamically handle runtime parameter changes."""
        try:
            for p in params:
                if p.name == 'publish_rate' and p.type_ in (p.Type.DOUBLE, p.Type.INTEGER):
                    new_rate = float(p.value)
                    if new_rate > 0:
                        self.publish_rate_ = new_rate
                        try:
                            self.timer_.cancel()
                        except Exception:
                            pass
                        self.timer_ = self.create_timer(1.0 / self.publish_rate_, self.timer_callback)
            return SetParametersResult(successful=True)
        except Exception as e:
            self.get_logger().warn(f'Failed to apply parameter changes: {e}')
            return SetParametersResult(successful=False)
    
    def _get_robot_pose_in_map(self) -> Optional[Tuple[float, float]]:
        """
        Get robot position in map frame using TF2 with caching.
        
        Returns:
            Tuple of (x, y) position in map frame, or None if transform fails
        """
        # Check cache first
        now = self.get_clock().now()
        cache_key = f"map_{self.center_frame_id_}"
        
        if cache_key in self._tf_cache_:
            cached_pose, cached_time = self._tf_cache_[cache_key]
            if (now - cached_time).nanoseconds / 1e9 < self._tf_cache_timeout_:
                if self.enable_debug_logging_:
                    self.get_logger().debug(f"Using cached robot pose: {cached_pose}")
                return cached_pose
        
        try:
            timeout = Duration(seconds=self.transform_timeout_)
            transform = self.tf_buffer_.lookup_transform(
                'map', self.center_frame_id_, rclpy.time.Time(), timeout)
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Cache the result
            self._tf_cache_[cache_key] = ((x, y), now)
            
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Robot pose in map: ({x:.3f}, {y:.3f})")
            
            return (x, y)
            
        except LookupException as e:
            # Throttle TF errors to avoid log spam
            now = self.get_clock().now()
            if (now - self._last_tf_error_time_).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"TF lookup failed - transform not available: {e}")
                self._last_tf_error_time_ = now
            return None
        except ConnectivityException as e:
            now = self.get_clock().now()
            if (now - self._last_tf_error_time_).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"TF connectivity error - transform tree not connected: {e}")
                self._last_tf_error_time_ = now
            return None
        except ExtrapolationException as e:
            now = self.get_clock().now()
            if (now - self._last_tf_error_time_).nanoseconds / 1e9 > 2.0:
                self.get_logger().warn(f"TF extrapolation error - transform too old: {e}")
                self._last_tf_error_time_ = now
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error getting robot pose: {e}")
            return None
    
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
    ) -> np.ndarray:
        """
        Rasterize an oriented bounding box (OBB) to grid indices.
        
        This function efficiently converts an oriented bounding box to a boolean mask
        using vectorized operations. The algorithm:
        1. Calculates OBB corners in local coordinates
        2. Rotates and translates to map coordinates
        3. Determines the bounding window for processing
        4. Uses vectorized point-in-polygon testing for all pixels in the window
        
        Time Complexity: O(w*h) where w,h are the bounding window dimensions
        Space Complexity: O(w*h) for the mask array
        
        Args:
            obb_center: OBB center [x, y] in map coordinates
            obb_size: OBB size [width, height] in map coordinates
            obb_rotation: OBB rotation angle in radians
            map_min_x: Map minimum X coordinate
            map_max_x: Map maximum X coordinate
            map_min_y: Map minimum Y coordinate
            map_max_y: Map maximum Y coordinate
            resolution: Map resolution in meters per pixel
            costmap_height: Costmap height in pixels
            costmap_width: Costmap width in pixels
            
        Returns:
            Boolean mask array of shape (height, width) indicating OBB coverage
        """
        # Calculate OBB corners in map coordinates
        cx, cy = obb_center[0], obb_center[1]
        w, h = obb_size[0], obb_size[1]
        
        # Calculate OBB corners in local coordinate system
        half_w, half_h = w / 2.0, h / 2.0
        corners_local = np.array([
            [-half_w, -half_h],
            [half_w, -half_h], 
            [half_w, half_h],
            [-half_w, half_h]
        ])
        
        # Rotate corners to map coordinate system (cache trigonometric values)
        cos_r, sin_r = np.cos(obb_rotation), np.sin(obb_rotation)
        rotation_matrix = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
        corners_rotated = corners_local @ rotation_matrix.T
        
        # Translate to map coordinates
        corners_map = corners_rotated + np.array([cx, cy])
        
        # Calculate bounding box of rotated OBB
        min_x, max_x = np.min(corners_map[:, 0]), np.max(corners_map[:, 0])
        min_y, max_y = np.min(corners_map[:, 1]), np.max(corners_map[:, 1])
        
        # Clamp to map boundaries
        min_x = max(min_x, map_min_x)
        max_x = min(max_x, map_max_x)
        min_y = max(min_y, map_min_y)
        max_y = min(max_y, map_max_y)
        
        # Check if OBB intersects with map
        if min_x >= max_x or min_y >= max_y:
            return np.zeros((costmap_height, costmap_width), dtype=bool)
        
        # Convert to grid coordinates
        min_col = max(0, int((min_x - map_min_x) / resolution))
        max_col = min(costmap_width - 1, int((max_x - map_min_x) / resolution))
        min_row = max(0, int((min_y - map_min_y) / resolution))
        max_row = min(costmap_height - 1, int((max_y - map_min_y) / resolution))
        
        # Create mask for the whole costmap and fast-path return if empty window
        mask = np.zeros((costmap_height, costmap_width), dtype=bool)
        if min_row > max_row or min_col > max_col:
            return mask

        # Vectorized test of pixel centers within the candidate window
        # Grid indices within the bounding window (inclusive)
        rows = np.arange(min_row, max_row + 1)
        cols = np.arange(min_col, max_col + 1)
        grid_cols, grid_rows = np.meshgrid(cols, rows)

        # Convert grid indices to map coordinates (pixel centers)
        px = map_min_x + (grid_cols + 0.5) * resolution
        py = map_min_y + (grid_rows + 0.5) * resolution

        # Transform points to OBB local coordinates by inverse rotation and translation
        # Reuse cached trigonometric values for inverse rotation (performance optimization)
        dx = px - cx
        dy = py - cy
        # cos(-r) = cos(r), sin(-r) = -sin(r) - avoid redundant cos/sin calls
        cos_neg_r = cos_r  # cos(-r) = cos(r)
        sin_neg_r = -sin_r  # sin(-r) = -sin(r)
        lx = cos_neg_r * dx - sin_neg_r * dy
        ly = sin_neg_r * dx + cos_neg_r * dy

        # Inside test in local AABB coordinates (vectorized point-in-box test)
        inside = (np.abs(lx) <= half_w) & (np.abs(ly) <= half_h)

        # Write window mask back to the full mask
        mask[min_row:max_row + 1, min_col:max_col + 1] = inside
        return mask



def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the semantic traversability costmap node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = SemanticTraversabilityCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
