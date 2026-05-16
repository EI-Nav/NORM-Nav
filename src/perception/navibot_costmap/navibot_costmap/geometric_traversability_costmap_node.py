#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Geometric Traversability Costmap Node.

Converts LIO point cloud to global costmap with ground-relative filtering.
Fixed map boundaries are defined in target_frame_id and transformed to input
point cloud frame for processing. Output map is published in input frame.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import numpy.typing as npt
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
try:
    from tf2_ros.exceptions import ConnectivityException, ExtrapolationException, LookupException
except ImportError:
    # Fallback for older ROS2 versions
    from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


# Constants
MIN_POINTS_FOR_GROUND_ESTIMATION = 10
MIN_GROUND_CANDIDATE_POINTS = 100
MIN_VALID_POINTS_THRESHOLD = 10
MAX_MAP_SIZE_PIXELS = 10000  # Maximum map size to prevent memory issues
MAX_MAP_SIZE_METERS = 500.0  # Maximum map size in meters


class GeometricTraversabilityCostmapNode(Node):
    """
    Convert laser map point cloud to geometric traversability costmap with ground-relative filtering.
    
    This node subscribes to LIO point cloud output, processes it in the input point
    cloud's native frame, and publishes an OccupancyGrid map. Fixed map boundaries
    are defined in target_frame_id and automatically transformed to the input frame
    for efficient processing without point cloud transformation.
    """
    
    def __init__(self) -> None:
        """Initialize the node with parameters and publishers/subscribers."""
        super().__init__('geometric_traversability_costmap_node')
        
        # Declare parameters
        self._declare_basic_parameters()
        self._declare_accumulation_parameters()
        self._declare_fixed_map_parameters()
        
        # Get parameter values
        self._get_parameter_values()
        
        # Validate parameters
        self._validate_parameters()
        
        # QoS profile for latched map topics
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.laser_map_callback, 1)
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid, self.output_topic, map_qos)
        
        # Internal state
        self.latest_points = None
        self.latest_header = None
        self.processing = False
        
        # Accumulation mode state
        self.accumulated_points = deque(maxlen=self.accumulation_window_size)
        self.frame_counter = 0
        
        # Fixed map boundary state
        self.map_bounds_initialized = False
        self.fixed_min_x = None
        self.fixed_max_x = None
        self.fixed_min_y = None
        self.fixed_max_y = None
        self.fixed_width_pixels = None
        self.fixed_height_pixels = None
        self.fixed_origin_x = None
        self.fixed_origin_y = None
        
        # Performance optimization caches
        self._cached_ground_grid_res_x = None
        self._cached_ground_grid_res_y = None
        self._cached_ground_grid_width = None
        self._cached_ground_grid_height = None
        
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self._log_initialization_info()
    
    def laser_map_callback(self, msg: PointCloud2) -> None:
        """
        Store latest received point cloud for processing.
        
        Point cloud is processed in its native frame without transformation.
        
        Args:
            msg: Input PointCloud2 message from LIO
        """
        try:
            points = self.pointcloud2_to_array(msg)
            if len(points) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
            
            if self.accumulation_mode:
                # Accumulation mode: accumulate every N frames
                self.frame_counter += 1
                if self.frame_counter >= self.accumulation_frame_interval:
                    self.accumulated_points.append(points)
                    self.frame_counter = 0
                
                # Update header from latest frame
                self.latest_header = msg.header
            else:
                # Single frame mode: keep only the latest
                self.latest_points = points
                self.latest_header = msg.header
            
        except Exception as e:
            self.get_logger().error(f'Error storing point cloud: {e}')
    
    def timer_callback(self) -> None:
        """Process and publish costmap at configured rate."""
        if self.accumulation_mode:
            # Accumulation mode: check if we have accumulated points
            if len(self.accumulated_points) == 0:
                self.get_logger().debug('No accumulated point cloud data yet')
                return
            if self.latest_header is None:
                return
        else:
            # Single frame mode: check for latest points
            if self.latest_points is None:
                self.get_logger().debug('No point cloud data received yet')
                return
        
        if self.processing:
            self.get_logger().warn('Still processing, skipping this cycle')
            return
        
        self.processing = True
        
        try:
            import time
            start_time = time.time()
            
            if self.accumulation_mode:
                # Merge all accumulated point clouds
                merged_points = np.vstack(list(self.accumulated_points))
                self.get_logger().debug(
                    f'Converting {len(merged_points)} points (from {len(self.accumulated_points)} frames) to costmap...')
                costmap = self.convert_to_costmap(merged_points, self.latest_header)
            else:
                # Single frame mode
                self.get_logger().debug(f'Converting {len(self.latest_points)} points to costmap...')
                costmap = self.convert_to_costmap(self.latest_points, self.latest_header)
            
            processing_time = time.time() - start_time
            self.costmap_publisher.publish(costmap)
            self.get_logger().info(f'Published costmap at {self.publish_rate} Hz (processing time: {processing_time:.3f}s)')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing costmap: {e}')
        finally:
            self.processing = False
    
    def pointcloud2_to_array(self, cloud_msg: PointCloud2) -> npt.NDArray[np.float32]:
        """
        Convert PointCloud2 message to Nx3 numpy array.
        
        Args:
            cloud_msg: Input point cloud message
            
        Returns:
            Numpy array of shape (N, 3) with [x, y, z] coordinates
        """
        points_list = []
        for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list, dtype=np.float32)
    
    def transform_boundaries(
        self, min_x: float, max_x: float, min_y: float, max_y: float,
        from_frame: str, to_frame: str, timestamp
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        Transform map boundaries from one frame to another with retry mechanism.
        
        Transforms the four corners of the boundary box and computes new min/max.
        Includes retry logic for improved reliability.
        
        Args:
            min_x: Minimum X boundary in source frame
            max_x: Maximum X boundary in source frame
            min_y: Minimum Y boundary in source frame
            max_y: Maximum Y boundary in source frame
            from_frame: Source coordinate frame
            to_frame: Target coordinate frame
            timestamp: Timestamp for TF lookup
            
        Returns:
            Tuple of (min_x, max_x, min_y, max_y) in target frame, or None if transform fails
        """
        max_retries = 3
        retry_delay = 0.1  # seconds
        
        for attempt in range(max_retries):
            try:
                timeout = Duration(seconds=self.transform_timeout)
                transform = self.tf_buffer.lookup_transform(
                    to_frame, from_frame, timestamp, timeout)
                
                # Define four corners of the boundary box
                corners = np.array([
                    [min_x, min_y, 0.0],
                    [max_x, min_y, 0.0],
                    [min_x, max_y, 0.0],
                    [max_x, max_y, 0.0]
                ], dtype=np.float32)
                
                # Apply rotation and translation
                trans = transform.transform.translation
                rot = transform.transform.rotation
                x, y, z, w = rot.x, rot.y, rot.z, rot.w
                
                # Convert quaternion to rotation matrix (3x3)
                # This is the standard quaternion to rotation matrix conversion
                rotation_matrix = np.array([
                    [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                    [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                    [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])
                
                # Apply transformation: R * corners + translation
                transformed_corners = np.dot(corners, rotation_matrix.T) + np.array([trans.x, trans.y, trans.z])
                
                # Compute new boundaries
                new_min_x = float(np.min(transformed_corners[:, 0]))
                new_max_x = float(np.max(transformed_corners[:, 0]))
                new_min_y = float(np.min(transformed_corners[:, 1]))
                new_max_y = float(np.max(transformed_corners[:, 1]))
                
                self.get_logger().debug(
                    f'Transformed boundaries from {from_frame} to {to_frame}: '
                    f'X=[{min_x:.2f}, {max_x:.2f}] -> [{new_min_x:.2f}, {new_max_x:.2f}], '
                    f'Y=[{min_y:.2f}, {max_y:.2f}] -> [{new_min_y:.2f}, {new_max_y:.2f}]')
                
                return (new_min_x, new_max_x, new_min_y, new_max_y)
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                if attempt < max_retries - 1:
                    self.get_logger().warn(f'TF transform failed (attempt {attempt + 1}/{max_retries}): {e}. Retrying...')
                    import time
                    time.sleep(retry_delay)
                    continue
                else:
                    self.get_logger().error(f'Failed to transform boundaries after {max_retries} attempts: {e}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Unexpected error in boundary transform: {e}')
                return None
        
        return None
    
    def convert_to_costmap(
        self, points: npt.NDArray[np.float32], header: Header
    ) -> OccupancyGrid:
        """
        Convert point cloud to OccupancyGrid with ground-relative filtering.
        
        Args:
            points: Point cloud array of shape (N, 3) with [x, y, z]
            header: ROS header for timestamp and frame information
            
        Returns:
            OccupancyGrid message ready for navigation
        """
        min_bounds = np.min(points, axis=0)
        max_bounds = np.max(points, axis=0)
        min_x, min_y, min_z = min_bounds
        max_x, max_y, max_z = max_bounds
        
        self.get_logger().debug(f'Current bounds: X=[{min_x:.2f}, {max_x:.2f}], '
                              f'Y=[{min_y:.2f}, {max_y:.2f}], Z=[{min_z:.2f}, {max_z:.2f}]')
        
        # Calculate map dimensions
        if self.use_fixed_map:
            input_frame = header.frame_id
            need_transform = (input_frame != self.target_frame_id)
            
            if not self.map_bounds_initialized:
                if self.use_custom_boundaries:
                    # User-defined boundaries are in target_frame_id
                    if need_transform:
                        # Transform boundaries to input frame
                        transformed = self.transform_boundaries(
                            self.fixed_map_min_x_param, self.fixed_map_max_x_param,
                            self.fixed_map_min_y_param, self.fixed_map_max_y_param,
                            self.target_frame_id, input_frame, header.stamp)
                        
                        if transformed is None:
                            self.get_logger().warn(
                                f'Failed to transform boundaries from {self.target_frame_id} to {input_frame}, '
                                f'using point cloud bounds instead')
                            self.fixed_min_x = min_x
                            self.fixed_max_x = max_x
                            self.fixed_min_y = min_y
                            self.fixed_max_y = max_y
                        else:
                            self.fixed_min_x, self.fixed_max_x, self.fixed_min_y, self.fixed_max_y = transformed
                            if need_transform:
                                self.get_logger().info(
                                    f'Initialized bounds: {self.target_frame_id} -> {input_frame}: '
                                    f'X=[{self.fixed_min_x:.2f}, {self.fixed_max_x:.2f}], '
                                    f'Y=[{self.fixed_min_y:.2f}, {self.fixed_max_y:.2f}]')
                                self.get_logger().info('Boundaries will be re-transformed each frame')
                    else:
                        # No transformation needed
                        self.fixed_min_x = self.fixed_map_min_x_param
                        self.fixed_max_x = self.fixed_map_max_x_param
                        self.fixed_min_y = self.fixed_map_min_y_param
                        self.fixed_max_y = self.fixed_map_max_y_param
                        self.get_logger().info(f'Applied user-defined fixed map bounds in {input_frame}: '
                                              f'X=[{self.fixed_min_x:.2f}, {self.fixed_max_x:.2f}], '
                                              f'Y=[{self.fixed_min_y:.2f}, {self.fixed_max_y:.2f}]')
                else:
                    # Initialize fixed map bounds from first frame (already in correct frame)
                    self.fixed_min_x = min_x
                    self.fixed_max_x = max_x
                    self.fixed_min_y = min_y
                    self.fixed_max_y = max_y
                    self.get_logger().info(
                        f'Auto-initialized fixed map bounds from first frame in {header.frame_id}: '
                        f'X=[{self.fixed_min_x:.2f}, {self.fixed_max_x:.2f}], '
                        f'Y=[{self.fixed_min_y:.2f}, {self.fixed_max_y:.2f}]')
                self.map_bounds_initialized = True
            elif need_transform and self.use_custom_boundaries:
                # Re-transform boundaries each frame when frames differ
                # This allows boundaries defined in robot frame (base_link) to follow the robot
                transformed = self.transform_boundaries(
                    self.fixed_map_min_x_param, self.fixed_map_max_x_param,
                    self.fixed_map_min_y_param, self.fixed_map_max_y_param,
                    self.target_frame_id, input_frame, header.stamp)
                
                if transformed is not None:
                    self.fixed_min_x, self.fixed_max_x, self.fixed_min_y, self.fixed_max_y = transformed
                    self.get_logger().debug(
                        f'Re-transformed bounds: {self.target_frame_id} -> {input_frame}: '
                        f'X=[{self.fixed_min_x:.2f}, {self.fixed_max_x:.2f}], '
                        f'Y=[{self.fixed_min_y:.2f}, {self.fixed_max_y:.2f}]')
                else:
                    self.get_logger().warn('Failed to re-transform boundaries, using previous bounds')
            elif self.expand_map:
                # Expand map bounds if needed (only when frames are the same)
                if min_x < self.fixed_min_x or max_x > self.fixed_max_x or \
                   min_y < self.fixed_min_y or max_y > self.fixed_max_y:
                    self.fixed_min_x = min(self.fixed_min_x, min_x)
                    self.fixed_max_x = max(self.fixed_max_x, max_x)
                    self.fixed_min_y = min(self.fixed_min_y, min_y)
                    self.fixed_max_y = max(self.fixed_max_y, max_y)
                    self.get_logger().info(f'Expanded map bounds: '
                                          f'X=[{self.fixed_min_x:.2f}, {self.fixed_max_x:.2f}], '
                                          f'Y=[{self.fixed_min_y:.2f}, {self.fixed_max_y:.2f}]')
            
            # Use fixed bounds
            min_x = self.fixed_min_x
            max_x = self.fixed_max_x
            min_y = self.fixed_min_y
            max_y = self.fixed_max_y
        
        world_width = (max_x - min_x) + self.margin_left + self.margin_right
        world_height = (max_y - min_y) + self.margin_top + self.margin_bottom
        origin_x = min_x - self.margin_left
        origin_y = min_y - self.margin_bottom
        width_pixels = int(world_width / self.resolution)
        height_pixels = int(world_height / self.resolution)
        
        # Check for excessive map size to prevent memory issues
        if width_pixels > MAX_MAP_SIZE_PIXELS or height_pixels > MAX_MAP_SIZE_PIXELS:
            self.get_logger().error(f'Map size too large: {width_pixels}x{height_pixels} pixels. '
                                  f'Maximum allowed: {MAX_MAP_SIZE_PIXELS}x{MAX_MAP_SIZE_PIXELS}')
            raise ValueError('Map size exceeds maximum allowed dimensions')
        
        if world_width > MAX_MAP_SIZE_METERS or world_height > MAX_MAP_SIZE_METERS:
            self.get_logger().error(f'Map size too large: {world_width:.2f}x{world_height:.2f} meters. '
                                  f'Maximum allowed: {MAX_MAP_SIZE_METERS}x{MAX_MAP_SIZE_METERS}')
            raise ValueError('Map size exceeds maximum allowed dimensions')
        
        self.get_logger().debug(f'Map size: {width_pixels}x{height_pixels} pixels')
        
        # Estimate ground and create grid
        ground_height_grid = self.estimate_ground_height_grid(
            points, origin_x, origin_y, width_pixels, height_pixels)
        grid = self.create_occupancy_grid_with_ground_filtering(
            points, origin_x, origin_y, width_pixels, height_pixels, ground_height_grid)
        
        # Build OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = header.frame_id
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = width_pixels
        occupancy_grid.info.height = height_pixels
        occupancy_grid.info.origin.position.x = origin_x
        occupancy_grid.info.origin.position.y = origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        occupancy_data = np.full((height_pixels, width_pixels), self.free_space_value, dtype=np.int8)
        occupancy_data[grid >= self.occupied_threshold] = 100
        occupancy_grid.data = occupancy_data.flatten().tolist()
        
        return occupancy_grid
    
    def estimate_ground_height_grid(
        self,
        points: npt.NDArray[np.float32],
        origin_x: float,
        origin_y: float,
        width_pixels: int,
        height_pixels: int
    ) -> npt.NDArray[np.float32]:
        """
        Estimate ground height using coarse grid and 5th percentile filtering.
        
        Divides the map into a coarse grid and estimates ground height for each cell
        by taking the 5th percentile of Z values from points below the height threshold.
        Missing cells are filled using nearest neighbor interpolation.
        
        Args:
            points: Point cloud array of shape (N, 3) with [x, y, z]
            origin_x: Map origin X coordinate in meters
            origin_y: Map origin Y coordinate in meters
            width_pixels: Map width in pixels
            height_pixels: Map height in pixels
            
        Returns:
            Ground height grid of shape (grid_height, grid_width)
        """
        grid_width = max(1, int(width_pixels * self.resolution / self.ground_grid_size))
        grid_height = max(1, int(height_pixels * self.resolution / self.ground_grid_size))
        
        self.get_logger().debug(f'Ground grid: {grid_width}x{grid_height}')
        
        ground_candidate_mask = points[:, 2] <= self.ground_estimation_height_threshold
        ground_candidate_points = points[ground_candidate_mask]
        
        original_count = len(points)
        filtered_count = len(ground_candidate_points)
        self.get_logger().debug(
            f'Ground estimation: filtered {original_count} -> {filtered_count} points '
            f'(Z <= {self.ground_estimation_height_threshold:.2f}m, removed {original_count - filtered_count} high points)')
        
        if filtered_count < MIN_GROUND_CANDIDATE_POINTS:
            self.get_logger().warn(f'Too few ground candidate points ({filtered_count}), using all points')
            ground_candidate_points = points
        
        # Convert points to ground grid coordinates
        grid_x = np.clip(((ground_candidate_points[:, 0] - origin_x) / self.ground_grid_size).astype(int), 0, grid_width - 1)
        grid_y = np.clip(((ground_candidate_points[:, 1] - origin_y) / self.ground_grid_size).astype(int), 0, grid_height - 1)
        # Convert 2D grid coordinates to 1D linear indices for efficient processing
        grid_indices = grid_y * grid_width + grid_x
        ground_height_grid = np.full((grid_height, grid_width), np.nan)
        
        # Sort points by grid index for efficient grouping
        sort_idx = np.argsort(grid_indices)
        sorted_indices = grid_indices[sort_idx]
        sorted_z_values = ground_candidate_points[sort_idx, 2]
        # Find unique grid cells and count points per cell
        unique_indices, _, counts = np.unique(
            sorted_indices, return_inverse=True, return_counts=True)
        
        current_pos = 0
        for linear_idx, count in zip(unique_indices, counts):
            if count > MIN_POINTS_FOR_GROUND_ESTIMATION:
                cell_z_values = sorted_z_values[current_pos:current_pos + count]
                gy = linear_idx // grid_width
                gx = linear_idx % grid_width
                ground_height_grid[gy, gx] = np.percentile(cell_z_values, 5)
            current_pos += count
        
        return self.interpolate_ground_heights(ground_height_grid)
    
    def interpolate_ground_heights(
        self, ground_grid: npt.NDArray[np.float32]
    ) -> npt.NDArray[np.float32]:
        """
        Interpolate missing ground heights using nearest neighbor.
        
        Args:
            ground_grid: Ground height grid with NaN values for missing cells
            
        Returns:
            Interpolated ground height grid with no NaN values
        """
        valid_mask = ~np.isnan(ground_grid)
        if not np.any(valid_mask):
            return np.full_like(ground_grid, 0.0)
        
        indices = distance_transform_edt(~valid_mask, return_distances=False, return_indices=True)
        return ground_grid[tuple(indices)]
    
    def create_occupancy_grid_with_ground_filtering(
        self,
        points: npt.NDArray[np.float32],
        origin_x: float,
        origin_y: float,
        width_pixels: int,
        height_pixels: int,
        ground_height_grid: npt.NDArray[np.float32]
    ) -> npt.NDArray[np.int32]:
        """
        Create occupancy grid filtering points by ground-relative height.
        
        Filters points based on their height relative to the estimated ground,
        keeping only those within [min_z_offset, max_z_offset] range.
        
        Args:
            points: Point cloud array of shape (N, 3) with [x, y, z]
            origin_x: Map origin X coordinate in meters
            origin_y: Map origin Y coordinate in meters
            width_pixels: Map width in pixels
            height_pixels: Map height in pixels
            ground_height_grid: Estimated ground height grid
            
        Returns:
            Occupancy grid array of shape (height_pixels, width_pixels) with point counts per cell
        """
        ground_grid_height, ground_grid_width = ground_height_grid.shape
        
        # Use cached values if available and dimensions haven't changed
        if (self._cached_ground_grid_width == ground_grid_width and 
            self._cached_ground_grid_height == ground_grid_height and
            self._cached_ground_grid_res_x is not None and
            self._cached_ground_grid_res_y is not None):
            ground_grid_res_x = self._cached_ground_grid_res_x
            ground_grid_res_y = self._cached_ground_grid_res_y
        else:
            ground_grid_res_x = (width_pixels * self.resolution) / ground_grid_width
            ground_grid_res_y = (height_pixels * self.resolution) / ground_grid_height
            # Cache the values
            self._cached_ground_grid_res_x = ground_grid_res_x
            self._cached_ground_grid_res_y = ground_grid_res_y
            self._cached_ground_grid_width = ground_grid_width
            self._cached_ground_grid_height = ground_grid_height
        
        # Pre-allocate arrays for better performance
        # Convert world coordinates to pixel coordinates for occupancy grid
        pixel_x = ((points[:, 0] - origin_x) / self.resolution).astype(int)
        pixel_y = ((points[:, 1] - origin_y) / self.resolution).astype(int)
        # Convert world coordinates to ground grid indices (coarser resolution)
        grid_x = np.clip(((points[:, 0] - origin_x) / ground_grid_res_x).astype(int), 0, ground_grid_width - 1)
        grid_y = np.clip(((points[:, 1] - origin_y) / ground_grid_res_y).astype(int), 0, ground_grid_height - 1)
        
        # Apply masks in order of computational cost (cheapest first)
        valid_map_mask = (pixel_x >= 0) & (pixel_x < width_pixels) & (pixel_y >= 0) & (pixel_y < height_pixels)
        
        # Early exit if no points are in valid map area
        if not np.any(valid_map_mask):
            self.get_logger().warn('No points in valid map area')
            return np.zeros((height_pixels, width_pixels), dtype=np.int32)
        
        # Get ground heights for valid points only
        valid_indices = np.where(valid_map_mask)[0]
        ground_heights = ground_height_grid[grid_y[valid_indices], grid_x[valid_indices]]
        relative_heights = points[valid_indices, 2] - ground_heights
        height_mask = (relative_heights >= self.min_z_offset) & (relative_heights <= self.max_z_offset)
        
        # Final valid points
        final_valid_indices = valid_indices[height_mask]
        valid_pixel_x = pixel_x[final_valid_indices]
        valid_pixel_y = pixel_y[final_valid_indices]
        
        num_valid_points = len(valid_pixel_x)
        
        # Early exit if no valid points
        if num_valid_points < MIN_VALID_POINTS_THRESHOLD:
            self.get_logger().warn(f'Too few valid points after filtering ({num_valid_points}), returning empty grid')
            return np.zeros((height_pixels, width_pixels), dtype=np.int32)
        
        # Use vectorized operations for counting
        linear_indices = valid_pixel_y * width_pixels + valid_pixel_x
        counts = np.bincount(linear_indices, minlength=width_pixels * height_pixels)
        return counts.reshape((height_pixels, width_pixels)).astype(np.int32)
    
    def _declare_basic_parameters(self) -> None:
        """Declare basic node parameters."""
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('margin_left', 10.0)
        self.declare_parameter('margin_right', 10.0)
        self.declare_parameter('margin_top', 10.0)
        self.declare_parameter('margin_bottom', 10.0)
        self.declare_parameter('occupied_threshold', 2)
        self.declare_parameter('free_space_value', 60)
        self.declare_parameter('min_z_offset', 0.1)
        self.declare_parameter('max_z_offset', 0.5)
        self.declare_parameter('ground_grid_size', 5.0)
        self.declare_parameter('ground_estimation_height_threshold', 0.5)
        self.declare_parameter('target_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 0.5)
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_topic', '/costmap/geometric')
        self.declare_parameter('transform_timeout', 0.1)
    
    def _declare_accumulation_parameters(self) -> None:
        """Declare point cloud accumulation parameters."""
        self.declare_parameter('accumulation_mode', True)
        self.declare_parameter('accumulation_frame_interval', 2)
        self.declare_parameter('accumulation_window_size', 50)
    
    def _declare_fixed_map_parameters(self) -> None:
        """Declare fixed map boundary parameters."""
        self.declare_parameter('use_fixed_map', True)
        self.declare_parameter('expand_map', False)
        self.declare_parameter('fixed_map_min_x', -20.0)
        self.declare_parameter('fixed_map_max_x', 20.0)
        self.declare_parameter('fixed_map_min_y', -20.0)
        self.declare_parameter('fixed_map_max_y', 20.0)
    
    def _get_parameter_values(self) -> None:
        """Get and store parameter values."""
        # Basic parameters
        self.resolution = self.get_parameter('resolution').value
        self.margin_left = self.get_parameter('margin_left').value
        self.margin_right = self.get_parameter('margin_right').value
        self.margin_top = self.get_parameter('margin_top').value
        self.margin_bottom = self.get_parameter('margin_bottom').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.free_space_value = self.get_parameter('free_space_value').value
        self.min_z_offset = self.get_parameter('min_z_offset').value
        self.max_z_offset = self.get_parameter('max_z_offset').value
        self.ground_grid_size = self.get_parameter('ground_grid_size').value
        self.ground_estimation_height_threshold = self.get_parameter('ground_estimation_height_threshold').value
        self.target_frame_id = self.get_parameter('target_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.transform_timeout = self.get_parameter('transform_timeout').value
        
        # Accumulation mode parameters
        self.accumulation_mode = self.get_parameter('accumulation_mode').value
        self.accumulation_frame_interval = self.get_parameter('accumulation_frame_interval').value
        self.accumulation_window_size = self.get_parameter('accumulation_window_size').value
        
        # Fixed map boundary parameters
        self.use_fixed_map = self.get_parameter('use_fixed_map').value
        self.expand_map = self.get_parameter('expand_map').value
        self.fixed_map_min_x_param = self.get_parameter('fixed_map_min_x').value
        self.fixed_map_max_x_param = self.get_parameter('fixed_map_max_x').value
        self.fixed_map_min_y_param = self.get_parameter('fixed_map_min_y').value
        self.fixed_map_max_y_param = self.get_parameter('fixed_map_max_y').value
    
    def _validate_parameters(self) -> None:
        """Validate parameter values and ranges."""
        # Validate basic parameters
        if self.resolution <= 0:
            raise ValueError(f'Resolution must be positive, got {self.resolution}')
        
        if any(margin < 0 for margin in [self.margin_left, self.margin_right, self.margin_top, self.margin_bottom]):
            raise ValueError('All margins must be non-negative')
        
        if self.min_z_offset >= self.max_z_offset:
            raise ValueError(f'min_z_offset ({self.min_z_offset}) must be less than max_z_offset ({self.max_z_offset})')
        
        if self.ground_grid_size <= 0:
            raise ValueError(f'Ground grid size must be positive, got {self.ground_grid_size}')
        
        if self.publish_rate <= 0:
            raise ValueError(f'Publish rate must be positive, got {self.publish_rate}')
        
        # Validate accumulation parameters
        if self.accumulation_frame_interval <= 0:
            raise ValueError(f'Accumulation frame interval must be positive, got {self.accumulation_frame_interval}')
        
        if self.accumulation_window_size <= 0:
            raise ValueError(f'Accumulation window size must be positive, got {self.accumulation_window_size}')
        
        # Check if user has set custom boundaries (not all zeros)
        self.use_custom_boundaries = not (
            self.fixed_map_min_x_param == 0.0 and 
            self.fixed_map_max_x_param == 0.0 and
            self.fixed_map_min_y_param == 0.0 and 
            self.fixed_map_max_y_param == 0.0
        )
        
        # Validate fixed boundary parameters if user has set them
        if self.use_custom_boundaries:
            if self.fixed_map_max_x_param <= self.fixed_map_min_x_param:
                self.get_logger().error(
                    f'Invalid fixed map boundaries: fixed_map_max_x ({self.fixed_map_max_x_param}) '
                    f'must be greater than fixed_map_min_x ({self.fixed_map_min_x_param})')
                raise ValueError('Invalid X boundary parameters')
            if self.fixed_map_max_y_param <= self.fixed_map_min_y_param:
                self.get_logger().error(
                    f'Invalid fixed map boundaries: fixed_map_max_y ({self.fixed_map_max_y_param}) '
                    f'must be greater than fixed_map_min_y ({self.fixed_map_min_y_param})')
                raise ValueError('Invalid Y boundary parameters')
    
    def _log_initialization_info(self) -> None:
        """Log initialization information in a concise format."""
        self.get_logger().info('Geometric Traversability Costmap Node initialized')
        self.get_logger().info(f'Input: {self.input_topic} -> Output: {self.output_topic}')
        self.get_logger().info(f'Fixed boundary reference frame: {self.target_frame_id}')
        self.get_logger().info('Processing & output: input point cloud native frame')
        self.get_logger().info(f'TF timeout: {self.transform_timeout}s')
        self.get_logger().info(f'Resolution: {self.resolution} m/pixel, Height: {self.min_z_offset}-{self.max_z_offset} m')
        self.get_logger().info(f'Ground estimation height threshold: {self.ground_estimation_height_threshold} m')
        self.get_logger().info(f'Margins: L={self.margin_left}, R={self.margin_right}, T={self.margin_top}, B={self.margin_bottom}')
        self.get_logger().info(f'Occupancy values: occupied(>={self.occupied_threshold})=100, free={self.free_space_value}')
        
        if self.accumulation_mode:
            self.get_logger().info('Accumulation mode: ENABLED')
            self.get_logger().info(f'  Frame interval: {self.accumulation_frame_interval}, Window size: {self.accumulation_window_size}')
        else:
            self.get_logger().info('Accumulation mode: DISABLED (single frame mode)')
        
        if self.use_fixed_map:
            mode_str = 'with expansion' if self.expand_map else 'fixed size'
            self.get_logger().info(f'Fixed map mode: ENABLED ({mode_str})')
            if self.use_custom_boundaries:
                self.get_logger().info(f'  User-defined boundaries: '
                                      f'X=[{self.fixed_map_min_x_param:.2f}, {self.fixed_map_max_x_param:.2f}], '
                                      f'Y=[{self.fixed_map_min_y_param:.2f}, {self.fixed_map_max_y_param:.2f}]')
                map_width = self.fixed_map_max_x_param - self.fixed_map_min_x_param
                map_height = self.fixed_map_max_y_param - self.fixed_map_min_y_param
                self.get_logger().info(f'  Map size: {map_width:.2f}m × {map_height:.2f}m')
            else:
                self.get_logger().info('  Boundaries will be auto-initialized from first frame')
        else:
            self.get_logger().info('Fixed map mode: DISABLED (dynamic boundaries)')


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the geometric traversability costmap node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = GeometricTraversabilityCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()