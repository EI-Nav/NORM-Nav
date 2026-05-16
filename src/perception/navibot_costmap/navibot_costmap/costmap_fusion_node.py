#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Costmap Fusion Node.

Fuses three costmap layers (geometric, semantic, directional) into a unified
navigation costmap. Supports multiple fusion modes and matrix-based operations.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import List, Optional, Dict, Any
from collections import OrderedDict

import time

import numpy as np
import numpy.typing as npt
import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class CostmapFusionNode(Node):
    """
    Fuse three costmap layers into a unified navigation costmap.
    
    This node subscribes to three costmap layers (geometric, semantic, directional)
    and fuses them using configurable matrix operations. Supports multiple fusion
    modes including fixed rate, triggered, and hybrid modes.
    """
    
    # Constants for parameter validation
    VALID_FUSION_MODES = ['fixed_rate', 'triggered', 'hybrid']
    VALID_REFERENCE_LAYERS = ['geometric', 'semantic', 'directional']
    
    # Constants for costmap values
    UNKNOWN_VALUE = -1
    FREE_SPACE = 0
    OCCUPIED = 100
    SEMANTIC_PENALTY_MAX = 100
    
    # Type annotations for instance variables
    latest_geometric_: Optional[OccupancyGrid]
    latest_semantic_: Optional[OccupancyGrid]
    latest_directional_: Optional[OccupancyGrid]
    processing_: bool
    
    # Timing for timeout detection
    last_geometric_time_: Optional[float]
    last_semantic_time_: Optional[float]
    last_directional_time_: Optional[float]
    
    def __init__(self) -> None:
        """Initialize the node with parameters and publishers/subscribers."""
        super().__init__('costmap_fusion_node')
        
        self._declare_parameters()
        self._load_parameters()
        self._initialize_components()
        
        self.get_logger().info('Costmap Fusion Node initialized')
    
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        # Input/Output configuration
        self.declare_parameter('geometric_topic', '/costmap/geometric')
        self.declare_parameter('semantic_topic', '/costmap/semantic')
        self.declare_parameter('directional_topic', '/costmap/directional')
        self.declare_parameter('output_topic', '/map')
        
        # Fusion mode configuration
        self.declare_parameter('fusion_mode', 'hybrid')  # fixed_rate / triggered / hybrid
        self.declare_parameter('fixed_rate', 5.0)  # Hz for fixed rate mode
        self.declare_parameter('trigger_on_geometric', True)
        self.declare_parameter('trigger_on_semantic', True)
        self.declare_parameter('trigger_on_directional', True)
        
        # Fusion algorithm configuration
        self.declare_parameter('fusion_algorithm', 'spatial_behavioral')  # spatial_behavioral (others removed)
        
        # Performance configuration
        self.declare_parameter('enable_debug_logging', False)
        self.declare_parameter('timeout_threshold', 5.0)  # seconds
        
        # Spatial behavioral fusion parameters
        self.declare_parameter('semantic_penalty_low', 20)
        self.declare_parameter('semantic_penalty_high', 100)
        
        # Map alignment configuration
        self.declare_parameter('reference_layer', 'geometric')  # geometric/semantic/directional
        self.declare_parameter('warn_on_misalignment', True)
        
        # Performance optimization configuration
        self.declare_parameter('enable_resampling_cache', True)
        self.declare_parameter('max_cache_entries', 10)
        self.declare_parameter('enable_detailed_timing', False)
        
        # Timing statistics configuration
        self.declare_parameter('enable_timing_stats', True)
    
    def _validate_parameters(self) -> None:
        """Validate all ROS parameters and raise exceptions for invalid values."""
        # Validate fusion mode
        if self.fusion_mode_ not in self.VALID_FUSION_MODES:
            raise ValueError(f"Invalid fusion_mode '{self.fusion_mode_}'. "
                           f"Must be one of: {self.VALID_FUSION_MODES}")
        
        # Validate reference layer
        if self.reference_layer_ not in self.VALID_REFERENCE_LAYERS:
            raise ValueError(f"Invalid reference_layer '{self.reference_layer_}'. "
                           f"Must be one of: {self.VALID_REFERENCE_LAYERS}")
        
        
        # Validate numeric parameters
        if self.fixed_rate_ <= 0:
            raise ValueError(f"fixed_rate must be positive, got: {self.fixed_rate_}")
        
        if self.timeout_threshold_ <= 0:
            raise ValueError(f"timeout_threshold must be positive, got: {self.timeout_threshold_}")
        
        if self.max_cache_entries_ <= 0:
            raise ValueError(f"max_cache_entries must be positive, got: {self.max_cache_entries_}")
        
        # Validate semantic penalty values
        if not (0 <= self.semantic_penalty_low_ <= self.SEMANTIC_PENALTY_MAX):
            raise ValueError(f"semantic_penalty_low must be in range [0, {self.SEMANTIC_PENALTY_MAX}], "
                           f"got: {self.semantic_penalty_low_}")
        
        if not (0 <= self.semantic_penalty_high_ <= self.SEMANTIC_PENALTY_MAX):
            raise ValueError(f"semantic_penalty_high must be in range [0, {self.SEMANTIC_PENALTY_MAX}], "
                           f"got: {self.semantic_penalty_high_}")
        
        if self.semantic_penalty_low_ >= self.semantic_penalty_high_:
            raise ValueError(f"semantic_penalty_low ({self.semantic_penalty_low_}) must be less than "
                           f"semantic_penalty_high ({self.semantic_penalty_high_})")
        
        self.get_logger().info("All parameters validated successfully")
    
    def _load_parameters(self) -> None:
        """Load and validate all ROS parameters."""
        # Input/Output configuration
        self.geometric_topic_ = self.get_parameter('geometric_topic').value
        self.semantic_topic_ = self.get_parameter('semantic_topic').value
        self.directional_topic_ = self.get_parameter('directional_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        
        # Fusion mode configuration
        self.fusion_mode_ = self.get_parameter('fusion_mode').value
        self.fixed_rate_ = self.get_parameter('fixed_rate').value
        self.trigger_on_geometric_ = self.get_parameter('trigger_on_geometric').value
        self.trigger_on_semantic_ = self.get_parameter('trigger_on_semantic').value
        self.trigger_on_directional_ = self.get_parameter('trigger_on_directional').value
        
        # Fusion algorithm configuration
        self.fusion_algorithm_ = self.get_parameter('fusion_algorithm').value
        
        # Performance configuration
        self.enable_debug_logging_ = self.get_parameter('enable_debug_logging').value
        self.timeout_threshold_ = self.get_parameter('timeout_threshold').value
        
        # Spatial behavioral fusion parameters
        self.semantic_penalty_low_ = self.get_parameter('semantic_penalty_low').value
        self.semantic_penalty_high_ = self.get_parameter('semantic_penalty_high').value
        
        # Map alignment configuration
        self.reference_layer_ = self.get_parameter('reference_layer').value
        self.warn_on_misalignment_ = self.get_parameter('warn_on_misalignment').value
        
        # Performance optimization configuration
        self.enable_resampling_cache_ = self.get_parameter('enable_resampling_cache').value
        self.max_cache_entries_ = self.get_parameter('max_cache_entries').value
        self.enable_detailed_timing_ = self.get_parameter('enable_detailed_timing').value
        
        # Timing statistics configuration
        self.enable_timing_stats_ = self.get_parameter('enable_timing_stats').value
        
        # Validate all parameters
        self._validate_parameters()
        
        # Set logger level
        if self.enable_debug_logging_:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Log key configuration parameters
        self.get_logger().info(f"Fusion mode: {self.fusion_mode_}")
        self.get_logger().info(f"Geometric topic: {self.geometric_topic_}")
        self.get_logger().info(f"Semantic topic: {self.semantic_topic_}")
        self.get_logger().info(f"Directional topic: {self.directional_topic_}")
        self.get_logger().info(f"Output topic: {self.output_topic_}")
        self.get_logger().info(f"Fusion algorithm: {self.fusion_algorithm_}")
    
    def _initialize_components(self) -> None:
        """Initialize ROS components (publishers, subscribers)."""
        # QoS profile for latched map topics
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Initialize subscribers for three costmap layers
        self.geometric_sub_ = self.create_subscription(
            OccupancyGrid, self.geometric_topic_, self.geometric_callback, 1)
        self.semantic_sub_ = self.create_subscription(
            OccupancyGrid, self.semantic_topic_, self.semantic_callback, 1)
        self.directional_sub_ = self.create_subscription(
            OccupancyGrid, self.directional_topic_, self.directional_callback, 1)
        
        # Initialize publisher for fused costmap
        self.fused_costmap_pub_ = self.create_publisher(
            OccupancyGrid, self.output_topic_, map_qos)
        
        # Internal state
        self.latest_geometric_: Optional[OccupancyGrid] = None
        self.latest_semantic_: Optional[OccupancyGrid] = None
        self.latest_directional_: Optional[OccupancyGrid] = None
        self.processing_: bool = False
        
        # Timing for timeout detection
        self.last_geometric_time_: Optional[float] = None
        self.last_semantic_time_: Optional[float] = None
        self.last_directional_time_: Optional[float] = None
        
        # Timing statistics
        self.fusion_count_: int = 0
        
        # Resampling cache for performance optimization (LRU cache)
        self.resampling_cache_: OrderedDict = OrderedDict()
        self.cache_hits_: int = 0
        self.cache_misses_: int = 0
        
        # Create timer for fixed rate mode
        if self.fusion_mode_ in ['fixed_rate', 'hybrid']:
            self.timer_ = self.create_timer(1.0 / self.fixed_rate_, self.timer_callback)
        
        self.get_logger().info("Subscribers and publishers initialized")
    
    def geometric_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback for receiving geometric costmap.
        
        Args:
            msg: Geometric costmap message
        """
        self.latest_geometric_ = msg
        self.last_geometric_time_ = time.time()
        if self.enable_debug_logging_:
            self.get_logger().debug("Received geometric costmap")
        
        # Trigger fusion if in triggered or hybrid mode
        if self.fusion_mode_ in ['triggered', 'hybrid'] and self.trigger_on_geometric_:
            self._trigger_fusion()
    
    def semantic_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback for receiving semantic costmap.
        
        Args:
            msg: Semantic costmap message
        """
        self.latest_semantic_ = msg
        self.last_semantic_time_ = time.time()
        if self.enable_debug_logging_:
            self.get_logger().debug("Received semantic costmap")
        
        # Trigger fusion if in triggered or hybrid mode
        if self.fusion_mode_ in ['triggered', 'hybrid'] and self.trigger_on_semantic_:
            self._trigger_fusion()
    
    def directional_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback for receiving directional costmap.
        
        Args:
            msg: Directional costmap message
        """
        self.latest_directional_ = msg
        self.last_directional_time_ = time.time()
        if self.enable_debug_logging_:
            self.get_logger().debug("Received directional costmap")
        
        # Trigger fusion if in triggered or hybrid mode
        if self.fusion_mode_ in ['triggered', 'hybrid'] and self.trigger_on_directional_:
            self._trigger_fusion()
    
    def timer_callback(self) -> None:
        """Timer callback for fixed rate fusion."""
        if self.fusion_mode_ in ['fixed_rate', 'hybrid']:
            self._trigger_fusion()
    
    def _trigger_fusion(self) -> None:
        """Trigger costmap fusion or publish reference layer based on availability."""
        if self.processing_:
            self.get_logger().warn('Still processing, skipping fusion trigger')
            return
        
        # First check if reference layer is available
        if not self._check_reference_layer_availability():
            self.get_logger().debug('Reference layer not available, skipping publication')
            return
        
        self.processing_ = True
        
        try:
            # Check if all three layers are available for fusion
            if self._check_costmap_availability():
                # All layers available - execute fusion
                self._execute_fusion_with_timing()
            else:
                # Some layers missing or timed out - publish reference layer
                self._publish_reference_layer()
        except (ValueError, RuntimeError, TypeError, AttributeError, IndexError) as e:
            self._handle_fusion_error(e)
        except Exception as e:  # pylint: disable=broad-except
            # Catch all other exceptions to ensure processing flag is reset
            self.get_logger().error(f'Unexpected error in fusion trigger: {e}')
        finally:
            self.processing_ = False
    
    def _check_costmap_availability(self) -> bool:
        """Check if all required costmap layers are available and not timed out."""
        current_time = time.time()
        
        # Check geometric costmap
        if self.latest_geometric_ is None:
            self.get_logger().debug('Geometric costmap not available')
            return False
        if self._is_costmap_timed_out(self.last_geometric_time_, current_time, 'geometric'):
            return False
        
        # Check semantic costmap
        if self.latest_semantic_ is None:
            self.get_logger().debug('Semantic costmap not available')
            return False
        if self._is_costmap_timed_out(self.last_semantic_time_, current_time, 'semantic'):
            return False
        
        # Check directional costmap
        if self.latest_directional_ is None:
            self.get_logger().debug('Directional costmap not available')
            return False
        if self._is_costmap_timed_out(self.last_directional_time_, current_time, 'directional'):
            return False
        
        return True
    
    def _is_costmap_timed_out(self, last_time: Optional[float], current_time: float, layer_name: str) -> bool:
        """Check if a costmap layer has timed out."""
        if last_time is None:
            return True
        
        time_diff = current_time - last_time
        if time_diff > self.timeout_threshold_:
            self.get_logger().warn(f'{layer_name} costmap timed out ({time_diff:.2f}s > {self.timeout_threshold_}s)')
            return True
        
        return False
    
    def _check_reference_layer_availability(self) -> bool:
        """Check if the reference layer is available and not timed out."""
        current_time = time.time()
        
        if self.reference_layer_ == 'geometric':
            if self.latest_geometric_ is None:
                self.get_logger().debug('Reference layer (geometric) not available')
                return False
            if self._is_costmap_timed_out(self.last_geometric_time_, current_time, 'geometric'):
                return False
        elif self.reference_layer_ == 'semantic':
            if self.latest_semantic_ is None:
                self.get_logger().debug('Reference layer (semantic) not available')
                return False
            if self._is_costmap_timed_out(self.last_semantic_time_, current_time, 'semantic'):
                return False
        else:  # directional
            if self.latest_directional_ is None:
                self.get_logger().debug('Reference layer (directional) not available')
                return False
            if self._is_costmap_timed_out(self.last_directional_time_, current_time, 'directional'):
                return False
        
        return True
    
    def _publish_reference_layer(self) -> None:
        """Publish the reference layer directly without fusion."""
        # Get reference layer data
        if self.reference_layer_ == 'geometric':
            reference_costmap = self.latest_geometric_
            missing_layers = []
            if self.latest_semantic_ is None:
                missing_layers.append('semantic')
            if self.latest_directional_ is None:
                missing_layers.append('directional')
        elif self.reference_layer_ == 'semantic':
            reference_costmap = self.latest_semantic_
            missing_layers = []
            if self.latest_geometric_ is None:
                missing_layers.append('geometric')
            if self.latest_directional_ is None:
                missing_layers.append('directional')
        else:  # directional
            reference_costmap = self.latest_directional_
            missing_layers = []
            if self.latest_geometric_ is None:
                missing_layers.append('geometric')
            if self.latest_semantic_ is None:
                missing_layers.append('semantic')
        
        if reference_costmap is None:
            self.get_logger().error(f"Reference layer ({self.reference_layer_}) is None, cannot publish")
            return
        
        # Create a copy of the reference costmap with updated timestamp
        published_costmap = OccupancyGrid()
        published_costmap.header.stamp = self.get_clock().now().to_msg()
        published_costmap.header.frame_id = reference_costmap.header.frame_id
        published_costmap.info = reference_costmap.info
        published_costmap.data = reference_costmap.data
        
        # Publish the reference layer
        self.fused_costmap_pub_.publish(published_costmap)
        
        # Log the reason for publishing reference layer
        if missing_layers:
            self.get_logger().info(f"Published reference layer ({self.reference_layer_}) due to missing layers: {', '.join(missing_layers)}")
        else:
            self.get_logger().info(f"Published reference layer ({self.reference_layer_}) due to layer timeout")
        
        self.get_logger().debug('Published reference layer')
    
    def _execute_fusion_with_timing(self) -> None:
        """Execute costmap fusion with detailed timing and statistics."""
        # Start timing if enabled
        fusion_start_time = time.time() if self.enable_timing_stats_ else None
        alignment_start_time = None
        
        # Fuse costmaps with detailed timing
        if self.enable_detailed_timing_:
            alignment_start_time = time.time()
        
        fused_costmap = self._fuse_costmaps(
            self.latest_geometric_,
            self.latest_semantic_,
            self.latest_directional_
        )
        
        # Publish fused costmap
        self.fused_costmap_pub_.publish(fused_costmap)
        self.get_logger().debug('Published fused costmap')
        
        # Record detailed timing statistics
        if self.enable_timing_stats_ and fusion_start_time is not None:
            self._record_timing_statistics(fusion_start_time, alignment_start_time)
    
    def _record_timing_statistics(self, fusion_start_time: float, alignment_start_time: Optional[float]) -> None:
        """Record and log timing statistics for fusion operations."""
        fusion_time = time.time() - fusion_start_time
        self.fusion_count_ += 1
        
        if self.enable_detailed_timing_ and alignment_start_time is not None:
            alignment_time = time.time() - alignment_start_time
            self.get_logger().info(f'Fusion #{self.fusion_count_} completed in {fusion_time*1000:.2f} ms (alignment: {alignment_time*1000:.2f} ms)')
        else:
            self.get_logger().info(f'Fusion #{self.fusion_count_} completed in {fusion_time*1000:.2f} ms')
        
        # Log cache statistics periodically
        if self.fusion_count_ % 10 == 0 and self.enable_resampling_cache_:
            self._log_cache_statistics()
    
    def _log_cache_statistics(self) -> None:
        """Log cache hit/miss statistics."""
        total_requests = self.cache_hits_ + self.cache_misses_
        hit_rate = (self.cache_hits_ / total_requests * 100) if total_requests > 0 else 0
        self.get_logger().info(f'Cache statistics: {self.cache_hits_}/{total_requests} hits ({hit_rate:.1f}% hit rate)')
    
    def _handle_fusion_error(self, error: Exception) -> None:
        """Handle fusion errors with appropriate logging."""
        if isinstance(error, ValueError):
            self.get_logger().error(f'Value error fusing costmaps: {error}')
        elif isinstance(error, RuntimeError):
            self.get_logger().error(f'Runtime error fusing costmaps: {error}')
        elif isinstance(error, (TypeError, AttributeError, IndexError)):
            self.get_logger().error(f'Data processing error fusing costmaps: {error}')
        else:
            # Catch all other exceptions to ensure processing flag is reset
            self.get_logger().error(f'Unexpected error fusing costmaps: {error}')
    
    def _fuse_costmaps(
        self, 
        geometric: OccupancyGrid, 
        semantic: OccupancyGrid, 
        directional: OccupancyGrid
    ) -> OccupancyGrid:
        """
        Fuse three costmap layers using matrix operations.
        
        Args:
            geometric: Geometric costmap layer
            semantic: Semantic costmap layer
            directional: Directional costmap layer
            
        Returns:
            Fused costmap as OccupancyGrid message
        """
        # 1. Check and align costmaps
        aligned_geo, aligned_sem, aligned_dir, reference_info = self._align_costmaps(
            geometric, semantic, directional
        )
        
        # 2. Execute fusion on aligned data
        fused_data = self._fuse_costmap_matrices(
            aligned_geo, aligned_sem, aligned_dir
        )
        
        # 3. Create fused costmap message using reference metadata
        fused_costmap = OccupancyGrid()
        fused_costmap.header.stamp = self.get_clock().now().to_msg()
        fused_costmap.header.frame_id = geometric.header.frame_id
        fused_costmap.info = reference_info  # Use reference layer metadata
        fused_costmap.data = fused_data.flatten().tolist()
        
        return fused_costmap
    
    def _fuse_costmap_matrices(
        self, 
        geometric: npt.NDArray[np.int8], 
        semantic: npt.NDArray[np.int8], 
        directional: npt.NDArray[np.int8]
    ) -> npt.NDArray[np.int8]:
        """
        Fuse costmap matrices using spatial behavioral fusion algorithm.
        
        Args:
            geometric: Geometric costmap matrix
            semantic: Semantic costmap matrix
            directional: Directional costmap matrix
            
        Returns:
            Fused costmap matrix
        """
        if self.fusion_algorithm_ != 'spatial_behavioral':
            self.get_logger().warn(f"Only spatial_behavioral fusion is supported, got: {self.fusion_algorithm_}. Using spatial_behavioral fusion.")
        
        # Implement spatial behavioral fusion based on formulas (16) and (17)
        return self._spatial_behavioral_fusion(geometric, semantic, directional)
    
    def _spatial_behavioral_fusion(
        self, 
        geometric: npt.NDArray[np.int8], 
        semantic: npt.NDArray[np.int8], 
        directional: npt.NDArray[np.int8]
    ) -> npt.NDArray[np.int8]:
        """
        Optimized spatial behavioral fusion with reduced temporary array allocation.
        
        This method implements the spatial behavioral fusion algorithm based on
        mathematical formulas (16) and (17) from the research paper. The algorithm
        combines geometric, semantic, and directional costmap information to create
        a unified navigation costmap.
        
        Mathematical Formulas:
        Formula (16): I(u,v) = 1/2 * C_geo ⊙ (1 - C_sem) ⊙ (2 - C_sem)
                      + semantic_penalty_low * C_sem ⊙ (2 - C_sem)
                      + 1/2 * semantic_penalty_high * C_sem ⊙ (C_sem - 1)
        
        Formula (17): C_spat(u,v) = 1(I == semantic_penalty_high) ⊙ semantic_penalty_high
                      + 1(I ≠ semantic_penalty_high) ⊙ 1(C_dir > 0) ⊙ C_dir
                      + 1(I ≠ semantic_penalty_high) ⊙ 1(C_dir == 0) ⊙ I
        
        Args:
            geometric: Geometric costmap matrix (C_geo) with values 0-100
            semantic: Semantic costmap matrix (C_sem) with values 0, 1, 2
            directional: Directional costmap matrix (C_dir) with values 0-100
            
        Returns:
            Fused costmap matrix (C_spat) with values 0-100
            
        Raises:
            ValueError: If costmap matrices have different shapes
            
        Note:
            This method is optimized for memory efficiency by:
            - Only processing intersection regions where all layers have valid data
            - Pre-allocating arrays to reduce temporary allocations
            - Using vectorized operations for maximum performance
        """
        # Ensure all matrices have the same shape
        if geometric.shape != semantic.shape or geometric.shape != directional.shape:
            self.get_logger().error("Costmap matrices have different shapes")
            return geometric
        
        # Create intersection mask (where all three layers have valid data)
        intersection_mask = (
            (geometric != self.UNKNOWN_VALUE) & 
            (semantic != self.UNKNOWN_VALUE) & 
            (directional != self.UNKNOWN_VALUE)
        )
        
        # Initialize result with geometric layer as base
        result = geometric.copy()
        
        # Apply fusion only in intersection regions
        if np.any(intersection_mask):
            # Pre-allocate arrays to reduce memory allocation
            intersection_indices = np.where(intersection_mask)
            num_intersection_pixels = len(intersection_indices[0])
            
            # Convert to float only for intersection regions to save memory
            C_geo = geometric[intersection_mask].astype(np.float64)
            C_sem = semantic[intersection_mask].astype(np.float64)
            C_dir = directional[intersection_mask].astype(np.float64)
            
            # Pre-allocate result array for intersection region
            C_spat = np.zeros(num_intersection_pixels, dtype=np.float64)
            
            # Formula (16): I(u,v) calculation - optimized to reduce temporary arrays
            # Term 1: 1/2 * C_geo ⊙ (1 - C_sem) ⊙ (2 - C_sem)
            term1 = 0.5 * C_geo * (1.0 - C_sem) * (2.0 - C_sem)
            
            # Term 2: semantic_penalty_low * C_sem ⊙ (2 - C_sem)
            term2 = self.semantic_penalty_low_ * C_sem * (2.0 - C_sem)
            
            # Term 3: 1/2 * semantic_penalty_high * C_sem ⊙ (C_sem - 1)
            term3 = 0.5 * self.semantic_penalty_high_ * C_sem * (C_sem - 1.0)
            
            # Combine terms to get I
            I = term1 + term2 + term3
            
            # Formula (17): C_spat(u,v) calculation using np.where for efficiency
            # Use np.where to avoid creating multiple indicator function arrays
            C_spat = np.where(
                I == self.semantic_penalty_high_,
                self.semantic_penalty_high_,
                np.where(C_dir > 0, C_dir, I)
            )
            
            # Apply fusion only in intersection regions
            result[intersection_mask] = C_spat.astype(np.int8)
        
        # Clamp values to valid range
        result = np.clip(result, 0, 100).astype(np.int8)
        
        if self.enable_debug_logging_:
            intersection_count = np.sum(intersection_mask)
            self.get_logger().debug(f"Spatial behavioral fusion applied to {intersection_count} intersection pixels")
        
        return result
    
    def _world_to_grid(self, x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            x: World X coordinate
            y: World Y coordinate
            origin_x: Map origin X coordinate
            origin_y: Map origin Y coordinate
            resolution: Map resolution in meters per pixel
            
        Returns:
            Grid coordinates (x, y)
        """
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        return grid_x, grid_y
    
    def _grid_to_world(self, grid_x: int, grid_y: int, origin_x: float, origin_y: float, resolution: float) -> tuple[float, float]:
        """
        Convert grid coordinates to world coordinates.
        
        Args:
            grid_x: Grid X coordinate
            grid_y: Grid Y coordinate
            origin_x: Map origin X coordinate
            origin_y: Map origin Y coordinate
            resolution: Map resolution in meters per pixel
            
        Returns:
            World coordinates (x, y)
        """
        x = origin_x + (grid_x + 0.5) * resolution
        y = origin_y + (grid_y + 0.5) * resolution
        return x, y
    
    def _resample_costmap(self, source_costmap: OccupancyGrid, target_info: MapMetaData) -> npt.NDArray[np.int8]:
        """
        Resample source costmap to target coordinate system using vectorized operations with caching.
        
        Args:
            source_costmap: Source costmap to resample
            target_info: Target map metadata (resolution, origin, size)
            
        Returns:
            Resampled costmap data as numpy array
        """
        # Extract source map data using frombuffer for better performance
        src_data = np.frombuffer(source_costmap.data, dtype=np.int8).reshape(
            (source_costmap.info.height, source_costmap.info.width))
        src_info = source_costmap.info
        
        if self.enable_debug_logging_:
            self.get_logger().debug(
                f"Resampling from {src_info.width}x{src_info.height} "
                f"to {target_info.width}x{target_info.height}")
        
        # Check cache if enabled
        if self.enable_resampling_cache_:
            cache_key = self._generate_cache_key(src_info, target_info)
            if cache_key in self.resampling_cache_:
                self.cache_hits_ += 1
                if self.enable_debug_logging_:
                    self.get_logger().debug(f"Cache hit for resampling key: {cache_key}")
                # Move to end (most recently used) and use cached coordinate mapping
                cached_mapping = self.resampling_cache_.pop(cache_key)
                self.resampling_cache_[cache_key] = cached_mapping  # Move to end
                return self._apply_cached_resampling(src_data, cached_mapping, target_info)
            else:
                self.cache_misses_ += 1
                if self.enable_debug_logging_:
                    self.get_logger().debug(f"Cache miss for resampling key: {cache_key}")
        
        # Vectorized resampling using meshgrid
        result = self._resample_costmap_vectorized(src_data, src_info, target_info)
        
        # Cache the coordinate mapping if enabled
        if self.enable_resampling_cache_:
            self._cache_resampling_mapping(cache_key, src_info, target_info)
        
        return result
    
    def _resample_costmap_vectorized(self, src_data: npt.NDArray[np.int8], src_info: MapMetaData, target_info: MapMetaData) -> npt.NDArray[np.int8]:
        """
        Vectorized resampling implementation for maximum performance.
        
        Args:
            src_data: Source costmap data array
            src_info: Source map metadata
            target_info: Target map metadata
            
        Returns:
            Resampled costmap data as numpy array
        """
        # Generate target grid coordinates using meshgrid
        target_x, target_y = np.meshgrid(
            np.arange(target_info.width, dtype=np.float64),
            np.arange(target_info.height, dtype=np.float64)
        )
        
        # Vectorized coordinate conversion from target grid to world coordinates
        world_x = target_info.origin.position.x + (target_x + 0.5) * target_info.resolution
        world_y = target_info.origin.position.y + (target_y + 0.5) * target_info.resolution
        
        # Vectorized coordinate conversion from world to source grid coordinates
        src_x = ((world_x - src_info.origin.position.x) / src_info.resolution).astype(np.int32)
        src_y = ((world_y - src_info.origin.position.y) / src_info.resolution).astype(np.int32)
        
        # Create validity mask for bounds checking
        valid_mask = (
            (src_x >= 0) & (src_x < src_info.width) & 
            (src_y >= 0) & (src_y < src_info.height)
        )
        
        # Initialize target data with unknown values
        target_data = np.full((target_info.height, target_info.width), self.UNKNOWN_VALUE, dtype=np.int8)
        
        # Use advanced indexing to copy valid values
        target_data[valid_mask] = src_data[src_y[valid_mask], src_x[valid_mask]]
        
        return target_data
    
    def _generate_cache_key(self, src_info: MapMetaData, target_info: MapMetaData) -> str:
        """Generate cache key for resampling parameters."""
        return f"{src_info.width}x{src_info.height}_{src_info.resolution}_{src_info.origin.position.x}_{src_info.origin.position.y}_to_{target_info.width}x{target_info.height}_{target_info.resolution}_{target_info.origin.position.x}_{target_info.origin.position.y}"
    
    def _cache_resampling_mapping(self, cache_key: str, src_info: MapMetaData, target_info: MapMetaData) -> None:
        """Cache coordinate mapping for resampling using LRU strategy."""
        # Limit cache size using LRU eviction
        if len(self.resampling_cache_) >= self.max_cache_entries_:
            # Remove least recently used entry (LRU)
            self.resampling_cache_.popitem(last=False)
        
        # Pre-compute coordinate mapping
        target_x, target_y = np.meshgrid(
            np.arange(target_info.width, dtype=np.float64),
            np.arange(target_info.height, dtype=np.float64)
        )
        
        world_x = target_info.origin.position.x + (target_x + 0.5) * target_info.resolution
        world_y = target_info.origin.position.y + (target_y + 0.5) * target_info.resolution
        
        src_x = ((world_x - src_info.origin.position.x) / src_info.resolution).astype(np.int32)
        src_y = ((world_y - src_info.origin.position.y) / src_info.resolution).astype(np.int32)
        
        valid_mask = (
            (src_x >= 0) & (src_x < src_info.width) & 
            (src_y >= 0) & (src_y < src_info.height)
        )
        
        # Cache the mapping
        self.resampling_cache_[cache_key] = {
            'src_x': src_x,
            'src_y': src_y,
            'valid_mask': valid_mask
        }
    
    def _apply_cached_resampling(self, src_data: npt.NDArray[np.int8], cached_mapping: Dict[str, Any], target_info: MapMetaData) -> npt.NDArray[np.int8]:
        """Apply cached coordinate mapping for resampling."""
        src_x = cached_mapping['src_x']
        src_y = cached_mapping['src_y']
        valid_mask = cached_mapping['valid_mask']
        
        target_data = np.full((target_info.height, target_info.width), self.UNKNOWN_VALUE, dtype=np.int8)
        target_data[valid_mask] = src_data[src_y[valid_mask], src_x[valid_mask]]
        
        return target_data
    
    def _align_costmaps(
        self, 
        geometric: OccupancyGrid, 
        semantic: OccupancyGrid, 
        directional: OccupancyGrid
    ) -> tuple[npt.NDArray[np.int8], npt.NDArray[np.int8], npt.NDArray[np.int8], MapMetaData]:
        """
        Align three costmaps to a unified coordinate system.
        
        Args:
            geometric: Geometric costmap layer
            semantic: Semantic costmap layer
            directional: Directional costmap layer
            
        Returns:
            Tuple of (aligned_geometric, aligned_semantic, aligned_directional, reference_info)
        """
        # Select reference map
        if self.reference_layer_ == 'geometric':
            reference = geometric
        elif self.reference_layer_ == 'semantic':
            reference = semantic
        else:
            reference = directional
        
        # Check if alignment is needed
        needs_alignment = (
            abs(geometric.info.resolution - semantic.info.resolution) > 1e-6 or
            abs(geometric.info.resolution - directional.info.resolution) > 1e-6 or
            abs(semantic.info.resolution - directional.info.resolution) > 1e-6 or
            abs(geometric.info.origin.position.x - semantic.info.origin.position.x) > 1e-6 or
            abs(geometric.info.origin.position.y - semantic.info.origin.position.y) > 1e-6 or
            abs(geometric.info.origin.position.x - directional.info.origin.position.x) > 1e-6 or
            abs(geometric.info.origin.position.y - directional.info.origin.position.y) > 1e-6 or
            geometric.info.width != semantic.info.width or
            geometric.info.height != semantic.info.height or
            geometric.info.width != directional.info.width or
            geometric.info.height != directional.info.height or
            semantic.info.width != directional.info.width or
            semantic.info.height != directional.info.height
        )
        
        if needs_alignment:
            if self.warn_on_misalignment_:
                self.get_logger().warn(
                    f"Costmaps need alignment. Using {self.reference_layer_} layer as reference.")
            
            # Resample all maps to reference coordinate system
            aligned_geometric = self._resample_costmap(geometric, reference.info)
            aligned_semantic = self._resample_costmap(semantic, reference.info)
            aligned_directional = self._resample_costmap(directional, reference.info)
            
            if self.enable_debug_logging_:
                self.get_logger().debug(
                    f"Aligned costmaps to reference: {reference.info.width}x{reference.info.height} "
                    f"at {reference.info.resolution}m/pixel")
        else:
            # No alignment needed, extract data directly using frombuffer for better performance
            aligned_geometric = np.frombuffer(geometric.data, dtype=np.int8).reshape(
                (geometric.info.height, geometric.info.width))
            aligned_semantic = np.frombuffer(semantic.data, dtype=np.int8).reshape(
                (semantic.info.height, semantic.info.width))
            aligned_directional = np.frombuffer(directional.data, dtype=np.int8).reshape(
                (directional.info.height, directional.info.width))
            
            if self.enable_debug_logging_:
                self.get_logger().debug("Costmaps already aligned, no resampling needed")
        
        return aligned_geometric, aligned_semantic, aligned_directional, reference.info


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the costmap fusion node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = CostmapFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
