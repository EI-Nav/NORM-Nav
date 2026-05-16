#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Object Modeling Node.

Synchronizes point cloud and GroundedSam messages to segment point clouds by object.
Uses camera-lidar calibration to project image detection masks onto 3D point clouds.
Outputs XYZRGB point cloud with different colors for different detected objects.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

# Standard library imports
from typing import Dict, Optional, Tuple

# Third-party imports
import numpy as np
import numpy.typing as npt
import rclpy

# ROS2 imports
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

# Local imports
from navibot_interfaces.msg import GroundedSam, OBBInfo2D, OBBInfo2DArray
from visualization_msgs.msg import MarkerArray

# Core modules
from .core import ObjectModelingTracker, fit_obb_2d
from .core.data_structures import ModelingParams
from .pipeline import Pipeline
from .ros_io import PublisherCache
# NOTE: All per-frame processing utilities are now used inside Pipeline


class ObjectModelingNode(Node):
    """Object modeling node for 3D object detection and tracking."""

    def __init__(self) -> None:
        """Initialize the object modeling node."""
        super().__init__("object_modeling")

        self._declare_parameters()
        self._load_parameters()
        self._initialize_components()

        self.get_logger().info("Object Modeling Node initialized successfully")

    def _declare_parameters(self) -> None:
        """Declare ROS parameters."""
        # Camera parameters
        self.declare_parameter("camera_matrix", [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0])
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # Topic parameters
        self.declare_parameter("pointcloud_topic", "/cloud_registered")
        self.declare_parameter("grounded_sam_topic", "/grounded_sam2_tracker/grounded_sam")
        self.declare_parameter("output_topic", "~/segmented_pointcloud")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop", 0.1)

        # Frame parameters
        self.declare_parameter("pointcloud_frame", "map")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("tf_timeout", 0.1)

        # Segmentation parameters
        self.declare_parameter("min_depth", 0.1)
        self.declare_parameter("max_depth", 50.0)
        self.declare_parameter("min_object_points", 10)
        self.declare_parameter("color_saturation", 0.85)
        self.declare_parameter("color_value", 0.95)
        self.declare_parameter("enable_debug_logging", False)
        self.declare_parameter("enable_cluster_filtering", True)
        self.declare_parameter("cluster_eps", 0.1)
        self.declare_parameter("cluster_min_samples", 8)
        self.declare_parameter("radius_outlier_removal_radius", 0.5)
        self.declare_parameter("radius_outlier_removal_min_neighbors", 3)

        # Modeling parameters
        self.declare_parameter("min_obb_points", 5)
        self.declare_parameter("obb_marker_scale", 1.0)
        self.declare_parameter("publish_obb_markers", True)
        self.declare_parameter("obb_marker_topic", "~/object_obb_markers")
        self.declare_parameter("obb_marker_lifetime", 0.1)
        self.declare_parameter("enable_cross_frame_association", True)
        self.declare_parameter("max_accumulation_frames", 10)
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("max_object_age", 30)
        self.declare_parameter("enable_object_fusion", True)
        self.declare_parameter("fusion_iou_threshold", 0.5)
        self.declare_parameter("accumulation_iou_threshold", 0.8)
        self.declare_parameter("warmup_frames_threshold", 5)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("publish_obb_info", True)
        self.declare_parameter("obb_info_topic", "~/object_obb_info")
        self.declare_parameter("obb_height_min", -0.5)
        self.declare_parameter("obb_height_max", 2.0)
        self.declare_parameter("obb_marker_height", 1.0)

    def _load_parameters(self) -> None:
        """Load ROS parameters."""
        # Camera parameters
        camera_matrix_flat = self.get_parameter("camera_matrix").value
        self.camera_matrix: npt.NDArray[np.float64] = np.array(camera_matrix_flat).reshape(3, 3)

        dist_coeffs_list = self.get_parameter("dist_coeffs").value
        self.dist_coeffs: npt.NDArray[np.float64] = np.array(dist_coeffs_list)

        self.image_width: int = self.get_parameter("image_width").value
        self.image_height: int = self.get_parameter("image_height").value

        # Topic parameters
        self.pointcloud_topic: str = self.get_parameter("pointcloud_topic").value
        self.grounded_sam_topic: str = self.get_parameter("grounded_sam_topic").value
        self.output_topic: str = self.get_parameter("output_topic").value
        self.syncqueue_size: int = self.get_parameter("sync_queue_size").value
        self.syncslop: float = self.get_parameter("sync_slop").value

        # Frame parameters
        self.pointcloud_frame: str = self.get_parameter("pointcloud_frame").value
        self.camera_frame: str = self.get_parameter("camera_frame").value
        self.base_link_frame: str = self.get_parameter("base_link_frame").value
        self.tf_timeout: float = self.get_parameter("tf_timeout").value

        # Segmentation parameters
        self.min_depth: float = self.get_parameter("min_depth").value
        self.max_depth: float = self.get_parameter("max_depth").value
        self.min_object_points: int = self.get_parameter("min_object_points").value
        self.color_saturation: float = self.get_parameter("color_saturation").value
        self.color_value: float = self.get_parameter("color_value").value
        self.enable_debug_logging: bool = self.get_parameter("enable_debug_logging").value
        self.enable_cluster_filtering: bool = self.get_parameter("enable_cluster_filtering").value
        self.cluster_eps: float = self.get_parameter("cluster_eps").value
        self.cluster_min_samples: int = self.get_parameter("cluster_min_samples").value
        self.radius_outlier_removal_radius: float = self.get_parameter("radius_outlier_removal_radius").value
        self.radius_outlier_removal_min_neighbors: int = self.get_parameter("radius_outlier_removal_min_neighbors").value

        # Modeling parameters
        self.min_obb_points: int = self.get_parameter("min_obb_points").value
        self.obb_marker_scale: float = self.get_parameter("obb_marker_scale").value
        self.publish_obb_markers: bool = self.get_parameter("publish_obb_markers").value
        self.obb_marker_topic: str = self.get_parameter("obb_marker_topic").value
        self.obb_marker_lifetime: float = self.get_parameter("obb_marker_lifetime").value
        self.enable_cross_frame_association: bool = self.get_parameter("enable_cross_frame_association").value
        self.max_accumulation_frames: int = self.get_parameter("max_accumulation_frames").value
        self.iou_threshold: float = self.get_parameter("iou_threshold").value
        self.max_object_age: int = self.get_parameter("max_object_age").value
        self.enable_object_fusion: bool = self.get_parameter("enable_object_fusion").value
        self.fusion_iou_threshold: float = self.get_parameter("fusion_iou_threshold").value
        self.accumulation_iou_threshold: float = self.get_parameter("accumulation_iou_threshold").value
        self.warmup_frames_threshold: int = self.get_parameter("warmup_frames_threshold").value
        self.publish_rate: float = self.get_parameter("publish_rate").value
        self.publish_obb_info: bool = self.get_parameter("publish_obb_info").value
        self.obb_info_topic: str = self.get_parameter("obb_info_topic").value
        self.obb_height_min: float = self.get_parameter("obb_height_min").value
        self.obb_height_max: float = self.get_parameter("obb_height_max").value
        self.obb_marker_height: float = self.get_parameter("obb_marker_height").value

        # Set logger level
        if self.enable_debug_logging:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            
        # Validate parameters
        self._validate_parameters()

        # Build centralized params for downstream modules (non-invasive)
        try:
            self.params: ModelingParams = ModelingParams.from_node(self)
        except Exception as e:
            # Keep node functional even if dataclass construction fails
            self.get_logger().warn(f"Failed to build ModelingParams: {e}")

    def _initialize_components(self) -> None:
        """Initialize ROS components."""
        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize color map for object IDs
        self.object_color_map: Dict[int, Tuple[int, int, int]] = {}

        # Initialize object modeling tracker if cross-frame association is enabled
        if self.enable_cross_frame_association:
            self.object_modeling_tracker = ObjectModelingTracker(
                max_accumulation_frames=self.max_accumulation_frames,
                iou_threshold=self.iou_threshold,
                max_object_age=self.max_object_age,
                accumulation_iou_threshold=self.accumulation_iou_threshold,
                warmup_frames_threshold=self.warmup_frames_threshold,
                fit_obb_callback=self._create_obb_callback(),
            )
        else:
            self.object_modeling_tracker = None

        # Create synchronized subscribers
        self.pc_sub = Subscriber(self, PointCloud2, self.pointcloud_topic)
        self.gsam_sub = Subscriber(self, GroundedSam, self.grounded_sam_topic)

        # Create time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.pc_sub, self.gsam_sub], queue_size=self.syncqueue_size, slop=self.syncslop
        )
        self.sync.registerCallback(self.sync_callback)

        # Create publisher for modeled point cloud
        self.modeled_pc_pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        # Create publisher for object model markers if enabled
        if self.publish_obb_markers:
            self.object_model_marker_pub = self.create_publisher(MarkerArray, self.obb_marker_topic, 10)

        # Create publisher for OBB info if enabled
        if self.publish_obb_info:
            self.obb_info_pub = self.create_publisher(OBBInfo2DArray, self.obb_info_topic, 10)

        # Initialize cache variables for timer-based publishing
        self.latest_modeled_pc: Optional[PointCloud2] = None
        self.latest_marker_array: Optional[MarkerArray] = None
        self.latest_obb_info_array: Optional[OBBInfo2DArray] = None

        # Create timer for publishing at specified rate
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Statistics
        self.frame_count: int = 0

        # Processing pipeline
        self.pipeline = Pipeline(self)
        self.publisher_cache = PublisherCache(self)

    def sync_callback(self, pc_msg: PointCloud2, gsam_msg: GroundedSam) -> None:
        """Process synchronized point cloud and GroundedSam messages."""
        self.frame_count += 1

        try:
            modeled_pc, marker_array, obb_info_array = self.pipeline.run(pc_msg, gsam_msg)
            if modeled_pc is not None:
                self.latest_modeled_pc = modeled_pc
            if marker_array is not None:
                self.latest_marker_array = marker_array
            if obb_info_array is not None:
                self.latest_obb_info_array = obb_info_array
        except (Exception, RuntimeError) as e:
            self.get_logger().error(f"Error processing frame {self.frame_count}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _get_cached_transform(self, timestamp) -> Optional[npt.NDArray[np.float32]]:
        """Get transform at specific timestamp."""
        from .core.geometry import get_transform_at_time
        return get_transform_at_time(self.pointcloud_frame, self.camera_frame, timestamp, self.tf_buffer, self.get_logger(), self.tf_timeout)

    def _get_transform_to_pointcloud_frame(self, source_frame: str, timestamp) -> Optional[npt.NDArray[np.float32]]:
        """Get transform from source frame to pointcloud frame at specific timestamp."""
        from .core.geometry import get_transform_at_time
        return get_transform_at_time(source_frame, self.pointcloud_frame, timestamp, self.tf_buffer, self.get_logger(), self.tf_timeout)

    def _validate_parameters(self) -> None:
        """Validate parameter ranges and values."""
        # Validate thresholds
        if not 0.0 <= self.iou_threshold <= 1.0:
            self.get_logger().warn(f"IOU threshold {self.iou_threshold} should be between 0.0 and 1.0")
            
        if not 0.0 <= self.accumulation_iou_threshold <= 1.0:
            self.get_logger().warn(f"Accumulation IOU threshold {self.accumulation_iou_threshold} should be between 0.0 and 1.0")
            
        if not 0.0 <= self.fusion_iou_threshold <= 1.0:
            self.get_logger().warn(f"Fusion IOU threshold {self.fusion_iou_threshold} should be between 0.0 and 1.0")
            
        # Validate frame parameters
        if self.max_object_age <= 0:
            self.get_logger().warn(f"Max object age {self.max_object_age} should be positive")
            
        if self.warmup_frames_threshold <= 0:
            self.get_logger().warn(f"Warmup frames threshold {self.warmup_frames_threshold} should be positive")
            
        # Validate depth range
        if self.min_depth >= self.max_depth:
            self.get_logger().warn(f"Min depth {self.min_depth} should be less than max depth {self.max_depth}")
            
        # Validate height range
        if self.obb_height_min >= self.obb_height_max:
            self.get_logger().warn(f"OBB height min {self.obb_height_min} should be less than max {self.obb_height_max}")

    def _extract_label_image(self, gsam_msg: GroundedSam) -> npt.NDArray[np.uint16]:
        """Extract label image from GroundedSam message."""
        return self.cv_bridge.imgmsg_to_cv2(gsam_msg.label_image, desired_encoding="mono16")

    def _create_obb_info_array(self, header: Header) -> OBBInfo2DArray:
        """Create OBBInfo2DArray message from tracked objects."""
        obb_info_array = OBBInfo2DArray()
        obb_info_array.header = header

        if not self.object_modeling_tracker:
            return obb_info_array

        # Get all objects (both active and inactive)
        all_objects = self.object_modeling_tracker.modeled_objects

        for obj_id, obj_state in all_objects.items():
            # Skip objects that are not warmed up
            if not obj_state.is_warmed_up:
                continue
                
            # Skip objects without valid OBB
            if (
                obj_state.last_obb_center is None
                or obj_state.last_obb_size is None
                or obj_state.last_obb_rotation is None
            ):
                continue

            # Create OBBInfo2D message
            obb_info = OBBInfo2D()
            obb_info.object_name = obj_state.object_name
            obb_info.object_id = int(obj_id)  # Ensure it's a Python int
            obb_info.center = [float(obj_state.last_obb_center[0]), float(obj_state.last_obb_center[1])]
            obb_info.size = [float(obj_state.last_obb_size[0]), float(obj_state.last_obb_size[1])]
            obb_info.rotation = float(obj_state.last_obb_rotation)

            obb_info_array.obb_array.append(obb_info)

        return obb_info_array

    def _fit_obb_for_aged_object(self, accumulated_points: npt.NDArray[np.float32], frame_id: str) -> Optional[Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32], float]]:
        """Fit OBB for aged objects using accumulated points."""
        if len(accumulated_points) < self.min_obb_points:
            return None
            
        # Use existing fit_obb_2d function to compute OBB from accumulated points
        return fit_obb_2d(
            accumulated_points, frame_id, self.base_link_frame, self.pointcloud_frame,
            self.min_obb_points, self.obb_height_min, self.obb_height_max,
            self.tf_buffer, self.get_logger()
        )

    def _create_obb_callback(self) -> callable:
        """Create a callback function for OBB fitting that can be used by the tracker."""
        def obb_callback(accumulated_points: npt.NDArray[np.float32], frame_count: int) -> Optional[Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32], float]]:
            # Use the current pointcloud frame as the frame_id for transform calculations
            # frame_count is not used in this implementation but kept for interface compatibility
            _ = frame_count  # Suppress unused parameter warning
            return self._fit_obb_for_aged_object(accumulated_points, self.pointcloud_frame)
        
        return obb_callback

    def publish_callback(self) -> None:
        """Timer callback for publishing cached data at specified rate."""
        self.publisher_cache.publish()


def main(args: Optional[list] = None) -> None:
    """Main entry point for the object modeling node."""
    rclpy.init(args=args)

    try:
        node = ObjectModelingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (Exception, RuntimeError) as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
