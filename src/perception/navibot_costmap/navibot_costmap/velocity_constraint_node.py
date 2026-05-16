#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Velocity Constraint Node.

Generates velocity constraints based on behavioral constraints.
Uses AABB information and velocity constraints from object modeling to create
velocity limit commands for navigation.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

from typing import List, Optional, Tuple

import numpy as np
import rclpy
from nav2_msgs.msg import SpeedLimit
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


class VelocityConstraintNode(Node):
    """
    Generate velocity constraints from behavioral constraints.
    
    This node subscribes to behavioral constraints containing AABB information
    and velocity constraints, then generates velocity limit commands based on
    robot position relative to constraint regions.
    """
    
    def __init__(self) -> None:
        """Initialize the node with parameters and publishers/subscribers."""
        super().__init__('velocity_constraint_node')
        
        # Initialize all instance attributes
        self.constraint_topic_: str = ''
        self.output_topic_: str = ''
        self.self_constraint_topic_: str = ''
        self.default_speed_percentage_: float = 100.0
        self.update_rate_: float = 10.0
        self.enable_debug_logging_: bool = True
        self.frame_id_: str = 'map'
        self.robot_frame_id_: str = 'base_link'
        self.transform_timeout_: float = 1.0
        self.margin_distance_: float = 0.5
        self.enable_caching_: bool = True
        
        # Internal state
        self.latest_constraints_: Optional[BehavioralConstraintArray] = None
        self.processing_: bool = False
        self.last_published_percentage_: Optional[float] = None
        self._last_tf_error_time_ = None
        self._cos_cache_: dict = {}
        self._sin_cache_: dict = {}
        
        # ROS components
        self.tf_buffer_: Optional[Buffer] = None
        self.tf_listener_: Optional[TransformListener] = None
        self.constraint_sub_ = None
        self.self_constraint_sub_ = None
        self.speed_limit_pub_ = None
        self.timer_ = None
        
        self._declare_parameters()
        self._load_parameters()
        self._initialize_components()
        
        self.get_logger().info('Velocity Constraint Node initialized')
    
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        # Input/Output configuration
        self.declare_parameter('constraint_topic', '/compensated_behavioral_constraints')
        self.declare_parameter('self_constraint_topic', '/behavioral_constraints')
        self.declare_parameter('output_topic', '/speed_limit')
        
        # Velocity configuration
        self.declare_parameter('default_speed_percentage', 100.0)  # 100% default speed
        
        # Update configuration
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('enable_debug_logging', True)
        
        # TF configuration
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('transform_timeout', 1.0)
        
        # OBB configuration
        self.declare_parameter('margin_distance', 0.5)  # Expansion margin for non-traversable areas
        
        # Performance configuration
        self.declare_parameter('enable_caching', True)  # Enable trigonometric caching
    
    def _load_parameters(self) -> None:
        """Load and validate all ROS parameters."""
        # Input/Output configuration
        self.constraint_topic_ = self.get_parameter('constraint_topic').value
        self.self_constraint_topic_ = self.get_parameter('self_constraint_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        
        # Velocity configuration
        self.default_speed_percentage_ = self.get_parameter('default_speed_percentage').value
        
        # Update configuration
        self.update_rate_ = self.get_parameter('update_rate').value
        self.enable_debug_logging_ = self.get_parameter('enable_debug_logging').value
        
        # TF configuration
        self.frame_id_ = self.get_parameter('frame_id').value
        self.robot_frame_id_ = self.get_parameter('robot_frame_id').value
        self.transform_timeout_ = self.get_parameter('transform_timeout').value
        
        # OBB configuration
        self.margin_distance_ = self.get_parameter('margin_distance').value
        
        # Performance configuration
        self.enable_caching_ = self.get_parameter('enable_caching').value
        
        # Set logger level
        if self.enable_debug_logging_:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Validate parameters
        if not self._validate_parameters():
            raise ValueError("Invalid parameter configuration detected")
        
        # Log key configuration parameters
        self.get_logger().info(f"Constraint topic: {self.constraint_topic_}")
        self.get_logger().info(f"Self constraint topic: {self.self_constraint_topic_}")
        self.get_logger().info(f"Output topic: {self.output_topic_}")
        self.get_logger().info(f"Default speed percentage: {self.default_speed_percentage_}")
        self.get_logger().info(f"Update rate: {self.update_rate_} Hz")
    
    def _validate_parameters(self) -> bool:
        """
        Validate all ROS parameters for correctness and reasonable ranges.
        
        Returns:
            bool: True if all parameters are valid, False otherwise
        """
        try:
            # Validate speed percentage
            if not (0.0 <= self.default_speed_percentage_ <= 100.0):
                self.get_logger().error(
                    f"Invalid default_speed_percentage: {self.default_speed_percentage_}. "
                    f"Must be in range [0.0, 100.0]")
                return False
            
            # Validate update rate
            if self.update_rate_ <= 0:
                self.get_logger().error(f"Invalid update rate: {self.update_rate_}. Must be > 0")
                return False
            
            # Validate margin distance
            if self.margin_distance_ < 0:
                self.get_logger().error(f"Invalid margin_distance: {self.margin_distance_}. Must be >= 0")
                return False
            
            # Validate transform timeout
            if self.transform_timeout_ <= 0:
                self.get_logger().error(f"Invalid transform_timeout: {self.transform_timeout_}. Must be > 0")
                return False
            
            # Validate frame IDs
            if not self.frame_id_ or not self.robot_frame_id_:
                self.get_logger().error("Frame IDs cannot be empty")
                return False
            
            # Validate topic names
            if not self.constraint_topic_ or not self.self_constraint_topic_ or not self.output_topic_:
                self.get_logger().error("Topic names cannot be empty")
                return False
            
            self.get_logger().debug("All parameters validated successfully")
            return True
            
        except (ValueError, TypeError, AttributeError) as e:
            self.get_logger().error(f"Parameter validation error: {e}")
            return False
    
    def _expand_obb_if_non_traversable(self, obb, traversability: int) -> Tuple[List[float], List[float]]:
        """
        Expand OBB size if the area is non-traversable.
        
        Args:
            obb: OBB information with center, size, rotation
            traversability: Traversability constraint (0=null, 1=traversable, 2=non-traversable)
            
        Returns:
            Tuple of (expanded_center, expanded_size)
        """
        center = [float(obb.center[0]), float(obb.center[1])]
        size = [float(obb.size[0]), float(obb.size[1])]
        
        # Expand for non-traversable areas
        if traversability == 2:
            margin = 2 * self.margin_distance_
            size = [size[0] + margin, size[1] + margin]
        
        return center, size
    
    def _is_point_in_obb(self, point: Tuple[float, float], center: List[float], 
                        size: List[float], rotation: float) -> bool:
        """
        Check if a point is inside an oriented bounding box.
        
        Args:
            point: Point coordinates (x, y) in map frame
            center: OBB center coordinates [x, y]
            size: OBB size [length, width]
            rotation: OBB rotation angle in radians
            
        Returns:
            bool: True if point is inside OBB, False otherwise
        """
        px, py = point
        cx, cy = center
        w, h = size
        
        # Transform point to OBB local coordinates
        dx = px - cx
        dy = py - cy
        
        # Use cached trigonometric values if caching is enabled
        if self.enable_caching_ and hasattr(self, '_cos_cache_') and hasattr(self, '_sin_cache_'):
            # Use cached values for common rotation angles
            rotation_key = round(rotation, 3)  # Round to avoid floating point precision issues
            if rotation_key in self._cos_cache_:
                cos_neg = self._cos_cache_[rotation_key]
                sin_neg = self._sin_cache_[rotation_key]
            else:
                cos_neg = np.cos(-rotation)
                sin_neg = np.sin(-rotation)
                self._cos_cache_[rotation_key] = cos_neg
                self._sin_cache_[rotation_key] = sin_neg
        else:
            # Calculate trigonometric values directly
            cos_neg = np.cos(-rotation)
            sin_neg = np.sin(-rotation)
        
        # Apply inverse rotation
        lx = cos_neg * dx - sin_neg * dy
        ly = sin_neg * dx + cos_neg * dy
        
        # Check if point is inside OBB bounds
        half_w, half_h = w / 2.0, h / 2.0
        return (abs(lx) <= half_w) and (abs(ly) <= half_h)
    
    
    def _calculate_velocity_limit(self, constraints: BehavioralConstraintArray, 
                                robot_position: Tuple[float, float]) -> float:
        """
        Calculate velocity limit based on robot position and constraints.
        
        Args:
            constraints: Behavioral constraint array
            robot_position: Robot position (x, y) in map frame
            
        Returns:
            float: Velocity percentage (0.0-100.0)
        """
        # Early return if no constraints
        if not hasattr(constraints, 'constraints') or not constraints.constraints:
            return self.default_speed_percentage_
        
        # Process constraints in order of priority
        for constraint in constraints.constraints:
            # Extract constraint values
            velocity_constrain = float(constraint.velocity_constrain)
            traversability = int(constraint.traversability_constrain)
            
            # Skip null velocity constraints
            if velocity_constrain == -999.0:
                continue
            
            # Check each OBB in the constraint
            for obb in constraint.obb_list:
                # Expand OBB if non-traversable
                center, size = self._expand_obb_if_non_traversable(obb, traversability)
                rotation = float(obb.rotation)
                
                # Check if robot is inside this OBB
                if self._is_point_in_obb(robot_position, center, size, rotation):
                    # Convert velocity_constrain (0.0-1.0) to percentage (0-100)
                    velocity_percentage = max(0.0, min(100.0, velocity_constrain * 100.0))
                    if self.enable_debug_logging_:
                        self.get_logger().debug(
                            f"Robot in constraint OBB: velocity_constrain={velocity_constrain:.2f}, "
                            f"traversability={traversability}, percentage={velocity_percentage:.1f}%")
                    return velocity_percentage
        
        # Robot not in any constraint OBB
        return self.default_speed_percentage_
    
    def _publish_velocity_limit(self, percentage: float) -> None:
        """
        Publish velocity limit message.
        
        Args:
            percentage: Velocity percentage (0.0-100.0)
        """
        try:
            msg = SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id_
            msg.percentage = True  # Use percentage mode
            msg.speed_limit = percentage  # Store percentage value in speed_limit field
            
            self.speed_limit_pub_.publish(msg)
            
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Published velocity limit: {percentage:.1f}%")
                
        except (ValueError, TypeError, AttributeError, RuntimeError) as e:
            self.get_logger().error(f"Failed to publish velocity limit: {e}")
    
    def _get_robot_pose_in_map(self) -> Optional[Tuple[float, float]]:
        """
        Get robot position in map frame using TF2.
        
        Returns:
            Tuple of (x, y) position in map frame, or None if transform fails
        """
        try:
            timeout = Duration(seconds=self.transform_timeout_)
            transform = self.tf_buffer_.lookup_transform(
                self.frame_id_, self.robot_frame_id_, rclpy.time.Time(), timeout)
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Validate position values
            if not (np.isfinite(x) and np.isfinite(y)):
                self.get_logger().warn(f"Invalid robot position: ({x}, {y})")
                return None
            
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Robot pose in map: ({x:.3f}, {y:.3f})")
            
            return (x, y)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            now = self.get_clock().now()
            # Reduce error log frequency to avoid spam
            if (now - self._last_tf_error_time_).nanoseconds / 1e9 > 5.0:
                self.get_logger().warn(f"Failed to get robot pose from TF: {e}")
                self._last_tf_error_time_ = now
            return None
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"Robot pose calculation error: {e}")
            return None
    
    def _initialize_components(self) -> None:
        """Initialize ROS components (publishers, subscribers)."""
        # QoS profile for velocity limit topic
        velocity_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        # Initialize TF2 components
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        # Initialize subscriber for behavioral constraints
        self.constraint_sub_ = self.create_subscription(
            BehavioralConstraintArray, self.constraint_topic_, self.constraint_callback, 1)
        
        # Initialize subscriber for self constraints
        self.self_constraint_sub_ = self.create_subscription(
            BehavioralConstraintArray, self.self_constraint_topic_, self.self_constraint_callback, 1)
        
        # Initialize publisher for velocity limits
        self.speed_limit_pub_ = self.create_publisher(
            SpeedLimit, self.output_topic_, velocity_qos)
        
        # Initialize TF error time tracking
        self._last_tf_error_time_ = self.get_clock().now()
        
        # Initialize trigonometric caching if enabled
        if self.enable_caching_:
            self.get_logger().info("Trigonometric caching enabled")
        
        # Create timer for periodic processing
        period = 1.0 / max(1e-6, float(self.update_rate_))
        self.timer_ = self.create_timer(period, self.timer_callback)
        
        
        self.get_logger().info("Subscribers and publishers initialized")
        
        # Publish initial default speed limit
        self._publish_velocity_limit(self.default_speed_percentage_)
        self.last_published_percentage_ = self.default_speed_percentage_
        self.get_logger().info(f"Published initial default speed limit: {self.default_speed_percentage_}%")
    
    def constraint_callback(self, msg: BehavioralConstraintArray) -> None:
        """
        Callback for receiving behavioral constraints.
        
        Args:
            msg: BehavioralConstraintArray message containing behavioral constraints
        """
        try:
            # Validate message structure
            if not hasattr(msg, 'constraints') or msg.constraints is None:
                self.get_logger().warn("Received invalid constraint message: missing constraints field")
                return
            
            # Validate constraints data
            if not isinstance(msg.constraints, (list, tuple)):
                self.get_logger().warn("Received invalid constraint message: constraints field is not a list")
                return
            
            # Store constraints
            self.latest_constraints_ = msg
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Received constraints: {len(msg.constraints)} constraints")
            
        except AttributeError as e:
            self.get_logger().error(f"Invalid constraint message structure: {e}")
        except ValueError as e:
            self.get_logger().error(f"Invalid constraint data values: {e}")
        except TypeError as e:
            self.get_logger().error(f"Constraint processing error: {e}")
    
    def self_constraint_callback(self, msg: BehavioralConstraintArray) -> None:
        """
        Callback for receiving self behavioral constraints.
        
        This callback processes constraints specifically targeting the robot itself ("self").
        When a constraint with object_list containing only "self" is found, it updates
        the default speed percentage and publishes the new velocity limit.
        
        Args:
            msg: BehavioralConstraintArray message containing behavioral constraints
        """
        try:
            # Validate message structure
            if not hasattr(msg, 'constraints') or msg.constraints is None:
                self.get_logger().warn("Received invalid self constraint message: missing constraints field")
                return
            
            # Validate constraints data
            if not isinstance(msg.constraints, (list, tuple)):
                self.get_logger().warn("Received invalid self constraint message: constraints field is not a list")
                return
            
            # Process constraints to find self-targeted ones
            for constraint in msg.constraints:
                # Check if this constraint targets only "self"
                if (hasattr(constraint, 'object_list') and 
                    len(constraint.object_list) == 1 and 
                    constraint.object_list[0] == "self"):
                    
                    # Check if velocity constraint is valid
                    if (hasattr(constraint, 'velocity_constrain') and 
                        constraint.velocity_constrain != -999.0):
                        
                        # Convert velocity_constrain (0.0-1.0) to percentage (0-100)
                        velocity_percentage = max(0.0, min(100.0, constraint.velocity_constrain * 100.0 + 1e-3))
                        
                        # Update default speed percentage
                        old_percentage = self.default_speed_percentage_
                        self.default_speed_percentage_ = velocity_percentage
                        
                        # Publish new velocity limit
                        self._publish_velocity_limit(velocity_percentage)
                        self.last_published_percentage_ = velocity_percentage
                        
                        self.get_logger().info(
                            f"Updated default speed from {old_percentage:.1f}% to {velocity_percentage:.1f}% "
                            f"based on self constraint")
                        
                        # Found and processed self constraint, return immediately
                        return
            
            if self.enable_debug_logging_:
                self.get_logger().debug(f"Received self constraints: {len(msg.constraints)} constraints, no self-targeted constraints found")
            
        except AttributeError as e:
            self.get_logger().error(f"Invalid self constraint message structure: {e}")
        except ValueError as e:
            self.get_logger().error(f"Invalid self constraint data values: {e}")
        except TypeError as e:
            self.get_logger().error(f"Self constraint processing error: {e}")
    
    def timer_callback(self) -> None:
        """Process and publish velocity limits at configured rate."""
        if self.latest_constraints_ is None:
            if self.enable_debug_logging_:
                self.get_logger().debug('No constraint data received yet')
            return
        
        if self.processing_:
            self.get_logger().warn('Still processing, skipping this cycle')
            return
        
        self.processing_ = True
        
        try:
            # Get robot position
            robot_position = self._get_robot_pose_in_map()
            if robot_position is None:
                self.get_logger().warn("Cannot get robot position, skipping velocity calculation")
                return
            
            # Calculate velocity limit
            velocity_percentage = self._calculate_velocity_limit(self.latest_constraints_, robot_position)
            
            # Publish if changed (with small tolerance to avoid floating point issues)
            if (self.last_published_percentage_ is None or 
                abs(velocity_percentage - self.last_published_percentage_) > 0.1):
                self._publish_velocity_limit(velocity_percentage)
                self.last_published_percentage_ = velocity_percentage
            
        except ValueError as e:
            self.get_logger().error(f'Invalid data in velocity calculation: {e}')
        except RuntimeError as e:
            self.get_logger().error(f'Runtime error in velocity calculation: {e}')
        except AttributeError as e:
            self.get_logger().error(f'Velocity calculation error: {e}')
        finally:
            self.processing_ = False
    
    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed."""
        try:
            # Cancel timer
            if hasattr(self, 'timer_') and self.timer_ is not None:
                self.timer_.cancel()
            
            # Clear caches
            if hasattr(self, '_cos_cache_'):
                self._cos_cache_.clear()
            if hasattr(self, '_sin_cache_'):
                self._sin_cache_.clear()
            
            # Clear internal state
            self.latest_constraints_ = None
            self.processing_ = False
            
            self.get_logger().info("Velocity constraint node resources cleaned up")
            
        except (ValueError, TypeError, AttributeError, RuntimeError) as e:
            self.get_logger().error(f"Error during node cleanup: {e}")
        finally:
            # Call parent destroy method
            super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for the velocity constraint node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    node = VelocityConstraintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
