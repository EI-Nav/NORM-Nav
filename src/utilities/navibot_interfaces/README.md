# navibot_interfaces

Custom ROS2 interfaces package for NaviBot navigation system.

## Overview

This package provides message types, service definitions, and action interfaces used across the NaviBot autonomous mobile robot navigation system. It enables standardized communication between different components for perception, planning, and task execution.

## Package Contents

### Messages

#### GroundedSam.msg
Perception results from Grounded SAM2 vision model for detection, segmentation, and tracking:
- **header**: Message header with timestamp and frame information
- **source_image**: Original RGB image captured by the camera
- **label_image**: Semantic segmentation label image (int16 format, pixel values correspond to detection indices)
- **class_names**: Array of detected object class names (e.g., "person", "chair", "table")
- **ids**: Unique tracking IDs for each detected object across video frames (int32 array)
- **confidences**: Detection confidence scores for each object class (range: 0.0-1.0)
- **boxes**: Bounding boxes for detected objects (sensor_msgs/RegionOfInterest)

**Dependencies:** `std_msgs`, `sensor_msgs`

#### BehavioralConstraint.msg
Behavioral constraint message with 2D OBB information for navigation systems:
- **header**: Message header with timestamp and frame information
- **object_list**: Array of object names being referenced in this constraint
- **direction_constrain**: Direction preference (-1.0=left, 0.0=center, 1.0=right, -999.0=null)
- **velocity_constrain**: Velocity requirement (0.0=slow, 0.5=normal, 1.0=fast, -999.0=null)
- **traversability_constrain**: Traversability constraint (0=null, 1=traversable, 2=non-traversable)
- **obb_list**: Array of 2D OBB information for objects (OBBInfo2D[])

**Dependencies:** `std_msgs`, `OBBInfo2D`

#### OBBInfo2D.msg
2D Oriented Bounding Box information for objects in behavioral constraints:
- **object_name**: Object name
- **object_id**: Object ID for tracking
- **class_name**: Class name from detection
- **center**: OBB center position in map frame [x, y] (float64[2])
- **size**: OBB size [length, width] (float64[2])
- **rotation**: OBB rotation angle (yaw in radians)

**Dependencies:** None

## Building

This package is built automatically as part of the NaviBot workspace:

```bash
cd /home/wjh/Research/NORM-Nav
colcon build --packages-select navibot_interfaces
```

## Usage

### C++ Example
```cpp
#include <navibot_interfaces/msg/grounded_sam.hpp>
#include <navibot_interfaces/msg/behavioral_constraint.hpp>

// Create and use the message type
navibot_interfaces::msg::GroundedSam perception_result;
perception_result.header.stamp = this->now();
perception_result.header.frame_id = "camera_link";
// ... populate other fields

// Publish the message
publisher_->publish(perception_result);

// Create behavioral constraint message
navibot_interfaces::msg::BehavioralConstraint constraint_msg;
constraint_msg.header.stamp = this->now();
constraint_msg.header.frame_id = "map";
constraint_msg.object_list = {"person", "chair"};
constraint_msg.direction_constrain = 0.0;  // center
constraint_msg.velocity_constrain = 0.5;  // normal
constraint_msg.traversability_constrain = 1;  // traversable
// ... populate obb_list

// Publish behavioral constraint
constraint_publisher_->publish(constraint_msg);
```

### Python Example
```python
from navibot_interfaces.msg import GroundedSam, BehavioralConstraint, OBBInfo2D

# Create and use the message type
perception_msg = GroundedSam()
perception_msg.header.stamp = self.get_clock().now().to_msg()
perception_msg.header.frame_id = 'camera_link'
# ... populate other fields

# Publish the message
self.publisher.publish(perception_msg)

# Create behavioral constraint message
constraint_msg = BehavioralConstraint()
constraint_msg.header.stamp = self.get_clock().now().to_msg()
constraint_msg.header.frame_id = 'map'
constraint_msg.object_list = ['person', 'chair']
constraint_msg.direction_constrain = 0.0  # center
constraint_msg.velocity_constrain = 0.5  # normal
constraint_msg.traversability_constrain = 1  # traversable

# Create OBB information
obb_info = OBBInfo2D()
obb_info.object_name = 'person'
obb_info.object_id = 1
obb_info.class_name = 'person'
obb_info.center = [1.0, 2.0]  # [x, y]
obb_info.size = [0.5, 0.3]  # [length, width]
obb_info.rotation = 0.0  # yaw in radians
constraint_msg.obb_list = [obb_info]

# Publish behavioral constraint
self.constraint_publisher.publish(constraint_msg)
```

## Dependencies

- ROS2 Humble or later
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `action_msgs`

## Author

**Wang Junhui**

## License

MIT License


