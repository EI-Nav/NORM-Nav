# navibot_pointcloud_to_laserscan

Converts 3D point clouds to 2D laser scans for the NaviBot navigation stack.

## Features

- **Real-time conversion**: Efficient point-cloud to `LaserScan` pipeline
- **Height filtering**: Configurable Z band to drop floor/ceiling noise
- **TF2 transforms**: Optional transform into a target frame
- **Dynamic subscription**: Starts/stops processing based on `LaserScan` subscriber count
- **Rich parameters**: Tuning for different robots and environments
- **Modern C++**: C++17, ROS 2 conventions

## Nodes

### 1. pointcloud_to_laserscan_node

Projects 3D `PointCloud2` into 2D `LaserScan` for classic 2D planners.

#### Subscribed topics
- `/cloud_in` (`sensor_msgs/msg/PointCloud2`) — input cloud

#### Published topics
- `/scan` (`sensor_msgs/msg/LaserScan`) — output scan

#### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| **Transform** |
| `target_frame` | string | "" | If non-empty, transform into this frame |
| `transform_tolerance` | double | 0.01 | TF lookup tolerance (s) |
| **Height** |
| `min_height` | double | -∞ | Minimum height (m) |
| `max_height` | double | +∞ | Maximum height (m) |
| **Scan geometry** |
| `angle_min` | double | -π | Min bearing (rad) |
| `angle_max` | double | π | Max bearing (rad) |
| `angle_increment` | double | π/180 | Angular step (rad) |
| `scan_time` | double | 1.0/30.0 | Scan duration (s) |
| **Range** |
| `range_min` | double | 0.0 | Min range (m) |
| `range_max` | double | +∞ | Max range (m) |
| **Infinity** |
| `use_inf` | bool | true | Use inf for no hit |
| `inf_epsilon` | double | 1.0 | Epsilon for inf ranges |

## Install

### Dependencies
```bash
sudo apt install ros-humble-sensor-msgs ros-humble-tf2-ros ros-humble-message-filters
```

### Build
```bash
colcon build --packages-select navibot_pointcloud_to_laserscan
source install/setup.bash
```

## Usage

### Basic
```bash
ros2 run navibot_pointcloud_to_laserscan pointcloud_to_laserscan_node

ros2 launch navibot_pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
```

### Parameter overrides
```bash
ros2 run navibot_pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -p target_frame:=base_link \
  -p min_height:=-0.5 \
  -p max_height:=2.0 \
  -p range_max:=15.0
```

### Remapping
```bash
ros2 run navibot_pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/velodyne/points \
  -r scan:=/laser_scan
```

## Examples

### Minimal pipeline
```bash
ros2 launch navibot_pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
ros2 topic echo /scan
```

### RViz
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix navibot_pointcloud_to_laserscan)/share/navibot_pointcloud_to_laserscan/rviz/pointcloud_to_laserscan.rviz
```

### Monitoring
```bash
ros2 topic hz /scan
ros2 node info /pointcloud_to_laserscan
```

## FAQ / troubleshooting

### Q1: No scan output
**Causes**: No input cloud, bad height limits, TF failure.

**Checks**:
```bash
ros2 topic echo /cloud_in
ros2 run tf2_tools view_frames
ros2 param set /pointcloud_to_laserscan min_height -1.0
ros2 param set /pointcloud_to_laserscan max_height 2.0
```

### Q2: High latency
**Causes**: Huge clouds, expensive TF, CPU load.

**Mitigation**: Downsample upstream, simplify frames, free CPU.

### Q3: Gaps in scan
**Causes**: Irregular cloud rate, tight height filter, wrong angular limits.

**Checks**:
```bash
ros2 topic hz /cloud_in
ros2 param set /pointcloud_to_laserscan min_height -2.0
ros2 param set /pointcloud_to_laserscan max_height 3.0
```

## Performance tips

1. Tune `min_height` / `max_height` to the environment
2. Narrow `angle_min` / `angle_max` if you do not need 360°
3. Set `scan_time` to match application needs
4. Prefer lightweight target frames when possible

## Technical notes

- **Method**: 3D→2D projection into polar bins
- **Frames**: Arbitrary frames via TF2
- **Filtering**: Height, range, angle, NaN handling
- **Implementation**: Efficient iteration and range math
- **Concurrency**: Atomics and smart pointers where appropriate

## Author

- **Wang Junhui**

## License

MIT License
