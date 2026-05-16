# Global Costmap Package

Global costmap package that converts a LiDAR point-cloud map into a navigation global costmap.

## Features

- **Live cloud-to-costmap**: Subscribes to `/laser_map` and converts to a grid in real time
- **Ground-relative height filtering**: Estimates ground height and filters floor/ceiling points
- **Tunable parameters**: Resolution, margins, height bands, and more
- **ROS 2 Nav2 compatible**: Publishes standard `nav_msgs/OccupancyGrid`

## Nodes

### 1. laser_map_to_costmap_node

ROS 2 node that converts a laser point-cloud map into a global costmap.

#### Subscribed topics

- `/laser_map` (`sensor_msgs/PointCloud2`)
  - Input laser map cloud

#### Published topics

- `/global_costmap/costmap` (`nav_msgs/OccupancyGrid`)
  - Output global costmap

#### Parameters


| Name                 | Type   | Default | Description                             |
| -------------------- | ------ | ------- | --------------------------------------- |
| `resolution`         | float  | 0.05    | Map resolution (m/cell)                 |
| `margin_left`        | float  | 10.0    | Left margin (m)                         |
| `margin_right`       | float  | 10.0    | Right margin (m)                        |
| `margin_top`         | float  | 10.0    | Top margin (m)                          |
| `margin_bottom`      | float  | 10.0    | Bottom margin (m)                       |
| `occupied_threshold` | int    | 1       | Min point count to mark a cell occupied |
| `min_z_offset`       | float  | 0.0     | Min height above ground (m)             |
| `max_z_offset`       | float  | 2.0     | Max height above ground (m)             |
| `ground_grid_size`   | float  | 5.0     | Ground estimation grid size (m)         |
| `frame_id`           | string | "map"   | Map frame                               |
| `publish_rate`       | float  | 1.0     | Publish rate (Hz)                       |


## Build

```bash
cd /path/to/your/workspace
colcon build --packages-select navibot_costmap
source install/setup.bash
```

## Usage

### Option 1: Run the node directly

```bash
ros2 run navibot_costmap laser_map_to_costmap_node
```

### Option 2: Launch with CLI overrides

```bash
ros2 launch navibot_costmap laser_map_to_costmap.launch.py \
  resolution:=0.05 \
  min_z_offset:=0.1 \
  max_z_offset:=2.5
```

### Option 3: Launch with a YAML params file

```bash
ros2 launch navibot_costmap laser_map_to_costmap_with_params.launch.py
```

Custom file:

```bash
ros2 launch navibot_costmap laser_map_to_costmap_with_params.launch.py \
  params_file:=/path/to/your/params.yaml
```

### Option 4: Edit package YAML

Edit `config/costmap_params_sim.yaml` (sim) or `config/costmap_params_real.yaml` (robot), then:

```bash
ros2 launch navibot_costmap laser_map_to_costmap_with_params.launch.py
```

## How it works

### Pipeline

1. **Receive cloud**: Subscribe to `/laser_map` and buffer the latest cloud
2. **Timer**: Fire at `publish_rate`
3. **Bounds**: Compute map extent from the cloud
4. **Ground estimate**: Grid-based ground height
5. **Height filter**: Keep points within `[min_z_offset, max_z_offset]` above ground
6. **Rasterize**: Project filtered points to 2D grid
7. **Publish**: Emit `OccupancyGrid` global costmap

### Publish-rate behavior

**Important**: The node uses **on-demand conversion** when publishing.

- **Cloud callback**: Only stores the latest cloud (lightweight)
- **Timer callback**: Runs conversion and publish at `publish_rate` (heavier work)
- **Benefits**:
  - Avoids redundant work if the cloud updates faster than you need to publish
  - Publish rate independent of input cloud rate
  - Saves CPU by converting only when publishing

**Example**:

- Cloud: 1 Hz
- Costmap publish: 0.1 Hz  
→ Roughly one conversion per 10 s despite 1 Hz input.

## Tuning

### Resolution

- **High precision**: `0.02`–`0.05` m/cell
- **Large areas**: `0.1`–`0.2` m/cell
- **Balanced**: `0.05` m/cell

### Height filtering

- **Indoor**:
  - `min_z_offset: 0.1` (reduce floor noise)
  - `max_z_offset: 2.0` (obstacles below ~2 m)
- **Outdoor**:
  - `min_z_offset: 0.0`
  - `max_z_offset: 2.5`

### Ground estimation

- **Flat floor**: `ground_grid_size: 10.0` (faster, coarser)
- **Uneven terrain**: `ground_grid_size: 2.0` (finer)
- **Suggested default**: `ground_grid_size: 5.0`

### Occupancy threshold

- **Dense clouds**: `occupied_threshold: 5`–`10`
- **Sparse clouds**: `occupied_threshold: 1`–`2`
- **Default**: `occupied_threshold: 1`

## Visualization

```bash
rviz2
```

Add:

1. **PointCloud2** — topic `/laser_map`
2. **Map** — topic `/global_costmap/costmap`

## Example YAML

See `config/costmap_params_sim.yaml` or `config/costmap_params_real.yaml`:

```yaml
/**:
  ros__parameters:
    resolution: 0.05
    margin_left: 10.0
    margin_right: 10.0
    margin_top: 10.0
    margin_bottom: 10.0
    occupied_threshold: 1
    min_z_offset: 0.0
    max_z_offset: 2.0
    ground_grid_size: 5.0
    frame_id: "map"
    publish_rate: 1.0
```

## Troubleshooting

### No cloud received

- `ros2 topic list | grep laser_map`
- `ros2 topic hz /laser_map`

### Empty costmap

- Widen `min_z_offset` / `max_z_offset`
- Lower `occupied_threshold`
- Inspect cloud Z range

### Performance

- Increase `resolution` (coarser map)
- Increase `ground_grid_size`
- Lower `publish_rate`

### Wrong bounds

- Increase `margin_*`
- Verify cloud frame

## Dependencies

- ROS 2 (Humble or newer)
- Python 3.8+
- numpy
- scipy
- sensor_msgs_py

## Author

- **Wang Junhui**
- Created: 2025-10-07

## License

MIT License