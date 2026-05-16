# NaviBot GroundedSAM2 Tracker

Real-time object tracking ROS2 node based on GroundingDINO and SAM2.

## Features

- **Zero-shot object detection**: Text-prompted object detection using GroundingDINO
- **Precise segmentation**: Instance segmentation using SAM2
- **Real-time tracking**: Hybrid detection+tracking mode for high frame rates
- **ID persistence**: Maintains object ID consistency across frames
- **Flexible configuration**: Supports multiple performance and accuracy modes

## Performance Modes

- **Pure detection mode** (`detection_interval=1`): ~3-4 FPS, saves GPU memory
- **Hybrid tracking mode** (`detection_interval>1`): ~10-20 FPS, full functionality

## Dependency Installation

```bash
conda create -n gsam2 python=3.10
conda activate gsam2
conda install -y -c conda-forge libstdcxx-ng
conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia -y
conda install -c conda-forge gdal
conda install openssl=3.0.3 -c conda-forge -y

pip install transformers opencv-python supervision chardet charset-normalizer "numpy<2"
pip install git+https://github.com/IDEA-Research/Grounded-SAM-2
```

## Usage

### Basic Launch

```bash
# Launch with default configuration
ros2 launch navibot_grounded_sam2 grounded_sam2_tracker.launch.py

# Launch with custom configuration file
ros2 launch navibot_grounded_sam2 grounded_sam2_tracker.launch.py \
    config_file:=/path/to/custom_config.yaml

# Launch with simulation time
ros2 launch navibot_grounded_sam2 grounded_sam2_tracker.launch.py \
    use_sim_time:=true
```

**Note**: Detection parameters (such as `prompt_text`, input topics, etc.) must be modified in the configuration file `config/tracker_params.yaml` and cannot be passed as launch parameters.

### Runtime Detection Prompt Updates

```bash
# Dynamically change detection targets
ros2 topic pub /grounded_sam2_tracker/prompt std_msgs/String "data: 'hand.'" --once
```

## Topics

### Subscribed Topics

- `/camera_sensor/image_raw` (`sensor_msgs/Image`): Input image stream (default, can be modified in config file)
- `/grounded_sam2_tracker/prompt` (`std_msgs/String`): Detection prompt text (optional, for runtime updates)
- `/behavioral_constraints` (`navibot_interfaces/BehavioralConstraintArray`): Behavioral constraints from instruction parsing (when `enable_constraint_subscription: true`)

### Published Topics

- `/grounded_sam2_tracker/output_image` (`sensor_msgs/Image`): Annotated output image with masks, boxes, and labels
- `/grounded_sam2_tracker/grounded_sam` (`navibot_interfaces/GroundedSam`): Structured detection results with label image, class names, IDs, confidences, and boxes
- `/grounded_sam2_tracker/label_image_debug` (`sensor_msgs/Image`): Debug topic for semantic segmentation mask (only when `enable_label_image_debug: true`)

## Parameters

### Model Configuration

- `grounding_model_id` (string): GroundingDINO model identifier
  - Default: `"IDEA-Research/grounding-dino-tiny"`
  - Options: `grounding-dino-tiny` (fast), `grounding-dino-base` (accurate)
- `sam2_model_id` (string): SAM2 model identifier
  - Default: `"facebook/sam2.1-hiera-small"`
  - Options: `hiera-tiny`, `hiera-small`, `hiera-base-plus`, `hiera-large`
- `device` (string): Inference device
  - Default: `"cuda"`
  - Options: `"cuda"`, `"cpu"`

### Detection Configuration

- `prompt_text` (string): Detection prompt text
  - Default: `"person."`
  - Format: Period-separated class list, e.g., `"car. truck. bus."`
- `box_threshold` (double): Bounding box confidence threshold [0.0-1.0]
  - Default: `0.35`
  - Recommended range: 0.25-0.45
- `text_threshold` (double): Text matching confidence threshold [0.0-1.0]
  - Default: `0.25`
  - Recommended range: 0.20-0.30

### Tracking Configuration

- `detection_interval` (int): Full detection interval (frames)
  - Default: `20`
  - `1`: Pure detection mode (saves memory)
  - `>1`: Hybrid detection+tracking mode
- `iou_threshold` (double): Tracking association IOU threshold [0.0-1.0]
  - Default: `0.3`
  - Recommended range: 0.25-0.40

### Display Configuration

- `stats_print_interval` (int): Statistics print interval (frames)
  - Default: `10`
- `overlay_alpha` (double): Mask overlay transparency [0.0-1.0]
  - Default: `0.5`
  - Recommended range: 0.3-0.7
- `info_box_width` (int): Information box width (pixels)
  - Default: `400`
- `info_box_height` (int): Information box height (pixels)
  - Default: `180`

### Topic Configuration

- `input_image_topic` (string): Input image topic
  - Default: `"/camera_sensor/image_raw"`
- `output_image_topic` (string): Output image topic
  - Default: `"~/output_image"`
- `grounded_sam_topic` (string): Structured detection results topic
  - Default: `"~/grounded_sam"`
- `prompt_topic` (string): Prompt update topic
  - Default: `"~/prompt"`
- `label_image_debug_topic` (string): Label image debug topic
  - Default: `"~/label_image_debug"`

### Performance Optimization Configuration

- `enable_visualization` (bool): Enable/disable visual overlay
  - Default: `true`
  - Effect: When disabled, saves 15-30% processing time if only detection data is needed
- `include_source_image_in_msg` (bool): Include source image in GroundedSam message
  - Default: `false`
  - Effect: When disabled, reduces message size by 30-50%
- `gpu_cache_clear_interval` (int): GPU cache clearing frequency (frames)
  - Default: `10`
  - Effect: Lower values save VRAM, higher values improve FPS
- `enable_torch_compile` (bool): Enable PyTorch 2.0+ model compilation
  - Default: `false`
  - Effect: 10-20% speedup after warmup, requires 8GB+ VRAM

### Constraint Integration Configuration

- `enable_constraint_subscription` (bool): Subscribe to behavioral constraints
  - Default: `true`
  - Effect: Automatically updates detection prompt based on parsed constraints
- `constraint_topic` (string): Behavioral constraints topic
  - Default: `"/behavioral_constraints"`

### Debug Configuration

- `enable_debug_logging` (bool): Enable debug logging
  - Default: `false`
  - Effect: Shows detailed frame-by-frame processing information
- `enable_label_image_debug` (bool): Enable label image debug publishing
  - Default: `false`
  - Effect: Publishes semantic segmentation mask for debugging

## Detection Data Format

The `/grounded_sam2_tracker/grounded_sam` topic provides structured detection results in ROS2 message format with the following information:

- `label_image`: Semantic segmentation mask (sensor_msgs/Image)
- `class_names`: List of detected object class names
- `ids`: List of object instance IDs
- `confidences`: List of detection confidence scores
- `boxes`: List of bounding box regions of interest
- `source_image`: Optional source image (if `include_source_image_in_msg: true`)

## Launch Parameters

The launch file supports the following parameters:

- `config_file` (string): Configuration file path
  - Default: `<package_share>/config/tracker_params.yaml`
- `use_sim_time` (bool): Use simulation time
  - Default: `false`

**Important**: All tracking parameters (such as `prompt_text`, `input_image_topic`, etc.) must be set in the YAML configuration file and cannot be passed as launch parameters.

## License

MIT License

## Author

Wang Junhui

## References

- [GroundedSAM2](https://github.com/IDEA-Research/Grounded-SAM-2)

