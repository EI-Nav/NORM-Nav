# ==============================================================================
# GroundedSam Message Definition
# ==============================================================================
# Perception results from Grounded SAM2 vision model for object detection and
# semantic segmentation in robot navigation and scene understanding tasks.
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

# Message header with timestamp and frame information
std_msgs/Header header

# Original RGB image captured by the camera (used as input for detection)
sensor_msgs/Image source_image

# Semantic label image where each pixel value corresponds to a detection index
# Data type: uint16 (mono16 encoding), each value maps to an entry in class_names array
# Pixel value 0 represents background, values 1+ correspond to object IDs
sensor_msgs/Image label_image

# Array of detected object class names (e.g., "person", "chair", "table")
# Index in this array corresponds to pixel values in label_image
string[] class_names

# Unique tracking IDs for each detected object across data flow
# Enables consistent object tracking and identification over time
# Each ID is a unique integer assigned to track the same object across frames
# Array length matches class_names array
int32[] ids

# Detection confidence scores for each detected object class
# Range: [0.0, 1.0], where 1.0 indicates highest confidence
# Array length matches class_names array
float64[] confidences

# Bounding boxes for detected objects in the source image
# Each ROI defines the rectangular region containing a detected object
sensor_msgs/RegionOfInterest[] boxes
