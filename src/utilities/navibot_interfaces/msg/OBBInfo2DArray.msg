# ==============================================================================
# OBBInfo2DArray Message Definition
# ==============================================================================
# Array of 2D Oriented Bounding Box information for objects in behavioral constraints.
# Used to provide spatial information about detected objects for navigation
# and path planning systems.
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

# Header with timestamp and frame information
std_msgs/Header header

# Array of 2D OBB information
OBBInfo2D[] obb_array
