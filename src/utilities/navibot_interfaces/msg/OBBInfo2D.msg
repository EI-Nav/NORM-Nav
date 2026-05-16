# ==============================================================================
# OBBInfo2D Message Definition
# ==============================================================================
# 2D Oriented Bounding Box information for objects in behavioral constraints.
# Used to provide spatial information about detected objects for navigation
# and path planning systems.
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

# Object name
string object_name

# Object ID for tracking
int32 object_id

# OBB center position in map frame [x, y] (2D)
float64[2] center

# OBB size [length, width] (2D)
float64[2] size

# OBB rotation angle (yaw in radians)
float64 rotation
