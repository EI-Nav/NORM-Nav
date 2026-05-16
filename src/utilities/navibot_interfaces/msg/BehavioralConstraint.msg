# ==============================================================================
# BehavioralConstraint Message Definition
# ==============================================================================
# Behavioral constraint message with 2D OBB information for navigation systems.
# Used to transmit structured behavioral constraints with spatial object
# information for path planning and navigation tasks.
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

# Header with timestamp and frame information
std_msgs/Header header

# Object list being referenced in this constraint
string[] object_list

# Direction constraint: -1.0 (left) to 1.0 (right), 0.0 (center)
# Special value: -999.0 indicates null/no constraint
float64 direction_constrain

# Velocity constraint: 0.0 (slow) to 1.0 (fast), 0.5 (normal)
# Special value: -999.0 indicates null/no constraint
float64 velocity_constrain

# Traversability constraint (integer enum)
# 0: null (no constraint)
# 1: traversable
# 2: non-traversable
int32 traversability_constrain

# OBB information for objects in object_list (2D bounding boxes)
OBBInfo2D[] obb_list
