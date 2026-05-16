# ==============================================================================
# BehavioralConstraintArray Message Definition
# ==============================================================================
# Array of behavioral constraints for navigation systems.
# Used to transmit multiple behavioral constraints in a single message
# for path planning and navigation tasks.
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

# Header with timestamp and frame information
std_msgs/Header header

# Array of behavioral constraints
BehavioralConstraint[] constraints
