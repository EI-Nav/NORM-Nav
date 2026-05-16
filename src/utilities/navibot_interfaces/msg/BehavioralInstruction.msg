# ==============================================================================
# BehavioralInstruction Message Definition
# ==============================================================================
# Behavioral instruction message for natural language instruction input.
# Used to transmit user instructions and pre-configured rules for behavioral
# constraint parsing and navigation systems.
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

# Header with timestamp and frame information
std_msgs/Header header

# Online constraints (real-time user instructions)
# These are instructions provided by users in real-time during navigation
string[] online_constraints

# Offline constraints (pre-configured rules)
# These are pre-defined behavioral rules and constraints
string[] offline_constraints
