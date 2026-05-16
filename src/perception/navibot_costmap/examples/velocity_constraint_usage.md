#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Velocity Constraint Node Usage Examples.

This file demonstrates how to use the velocity constraint node in different scenarios.

Author: Wang Junhui <wjh_9696@163.com>
License: MIT
"""

# Example 1: Basic usage with default parameters
# ros2 launch navibot_costmap velocity_constraint_bringup.launch.py

# Example 2: With custom parameter file
# ros2 launch navibot_costmap velocity_constraint_bringup.launch.py \
#   velocity_params_file:=/path/to/custom_params.yaml

# Example 3: For real robot (disable sim time)
# ros2 launch navibot_costmap velocity_constraint_bringup.launch.py \
#   use_sim_time:=False

# Example 4: With custom topics
# ros2 launch navibot_costmap velocity_constraint_bringup.launch.py \
#   constraint_topic:=/my_behavioral_constraints \
#   output_topic:=/my_speed_limit

# Example 5: Complete custom configuration
# ros2 launch navibot_costmap velocity_constraint_bringup.launch.py \
#   velocity_params_file:=/path/to/custom_params.yaml \
#   use_sim_time:=False \
#   constraint_topic:=/compensated_behavioral_constraints \
#   output_topic:=/speed_limit

# Monitoring commands:
# ros2 topic echo /speed_limit
# ros2 topic hz /speed_limit
# ros2 topic info /speed_limit

# Testing with rviz2:
# ros2 run rviz2 rviz2
# Add RobotModel, TF, and PointCloud2 displays
# Subscribe to /compensated_behavioral_constraints to visualize OBBs
