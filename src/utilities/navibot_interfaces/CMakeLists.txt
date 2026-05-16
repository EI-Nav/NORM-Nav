# ==============================================================================
# CMake Configuration for navibot_interfaces Package
# ==============================================================================
#
# @file CMakeLists.txt
# @brief Build configuration for custom ROS2 interfaces in NaviBot navigation system
#
# This package defines custom message types, service definitions, and action
# interfaces used across the NaviBot navigation system, including:
#   - Messages: GroundedSam (perception results), BehavioralConstraint (behavioral constraints)
#   - Services: ParseInstructions (behavioral instruction parsing)
#   - Actions: Navigation tasks, exploration, and delivery operations
#
# @author Wang Junhui <wjh_9696@163.com>
# @license MIT
# ==============================================================================

cmake_minimum_required(VERSION 3.16)
project(navibot_interfaces)

# ==============================================================================
# Compiler Configuration
# ==============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# Dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ==============================================================================
# Interface Generation
# ==============================================================================
# Generate custom interfaces for messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
    # Message definitions
    "msg/GroundedSam.msg"
    "msg/OBBInfo2D.msg"
    "msg/OBBInfo2DArray.msg"
    "msg/BehavioralConstraint.msg"
    "msg/BehavioralConstraintArray.msg"
    "msg/BehavioralInstruction.msg"
    
    # Service definitions
    "srv/ParseInstructions.srv"
    
    # Dependencies for generated interfaces
    DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

# ==============================================================================
# Export Configuration
# ==============================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
