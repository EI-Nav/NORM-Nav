# CMakeLists.txt for navibot_pointcloud_to_laserscan package
# Point cloud to laser scan conversion for NaviBot navigation system

cmake_minimum_required(VERSION 3.5)
project(navibot_pointcloud_to_laserscan)

# Compiler options for modern C++ and optimization
add_compile_options(-std=c++17 -O3 -Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create shared library for point cloud to laser scan conversion
ament_auto_add_library(navibot_pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

# Register as ROS2 component for dynamic loading
rclcpp_components_register_node(navibot_pointcloud_to_laserscan
  PLUGIN "navibot_pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

# Setup testing if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install package resources
ament_auto_package(
  INSTALL_TO_SHARE
    launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
