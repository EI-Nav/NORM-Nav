/**
 * @file pointcloud_to_laserscan_node.cpp
 * @brief Convert 3D point cloud to 2D laser scan for navigation
 * 
 * This node subscribes to PointCloud2 messages and converts them to
 * LaserScan messages by projecting 3D points onto a 2D plane.
 * Adapted from ros-perception/pointcloud_to_laserscan for NaviBot.
 * 
 * @author Wang Junhui <wjh_9696@163.com>
 * @license MIT
 * @date 2025-10-21
 */

#include "navibot_pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace navibot_pointcloud_to_laserscan
{

// Constants for better code maintainability
namespace {
  constexpr int kDefaultTimeoutMs = 100;  ///< Default timeout for subscription listener thread
  constexpr double kDefaultTolerance = 0.01;  ///< Default transform tolerance in seconds
  constexpr double kDefaultAngleIncrement = M_PI / 180.0;  ///< Default angle increment (1 degree)
  constexpr double kDefaultScanTime = 1.0 / 30.0;  ///< Default scan time (30 Hz)
  constexpr double kDefaultInfEpsilon = 1.0;  ///< Default epsilon for infinity representation
}

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  // Declare ROS2 parameters with improved descriptions
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", kDefaultTolerance);
  
  // Set input queue size based on hardware concurrency for optimal performance
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  
  // Height filtering parameters
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  
  // Laser scan geometry parameters
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", kDefaultAngleIncrement);
  scan_time_ = this->declare_parameter("scan_time", kDefaultScanTime);
  
  // Range filtering parameters
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  
  // Infinity representation parameters
  inf_epsilon_ = this->declare_parameter("inf_epsilon", kDefaultInfEpsilon);
  use_inf_ = this->declare_parameter("use_inf", true);

  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_profile);

  using std::placeholders::_1;
  
  // Setup coordinate transformation if target frame is specified
  if (!target_frame_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Setting up coordinate transformation to frame: %s", target_frame_.c_str());
    
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    
    // Setup message filter for transform-aware processing
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloud_callback, this, _1));
  } else {  
    // Direct subscription without coordinate transformation
    RCLCPP_INFO(this->get_logger(), "Using direct point cloud subscription (no coordinate transformation)");
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloud_callback, this, _1));
  }

  // Start subscription listener thread for dynamic subscription management
  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscription_listener_thread_loop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscription_listener_thread_loop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(kDefaultTimeoutMs);
  while (rclcpp::ok(context) && alive_.load()) {
    // Check total subscription count (including intra-process subscriptions)
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
      
    if (subscription_count > 0) {
      // Start point cloud subscription if not already active
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got %d subscriber(s) to laserscan, starting pointcloud subscriber", subscription_count);
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos_profile.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      // Stop point cloud subscription when no laser scan subscribers
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    
    // Wait for graph changes with timeout
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  
  // Ensure clean shutdown
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloud_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // Create laser scan message with basic configuration
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  
  // Set target frame if coordinate transformation is enabled
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  // Configure laser scan geometry parameters
  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;  // No time increment for static conversion
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // Calculate number of laser scan rays based on angle range and increment
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // Initialize ranges with default values (infinity or max_range + epsilon)
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform point cloud to target frame if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
      RCLCPP_DEBUG(this->get_logger(), "Successfully transformed point cloud to frame: %s", 
                   target_frame_.c_str());
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), 
                         "Transform failure from " << cloud_msg->header.frame_id << 
                         " to " << target_frame_ << ": " << ex.what());
      return;
    }
  }

  // Process each point in the point cloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Skip points with invalid (NaN) coordinates
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Rejected point with NaN coordinates: (%f, %f, %f)",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    // Apply height filtering
    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Rejected point for height %f not in range [%f, %f]",
        *iter_z, min_height_, max_height_);
      continue;
    }

    // Calculate 2D range from origin
    double range = hypot(*iter_x, *iter_y);
    
    // Apply range filtering
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Rejected point for range %f below minimum %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Rejected point for range %f above maximum %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    // Calculate angle and check if within scan range
    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Rejected point for angle %f not in range [%f, %f]",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // Update laser scan range if this point is closer to the origin
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  
  // Publish the converted laser scan
  pub_->publish(std::move(scan_msg));
}

}  // namespace navibot_pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(navibot_pointcloud_to_laserscan::PointCloudToLaserScanNode)
