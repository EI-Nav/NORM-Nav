/**
 * @file pointcloud_to_laserscan_node.hpp
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

#ifndef NAVIBOT_POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
#define NAVIBOT_POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace navibot_pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> MessageFilter;

/**
 * @class PointCloudToLaserScanNode
 * @brief Convert 3D point cloud to 2D laser scan for navigation
 * 
 * This class processes incoming pointclouds into laserscans by projecting
 * 3D points onto a 2D plane. It supports height filtering, coordinate frame
 * transformation, and configurable scan parameters.
 * 
 * Adapted from ros-perception/pointcloud_to_laserscan for NaviBot navigation system.
 */
class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new PointCloudToLaserScanNode object
   * @param options Node options for ROS2 node initialization
   */
  explicit PointCloudToLaserScanNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Destroy the PointCloudToLaserScanNode object
   */
  ~PointCloudToLaserScanNode() override;

private:
  /**
   * @brief Process incoming point cloud and convert to laser scan
   * @param cloud_msg Input point cloud message
   */
  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

  /**
   * @brief Monitor subscription status and manage point cloud subscription
   * 
   * This method runs in a separate thread to dynamically subscribe/unsubscribe
   * to point cloud topics based on laser scan subscriber count.
   */
  void subscription_listener_thread_loop();

  // TF2 components for coordinate transformation
  std::unique_ptr<tf2_ros::Buffer> tf2_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  
  // ROS2 communication components
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
  std::unique_ptr<MessageFilter> message_filter_;

  // Thread management
  std::thread subscription_listener_thread_;
  std::atomic_bool alive_{true};

  // ROS Parameters
  int input_queue_size_;                    ///< Input queue size for message filtering
  std::string target_frame_;               ///< Target coordinate frame for transformation
  double tolerance_;                        ///< Transform tolerance in seconds
  double min_height_;                       ///< Minimum height filter (meters)
  double max_height_;                       ///< Maximum height filter (meters)
  double angle_min_;                        ///< Minimum scan angle (radians)
  double angle_max_;                        ///< Maximum scan angle (radians)
  double angle_increment_;                  ///< Angle increment between rays (radians)
  double scan_time_;                        ///< Scan time in seconds
  double range_min_;                        ///< Minimum range (meters)
  double range_max_;                        ///< Maximum range (meters)
  bool use_inf_;                           ///< Use infinity for unobstructed rays
  double inf_epsilon_;                      ///< Epsilon value for infinity representation
};

}  // namespace navibot_pointcloud_to_laserscan

#endif  // NAVIBOT_POINTCLOUD_TO_LASERSCAN__POINTCLOUD_TO_LASERSCAN_NODE_HPP_
