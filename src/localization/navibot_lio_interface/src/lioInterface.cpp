#/**
# * @file lioInterface.cpp
# * @brief Bridge FAST-LIO/LOAM outputs to ROS 2 topics and TF
# *
# * This node subscribes to a state estimation (odometry) topic and a registered
# * point cloud topic, optionally applies axis flipping to match coordinate
# * conventions, republishes the point cloud, publishes an Odometry message with
# * configurable frame ids, and broadcasts TF between odom and base_link using a
# * cached static transform from cloud frame (e.g., imu_link) to base_link.
# *
# * @author Wang Junhui <wjh_9696@163.com>
# * @license MIT
# * @date 2025-10-30
# */
#
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// TF buffer and listener for querying static transform imu_link->base_link
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

class LoamInterface : public rclcpp::Node
{
public:
  LoamInterface()
  : Node("loamInterface"), tfBuffer(this->get_clock()), tfListener(tfBuffer)
  {
    // Declare Parameters (keep backward-compatible defaults)
    this->declare_parameter<std::string>("stateEstimationTopic", stateEstimationTopic);
    this->declare_parameter<std::string>("registeredScanTopic", registeredScanTopic);
    this->declare_parameter<bool>("flipStateEstimation", flipStateEstimation);
    this->declare_parameter<bool>("flipRegisteredScan", flipRegisteredScan);
    this->declare_parameter<bool>("sendTF", sendTF);
    this->declare_parameter<bool>("reverseTF", reverseTF);
    // New: parameterize frames and output topics
    this->declare_parameter<std::string>("cloud_frame_id", cloud_frame_id);
    this->declare_parameter<std::string>("odom_frame_id", odom_frame_id);
    this->declare_parameter<std::string>("base_link_frame_id", base_link_frame_id);
    this->declare_parameter<std::string>("pub_cloud_topic", pub_cloud_topic);
    this->declare_parameter<std::string>("pub_odom_topic", pub_odom_topic);

    // Initialize Parameters
    this->get_parameter("stateEstimationTopic", stateEstimationTopic);
    this->get_parameter("registeredScanTopic", registeredScanTopic);
    this->get_parameter("flipStateEstimation", flipStateEstimation);
    this->get_parameter("flipRegisteredScan", flipRegisteredScan);
    this->get_parameter("sendTF", sendTF);
    this->get_parameter("reverseTF", reverseTF);
    this->get_parameter("cloud_frame_id", cloud_frame_id);
    this->get_parameter("odom_frame_id", odom_frame_id);
    this->get_parameter("base_link_frame_id", base_link_frame_id);
    this->get_parameter("pub_cloud_topic", pub_cloud_topic);
    this->get_parameter("pub_odom_topic", pub_odom_topic);

    tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Use parameterized output topics
    pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_cloud_topic, 5);
    pubOdometry = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 5);

    // Allow live update of boolean switches
    on_set_param_cb_handle = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "flipStateEstimation") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) { result.successful = false; result.reason = "flipStateEstimation must be bool"; break; }
            flipStateEstimation = p.as_bool();
          } else if (p.get_name() == "flipRegisteredScan") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) { result.successful = false; result.reason = "flipRegisteredScan must be bool"; break; }
            flipRegisteredScan = p.as_bool();
          } else if (p.get_name() == "sendTF") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) { result.successful = false; result.reason = "sendTF must be bool"; break; }
            sendTF = p.as_bool();
          } else if (p.get_name() == "reverseTF") {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) { result.successful = false; result.reason = "reverseTF must be bool"; break; }
            reverseTF = p.as_bool();
          }
        }
        return result;
      }
    );

    // Initialize members
    laserCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      registeredScanTopic, rclcpp::SensorDataQoS(), std::bind(&LoamInterface::laserCloudHandler, this, std::placeholders::_1));
    subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      stateEstimationTopic, rclcpp::SensorDataQoS(), std::bind(&LoamInterface::odometryHandler, this, std::placeholders::_1));

  }

private:

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn)
  {
    // Convert incoming cloud into PCL structure
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);

    if (flipRegisteredScan) {
      // Flip axes when upstream registration uses a different convention
      const auto n = laserCloud->points.size();
      for (size_t i = 0; i < n; ++i) {
        std::swap(laserCloud->points[i].x, laserCloud->points[i].z);
        std::swap(laserCloud->points[i].z, laserCloud->points[i].y);
      }
    }

    // Publish registered scan
    sensor_msgs::msg::PointCloud2 laserCloud2;
    pcl::toROSMsg(*laserCloud, laserCloud2);
    laserCloud2.header.stamp = laserCloudIn->header.stamp;
    laserCloud2.header.frame_id = cloud_frame_id;
    pubLaserCloud->publish(laserCloud2);
  }

  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    odomData = *odom;

    if (flipStateEstimation) {
      tf2::Matrix3x3(tf2::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

      pitch = -pitch;
      yaw = -yaw;

      tf2::Quaternion quat_tf;
      quat_tf.setRPY(roll, pitch, yaw);
      tf2::convert(quat_tf, geoQuat);

      odomData.pose.pose.orientation = geoQuat;
      odomData.pose.pose.position.x = odom->pose.pose.position.z;
      odomData.pose.pose.position.y = odom->pose.pose.position.x;
      odomData.pose.pose.position.z = odom->pose.pose.position.y;
    }

    // Goal: publish odom->base_link, with child_frame_id set to base_link.
    // Build T_oi (odom->cloud_frame, typically imu_link), then multiply
    // with static T_ib (cloud_frame->base_link) to obtain T_ob.
    tf2::Transform T_oi;
    T_oi.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    T_oi.setOrigin(tf2::Vector3(
      odomData.pose.pose.position.x,
      odomData.pose.pose.position.y,
      odomData.pose.pose.position.z));

    // Lazily look up and cache the static transform cloud_frame->base_link
    if (!has_T_ib_cached) {
      try {
        if (tfBuffer.canTransform(cloud_frame_id, base_link_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
          auto T_ib_msg = tfBuffer.lookupTransform(cloud_frame_id, base_link_frame_id, rclcpp::Time(0));
          const auto & t = T_ib_msg.transform.translation;
          const auto & q = T_ib_msg.transform.rotation;
          T_ib_cached.setOrigin(tf2::Vector3(t.x, t.y, t.z));
          T_ib_cached.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
          has_T_ib_cached = true;
          if (!has_logged_T_ib_ready) {
            RCLCPP_INFO(this->get_logger(), "Cached static TF %s -> %s", cloud_frame_id.c_str(), base_link_frame_id.c_str());
            has_logged_T_ib_ready = true;
          }
        }
      } catch (const tf2::TransformException &ex) {
        auto clock = this->get_clock();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 2000, "TF %s->%s lookup failed: %s", cloud_frame_id.c_str(), base_link_frame_id.c_str(), ex.what());
      }
    }

    bool got_static_tf = has_T_ib_cached;
    tf2::Transform T_ob;
    if (got_static_tf) {
      T_ob = T_oi * T_ib_cached;  // odom->base_link
    }

    // Publish odometry: frame_id = odom, child_frame_id = base_link
    odomData.header.frame_id = odom_frame_id;
    odomData.child_frame_id = base_link_frame_id;
    if (got_static_tf) {
      // Overwrite pose using T_ob so that message is self-consistent with TF
      const tf2::Vector3 & p = T_ob.getOrigin();
      const tf2::Quaternion & q = T_ob.getRotation();
      odomData.pose.pose.position.x = p.x();
      odomData.pose.pose.position.y = p.y();
      odomData.pose.pose.position.z = p.z();
      odomData.pose.pose.orientation.x = q.x();
      odomData.pose.pose.orientation.y = q.y();
      odomData.pose.pose.orientation.z = q.z();
      odomData.pose.pose.orientation.w = q.w();
    }
    pubOdometry->publish(odomData);

    // Publish TF: odom->base_link or its inverse
    if (sendTF && got_static_tf) {
      if (!reverseTF) {
        transformTfGeom.transform = tf2::toMsg(T_ob);
        transformTfGeom.header.frame_id = odom_frame_id;
        transformTfGeom.child_frame_id = base_link_frame_id;
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      } else {
        auto T_bo = T_ob.inverse();
        transformTfGeom.transform = tf2::toMsg(T_bo);
        transformTfGeom.header.frame_id = base_link_frame_id;
        transformTfGeom.child_frame_id = odom_frame_id;
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;
  // TF buffer and listener for TF queries
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  // Cached static transform (cloud_frame->base_link)
  tf2::Transform T_ib_cached;
  bool has_T_ib_cached = false;
  bool has_logged_T_ib_ready = false;

  // Former globals folded into this scope (member-like locals).
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
  nav_msgs::msg::Odometry odomData;
  geometry_msgs::msg::TransformStamped transformTfGeom;

  const double PI = 3.1415926;

  string stateEstimationTopic = "/integrated_to_init";
  string registeredScanTopic = "/velodyne_cloud_registered";
  bool flipStateEstimation = true;
  bool flipRegisteredScan = true;
  bool sendTF = true;
  bool reverseTF = false;

  // New: parameterized frames and output topics (defaults preserve current behavior)
  string cloud_frame_id = "imu_link";
  string odom_frame_id = "odom";
  string base_link_frame_id = "base_link";
  string pub_cloud_topic = "/cloud";
  string pub_odom_topic = "/state_estimation";

  // Allow live update of boolean switches via parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_cb_handle;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoamInterface>());
  rclcpp::shutdown();
  
  return 0;
}