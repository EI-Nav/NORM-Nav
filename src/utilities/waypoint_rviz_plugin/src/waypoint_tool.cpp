/**
 * @file waypoint_tool.cpp
 * @brief Implementation of enhanced Goal/Waypoint RViz2 tool
 * 
 * @author Wang Junhui <wjh_9696@163.com>
 * @license MIT
 */

#include <waypoint_tool.hpp>

#include <string>
#include <cmath>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <QColor>

namespace waypoint_rviz_plugin
{

WaypointTool::WaypointTool()
: rviz_default_plugins::tools::PoseTool(), 
  qos_profile_(5)
{
  // Set shortcut key to 'g' (similar to Nav2 Goal)
  shortcut_key_ = 'g';

  // Create topic property for goal pose
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", 
    "goal_pose", 
    "The topic on which to publish navigation goals.",
    getPropertyContainer(), 
    SLOT(updateTopic()), 
    this);
  
  // Create QoS profile property
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, 
    qos_profile_);
  
  // Create marker shape property
  marker_shape_property_ = new rviz_common::properties::EnumProperty(
    "Marker Shape",
    "Arrow",
    "Shape of the goal marker visualization.",
    getPropertyContainer(),
    nullptr,
    this);
  marker_shape_property_->addOption("Arrow", visualization_msgs::msg::Marker::ARROW);
  marker_shape_property_->addOption("Cylinder", visualization_msgs::msg::Marker::CYLINDER);
  marker_shape_property_->addOption("Sphere", visualization_msgs::msg::Marker::SPHERE);
  marker_shape_property_->addOption("Cube", visualization_msgs::msg::Marker::CUBE);
  
  // Create marker scale properties
  marker_scale_x_property_ = new rviz_common::properties::FloatProperty(
    "Scale X",
    1.5,
    "Marker scale in X direction (length for arrow).",
    getPropertyContainer(),
    nullptr,
    this);
  marker_scale_x_property_->setMin(0.1);
  marker_scale_x_property_->setMax(10.0);
  
  marker_scale_y_property_ = new rviz_common::properties::FloatProperty(
    "Scale Y",
    0.3,
    "Marker scale in Y direction (width).",
    getPropertyContainer(),
    nullptr,
    this);
  marker_scale_y_property_->setMin(0.1);
  marker_scale_y_property_->setMax(10.0);
  
  marker_scale_z_property_ = new rviz_common::properties::FloatProperty(
    "Scale Z",
    0.3,
    "Marker scale in Z direction (height).",
    getPropertyContainer(),
    nullptr,
    this);
  marker_scale_z_property_->setMin(0.1);
  marker_scale_z_property_->setMax(10.0);
  
  // Create marker color property (default: cyan)
  marker_color_property_ = new rviz_common::properties::ColorProperty(
    "Marker Color",
    QColor(0, 255, 255),
    "Color of the goal marker.",
    getPropertyContainer(),
    nullptr,
    this);
  
  // Create marker alpha property
  marker_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Marker Alpha",
    1.0,
    "Transparency of the goal marker (0.0 = transparent, 1.0 = opaque).",
    getPropertyContainer(),
    nullptr,
    this);
  marker_alpha_property_->setMin(0.0);
  marker_alpha_property_->setMax(1.0);
}

WaypointTool::~WaypointTool() = default;

void WaypointTool::onInitialize()
{
  // Initialize base class
  rviz_default_plugins::tools::PoseTool::onInitialize();
  
  // Initialize QoS profile property
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->qos_profile_ = profile;
    });
  
  // Set tool name
  setName("Navigation Goal");
  
  // Create publishers
  updateTopic();
}

void WaypointTool::updateTopic()
{
  // Get ROS node
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  
  // Create goal pose publisher (Nav2 compatible)
  goal_pub_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), 
    qos_profile_);
  
  // Create marker publisher for highlighted visualization
  marker_pub_ = raw_node->create_publisher<visualization_msgs::msg::Marker>(
    topic_property_->getStdString() + "_marker", 
    qos_profile_);
  
  // Get clock for timestamping
  clock_ = raw_node->get_clock();
  
  RCLCPP_INFO(
    raw_node->get_logger(), 
    "Enhanced Goal Tool publishing to: %s", 
    topic_property_->getStdString().c_str());
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  // Get current timestamp
  auto stamp = clock_->now();
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  
  // Create and publish goal pose message (Nav2 compatible)
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.frame_id = fixed_frame;
  goal_msg.header.stamp = stamp;
  goal_msg.pose.position.x = x;
  goal_msg.pose.position.y = y;
  goal_msg.pose.position.z = 0.0;
  
  // Convert theta to quaternion
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  goal_msg.pose.orientation = tf2::toMsg(quat);
  
  // Publish goal pose
  goal_pub_->publish(goal_msg);
  
  // First, delete all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = fixed_frame;
  delete_marker.header.stamp = stamp;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_pub_->publish(delete_marker);
  
  // Create and publish highlighted marker for new goal
  auto marker = createGoalMarker(x, y, theta);
  marker.header.frame_id = fixed_frame;
  marker.header.stamp = stamp;
  marker_pub_->publish(marker);
  
  // Log goal publication
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  RCLCPP_INFO(
    raw_node->get_logger(),
    "Published goal: [%.2f, %.2f, %.2f rad]", 
    x, y, theta);
}

visualization_msgs::msg::Marker WaypointTool::createGoalMarker(
  double x, 
  double y, 
  double theta)
{
  visualization_msgs::msg::Marker marker;
  
  // Basic marker properties - use fixed ID so new goals replace old ones
  marker.id = 0;
  
  // Get marker type from property
  marker.type = static_cast<uint8_t>(marker_shape_property_->getOptionInt());
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent marker
  
  // Set position
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  
  // Set orientation from theta
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  marker.pose.orientation = tf2::toMsg(quat);
  
  // Get scale from properties
  marker.scale.x = marker_scale_x_property_->getFloat();
  marker.scale.y = marker_scale_y_property_->getFloat();
  marker.scale.z = marker_scale_z_property_->getFloat();
  
  // Get color from property
  QColor color = marker_color_property_->getColor();
  marker.color.r = color.redF();
  marker.color.g = color.greenF();
  marker.color.b = color.blueF();
  
  // Get alpha from property
  marker.color.a = marker_alpha_property_->getFloat();
  
  return marker;
}

}  // namespace waypoint_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::WaypointTool, rviz_common::Tool)
