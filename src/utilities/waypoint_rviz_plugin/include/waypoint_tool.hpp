/**
 * @file waypoint_tool.hpp
 * @brief Enhanced Goal/Waypoint RViz2 tool with highlighted visualization
 * 
 * This tool provides similar functionality to Nav2 Goal tool but with enhanced
 * visualization markers to make the goal point more prominent in RViz2.
 * 
 * @author Wang Junhui <wjh_9696@163.com>
 * @license MIT
 */

#ifndef WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_HPP
#define WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_HPP

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/tool.hpp>

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
class EnumProperty;
class FloatProperty;
class ColorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace waypoint_rviz_plugin
{

/**
 * @brief Enhanced goal/waypoint tool with highlighted visualization
 * 
 * Publishes PoseStamped messages to /goal_pose (Nav2 compatible) and
 * visualization markers to make the goal more prominent in RViz2.
 */
class WaypointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   */
  WaypointTool();
  
  /**
   * @brief Destructor
   */
  ~WaypointTool() override;
  
  /**
   * @brief Initialize the tool
   */
  void onInitialize() override;

protected:
  /**
   * @brief Called when a pose is set by the user
   * @param x X coordinate in the fixed frame
   * @param y Y coordinate in the fixed frame
   * @param theta Orientation angle (yaw) in radians
   */
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  /**
   * @brief Update topic publishers when topic property changes
   */
  void updateTopic();

private:
  /**
   * @brief Create a highlighted marker for the goal visualization
   * @param x X coordinate
   * @param y Y coordinate
   * @param theta Orientation angle
   * @return Visualization marker message
   */
  visualization_msgs::msg::Marker createGoalMarker(double x, double y, double theta);

  /// Publisher for goal pose (Nav2 compatible)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  
  /// Publisher for visualization marker (highlighted display)
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  /// ROS clock for timestamping
  rclcpp::Clock::SharedPtr clock_;
  
  /// Property for configuring goal topic
  rviz_common::properties::StringProperty* topic_property_;
  
  /// Property for configuring QoS profile
  rviz_common::properties::QosProfileProperty* qos_profile_property_;

  /// QoS profile for publishers
  rclcpp::QoS qos_profile_;
  
  // Marker appearance properties
  /// Property for marker shape type
  rviz_common::properties::EnumProperty* marker_shape_property_;
  
  /// Property for marker scale X (length for arrow)
  rviz_common::properties::FloatProperty* marker_scale_x_property_;
  
  /// Property for marker scale Y (width)
  rviz_common::properties::FloatProperty* marker_scale_y_property_;
  
  /// Property for marker scale Z (height)
  rviz_common::properties::FloatProperty* marker_scale_z_property_;
  
  /// Property for marker color
  rviz_common::properties::ColorProperty* marker_color_property_;
  
  /// Property for marker alpha (transparency)
  rviz_common::properties::FloatProperty* marker_alpha_property_;
};

}  // namespace waypoint_rviz_plugin

#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_HPP
