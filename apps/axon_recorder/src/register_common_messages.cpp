#include "register_common_messages.hpp"

#include "message_factory.hpp"

#if defined(AXON_ROS1)
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#elif defined(AXON_ROS2)
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#endif

namespace axon {
namespace recorder {

/**
 * Register common ROS message types with the message factory
 * This enables full typed subscription support for these messages
 */
void register_common_message_types() {
#if defined(AXON_ROS1)
  // Standard messages (ROS 1 format: package/MessageName)
  MessageFactory::register_message_type<std_msgs::String>("std_msgs/String");
  MessageFactory::register_message_type<std_msgs::Int32>("std_msgs/Int32");
  MessageFactory::register_message_type<std_msgs::Float64>("std_msgs/Float64");
  MessageFactory::register_message_type<std_msgs::Bool>("std_msgs/Bool");
  MessageFactory::register_message_type<std_msgs::Header>("std_msgs/Header");

  // Sensor messages
  MessageFactory::register_message_type<sensor_msgs::Image>("sensor_msgs/Image");
  MessageFactory::register_message_type<sensor_msgs::PointCloud2>("sensor_msgs/PointCloud2");

  // Navigation messages
  MessageFactory::register_message_type<nav_msgs::Odometry>("nav_msgs/Odometry");

  // Geometry messages
  MessageFactory::register_message_type<geometry_msgs::Pose>("geometry_msgs/Pose");
  MessageFactory::register_message_type<geometry_msgs::Twist>("geometry_msgs/Twist");

#elif defined(AXON_ROS2)
  // Standard messages (ROS 2 format: package/msg/MessageName)
  MessageFactory::register_message_type<std_msgs::msg::String>("std_msgs/msg/String");
  MessageFactory::register_message_type<std_msgs::msg::Int32>("std_msgs/msg/Int32");
  MessageFactory::register_message_type<std_msgs::msg::Float64>("std_msgs/msg/Float64");
  MessageFactory::register_message_type<std_msgs::msg::Bool>("std_msgs/msg/Bool");
  MessageFactory::register_message_type<std_msgs::msg::Header>("std_msgs/msg/Header");

  // Sensor messages
  MessageFactory::register_message_type<sensor_msgs::msg::Image>("sensor_msgs/msg/Image");
  MessageFactory::register_message_type<sensor_msgs::msg::PointCloud2>("sensor_msgs/msg/PointCloud2"
  );

  // Navigation messages
  MessageFactory::register_message_type<nav_msgs::msg::Odometry>("nav_msgs/msg/Odometry");

  // Geometry messages
  MessageFactory::register_message_type<geometry_msgs::msg::Pose>("geometry_msgs/msg/Pose");
  MessageFactory::register_message_type<geometry_msgs::msg::Twist>("geometry_msgs/msg/Twist");

  // Also register with ROS 1 style names for config compatibility
  // This allows configs to use "sensor_msgs/Image" instead of "sensor_msgs/msg/Image"
  MessageFactory::register_message_type<std_msgs::msg::String>("std_msgs/String");
  MessageFactory::register_message_type<std_msgs::msg::Int32>("std_msgs/Int32");
  MessageFactory::register_message_type<std_msgs::msg::Float64>("std_msgs/Float64");
  MessageFactory::register_message_type<std_msgs::msg::Bool>("std_msgs/Bool");
  MessageFactory::register_message_type<std_msgs::msg::Header>("std_msgs/Header");
  MessageFactory::register_message_type<sensor_msgs::msg::Image>("sensor_msgs/Image");
  MessageFactory::register_message_type<sensor_msgs::msg::PointCloud2>("sensor_msgs/PointCloud2");
  MessageFactory::register_message_type<nav_msgs::msg::Odometry>("nav_msgs/Odometry");
  MessageFactory::register_message_type<geometry_msgs::msg::Pose>("geometry_msgs/Pose");
  MessageFactory::register_message_type<geometry_msgs::msg::Twist>("geometry_msgs/Twist");
#endif
}

}  // namespace recorder
}  // namespace axon
