#include "message_factory.hpp"
#include "register_common_messages.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace axon {
namespace ros1 {

/**
 * Register common ROS message types with the message factory
 * This enables full typed subscription support for these messages
 */
void register_common_message_types() {
    // Standard messages
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
}

} // namespace ros1
} // namespace axon

