#include "ros_interface.hpp"
#include <cstdlib>
#include <string>

namespace axon {
namespace common {

RosInterfaceFactory::RosVersion RosInterfaceFactory::detect_ros_version() {
    // Check for ROS 1
    const char* ros_distro = std::getenv("ROS_DISTRO");
    if (ros_distro) {
        // ROS 1 distros: noetic, melodic, kinetic, etc.
        std::string distro(ros_distro);
        if (distro == "noetic" || distro == "melodic" || distro == "kinetic") {
            return RosVersion::ROS1;
        }
    }
    
    // Check for ROS 2
    const char* ros_version = std::getenv("ROS_VERSION");
    if (ros_version) {
        std::string version(ros_version);
        if (version == "2") {
            return RosVersion::ROS2;
        }
    }
    
    // Default to ROS 1 for backward compatibility
    return RosVersion::ROS1;
}

// Factory implementation is in ros1_interface.cpp and ros2_interface.cpp
// This file provides the detection function only

} // namespace common
} // namespace axon
