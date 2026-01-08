#!/bin/bash

# Build script for unified subscriber_test
# This builds the test program that works with BOTH ROS1 and ROS2 plugins
# WITHOUT requiring ROS1 or ROS2 libraries at compile time

set -e

echo "=========================================="
echo "Building Unified Subscriber Test"
echo "=========================================="
echo ""

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake ..

# Build
echo "Building..."
make

echo ""
echo "=========================================="
echo "Build completed successfully!"
echo "=========================================="
echo ""
echo "The unified subscriber_test binary can now work with both ROS1 and ROS2 plugins."
echo ""
echo "Usage:"
echo "  ./subscriber_test ros1 [topic_name] [message_type]"
echo "  ./subscriber_test ros2 [topic_name] [message_type]"
echo ""
echo "Examples:"
echo "  ./subscriber_test ros2"
echo "  ./subscriber_test ros2 /chatter std_msgs/msg/String"
echo "  ./subscriber_test ros1 /chatter std_msgs/String"
echo ""
echo "IMPORTANT: Make sure to build the ROS plugins first:"
echo "  - For ROS2: ./build.sh (from project root)"
echo "  - For ROS1: cd ros1 && ./build.sh"
echo ""
