#!/bin/bash

# Convenience script to run unified subscriber_test with ROS2 plugin
# This script sources the ROS2 environment and runs the test

set -e

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
  source /opt/ros/iron/setup.bash
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
  source /opt/ros/galactic/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
  source /opt/ros/foxy/setup.bash
else
  echo "Error: ROS2 setup.bash not found!"
  echo "Please make sure ROS2 is installed."
  exit 1
fi

# Change to build directory
cd "$(dirname "$0")/build"

# Copy ROS2 plugin if available
if [ -f "../../middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so" ]; then
  cp ../../middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so . 2>/dev/null || true
elif [ -f "../../middlewares/ros2/build/ros2_plugin/libros2_plugin.so" ]; then
  cp ../../middlewares/ros2/build/ros2_plugin/libros2_plugin.so . 2>/dev/null || true
fi

# Run with current directory in library path
LD_LIBRARY_PATH=:$LD_LIBRARY_PATH ./subscriber_test ros2 "$@"
