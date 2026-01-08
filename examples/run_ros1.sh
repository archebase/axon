#!/bin/bash

# Convenience script to run unified subscriber_test with ROS1 plugin
# This script sources the ROS1 environment and runs the test

set -e

# Source ROS1 environment
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
  source /opt/ros/melodic/setup.bash
elif [ -f "/opt/ros/kinetic/setup.bash" ]; then
  source /opt/ros/kinetic/setup.bash
else
  echo "Error: ROS1 setup.bash not found!"
  echo "Please make sure ROS1 is installed."
  exit 1
fi

# Change to build directory
cd "$(dirname "$0")/build"

# Copy ROS1 plugin if available
if [ -f "../../middlewares/ros1/devel/lib/libros1_plugin.so" ]; then
  cp ../../middlewares/ros1/devel/lib/libros1_plugin.so . 2>/dev/null || true
elif [ -f "../../middlewares/ros1/build/libros1_plugin.so" ]; then
  cp ../../middlewares/ros1/build/libros1_plugin.so . 2>/dev/null || true
fi

# Run with current directory in library path
LD_LIBRARY_PATH=:$LD_LIBRARY_PATH ./subscriber_test ros1 "$@"
