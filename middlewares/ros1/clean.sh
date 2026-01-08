#!/bin/bash
# Clean script for ROS1

echo "Cleaning ROS1 build artifacts..."

cd ros1

# Remove catkin build artifacts
rm -rf build/
rm -rf devel/
rm -rf *.pcdump

# Remove example build artifacts
rm -rf examples/ros1/build/
rm -rf examples/ros1/libros1_plugin.so
rm -rf examples/ros1/subscriber_test

echo "Clean completed."
