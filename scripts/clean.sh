#!/bin/bash

# Axon Clean Script
# This script removes all build artifacts

echo "========================================"
echo "  Axon Clean Script"
echo "========================================"
echo ""

echo "Removing ROS2 workspace build artifacts..."
cd middlewares/ros2
rm -rf build install log
echo "✅ Cleaned middlewares/ros2/"
echo ""

cd ../..

echo "Removing ROS1 workspace build artifacts..."
cd middlewares/ros1
rm -rf build devel
echo "✅ Cleaned middlewares/ros1/"
echo ""

cd ../..

echo "Removing example build artifacts..."
cd examples
rm -rf build
echo "✅ Cleaned examples/"
echo ""

cd ..

echo "Removing root build artifacts (if any)..."
rm -rf build install log
echo "✅ Cleaned project root"
echo ""

echo "========================================"
echo "  Clean Complete!"
echo "========================================"
echo ""
echo "All build artifacts have been removed."
echo "Run 'make build' or './build.sh' to rebuild."
echo ""
