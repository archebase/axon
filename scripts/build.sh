#!/bin/bash

# Axon Build Script
# This script builds the ROS2 plugin and unified test program

set -e  # Exit on error

echo "========================================"
echo "  Axon ROS2 Plugin Build Script"
echo "========================================"
echo ""

# Source ROS2 environment
echo "[1/3] Sourcing ROS2 environment..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble environment sourced"
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
    echo "✅ ROS2 Iron environment sourced"
else
    echo "❌ Error: ROS2 not found"
    exit 1
fi
echo ""

# Build ROS2 plugin in workspace
echo "[2/3] Building ROS2 plugin in middlewares/ros2..."
cd middlewares/ros2
colcon build --packages-select ros2_plugin
echo "✅ Plugin built successfully"
echo ""

cd ../..

# Build unified test program
echo "[3/3] Building unified test program..."
cd examples
./build.sh
cd ..
echo "✅ Unified test program built successfully"
echo ""

echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo ""
echo "Plugin library:  middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so"
echo "Test binary:     examples/build/subscriber_test"
echo ""
echo "To run the unified test:"
echo "  make run-ros2"
echo ""
echo "Or manually:"
echo "  cd examples"
echo "  ./run_ros2.sh /chatter std_msgs/msg/String"
echo ""
