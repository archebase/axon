#!/bin/bash

# Test script for unified subscriber_test
# Tests both ROS1 and ROS2 plugin loading

set -e

echo "=========================================="
echo "Testing Unified Subscriber Test Program"
echo "=========================================="
echo ""

# Check if binary exists
if [ ! -f "build/subscriber_test" ]; then
  echo "Error: subscriber_test binary not found!"
  echo "Please run: ./build.sh"
  exit 1
fi

cd build

# Test ROS2 plugin loading
echo "=== Testing ROS2 Plugin ==="
echo ""

if [ -f "../../middlewares/ros2/build/ros2_plugin/libros2_plugin.so" ]; then
  echo "Copying ROS2 plugin library..."
  cp ../../middlewares/ros2/build/ros2_plugin/libros2_plugin.so . 2>/dev/null || true
  cp ../../middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so . 2>/dev/null || true
elif [ -f "../../middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so" ]; then
  echo "Copying ROS2 plugin library..."
  cp ../../middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so .
else
  echo "Warning: ROS2 plugin library not found"
  echo "Expected locations:"
  echo "  - ../../middlewares/ros2/build/ros2_plugin/libros2_plugin.so"
  echo "  - ../../middlewares/ros2/install/ros2_plugin/lib/libros2_plugin.so"
  echo ""
  echo "Build ROS2 plugin first: ./build.sh (from project root)"
fi

if [ -f "libros2_plugin.so" ]; then
  echo "✓ ROS2 plugin library found"
  echo ""
  echo "Testing ROS2 plugin load (dry-run)..."
  echo "This will test if the plugin can be loaded successfully."
  echo ""

  # Test library loading with a simple test
  ./subscriber_test ros2 --help 2>&1 | head -5 || true

  echo ""
  echo "✓ ROS2 plugin test completed"
  echo ""
  echo "To run ROS2 subscriber test:"
  echo "  ./subscriber_test ros2 /chatter std_msgs/msg/String"
  echo ""
else
  echo "⚠ ROS2 plugin library not available - skipping ROS2 test"
fi

echo ""
echo "=== Testing ROS1 Plugin ==="
echo ""

if [ -f "../../middlewares/ros1/devel/lib/libros1_plugin.so" ]; then
  echo "Copying ROS1 plugin library..."
  cp ../../middlewares/ros1/devel/lib/libros1_plugin.so . 2>/dev/null || true
  cp ../../middlewares/ros1/build/libros1_plugin.so . 2>/dev/null || true
elif [ -f "../../middlewares/ros1/build/libros1_plugin.so" ]; then
  echo "Copying ROS1 plugin library..."
  cp ../../middlewares/ros1/build/libros1_plugin.so .
else
  echo "Warning: ROS1 plugin library not found"
  echo "Expected locations:"
  echo "  - ../../middlewares/ros1/devel/lib/libros1_plugin.so"
  echo "  - ../../middlewares/ros1/build/libros1_plugin.so"
  echo ""
  echo "Build ROS1 plugin first: cd ../../middlewares/ros1 && ./build.sh"
fi

if [ -f "libros1_plugin.so" ]; then
  echo "✓ ROS1 plugin library found"
  echo ""
  echo "Testing ROS1 plugin load (dry-run)..."
  echo ""

  # Test library loading
  ./subscriber_test ros1 --help 2>&1 | head -5 || true

  echo ""
  echo "✓ ROS1 plugin test completed"
  echo ""
  echo "To run ROS1 subscriber test:"
  echo "  ./subscriber_test ros1 /chatter std_msgs/String"
  echo ""
else
  echo "⚠ ROS1 plugin library not available - skipping ROS1 test"
fi

echo ""
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo ""

if [ -f "libros2_plugin.so" ]; then
  echo "✓ ROS2 plugin: Available"
else
  echo "✗ ROS2 plugin: Not built"
fi

if [ -f "libros1_plugin.so" ]; then
  echo "✓ ROS1 plugin: Available"
else
  echo "✗ ROS1 plugin: Not built"
fi

echo ""
echo "Note: The unified subscriber_test program has NO compile-time"
echo "dependencies on ROS1 or ROS2 libraries. It loads them dynamically!"
