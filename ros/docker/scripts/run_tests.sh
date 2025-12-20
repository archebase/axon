#!/bin/bash
set -e

# Run tests script for ROS Docker containers
# This script runs only ROS/C++ tests - Rust tests are run separately

echo "============================================"
echo "Running ROS tests for ROS ${ROS_VERSION} (${ROS_DISTRO})"
echo "============================================"

# Source ROS environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "Sourced ROS ${ROS_DISTRO} environment"
else
    echo "ERROR: ROS setup.bash not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

# Navigate to workspace
cd /workspace/axon

# Build and run C++ tests
echo ""
echo "============================================"
echo "Building and running C++ tests..."
echo "============================================"
# Clean and recreate test build directory to avoid CMake cache conflicts
rm -rf ros/axon_recorder/test/build
mkdir -p ros/axon_recorder/test/build
cd ros/axon_recorder/test/build
echo "Running CMake from: $(pwd)"
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
ctest --output-on-failure
cd /workspace/axon
echo "✓ C++ tests passed"

echo ""
echo "============================================"
echo "All ROS tests passed!"
echo "============================================"
