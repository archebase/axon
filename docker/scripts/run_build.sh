#!/bin/bash
set -e

# Build script for Docker containers
# This script is executed inside the Docker container to build the project

echo "============================================"
echo "Building for ROS ${ROS_VERSION} (${ROS_DISTRO})"
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

# Build Rust library
echo ""
echo "============================================"
echo "Building Rust bridge library..."
echo "============================================"
cd src/bridge
cargo build
cd /workspace/axon
echo "✓ Rust library built"

# Build C++ code
echo ""
echo "============================================"
echo "Building C++ code..."
echo "============================================"
# Clean build directory to avoid CMake cache conflicts between ROS versions
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=${ROS_VERSION}
cmake --build . -j$(nproc)
cd /workspace/axon
echo "✓ C++ code built"

echo ""
echo "============================================"
echo "Build complete!"
echo "============================================"
