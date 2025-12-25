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

# Build C FFI library (Rust)
echo ""
echo "============================================"
echo "Building C FFI library (Rust)..."
echo "============================================"
if [ -f "c/Cargo.toml" ]; then
    cd c
    cargo build
    cd /workspace/axon
    echo "✓ C FFI library built"
else
    echo "⚠ C FFI library not found (Cargo.toml missing), skipping..."
fi

# Build C++ code
echo ""
echo "============================================"
echo "Building C++ code..."
echo "============================================"
if [ "${ROS_VERSION}" = "2" ]; then
    # ROS2: use colcon
    colcon build --base-paths . \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=${ROS_VERSION} \
        --event-handlers console_direct+ \
        --executor parallel
else
    # ROS1: use cmake
    rm -rf build
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=${ROS_VERSION}
    cmake --build . -j$(nproc)
    cd /workspace/axon
fi
echo "✓ C++ code built"

echo ""
echo "============================================"
echo "Build complete!"
echo "============================================"
