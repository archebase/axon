#!/bin/bash
set -e

# Run tests script for Docker containers
# This script is executed inside the Docker container to run all tests

echo "============================================"
echo "Running tests for ROS ${ROS_VERSION} (${ROS_DISTRO})"
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

# Run Rust tests
echo ""
echo "============================================"
echo "Running Rust tests..."
echo "============================================"
cd src/bridge
cargo test
cd /workspace/axon
echo "✓ Rust tests passed"

# Build and run C++ tests
echo ""
echo "============================================"
echo "Building and running C++ tests..."
echo "============================================"
# Clean and recreate test build directory to avoid CMake cache conflicts
rm -rf test/build
mkdir -p test/build
cd test/build
cmake ..
cmake --build . -j$(nproc)
ctest --output-on-failure
cd /workspace/axon
echo "✓ C++ tests passed"

# Run integration tests if they exist
if [ -f "test/integration/test_rust_bridge.sh" ]; then
    echo ""
    echo "============================================"
    echo "Running integration tests..."
    echo "============================================"
    bash test/integration/test_rust_bridge.sh
    echo "✓ Integration tests passed"
fi

echo ""
echo "============================================"
echo "All tests passed!"
echo "============================================"
