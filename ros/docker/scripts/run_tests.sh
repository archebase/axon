#!/bin/bash
set -e

# =============================================================================
# ROS Test Runner
# =============================================================================
# This script runs tests using ROS's native test infrastructure:
#   - colcon test (ROS 2)
#   - catkin build --catkin-make-args run_tests (ROS 1)
#
# This matches how CI runs tests via industrial_ci.
# =============================================================================

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

cd /workspace/axon

# =============================================================================
# Part 1: Build and Run ROS Tests
# =============================================================================
# This is equivalent to what industrial_ci does in ros-unit-tests job
# =============================================================================
echo ""
echo "============================================"
echo "Part 1: Building and running ROS tests..."
echo "============================================"

if [ "${ROS_VERSION}" = "1" ]; then
    # ROS 1 - Use catkin build
    echo "Building with catkin build (ROS 1)..."

    cd /workspace/axon/ros
    # Clean previous build (union of ROS1 and ROS2 build artifacts)
    rm -rf build devel install log

    source /opt/ros/${ROS_DISTRO}/setup.bash
    catkin build --no-notify -DCMAKE_BUILD_TYPE=Release
    source devel/setup.bash

    echo "Running catkin tests..."
    catkin build --no-notify --catkin-make-args run_tests
    catkin_test_results --verbose
    echo "✓ ROS 1 tests passed"

else
    # ROS 2 - Use colcon
    echo "Building with colcon (ROS 2)..."
    
    cd /workspace/axon
    rm -rf build devel install log
    
    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build \
        --packages-select axon_recorder \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --base-paths ros
    
    source install/setup.bash
    
    echo "Running colcon tests..."
    colcon test \
        --packages-select axon_recorder \
        --event-handlers console_direct+ \
        --base-paths ros
    colcon test-result --verbose
    echo "✓ ROS 2 tests passed"
fi

echo ""
echo "============================================"
echo "All unit/integration tests passed!"
echo "============================================"
echo ""
echo "NOTE: E2E tests (ROS service calls) run separately via run_e2e_with_coverage.sh"
echo "      and are only executed on Humble via e2e-tests.yml workflow."
