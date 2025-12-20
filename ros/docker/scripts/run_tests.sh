#!/bin/bash
set -e

# =============================================================================
# ROS Test Runner
# =============================================================================
# This script runs tests using ROS's native test infrastructure:
#   - colcon test (ROS 2)
#   - catkin_make run_tests (ROS 1)
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
    # ROS 1 - Use catkin
    echo "Building with catkin (ROS 1)..."
    
    rm -rf /workspace/catkin_ws
    mkdir -p /workspace/catkin_ws/src
    cd /workspace/catkin_ws
    ln -sf /workspace/axon/ros/axon_recorder src/axon_recorder
    
    source /opt/ros/${ROS_DISTRO}/setup.bash
    catkin_make -DCMAKE_BUILD_TYPE=Release
    source devel/setup.bash
    
    echo "Running catkin tests..."
    catkin_make run_tests
    catkin_test_results --verbose
    echo "✓ ROS 1 tests passed"
    
else
    # ROS 2 - Use colcon
    echo "Building with colcon (ROS 2)..."
    
    cd /workspace/axon
    rm -rf build install log
    
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

# =============================================================================
# Part 2: Integration Tests (End-to-end ROS service calls)
# =============================================================================
# This is equivalent to what industrial_ci does in ros-integration-tests job
# Uses the shared run_integration_tests.sh script
# =============================================================================
echo ""
echo "============================================"
echo "Part 2: Running Integration Tests..."
echo "============================================"

INTEGRATION_RUNNER="/workspace/axon/ros/axon_recorder/test/integration/run_integration_tests.sh"

if [ -f "${INTEGRATION_RUNNER}" ]; then
    chmod +x "${INTEGRATION_RUNNER}"
    "${INTEGRATION_RUNNER}"
else
    echo "Note: Integration test runner not found, skipping..."
fi

echo ""
echo "============================================"
echo "All tests passed!"
echo "============================================"
