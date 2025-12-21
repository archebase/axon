#!/bin/bash
# ============================================================================
# Integration Test Runner
# ============================================================================
# This script starts the axon_recorder_node, runs integration tests, and
# cleans up. It's used by both CI and local Docker testing.
#
# Usage:
#   ./run_integration_tests.sh [source_path]
#
# Arguments:
#   source_path - Optional path to the source directory (default: script's location)
#
# Environment:
#   ROS_VERSION - Must be set to "1" or "2"
#   ROS_DISTRO  - Must be set (e.g., "noetic", "humble")
#
# Prerequisites:
#   - ROS workspace must be sourced before calling this script
#   - axon_recorder package must be built and installed
# ============================================================================

set -eo pipefail

# Determine script directory and source path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_PATH="${1:-$SCRIPT_DIR}"

# Auto-detect ROS version from ROS_DISTRO if ROS_VERSION not set
if [ -z "$ROS_VERSION" ]; then
    if [ -n "$ROS_DISTRO" ]; then
        case "$ROS_DISTRO" in
            noetic|melodic|kinetic)
                ROS_VERSION=1
                ;;
            *)
                ROS_VERSION=2
                ;;
        esac
        echo "Auto-detected ROS_VERSION=$ROS_VERSION from ROS_DISTRO=$ROS_DISTRO"
    else
        echo "ERROR: Neither ROS_VERSION nor ROS_DISTRO is set"
        exit 1
    fi
fi

echo "============================================"
echo "Running Integration Tests (ROS $ROS_VERSION)"
echo "============================================"

# Source ROS workspace if not already sourced
# This handles both CI (/root/target_ws) and local Docker (/workspace) paths
set +eu
if [ -n "$ROS_DISTRO" ]; then
    . /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true
fi
for ws in /root/target_ws /workspace/axon /workspace/catkin_ws; do
    if [ -f "$ws/install/setup.bash" ]; then
        echo "Sourcing $ws/install/setup.bash"
        . "$ws/install/setup.bash"
        # For ROS 2: Ensure workspace is in AMENT_PREFIX_PATH
        if [ "$ROS_VERSION" = "2" ]; then
            export AMENT_PREFIX_PATH="$ws/install/axon_recorder:$ws/install:$AMENT_PREFIX_PATH"
        fi
        break
    elif [ -f "$ws/devel/setup.bash" ]; then
        echo "Sourcing $ws/devel/setup.bash"
        . "$ws/devel/setup.bash"
        break
    fi
done
set -e

# Debug: Show what's installed
echo "Checking installed packages..."
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
ls -la /root/target_ws/install/ 2>/dev/null | head -10 || echo "No /root/target_ws/install/"
ls -la /root/target_ws/install/axon_recorder/ 2>/dev/null | head -5 || echo "No axon_recorder install dir"

# Verify package is available
echo "Checking if axon_recorder is available..."
if [ "$ROS_VERSION" = "2" ]; then
    echo "ros2 pkg list output:"
    ros2 pkg list 2>&1 | head -20
    ros2 pkg list 2>/dev/null | grep -q axon_recorder || echo "WARNING: axon_recorder not found in ros2 pkg list"
else
    rospack find axon_recorder 2>/dev/null || echo "WARNING: axon_recorder not found via rospack"
fi

# Path to the test script
TEST_SCRIPT="$SOURCE_PATH/test_ros_services.sh"
if [ ! -f "$TEST_SCRIPT" ]; then
    # Try relative to script directory
    TEST_SCRIPT="$SCRIPT_DIR/test_ros_services.sh"
fi

if [ ! -f "$TEST_SCRIPT" ]; then
    echo "ERROR: test_ros_services.sh not found"
    echo "Searched: $SOURCE_PATH/test_ros_services.sh"
    echo "Searched: $SCRIPT_DIR/test_ros_services.sh"
    exit 1
fi

chmod +x "$TEST_SCRIPT"

# Start node based on ROS version
echo "Starting axon_recorder_node..."

if [ "$ROS_VERSION" = "1" ]; then
    # ROS 1 - Need roscore
    roscore &
    ROSCORE_PID=$!
    sleep 2
    
    # Use __name:= to set node name so services appear at /axon_recorder/...
    rosrun axon_recorder axon_recorder_node __name:=axon_recorder &
    NODE_PID=$!
else
    # ROS 2 - Direct node start
    ros2 run axon_recorder axon_recorder_node &
    NODE_PID=$!
fi

sleep 3
echo "Node started (PID: $NODE_PID)"

# Run integration tests
echo "Running test_ros_services.sh..."
TEST_RESULT=0
if ! "$TEST_SCRIPT"; then
    TEST_RESULT=1
fi

# Cleanup
echo "Cleaning up..."
kill $NODE_PID 2>/dev/null || true

if [ "$ROS_VERSION" = "1" ]; then
    kill $ROSCORE_PID 2>/dev/null || true
    killall roscore 2>/dev/null || true
fi

# Wait for processes to terminate
sleep 1

if [ $TEST_RESULT -eq 0 ]; then
    echo "✓ Integration tests passed"
else
    echo "✗ Integration tests failed"
fi

exit $TEST_RESULT

