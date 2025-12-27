#!/bin/bash
# ============================================================================
# E2E Test Runner
# ============================================================================
# This script starts the axon_recorder_node, runs E2E tests via ROS service
# calls, and cleans up. It's used by both CI and local Docker testing.
#
# Usage:
#   ./run_e2e_tests.sh [source_path]
#
# Arguments:
#   source_path - Optional path to the source directory (default: script's location)
#
# Environment:
#   ROS_VERSION - Must be set to "1" or "2" (or auto-detected from ROS_DISTRO)
#   ROS_DISTRO  - Must be set (e.g., "noetic", "humble")
#
# Prerequisites:
#   - ROS workspace should be sourced before calling this script (will attempt to source if not)
#   - axon_recorder package must be built and installed
# ============================================================================

set -eo pipefail

# Determine script directory and source path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_PATH="${1:-$SCRIPT_DIR}"

# Try to source workspace library if available (for CI/Docker environments)
# If not available, we'll handle workspace sourcing manually
LIB_DIR="/workspace/axon/ros/docker/scripts/lib"
if [ -f "${LIB_DIR}/ros_workspace_lib.sh" ]; then
    source "${LIB_DIR}/ros_workspace_lib.sh"
    USE_WORKSPACE_LIB=true
else
    USE_WORKSPACE_LIB=false
fi

# Try to source diagnostics library if available
if [ -f "${LIB_DIR}/ros_diagnostics_lib.sh" ]; then
    source "${LIB_DIR}/ros_diagnostics_lib.sh"
    USE_DIAGNOSTICS_LIB=true
else
    USE_DIAGNOSTICS_LIB=false
fi

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

# Source ROS workspace using library if available, otherwise fall back to manual sourcing
if [ "$USE_WORKSPACE_LIB" = true ]; then
    # Use library functions for workspace sourcing
    ros_workspace_source_base || true  # Non-fatal if already sourced
    
    WORKSPACE_ROOT="/workspace/axon"
    if ! ros_workspace_source_workspace "${WORKSPACE_ROOT}"; then
        echo "WARNING: Failed to source workspace using library, trying manual sourcing..."
        USE_WORKSPACE_LIB=false
    fi
fi

# Fallback to manual workspace sourcing if library not available or failed
if [ "$USE_WORKSPACE_LIB" = false ]; then
    set +eu
    if [ -n "$ROS_DISTRO" ]; then
        . /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true
    fi
    # Check workspace paths in order of preference
    for ws in /root/target_ws /workspace/axon/ros /workspace/axon /workspace/catkin_ws; do
        if [ -f "$ws/install/setup.bash" ]; then
            echo "Sourcing $ws/install/setup.bash"
            . "$ws/install/setup.bash"
            break
        elif [ -f "$ws/devel/setup.bash" ]; then
            echo "Sourcing $ws/devel/setup.bash"
            . "$ws/devel/setup.bash"
            break
        fi
    done
    set -e
fi

# Debug: Show what's installed
echo "Checking installed packages..."
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
ls -la /root/target_ws/install/ 2>/dev/null | head -15 || echo "No /root/target_ws/install/"
echo "--- axon_recorder install dir ---"
ls -la /root/target_ws/install/axon_recorder/ 2>/dev/null || echo "No axon_recorder install dir"
echo "--- share directory (needed for ament index) ---"
ls -la /root/target_ws/install/axon_recorder/share/ 2>/dev/null || echo "No share dir"
echo "--- ament_index (needed for ros2 pkg list) ---"
ls -la /root/target_ws/install/axon_recorder/share/ament_index/resource_index/packages/ 2>/dev/null || echo "No ament_index"
echo "--- lib directory (executables) ---"
ls -la /root/target_ws/install/axon_recorder/lib/axon_recorder/ 2>/dev/null || echo "No lib/axon_recorder dir"

# Verify package is available
if [ "$USE_WORKSPACE_LIB" = true ]; then
    ros_workspace_verify_package || echo "WARNING: Package verification failed"
else
    echo "Checking if axon_recorder is available..."
    if [ "$ROS_VERSION" = "2" ]; then
        if ros2 pkg list 2>/dev/null | grep -q "^axon_recorder$"; then
            echo "✓ axon_recorder found in ros2 pkg list"
        else
            echo "WARNING: axon_recorder not found in ros2 pkg list"
            echo "First 30 packages:"
            ros2 pkg list 2>/dev/null | head -30 || true
        fi
    else
        if rospack find axon_recorder 2>/dev/null; then
            echo "✓ axon_recorder found via rospack"
        else
            echo "WARNING: axon_recorder not found via rospack"
        fi
    fi
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
    # ROS 2 - Set node name to match ROS 1 (so services appear at /axon_recorder/...)
    ros2 run axon_recorder axon_recorder_node --ros-args -r __node:=axon_recorder &
    NODE_PID=$!
fi

sleep 3
echo "Node started (PID: $NODE_PID)"

# Run diagnostics if library is available (useful for both ROS1 and ROS2 issues)
if [ "$USE_DIAGNOSTICS_LIB" = true ]; then
    echo ""
    echo "Running diagnostics before tests..."
    ros_diagnostics_full "axon_recorder" "axon_recorder/recording_control" || {
        echo "WARNING: Some diagnostic checks failed, but continuing with tests..."
    }
    echo ""
fi

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

