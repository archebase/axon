#!/bin/bash
# =============================================================================
# Performance Test Runner for Docker
# =============================================================================
# This script builds the axon_recorder package and runs performance tests.
# It is designed to run inside a ROS Docker container.
#
# Usage:
#   /usr/local/bin/run_perf_tests.sh [options]
#
# Options:
#   --duration <sec>     Test duration (default: 10)
#   --imu-rate <hz>      IMU rate (default: 1000)
#   --camera-rate <hz>   Camera rate (default: 30)
#   --num-cameras <n>    Number of cameras (default: 3)
#   --output <file>      JSON output file
#   --skip-build         Skip building the workspace
# =============================================================================

set -e

# Default configuration
DURATION=10
IMU_RATE=1000
CAMERA_RATE=30
NUM_CAMERAS=3
OUTPUT_FILE=""
SKIP_BUILD=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --imu-rate)
            IMU_RATE="$2"
            shift 2
            ;;
        --camera-rate)
            CAMERA_RATE="$2"
            shift 2
            ;;
        --num-cameras)
            NUM_CAMERAS="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "============================================"
echo "  Axon Performance Test Suite (Docker)"
echo "============================================"
echo "Configuration:"
echo "  ROS Distro:    ${ROS_DISTRO}"
echo "  ROS Version:   ${ROS_VERSION}"
echo "  Duration:      ${DURATION}s"
echo "  IMU Rate:      ${IMU_RATE} Hz"
echo "  Camera Rate:   ${CAMERA_RATE} Hz"
echo "  Num Cameras:   ${NUM_CAMERAS}"
echo "  Skip Build:    ${SKIP_BUILD}"
echo "============================================"

# Source ROS environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "✓ Sourced ROS ${ROS_DISTRO} environment"
else
    echo "ERROR: ROS setup.bash not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

# Navigate to workspace
cd /workspace/axon

# =============================================================================
# Build Phase
# =============================================================================
if [ "$SKIP_BUILD" = false ]; then
    echo ""
    echo "============================================"
    echo "Building Rust Lance bridge library..."
    echo "============================================"
    
    # Build the Rust C FFI library first (required by axon_recorder)
    # Note: Cargo workspace outputs to /workspace/axon/target/, not /workspace/axon/c/target/
    cd /workspace/axon
    
    echo "Building with cargo..."
    cargo build --release -p axon-lance-ffi 2>&1
    BUILD_EXIT=$?
    
    # Verify the library was created (workspace outputs to root target/)
    if [ -f "target/release/liblance_writer_bridge.so" ]; then
        echo "✓ Built Rust Lance bridge library"
        ls -la target/release/liblance_writer_bridge*
    else
        echo "ERROR: Rust library not found after build"
        echo "Cargo exit code: $BUILD_EXIT"
        echo "Contents of target/release/:"
        ls -la target/release/ 2>/dev/null || echo "Directory does not exist"
        exit 1
    fi
    
    echo ""
    echo "============================================"
    echo "Building axon_recorder package..."
    echo "============================================"
    
    # Clean previous build to ensure fresh package metadata
    echo "Cleaning previous build artifacts..."
    rm -rf /workspace/ros_ws/build /workspace/ros_ws/install /workspace/ros_ws/log
    
    # Build in the axon/ros directory to preserve relative paths in CMakeLists.txt
    # The CMakeLists.txt uses paths like ../../cpp/axon_arrow which need the full repo structure
    cd /workspace/axon/ros
    
    if [ "${ROS_VERSION}" = "2" ]; then
        # Build with colcon, keeping build artifacts in a separate directory
        # Use --build-base and --install-base to avoid polluting the source tree
        colcon build \
            --packages-select axon_recorder \
            --build-base /workspace/ros_ws/build \
            --install-base /workspace/ros_ws/install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
        source /workspace/ros_ws/install/setup.bash
        echo "✓ Built axon_recorder with colcon"
    else
        # ROS 1 build with catkin
        # Create catkin workspace structure
        mkdir -p /workspace/ros_ws/src
        if [ ! -L /workspace/ros_ws/src/axon_recorder ]; then
            ln -sf /workspace/axon/ros/axon_recorder /workspace/ros_ws/src/axon_recorder
        fi
        # Also symlink cpp and c directories to preserve relative paths
        if [ ! -L /workspace/ros_ws/src/cpp ]; then
            ln -sf /workspace/axon/cpp /workspace/ros_ws/src/cpp
        fi
        if [ ! -L /workspace/ros_ws/src/c ]; then
            ln -sf /workspace/axon/c /workspace/ros_ws/src/c
        fi
        cd /workspace/ros_ws
        catkin_make -DCMAKE_BUILD_TYPE=Release
        source devel/setup.bash
        echo "✓ Built axon_recorder with catkin"
    fi
    
    cd /workspace/axon
else
    # Source existing build if available
    if [ -f /workspace/ros_ws/install/setup.bash ]; then
        source /workspace/ros_ws/install/setup.bash
    elif [ -f /workspace/ros_ws/devel/setup.bash ]; then
        source /workspace/ros_ws/devel/setup.bash
    fi
fi

# =============================================================================
# Run Performance Tests
# =============================================================================
echo ""
echo "============================================"
echo "Running Performance Tests..."
echo "============================================"

# Build common args
COMMON_ARGS="--duration $DURATION --imu-rate $IMU_RATE --camera-rate $CAMERA_RATE --num-cameras $NUM_CAMERAS"

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    if [[ -n "$PUB_PID" ]]; then
        kill $PUB_PID 2>/dev/null || true
    fi
    if [[ -n "$RECORDER_PID" ]]; then
        kill $RECORDER_PID 2>/dev/null || true
    fi
}

trap cleanup EXIT

if [ "${ROS_VERSION}" = "2" ]; then
    # ROS 2 execution
    echo "Starting synthetic publisher (ROS 2)..."
    ros2 run axon_recorder synthetic_publisher $COMMON_ARGS &
    PUB_PID=$!
    sleep 2
    
    echo "Starting axon_recorder..."
    ros2 run axon_recorder axon_recorder_node &
    RECORDER_PID=$!
    sleep 2
    
    echo "Starting performance test..."
    if [[ -n "$OUTPUT_FILE" ]]; then
        ros2 run axon_recorder perf_test_node $COMMON_ARGS --output "$OUTPUT_FILE"
    else
        ros2 run axon_recorder perf_test_node $COMMON_ARGS
    fi
else
    # ROS 1 execution
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    
    echo "Starting synthetic publisher (ROS 1)..."
    rosrun axon_recorder synthetic_publisher $COMMON_ARGS &
    PUB_PID=$!
    sleep 2
    
    echo "Starting axon_recorder..."
    rosrun axon_recorder axon_recorder_node &
    RECORDER_PID=$!
    sleep 2
    
    echo "Starting performance test..."
    if [[ -n "$OUTPUT_FILE" ]]; then
        rosrun axon_recorder perf_test_node $COMMON_ARGS --output "$OUTPUT_FILE"
    else
        rosrun axon_recorder perf_test_node $COMMON_ARGS
    fi
    
    # Cleanup roscore
    kill $ROSCORE_PID 2>/dev/null || true
fi

EXIT_CODE=$?

echo ""
echo "============================================"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo "  Performance Test: PASSED ✓"
else
    echo "  Performance Test: FAILED ✗"
fi
echo "============================================"

exit $EXIT_CODE
