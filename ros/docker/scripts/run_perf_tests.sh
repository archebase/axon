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
        # ls -la target/release/liblance_writer_bridge*
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

# Build common args for publisher
PUB_ARGS="--duration $DURATION --imu-rate $IMU_RATE --camera-rate $CAMERA_RATE --num-cameras $NUM_CAMERAS"
MONITOR_ARGS="--duration $DURATION"

if [[ -n "$OUTPUT_FILE" ]]; then
    MONITOR_ARGS="$MONITOR_ARGS --output $OUTPUT_FILE"
fi

# Stats file paths
STATS_DIR="/data/recordings"
PUBLISHER_STATS="$STATS_DIR/publisher_stats.json"
RECORDER_STATS="$STATS_DIR/recorder_stats.json"

# Prepare stats directory
mkdir -p "$STATS_DIR"
rm -f "$PUBLISHER_STATS" "$RECORDER_STATS" 2>/dev/null || true

# Cleanup function - gracefully stop processes to allow stats writing
# Note: We use pkill to find and kill processes by name, as ros2 run spawns
# child processes and the captured $! PID is just the wrapper.
cleanup() {
    echo ""
    echo "Cleaning up processes..."
    
    # Stop publisher first (by name)
    if pgrep -f "synthetic_publisher" > /dev/null 2>&1; then
        echo "Stopping publisher..."
        pkill -TERM -f "synthetic_publisher" 2>/dev/null || true
        sleep 1
        pkill -9 -f "synthetic_publisher" 2>/dev/null || true
    fi
    
    # Stop recorder with SIGTERM to trigger graceful shutdown (writes stats)
    # Use "axon_recorder" pattern (not "axon_recorder_node") to match the actual process
    if pgrep -f "axon_recorder" > /dev/null 2>&1; then
        echo "Stopping recorder (waiting for stats file)..."
        pkill -TERM -f "axon_recorder" 2>/dev/null || true
        
        # Wait for recorder to write stats file (up to 10 seconds)
        for i in {1..20}; do
            if [[ -f "$RECORDER_STATS" ]]; then
                echo "Recorder stats file written"
                break
            fi
            sleep 0.5
        done
        
        # Force kill if still running
        pkill -9 -f "axon_recorder" 2>/dev/null || true
    fi
    
    # Stop monitor (by name)
    if pgrep -f "perf_test_node" > /dev/null 2>&1; then
        echo "Stopping monitor..."
        pkill -TERM -f "perf_test_node" 2>/dev/null || true
        sleep 1
        pkill -9 -f "perf_test_node" 2>/dev/null || true
    fi
    
    # Clean up wrapper PIDs if still running
    [[ -n "$PUB_PID" ]] && kill -9 $PUB_PID 2>/dev/null || true
    [[ -n "$RECORDER_PID" ]] && kill -9 $RECORDER_PID 2>/dev/null || true
    [[ -n "$MONITOR_PID" ]] && kill -9 $MONITOR_PID 2>/dev/null || true
    
    # ROS 1: cleanup roscore
    if [[ -n "$ROSCORE_PID" ]]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
}

trap cleanup EXIT

if [ "${ROS_VERSION}" = "2" ]; then
    # =========================================================================
    # ROS 2 execution with proper synchronization
    # =========================================================================
    # Note: ros2 run spawns child processes, so we use pkill to find and kill
    # processes by name. This ensures the actual node executable receives SIGTERM.
    
    # Step 1: Start recorder FIRST (it needs to be ready before publisher starts)
    echo ""
    echo "Step 1: Starting axon_recorder..."
    ros2 run axon_recorder axon_recorder_node &
    RECORDER_PID=$!
    echo "Recorder wrapper PID: $RECORDER_PID"
    sleep 3  # Wait for recorder to initialize
    
    # Step 2: Start performance monitor (finds recorder PID for CPU/memory tracking)
    echo ""
    echo "Step 2: Starting performance monitor..."
    ros2 run axon_recorder perf_test_node $MONITOR_ARGS &
    MONITOR_PID=$!
    echo "Monitor wrapper PID: $MONITOR_PID"
    sleep 1
    
    # Step 3: Start publisher (runs for DURATION seconds, then exits)
    echo ""
    echo "Step 3: Starting synthetic publisher..."
    ros2 run axon_recorder synthetic_publisher $PUB_ARGS &
    PUB_PID=$!
    echo "Publisher wrapper PID: $PUB_PID"
    
    # Step 4: Wait for publisher to complete (it runs for exact duration)
    echo ""
    echo "Step 4: Waiting for publisher to complete ($DURATION seconds)..."
    wait $PUB_PID 2>/dev/null || true
    echo "Publisher finished"
    PUB_PID=""
    
    # Step 5: Signal recorder to stop gracefully (triggers stats file write)
    # Use pkill to find and kill the actual recorder node by name
    echo ""
    echo "Step 5: Stopping recorder..."
    
    # Debug: show all matching processes
    echo "Searching for recorder processes..."
    pgrep -af "axon_recorder" || echo "  (no processes found with pgrep -af)"
    
    # Find the ACTUAL recorder node PID (not the Python ros2 run wrapper)
    # The actual node is the one WITHOUT "python" or "ros2 run" in its command line
    # It should be something like: /workspace/.../axon_recorder_node
    ACTUAL_RECORDER_PID=""
    while IFS= read -r line; do
        pid=$(echo "$line" | awk '{print $1}')
        cmd=$(echo "$line" | cut -d' ' -f2-)
        # Skip if it's a python process (the ros2 run wrapper)
        if [[ "$cmd" != *"python"* ]] && [[ "$cmd" != *"ros2 run"* ]]; then
            if [[ "$cmd" == *"axon_recorder"* ]]; then
                ACTUAL_RECORDER_PID="$pid"
                echo "Found actual recorder node: PID=$pid CMD=$cmd"
                break
            fi
        fi
    done < <(pgrep -af "axon_recorder" 2>/dev/null || true)
    
    if [[ -n "$ACTUAL_RECORDER_PID" ]]; then
        echo "Sending SIGTERM to actual recorder PID: $ACTUAL_RECORDER_PID"
        kill -TERM $ACTUAL_RECORDER_PID 2>/dev/null || true
        
        # Wait for recorder stats file
        for i in {1..20}; do
            if [[ -f "$RECORDER_STATS" ]]; then
                echo "Recorder stats file ready"
                break
            fi
            echo "Waiting for recorder stats... ($i)"
            sleep 0.5
        done
        
        # Wait for recorder to exit (with timeout)
        for i in {1..10}; do
            if ! kill -0 $ACTUAL_RECORDER_PID 2>/dev/null; then
                echo "Recorder node exited"
                break
            fi
            sleep 0.5
        done
    else
        echo "Warning: Could not find actual recorder node process"
        echo "Trying pkill as fallback (will kill all matching processes)..."
        pkill -TERM -f "axon_recorder_node" 2>/dev/null || true
        
        # Wait for stats file anyway
        for i in {1..20}; do
            if [[ -f "$RECORDER_STATS" ]]; then
                echo "Recorder stats file ready"
                break
            fi
            echo "Waiting for recorder stats... ($i)"
            sleep 0.5
        done
    fi
    
    # Clean up wrapper process if still running
    kill -9 $RECORDER_PID 2>/dev/null || true
    wait $RECORDER_PID 2>/dev/null || true
    RECORDER_PID=""
    
    # Step 6: Wait for monitor to collect results and exit
    echo ""
    echo "Step 6: Waiting for performance monitor to finish..."
    wait $MONITOR_PID 2>/dev/null
    EXIT_CODE=$?
    MONITOR_PID=""
    
else
    # =========================================================================
    # ROS 1 execution
    # =========================================================================
    # Note: Using pkill for process management, same as ROS 2
    echo "Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    
    echo "Starting axon_recorder..."
    rosrun axon_recorder axon_recorder_node &
    RECORDER_PID=$!
    sleep 2
    
    echo "Starting performance monitor..."
    rosrun axon_recorder perf_test_node $MONITOR_ARGS &
    MONITOR_PID=$!
    sleep 1
    
    echo "Starting synthetic publisher..."
    rosrun axon_recorder synthetic_publisher $PUB_ARGS &
    PUB_PID=$!
    
    # Wait for publisher
    wait $PUB_PID 2>/dev/null || true
    PUB_PID=""
    
    # Stop recorder by name
    pkill -TERM -f "axon_recorder_node" 2>/dev/null || true
    
    # Wait for recorder stats file
    for i in {1..10}; do
        if [[ -f "$RECORDER_STATS" ]]; then
            echo "Recorder stats file ready"
            break
        fi
        echo "Waiting for recorder stats... ($i)"
        sleep 0.5
    done
    
    wait $RECORDER_PID 2>/dev/null || true
    RECORDER_PID=""
    
    # Wait for monitor
    wait $MONITOR_PID 2>/dev/null
    EXIT_CODE=$?
    MONITOR_PID=""
fi

echo ""
echo "============================================"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo "  Performance Test: PASSED ✓"
else
    echo "  Performance Test: FAILED ✗"
fi
echo "============================================"

exit $EXIT_CODE
