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
#   --flamegraph         Generate CPU flamegraph for recorder process
#   --flamegraph-freq    Sampling frequency for perf (default: 99 Hz)
#   --asan               Enable Address Sanitizer build and checks
#   --asan-lite          Use reduced load with ASAN (faster debugging)
#
# ASAN Mode:
#   When --asan is enabled, the code is built with Address Sanitizer to detect:
#   - Double-free errors
#   - Use-after-free errors
#   - Buffer overflows (heap/stack/global)
#   - Memory leaks
#   - Use of uninitialized memory
# =============================================================================

set -e

# Default configuration
DURATION=10
IMU_RATE=1000
CAMERA_RATE=30
NUM_CAMERAS=3
OUTPUT_FILE=""
SKIP_BUILD=false
FLAMEGRAPH_ENABLED=false
FLAMEGRAPH_FREQ=99
ASAN_ENABLED=false
ASAN_LITE=false

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
        --flamegraph)
            FLAMEGRAPH_ENABLED=true
            shift
            ;;
        --flamegraph-freq)
            FLAMEGRAPH_FREQ="$2"
            shift 2
            ;;
        --asan)
            ASAN_ENABLED=true
            shift
            ;;
        --asan-lite)
            ASAN_ENABLED=true
            ASAN_LITE=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# ASAN mode: don't exit on error - we want to capture ASAN output
if [ "$ASAN_ENABLED" = true ]; then
    set +e
fi

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
echo "  Flamegraph:    ${FLAMEGRAPH_ENABLED}"
if [ "$FLAMEGRAPH_ENABLED" = true ]; then
echo "  Flamegraph Hz: ${FLAMEGRAPH_FREQ}"
fi
echo "  ASAN:          ${ASAN_ENABLED}"
if [ "$ASAN_ENABLED" = true ]; then
echo "  ASAN Lite:     ${ASAN_LITE}"
fi
echo "============================================"

# Apply ASAN lite mode: use reduced load for faster debugging
if [ "$ASAN_LITE" = true ]; then
    echo ""
    echo "ASAN lite mode: Using reduced load for faster debugging"
    IMU_RATE=100
    CAMERA_RATE=10
    NUM_CAMERAS=1
    echo "  IMU Rate:      ${IMU_RATE} Hz"
    echo "  Camera Rate:   ${CAMERA_RATE} Hz"
    echo "  Num Cameras:   ${NUM_CAMERAS}"
fi

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
    echo "Building axon_recorder package..."
    echo "============================================"
    
    # Determine build type and extra flags based on ASAN mode
    if [ "$ASAN_ENABLED" = true ]; then
        echo ""
        echo "Building with Address Sanitizer enabled..."
        # Clean previous build to ensure ASAN flags are applied
        rm -rf /workspace/ros_ws/build /workspace/ros_ws/install
        mkdir -p /workspace/ros_ws/build /workspace/ros_ws/install

        CMAKE_BUILD_TYPE="RelWithDebInfo"
        CMAKE_EXTRA_ARGS="-DENABLE_ASAN=ON"
    else
        CMAKE_BUILD_TYPE="Release"
        CMAKE_EXTRA_ARGS=""
    fi

    if [ "${ROS_VERSION}" = "1" ]; then
        # ROS 1 - Use catkin build
        echo "Building with catkin build (ROS 1)..."

        cd /workspace/axon/ros
        # Clean previous build (union of ROS1 and ROS2 build artifacts)
        rm -rf build devel install log logs

        source /opt/ros/${ROS_DISTRO}/setup.bash
        catkin build --no-notify -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ${CMAKE_EXTRA_ARGS}
        source devel/setup.bash

        if [ "$ASAN_ENABLED" = true ]; then
            echo "✓ Built axon_recorder with catkin build (ASAN enabled)"
        else
            echo "✓ Built axon_recorder with catkin build"
        fi
    else
        # ROS 2 - Use colcon
        echo "Building with colcon (ROS 2)..."

        cd /workspace/axon/ros
        rm -rf build devel install log logs

        source /opt/ros/${ROS_DISTRO}/setup.bash
        colcon build \
            --packages-select axon_recorder \
            --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} ${CMAKE_EXTRA_ARGS}

        source install/setup.bash

        if [ "$ASAN_ENABLED" = true ]; then
            echo "✓ Built axon_recorder with colcon (ASAN enabled)"
        else
            echo "✓ Built axon_recorder with colcon"
        fi
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
# ASAN Setup
# =============================================================================
ASAN_LOG="/tmp/asan_recorder.log"

if [ "$ASAN_ENABLED" = true ]; then
    echo ""
    echo "============================================"
    echo "Setting up Address Sanitizer..."
    echo "============================================"
    
    # Set ASAN options for detailed output
    # - new_delete_type_mismatch=0: Suppress false positives from ROS2 system libraries
    #   (ROS2 libs are not built with ASAN, causing allocator mismatch errors)
    # - halt_on_error=0: Don't halt, continue to capture more errors
    # - detect_leaks=0: Disable leak detection (also triggers false positives with ROS2)
    # - verify_asan_link_order=0: Don't verify linking order (helps with mixed ASAN/non-ASAN)
    export ASAN_OPTIONS="detect_leaks=0:print_stats=0:halt_on_error=0:new_delete_type_mismatch=0:alloc_dealloc_mismatch=0:verify_asan_link_order=0:replace_intrin=0"
    echo "ASAN_OPTIONS=${ASAN_OPTIONS}"
    echo "ASAN log: ${ASAN_LOG}"
    rm -f "$ASAN_LOG"
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

# Flamegraph output paths
FLAMEGRAPH_DIR="$STATS_DIR/flamegraph"
PERF_DATA="$FLAMEGRAPH_DIR/perf.data"
PERF_SCRIPT="$FLAMEGRAPH_DIR/perf.script"
FLAMEGRAPH_SVG="$FLAMEGRAPH_DIR/recorder_flamegraph.svg"
FLAMEGRAPH_FOLDED="$FLAMEGRAPH_DIR/perf.folded"

# Prepare stats directory
mkdir -p "$STATS_DIR"
rm -f "$PUBLISHER_STATS" "$RECORDER_STATS" 2>/dev/null || true

# =============================================================================
# Flamegraph Setup
# =============================================================================
PERF_PID=""
PERF_AVAILABLE=false

if [ "$FLAMEGRAPH_ENABLED" = true ]; then
    echo ""
    echo "============================================"
    echo "Setting up Flamegraph profiling..."
    echo "============================================"
    
    # Check for perf command
    if ! command -v perf &> /dev/null; then
        echo "WARNING: 'perf' command not found."
        echo "Install with: apt-get install linux-tools-generic"
    else
        # Test if perf actually works (kernel module check)
        # perf requires kernel-specific tools - test with a simple command
        if perf list hw 2>&1 | grep -q "not supported\|WARNING.*not found for kernel"; then
            echo "WARNING: 'perf' found but kernel tools are missing."
            echo ""
            echo "This typically happens when running Docker on:"
            echo "  - macOS with Docker Desktop or OrbStack"
            echo "  - WSL2 with mismatched kernel"
            echo "  - Cloud VMs with custom kernels"
            echo ""
            echo "Flamegraph profiling requires native Linux with matching kernel tools."
            echo "The performance test will continue without flamegraph."
        elif ! perf record -o /dev/null -- sleep 0.01 2>/dev/null; then
            echo "WARNING: 'perf record' test failed."
            echo "This may be due to:"
            echo "  - Missing kernel perf support"
            echo "  - Insufficient permissions (try --privileged)"
            echo "  - Docker on non-Linux host"
        else
            PERF_AVAILABLE=true
        fi
    fi
    
    if [ "$PERF_AVAILABLE" = true ]; then
        # Check for FlameGraph scripts
        if [ ! -d "/opt/FlameGraph" ]; then
            echo "Installing FlameGraph scripts..."
            git clone --depth 1 https://github.com/brendangregg/FlameGraph.git /opt/FlameGraph 2>/dev/null || {
                echo "WARNING: Could not install FlameGraph scripts."
                PERF_AVAILABLE=false
            }
        fi
    fi
    
    if [ "$PERF_AVAILABLE" = true ]; then
        mkdir -p "$FLAMEGRAPH_DIR"
        rm -f "$PERF_DATA" "$PERF_SCRIPT" "$FLAMEGRAPH_SVG" "$FLAMEGRAPH_FOLDED" 2>/dev/null || true
        echo "✓ Flamegraph profiling ready"
        echo "  Output: $FLAMEGRAPH_SVG"
    else
        echo ""
        echo "Flamegraph will be SKIPPED for this run."
        echo "Performance test will continue without CPU profiling."
        FLAMEGRAPH_ENABLED=false
    fi
fi

# Function to start perf recording for a given PID
start_perf_recording() {
    local target_pid=$1
    if [ "$FLAMEGRAPH_ENABLED" = true ] && [ "$PERF_AVAILABLE" = true ] && [ -n "$target_pid" ]; then
        echo "Starting perf record for PID $target_pid (freq: ${FLAMEGRAPH_FREQ} Hz)..."
        # Use -g for call graphs, -F for frequency, -p for PID
        # --call-graph dwarf provides better stack traces for C++
        perf record -F "$FLAMEGRAPH_FREQ" -g --call-graph dwarf -p "$target_pid" -o "$PERF_DATA" 2>/dev/null &
        PERF_PID=$!
        
        # Verify perf actually started
        sleep 0.5
        if kill -0 "$PERF_PID" 2>/dev/null; then
            echo "Perf recording started (PID: $PERF_PID)"
        else
            echo "WARNING: Perf recording failed to start"
            PERF_PID=""
        fi
    fi
}

# Function to stop perf and generate flamegraph
stop_perf_and_generate_flamegraph() {
    if [ "$FLAMEGRAPH_ENABLED" = true ] && [ "$PERF_AVAILABLE" = true ] && [ -n "$PERF_PID" ]; then
        echo ""
        echo "============================================"
        echo "Generating Flamegraph..."
        echo "============================================"
        
        # Stop perf recording gracefully
        echo "Stopping perf recording..."
        kill -INT "$PERF_PID" 2>/dev/null || true
        wait "$PERF_PID" 2>/dev/null || true
        PERF_PID=""
        
        if [ -f "$PERF_DATA" ]; then
            echo "Processing perf data..."
            
            # Convert perf data to script format
            perf script -i "$PERF_DATA" > "$PERF_SCRIPT" 2>/dev/null || {
                echo "WARNING: perf script failed"
                return
            }
            
            # Generate folded stacks
            /opt/FlameGraph/stackcollapse-perf.pl "$PERF_SCRIPT" > "$FLAMEGRAPH_FOLDED" 2>/dev/null || {
                echo "WARNING: stackcollapse-perf.pl failed"
                return
            }
            
            # Generate SVG flamegraph
            /opt/FlameGraph/flamegraph.pl \
                --title "Axon Recorder CPU Flamegraph" \
                --subtitle "Duration: ${DURATION}s, IMU: ${IMU_RATE}Hz, Cameras: ${NUM_CAMERAS}x${CAMERA_RATE}Hz" \
                --width 1800 \
                --hash \
                "$FLAMEGRAPH_FOLDED" > "$FLAMEGRAPH_SVG" 2>/dev/null || {
                echo "WARNING: flamegraph.pl failed"
                return
            }
            
            if [ -f "$FLAMEGRAPH_SVG" ]; then
                echo "✓ Flamegraph generated: $FLAMEGRAPH_SVG"
                
                # Print some stats
                local num_samples=$(wc -l < "$FLAMEGRAPH_FOLDED")
                echo "  Samples collected: $num_samples"
                
                # Show top functions
                echo ""
                echo "Top CPU consumers:"
                echo "----------------------------------------"
                sort -t';' -k2 -rn "$FLAMEGRAPH_FOLDED" | head -10 | while read line; do
                    func=$(echo "$line" | rev | cut -d';' -f1 | rev | cut -d' ' -f1)
                    count=$(echo "$line" | awk '{print $NF}')
                    printf "  %6d  %s\n" "$count" "$func"
                done
                echo "----------------------------------------"
            fi
        else
            echo "WARNING: No perf data collected"
        fi
    fi
}

# Cleanup function - gracefully stop processes to allow stats writing
# Note: We use pkill to find and kill processes by name, as ros2 run spawns
# child processes and the captured $! PID is just the wrapper.
cleanup() {
    echo ""
    echo "Cleaning up processes..."

    # Stop perf recording first (if running)
    if [[ -n "$PERF_PID" ]]; then
        echo "Stopping perf recording..."
        kill -INT "$PERF_PID" 2>/dev/null || true
        wait "$PERF_PID" 2>/dev/null || true
        PERF_PID=""
    fi

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
    if [ "$ASAN_ENABLED" = true ]; then
        ros2 run axon_recorder axon_recorder_node 2>&1 | tee "$ASAN_LOG" &
    else
        ros2 run axon_recorder axon_recorder_node &
    fi
    RECORDER_PID=$!
    echo "Recorder wrapper PID: $RECORDER_PID"
    sleep 3  # Wait for recorder to initialize
    
    # Find the actual recorder node PID for flamegraph profiling
    if [ "$FLAMEGRAPH_ENABLED" = true ]; then
        echo "Looking for actual recorder node PID for profiling..."
        ACTUAL_RECORDER_PID_FOR_PERF=""
        for attempt in {1..10}; do
            while IFS= read -r line; do
                pid=$(echo "$line" | awk '{print $1}')
                cmd=$(echo "$line" | cut -d' ' -f2-)
                if [[ "$cmd" != *"python"* ]] && [[ "$cmd" != *"ros2 run"* ]]; then
                    if [[ "$cmd" == *"axon_recorder_node"* ]]; then
                        ACTUAL_RECORDER_PID_FOR_PERF="$pid"
                        echo "Found recorder node for profiling: PID=$pid"
                        break 2
                    fi
                fi
            done < <(pgrep -af "axon_recorder" 2>/dev/null || true)
            sleep 0.5
        done
        
        if [ -n "$ACTUAL_RECORDER_PID_FOR_PERF" ]; then
            start_perf_recording "$ACTUAL_RECORDER_PID_FOR_PERF"
        else
            echo "WARNING: Could not find recorder PID for profiling"
        fi
    fi

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

# Generate flamegraph if enabled
stop_perf_and_generate_flamegraph

echo ""
echo "============================================"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo "  Performance Test: PASSED ✓"
else
    echo "  Performance Test: FAILED ✗"
fi
echo "============================================"

# Print flamegraph location if generated
if [ "$FLAMEGRAPH_ENABLED" = true ] && [ -f "$FLAMEGRAPH_SVG" ]; then
    echo ""
    echo "Flamegraph saved to: $FLAMEGRAPH_SVG"
    echo "Open in browser to view interactive CPU profile"
fi

# =============================================================================
# ASAN Error Analysis
# =============================================================================
if [ "$ASAN_ENABLED" = true ]; then
    echo ""
    echo "============================================"
    echo "ASAN Analysis..."
    echo "============================================"
    
    ASAN_ERRORS=0
    if [ -f "$ASAN_LOG" ]; then
        if grep -q "ERROR: AddressSanitizer" "$ASAN_LOG" 2>/dev/null; then
            ASAN_ERRORS=1
        fi
        if grep -q "ERROR: LeakSanitizer" "$ASAN_LOG" 2>/dev/null; then
            ASAN_ERRORS=1
        fi
    fi
    
    if [ $ASAN_ERRORS -eq 1 ]; then
        echo ""
        echo "╔══════════════════════════════════════════════════════════╗"
        echo "║   !!! ASAN DETECTED MEMORY ERRORS !!!                    ║"
        echo "╚══════════════════════════════════════════════════════════╝"
        echo ""
        echo "=== ASAN Error Report ==="
        echo ""
        # Extract and display ASAN error report
        grep -A 200 "ERROR: AddressSanitizer\|ERROR: LeakSanitizer" "$ASAN_LOG" 2>/dev/null || true
        echo ""
        echo "=== End of ASAN Report ==="
        echo ""
        echo "Full log: $ASAN_LOG"
        EXIT_CODE=1
    else
        # Check for the double free message even if ASAN didn't catch it
        if [ -f "$ASAN_LOG" ] && grep -q "double free\|free():.*double" "$ASAN_LOG" 2>/dev/null; then
            echo ""
            echo "╔══════════════════════════════════════════════════════════╗"
            echo "║   !!! DOUBLE FREE DETECTED (runtime) !!!                 ║"
            echo "╚══════════════════════════════════════════════════════════╝"
            echo ""
            echo "Double free was detected but ASAN may not have full details."
            echo "This can happen if the error is in a library not built with ASAN."
            echo ""
            grep -B5 -A10 "double free\|free():.*double" "$ASAN_LOG" 2>/dev/null || true
            echo ""
            echo "Full log: $ASAN_LOG"
            EXIT_CODE=1
        else
            echo "✓ No ASAN errors detected in this run."
            echo ""
            echo "Note: If you saw 'double free' in the output but ASAN didn't report it,"
            echo "the error may be in a library not compiled with ASAN (e.g., ROS2)."
            echo ""
            echo "Full log: $ASAN_LOG"
        fi
    fi
fi

exit $EXIT_CODE
