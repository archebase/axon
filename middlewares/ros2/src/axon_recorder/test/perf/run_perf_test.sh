#!/bin/bash
# =============================================================================
# Performance Test Runner
# =============================================================================
# This script runs the full performance test suite:
# 1. Starts the axon_recorder first
# 2. Starts the synthetic publisher (IMU + cameras)
# 3. Runs the performance monitor
# 4. Waits for publisher to complete, then signals recorder to stop
# 5. Collects and reports results
#
# Key improvement: Proper synchronization between components
# - Recorder starts first and is ready to receive
# - Publisher runs for exact duration, then writes stats and exits
# - Recorder is signaled to stop, writes stats, then exits
# - Monitor reads stats files and reports results
#
# Usage:
#   ./run_perf_test.sh [options]
#
# Options:
#   --duration <sec>     Test duration (default: 10)
#   --imu-rate <hz>      IMU rate (default: 1000)
#   --camera-rate <hz>   Camera rate (default: 30)
#   --num-cameras <n>    Number of cameras (default: 3)
#   --output <file>      JSON output file
#   --ros1               Use ROS 1 commands
#   --ros2               Use ROS 2 commands (default)

set -e

# Default configuration
DURATION=10
IMU_RATE=1000
CAMERA_RATE=30
NUM_CAMERAS=3
OUTPUT_FILE=""
ROS_VERSION="ros2"
STATS_DIR="/data/recordings"

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
        --ros1)
            ROS_VERSION="ros1"
            shift
            ;;
        --ros2)
            ROS_VERSION="ros2"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "============================================"
echo "  Axon Performance Test Suite"
echo "============================================"
echo "Configuration:"
echo "  ROS Version:   $ROS_VERSION"
echo "  Duration:      ${DURATION}s"
echo "  IMU Rate:      ${IMU_RATE} Hz"
echo "  Camera Rate:   ${CAMERA_RATE} Hz"
echo "  Num Cameras:   $NUM_CAMERAS"
echo "  Stats Dir:     $STATS_DIR"
echo "============================================"

# Cleanup function - ensure processes are stopped gracefully
# Note: We use pkill to find and kill processes by name, as ros2 run spawns
# child processes and the captured $! PID is just the wrapper.
cleanup() {
    echo ""
    echo "Cleaning up..."
    
    # Stop publisher by name
    if pgrep -f "synthetic_publisher" > /dev/null 2>&1; then
        echo "Stopping publisher..."
        pkill -TERM -f "synthetic_publisher" 2>/dev/null || true
        sleep 1
        pkill -9 -f "synthetic_publisher" 2>/dev/null || true
    fi
    
    # Stop recorder by name (use "axon_recorder" pattern to match the actual process)
    if pgrep -f "axon_recorder" > /dev/null 2>&1; then
        echo "Stopping recorder..."
        pkill -TERM -f "axon_recorder" 2>/dev/null || true
        # Wait for recorder to write stats file
        sleep 2
        pkill -9 -f "axon_recorder" 2>/dev/null || true
    fi
    
    # Stop monitor by name
    if pgrep -f "perf_test_node" > /dev/null 2>&1; then
        echo "Stopping monitor..."
        pkill -TERM -f "perf_test_node" 2>/dev/null || true
        sleep 1
        pkill -9 -f "perf_test_node" 2>/dev/null || true
    fi
    
    # Clean up wrapper PIDs
    [[ -n "$PUB_PID" ]] && kill -9 $PUB_PID 2>/dev/null || true
    [[ -n "$RECORDER_PID" ]] && kill -9 $RECORDER_PID 2>/dev/null || true
    [[ -n "$MONITOR_PID" ]] && kill -9 $MONITOR_PID 2>/dev/null || true
}

trap cleanup EXIT

# Prepare stats directory
echo "Preparing stats directory..."
mkdir -p "$STATS_DIR"

# Clean up old stats files
rm -f "$STATS_DIR/publisher_stats.json" 2>/dev/null || true
rm -f "$STATS_DIR/recorder_stats.json" 2>/dev/null || true

# Build common args for publisher
PUB_ARGS="--duration $DURATION --imu-rate $IMU_RATE --camera-rate $CAMERA_RATE --num-cameras $NUM_CAMERAS"

# Build args for monitor
MONITOR_ARGS="--duration $DURATION"
if [[ -n "$OUTPUT_FILE" ]]; then
    MONITOR_ARGS="$MONITOR_ARGS --output $OUTPUT_FILE"
fi

if [[ "$ROS_VERSION" == "ros1" ]]; then
    echo "ROS 1 performance test not yet fully supported"
    exit 1
else
    # ROS 2 execution with proper synchronization
    # Note: ros2 run spawns child processes, so we use pkill to find and kill
    # processes by name. This ensures the actual node executable receives SIGTERM.
    
    # Step 1: Start recorder FIRST so it's ready to receive messages
    echo ""
    echo "Step 1: Starting axon_recorder..."
    ros2 run axon_recorder axon_recorder &
    RECORDER_PID=$!
    echo "Recorder wrapper PID: $RECORDER_PID"
    
    # Wait for recorder to initialize
    sleep 3
    
    # Step 2: Start performance monitor (it will find recorder PID)
    echo ""
    echo "Step 2: Starting performance monitor..."
    ros2 run axon_recorder perf_test_node $MONITOR_ARGS &
    MONITOR_PID=$!
    echo "Monitor wrapper PID: $MONITOR_PID"
    
    # Wait for monitor to initialize
    sleep 1
    
    # Step 3: Start publisher (will run for exact duration then exit)
    echo ""
    echo "Step 3: Starting synthetic publisher..."
    ros2 run axon_recorder synthetic_publisher $PUB_ARGS &
    PUB_PID=$!
    echo "Publisher wrapper PID: $PUB_PID"
    
    # Step 4: Wait for publisher to complete (it runs for DURATION seconds)
    echo ""
    echo "Step 4: Waiting for publisher to complete ($DURATION seconds)..."
    wait $PUB_PID 2>/dev/null || true
    PUB_PID=""
    echo "Publisher finished"
    
    # Step 5: Wait a moment, then signal recorder to stop gracefully
    # Use pkill to find and kill the actual recorder node by name
    echo ""
    echo "Step 5: Signaling recorder to stop..."
    sleep 1
    
    # Debug: show all matching processes
    echo "Searching for recorder processes..."
    pgrep -af "axon_recorder" || echo "  (no processes found with pgrep -af)"
    
    # Find the ACTUAL recorder node PID (not the Python ros2 run wrapper)
    # The actual node is the one WITHOUT "python" or "ros2 run" in its command line
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
        
        # Wait for recorder to flush and write stats
        for i in {1..20}; do
            if [[ -f "$STATS_DIR/recorder_stats.json" ]]; then
                echo "Recorder stats file written"
                break
            fi
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
        echo "Trying pkill as fallback..."
        pkill -TERM -f "axon_recorder_node" 2>/dev/null || true
        
        # Wait for stats file anyway
        for i in {1..20}; do
            if [[ -f "$STATS_DIR/recorder_stats.json" ]]; then
                echo "Recorder stats file written"
                break
            fi
            sleep 0.5
        done
    fi
    
    # Clean up wrapper process
    kill -9 $RECORDER_PID 2>/dev/null || true
    RECORDER_PID=""
    
    # Step 6: Wait for monitor to finish collecting results
    echo ""
    echo "Step 6: Waiting for performance monitor to finish..."
    wait $MONITOR_PID 2>/dev/null
    EXIT_CODE=$?
    MONITOR_PID=""
fi

echo ""
echo "============================================"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo "  Performance Test: PASSED"
else
    echo "  Performance Test: FAILED"
fi
echo "============================================"

# Show stats files if they exist
echo ""
echo "Stats files:"
if [[ -f "$STATS_DIR/publisher_stats.json" ]]; then
    echo "--- Publisher Stats ---"
    cat "$STATS_DIR/publisher_stats.json"
fi
if [[ -f "$STATS_DIR/recorder_stats.json" ]]; then
    echo "--- Recorder Stats ---"
    cat "$STATS_DIR/recorder_stats.json"
fi

exit $EXIT_CODE
