#!/bin/bash
# =============================================================================
# Performance Test Runner
# =============================================================================
# This script runs the full performance test suite:
# 1. Starts the synthetic publisher (IMU + cameras)
# 2. Starts the axon_recorder
# 3. Runs the performance test node
# 4. Reports results
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
echo "============================================"

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

# Build common args
COMMON_ARGS="--duration $DURATION --imu-rate $IMU_RATE --camera-rate $CAMERA_RATE --num-cameras $NUM_CAMERAS"

if [[ "$ROS_VERSION" == "ros1" ]]; then
    # ROS 1 execution
    echo "Starting synthetic publisher (ROS 1)..."
    rosrun axon_recorder synthetic_publisher $COMMON_ARGS &
    PUB_PID=$!
    sleep 2
    
    echo "Starting axon_recorder..."
    rosrun axon_recorder axon_recorder &
    RECORDER_PID=$!
    sleep 2
    
    echo "Starting performance test..."
    if [[ -n "$OUTPUT_FILE" ]]; then
        rosrun axon_recorder perf_test_node $COMMON_ARGS --output "$OUTPUT_FILE"
    else
        rosrun axon_recorder perf_test_node $COMMON_ARGS
    fi
    
else
    # ROS 2 execution
    echo "Starting synthetic publisher (ROS 2)..."
    ros2 run axon_recorder synthetic_publisher $COMMON_ARGS &
    PUB_PID=$!
    sleep 2
    
    echo "Starting axon_recorder..."
    ros2 run axon_recorder axon_recorder &
    RECORDER_PID=$!
    sleep 2
    
    echo "Starting performance test..."
    if [[ -n "$OUTPUT_FILE" ]]; then
        ros2 run axon_recorder perf_test_node $COMMON_ARGS --output "$OUTPUT_FILE"
    else
        ros2 run axon_recorder perf_test_node $COMMON_ARGS
    fi
fi

EXIT_CODE=$?

echo "============================================"
if [[ $EXIT_CODE -eq 0 ]]; then
    echo "  Performance Test: PASSED"
else
    echo "  Performance Test: FAILED"
fi
echo "============================================"

exit $EXIT_CODE
