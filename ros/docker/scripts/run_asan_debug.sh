#!/bin/bash
# =============================================================================
# ASAN Debug Build and Test Script
# =============================================================================
# This script builds the axon_recorder with Address Sanitizer enabled and runs
# a simplified test to capture detailed memory error information.
#
# Usage:
#   ./run_asan_debug.sh [--duration <seconds>] [--skip-build] [--full-load]
#
# ASAN will detect:
#   - Double-free errors
#   - Use-after-free errors  
#   - Buffer overflows (heap/stack/global)
#   - Memory leaks
#   - Use of uninitialized memory
# =============================================================================

# Don't exit on error - we want to capture ASAN output
set +e

# Configuration
DURATION=${DURATION:-10}
SKIP_BUILD=${SKIP_BUILD:-0}
FULL_LOAD=${FULL_LOAD:-0}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=1
            shift
            ;;
        --full-load)
            FULL_LOAD=1
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "╔══════════════════════════════════════════════════════════╗"
echo "║           ASAN DEBUG BUILD                               ║"
echo "╠══════════════════════════════════════════════════════════╣"
echo "║ Duration: ${DURATION}s"
echo "║ Full Load: ${FULL_LOAD}"
echo "║ ASAN_OPTIONS: ${ASAN_OPTIONS:-detect_leaks=1:halt_on_error=0}"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# Source ROS environment
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "Sourced ROS ${ROS_DISTRO}"
fi

# Verify ASAN is enabled
if [ "$ENABLE_ASAN" != "1" ]; then
    echo "WARNING: ENABLE_ASAN is not set. Setting it now..."
    export ENABLE_ASAN=1
fi

# Workspace directories
# Build from original source to preserve relative paths in CMakeLists.txt
WORKSPACE=/workspace/ros_ws
BUILD_BASE=${WORKSPACE}/build
INSTALL_BASE=${WORKSPACE}/install

mkdir -p ${BUILD_BASE}
mkdir -p ${INSTALL_BASE}

cd /workspace/axon

# Build with ASAN
if [ "$SKIP_BUILD" != "1" ]; then
    echo ""
    echo "============================================"
    echo "Building Rust Lance bridge library..."
    echo "============================================"
    
    # Build Rust library first (required by axon_recorder)
    cd /workspace/axon
    cargo build --release -p axon-lance-ffi 2>&1 | tee /tmp/asan_cargo.log
    
    if [ ! -f "target/release/liblance_writer_bridge.so" ]; then
        echo "ERROR: Rust library not found after build"
        exit 1
    fi
    echo "✓ Built Rust Lance bridge library"
    
    echo ""
    echo "============================================"
    echo "Building with Address Sanitizer..."
    echo "============================================"
    
    # Clean previous ASAN build to ensure flags are applied
    rm -rf ${BUILD_BASE} ${INSTALL_BASE}
    mkdir -p ${BUILD_BASE} ${INSTALL_BASE}
    
    # Build from ros directory to preserve CMakeLists.txt relative paths
    cd /workspace/axon/ros
    
    # Build with ASAN enabled using colcon
    # Use RelWithDebInfo for reasonable performance with debug info
    colcon build \
        --packages-select axon_recorder \
        --build-base ${BUILD_BASE} \
        --install-base ${INSTALL_BASE} \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -DENABLE_ASAN=ON \
        --event-handlers console_direct+ \
        2>&1 | tee /tmp/asan_build.log
    
    BUILD_RESULT=${PIPESTATUS[0]}
    if [ $BUILD_RESULT -ne 0 ]; then
        echo "ERROR: Build failed! Check /tmp/asan_build.log"
        cat /tmp/asan_build.log | tail -50
        exit 1
    fi
    
    echo ""
    echo "✓ Build completed successfully with ASAN enabled."
    
    cd /workspace/axon
fi

# Source workspace
source ${INSTALL_BASE}/setup.bash

echo ""
echo "============================================"
echo "Running ASAN Debug Test..."
echo "============================================"
echo ""
echo "ASAN will print detailed stack traces if memory errors are found."
echo ""

# Create test output directory
mkdir -p /data/recordings
rm -rf /data/recordings/*

# Set ASAN options for detailed output
# - new_delete_type_mismatch=0: Suppress false positives from ROS2 system libraries
#   (ROS2 libs are not built with ASAN, causing allocator mismatch errors)
# - halt_on_error=0: Don't halt, continue to capture more errors
# - detect_leaks=0: Disable leak detection (also triggers false positives with ROS2)
# - verify_asan_link_order=0: Don't verify linking order (helps with mixed ASAN/non-ASAN)
export ASAN_OPTIONS="detect_leaks=0:print_stats=0:halt_on_error=0:new_delete_type_mismatch=0:alloc_dealloc_mismatch=0:verify_asan_link_order=0:replace_intrin=0"

# Also set for child processes
echo "ASAN_OPTIONS=${ASAN_OPTIONS}"

# Determine load parameters
if [ "$FULL_LOAD" == "1" ]; then
    # Full load - matches perf test
    IMU_RATE=1000
    CAMERA_RATE=30
    NUM_CAMERAS=3
    RES_WIDTH=1920
    RES_HEIGHT=1080
    echo "Using FULL LOAD configuration (matches perf test)"
else
    # Reduced load for faster debugging
    IMU_RATE=100
    CAMERA_RATE=10
    NUM_CAMERAS=1
    RES_WIDTH=640
    RES_HEIGHT=480
    echo "Using REDUCED LOAD configuration (faster debugging)"
fi

# Start synthetic publisher in background
echo ""
echo "Starting synthetic publisher..."
echo "  IMU: ${IMU_RATE} Hz, Camera: ${CAMERA_RATE} Hz x ${NUM_CAMERAS}"
echo "  Resolution: ${RES_WIDTH}x${RES_HEIGHT}"
echo ""

ros2 run axon_recorder synthetic_publisher \
    --ros-args \
    -p imu_rate:=${IMU_RATE} \
    -p camera_rate:=${CAMERA_RATE} \
    -p num_cameras:=${NUM_CAMERAS} \
    -p resolution_width:=${RES_WIDTH} \
    -p resolution_height:=${RES_HEIGHT} \
    -p test_duration:=${DURATION} \
    2>&1 &
PUBLISHER_PID=$!

# Give publisher time to start
sleep 2

# Check if publisher started
if ! kill -0 $PUBLISHER_PID 2>/dev/null; then
    echo "ERROR: Synthetic publisher failed to start"
    exit 1
fi

echo "✓ Synthetic publisher started (PID: $PUBLISHER_PID)"

# Run recorder with ASAN
echo ""
echo "============================================"
echo "Starting axon_recorder with ASAN..."
echo "============================================"
echo ""

# Run recorder in foreground to capture all output
ros2 run axon_recorder axon_recorder_node 2>&1 | tee /tmp/asan_recorder.log &
RECORDER_PID=$!

# Wait for test duration plus buffer
echo ""
echo "Test running for ${DURATION} seconds..."
echo "(Watch for ASAN error messages above)"
echo ""

sleep $((DURATION + 5))

# Cleanup
echo ""
echo "============================================"
echo "Cleaning up..."
echo "============================================"

# Send SIGTERM first for graceful shutdown
kill -TERM $PUBLISHER_PID 2>/dev/null || true
kill -TERM $RECORDER_PID 2>/dev/null || true

sleep 2

# Force kill if still running
kill -9 $PUBLISHER_PID 2>/dev/null || true
kill -9 $RECORDER_PID 2>/dev/null || true

# Wait for processes to terminate
wait $PUBLISHER_PID 2>/dev/null || true
wait $RECORDER_PID 2>/dev/null || true

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║           ASAN TEST COMPLETE                             ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# Check if ASAN detected any issues
ASAN_ERRORS=0
if grep -q "ERROR: AddressSanitizer" /tmp/asan_recorder.log 2>/dev/null; then
    ASAN_ERRORS=1
fi
if grep -q "ERROR: LeakSanitizer" /tmp/asan_recorder.log 2>/dev/null; then
    ASAN_ERRORS=1
fi

if [ $ASAN_ERRORS -eq 1 ]; then
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║   !!! ASAN DETECTED MEMORY ERRORS !!!                    ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""
    echo "=== ASAN Error Report ==="
    echo ""
    # Extract and display ASAN error report
    grep -A 200 "ERROR: AddressSanitizer\|ERROR: LeakSanitizer" /tmp/asan_recorder.log 2>/dev/null || true
    echo ""
    echo "=== End of ASAN Report ==="
    echo ""
    echo "Full log: /tmp/asan_recorder.log"
    exit 1
else
    # Check for the double free message even if ASAN didn't catch it
    if grep -q "double free\|free():.*double" /tmp/asan_recorder.log 2>/dev/null; then
        echo "╔══════════════════════════════════════════════════════════╗"
        echo "║   !!! DOUBLE FREE DETECTED (runtime) !!!                 ║"
        echo "╚══════════════════════════════════════════════════════════╝"
        echo ""
        echo "Double free was detected but ASAN may not have full details."
        echo "This can happen if the error is in a library not built with ASAN."
        echo ""
        grep -B5 -A10 "double free\|free():.*double" /tmp/asan_recorder.log 2>/dev/null || true
        echo ""
        echo "Full log: /tmp/asan_recorder.log"
        exit 1
    fi
    
    echo "✓ No ASAN errors detected in this run."
    echo ""
    echo "Note: If you saw 'double free' in the output but ASAN didn't report it,"
    echo "the error may be in a library not compiled with ASAN (e.g., Arrow, ROS2)."
    echo ""
    echo "Full log: /tmp/asan_recorder.log"
fi

