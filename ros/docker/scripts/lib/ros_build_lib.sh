#!/bin/bash
# =============================================================================
# ROS Build Library
# =============================================================================
# Shared functions for building axon_recorder package in Docker.
# Handles both ROS 1 (catkin) and ROS 2 (colcon) build systems.
#
# Functions:
#   ros_build_package - Build axon_recorder package
# =============================================================================

set -eo pipefail

# =============================================================================
# Helper Functions
# =============================================================================

ros_build_log() {
    local level="$1"
    shift
    echo "[ros_build] $level: $*" >&2
}

ros_build_error() {
    ros_build_log "ERROR" "$@"
    exit 1
}

# =============================================================================
# Build Package
# =============================================================================
#
# Builds the axon_recorder package using the appropriate build system.
#
# Arguments:
#   ROS_VERSION - ROS version (1 or 2)
#   ROS_DISTRO  - ROS distribution (e.g., noetic, humble)
#   WORKSPACE_ROOT - Root of the workspace (default: /workspace/axon)
#   CLEAN_BUILD - If set to "true", clean build artifacts first (default: false)
#   ENABLE_COVERAGE - If set to "true", enable coverage instrumentation (default: false)
#   BUILD_TYPE - CMake build type (default: Release, or Debug if coverage enabled)
#   EXTRA_CMAKE_ARGS - Extra CMake arguments (optional, e.g., "-DENABLE_ASAN=ON")
#
# Environment:
#   Sets WORKSPACE_BUILD_DIR - Directory where build artifacts are located
#   Sets WORKSPACE_INSTALL_DIR - Directory where install artifacts are located
#   Sources the workspace setup file after building
#
ros_build_package() {
    local ros_version="${ROS_VERSION:-2}"
    local ros_distro="${ROS_DISTRO:-humble}"
    local workspace_root="${1:-/workspace/axon}"
    local clean_build="${2:-false}"
    local enable_coverage="${3:-false}"
    local build_type="${4:-}"
    local extra_cmake_args="${5:-}"
    
    # Determine build type
    if [ -z "$build_type" ]; then
        if [ "$enable_coverage" = "true" ]; then
            build_type="Debug"
        else
            build_type="Release"
        fi
    fi
    
    ros_build_log "INFO" "Building axon_recorder package"
    ros_build_log "INFO" "  ROS_VERSION: $ros_version"
    ros_build_log "INFO" "  ROS_DISTRO: $ros_distro"
    ros_build_log "INFO" "  Workspace: $workspace_root"
    ros_build_log "INFO" "  Clean build: $clean_build"
    ros_build_log "INFO" "  Coverage: $enable_coverage"
    ros_build_log "INFO" "  Build type: $build_type"
    
    # Source ROS base environment
    if [ ! -f "/opt/ros/${ros_distro}/setup.bash" ]; then
        ros_build_error "ROS setup.bash not found at /opt/ros/${ros_distro}/setup.bash"
    fi
    source /opt/ros/${ros_distro}/setup.bash
    
    if [ "$ros_version" = "1" ]; then
        # ROS 1 - Use catkin build
        ros_build_log "INFO" "Building with catkin (ROS 1)..."
        
        local catkin_ws="${workspace_root}/ros"
        if [ ! -d "$catkin_ws" ]; then
            ros_build_error "Catkin workspace not found at $catkin_ws"
        fi
        
        cd "$catkin_ws"
        
        # Clean if requested
        if [ "$clean_build" = "true" ]; then
            ros_build_log "INFO" "Cleaning previous build artifacts..."
            rm -rf build devel install log logs
        fi
        
        # Build with catkin
        local catkin_args=(
            --no-notify
            -DCMAKE_BUILD_TYPE="$build_type"
        )
        
        if [ "$enable_coverage" = "true" ]; then
            catkin_args+=(-DENABLE_COVERAGE=ON)
        fi
        
        # Add extra CMake args if provided
        if [ -n "$extra_cmake_args" ]; then
            catkin_args+=($extra_cmake_args)
        fi
        
        ros_build_log "INFO" "Running: catkin build ${catkin_args[*]}"
        catkin build "${catkin_args[@]}" || {
            ros_build_error "catkin build failed"
        }
        
        # Set workspace directories
        export WORKSPACE_BUILD_DIR="${PWD}/build/axon_recorder"
        export WORKSPACE_INSTALL_DIR="${PWD}/devel"
        
        # Source workspace
        if [ -f "${WORKSPACE_INSTALL_DIR}/setup.bash" ]; then
            source "${WORKSPACE_INSTALL_DIR}/setup.bash"
            ros_build_log "INFO" "Sourced workspace: ${WORKSPACE_INSTALL_DIR}/setup.bash"
        else
            ros_build_error "Workspace setup.bash not found at ${WORKSPACE_INSTALL_DIR}/setup.bash"
        fi
        
    else
        # ROS 2 - Use colcon
        ros_build_log "INFO" "Building with colcon (ROS 2)..."
        
        if [ ! -d "$workspace_root" ]; then
            ros_build_error "Workspace root not found at $workspace_root"
        fi
        
        # For ROS 2, build from the ros subdirectory if it exists (matches original behavior)
        # This ensures build artifacts are in the same location as before
        local build_dir="$workspace_root"
        if [ -d "${workspace_root}/ros" ]; then
            build_dir="${workspace_root}/ros"
        fi
        
        cd "$build_dir"
        
        # Clean if requested
        if [ "$clean_build" = "true" ]; then
            ros_build_log "INFO" "Cleaning previous build artifacts..."
            rm -rf build devel install log logs
        fi
        
        # Build with colcon
        local colcon_args=(
            --packages-select axon_recorder
            --cmake-args
            -DCMAKE_BUILD_TYPE="$build_type"
        )
        
        if [ "$enable_coverage" = "true" ]; then
            colcon_args+=(-DENABLE_COVERAGE=ON)
        fi
        
        # Add extra CMake args if provided
        if [ -n "$extra_cmake_args" ]; then
            colcon_args+=($extra_cmake_args)
        fi
        
        ros_build_log "INFO" "Running: colcon build ${colcon_args[*]}"
        colcon build "${colcon_args[@]}" || {
            ros_build_error "colcon build failed"
        }
        
        # Set workspace directories (build artifacts are in current directory)
        export WORKSPACE_BUILD_DIR="${PWD}/build/axon_recorder"
        export WORKSPACE_INSTALL_DIR="${PWD}/install"
        
        # Source workspace
        if [ -f "${WORKSPACE_INSTALL_DIR}/setup.bash" ]; then
            source "${WORKSPACE_INSTALL_DIR}/setup.bash"
            ros_build_log "INFO" "Sourced workspace: ${WORKSPACE_INSTALL_DIR}/setup.bash"
            
            # For ROS 2: Ensure workspace is in AMENT_PREFIX_PATH
            export AMENT_PREFIX_PATH="${WORKSPACE_INSTALL_DIR}/axon_recorder:${WORKSPACE_INSTALL_DIR}:${AMENT_PREFIX_PATH}"
        else
            ros_build_error "Workspace setup.bash not found at ${WORKSPACE_INSTALL_DIR}/setup.bash"
        fi
    fi
    
    ros_build_log "INFO" "Build completed successfully"
    ros_build_log "INFO" "  Build directory: ${WORKSPACE_BUILD_DIR}"
    ros_build_log "INFO" "  Install directory: ${WORKSPACE_INSTALL_DIR}"
}
