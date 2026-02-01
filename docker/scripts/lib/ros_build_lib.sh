#!/bin/bash
# =============================================================================
# ROS Build Library
# =============================================================================
# Shared functions for building axon_recorder package in Docker.
# Uses unified CMake build system for all components.
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
# Builds the axon_recorder package using unified CMake build system.
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

    if [ ! -d "$workspace_root" ]; then
        ros_build_error "Workspace root not found at $workspace_root"
    fi

    cd "$workspace_root"

    # Build directory
    local build_dir="${workspace_root}/build"

    # Clean if requested
    if [ "$clean_build" = "true" ]; then
        ros_build_log "INFO" "Cleaning previous build artifacts..."
        rm -rf "$build_dir"

        # Also clean C++ library build artifacts in core directories
        if [ -d "${workspace_root}/core" ]; then
            ros_build_log "INFO" "Cleaning C++ library build artifacts..."
            find "${workspace_root}/core" -type f -name "CMakeCache.txt" -delete 2>/dev/null || true
            find "${workspace_root}/core" -type d -name "CMakeFiles" -exec rm -rf {} + 2>/dev/null || true
            find "${workspace_root}/core" -type f -name "*.gcda" -delete 2>/dev/null || true
            find "${workspace_root}/core" -type f -name "*.gcno" -delete 2>/dev/null || true
        fi

        # Clean coverage data files
        find "${workspace_root}" -type f -name "*.gcda" -delete 2>/dev/null || true
        find "${workspace_root}" -type f -name "*.gcno" -delete 2>/dev/null || true
    fi

    # Create build directory
    mkdir -p "$build_dir"
    cd "$build_dir"

    # Configure CMake
    local cmake_args=(
        -DCMAKE_BUILD_TYPE="$build_type"
        -DAXON_BUILD_TESTS=ON
    )

    if [ "$enable_coverage" = "true" ]; then
        cmake_args+=(-DAXON_ENABLE_COVERAGE=ON)
    else
        cmake_args+=(-DAXON_ENABLE_COVERAGE=OFF)
    fi

    # Add ROS plugin based on ROS version
    if [ "$ros_version" = "1" ]; then
        cmake_args+=(-DAXON_BUILD_ROS1_PLUGIN=ON)
        cmake_args+=(-DAXON_BUILD_ROS2_PLUGIN=OFF)
        ros_build_log "INFO" "Building ROS1 plugin..."
    else
        cmake_args+=(-DAXON_BUILD_ROS1_PLUGIN=OFF)
        cmake_args+=(-DAXON_BUILD_ROS2_PLUGIN=ON)
        ros_build_log "INFO" "Building ROS2 plugin..."
    fi

    cmake_args+=(-DAXON_BUILD_ZENOH_PLUGIN=OFF)

    # Add extra CMake args if provided
    if [ -n "$extra_cmake_args" ]; then
        cmake_args+=($extra_cmake_args)
    fi

    ros_build_log "INFO" "Running: cmake ${cmake_args[*]} .."
    cmake "${cmake_args[@]}" .. || {
        ros_build_error "CMake configure failed"
    }

    # Build
    ros_build_log "INFO" "Building..."
    cmake --build . -j$(nproc) || {
        ros_build_error "Build failed"
    }

    # Set workspace directories
    export WORKSPACE_BUILD_DIR="$build_dir"
    export WORKSPACE_INSTALL_DIR="$build_dir"

    # For ROS2, set AMENT_PREFIX_PATH
    if [ "$ros_version" = "2" ]; then
        export AMENT_PREFIX_PATH="${build_dir}:${AMENT_PREFIX_PATH}"
    fi

    ros_build_log "INFO" "Build completed successfully"
    ros_build_log "INFO" "  Build directory: ${WORKSPACE_BUILD_DIR}"
    ros_build_log "INFO" "  Install directory: ${WORKSPACE_INSTALL_DIR}"
}
