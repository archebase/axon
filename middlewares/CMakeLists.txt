# =============================================================================
# Axon Middlewares - Plugin Build Configuration
# =============================================================================
# This file provides unified CMake configuration for all middleware plugins.
# Each plugin is built as a standalone shared library that can be loaded
# dynamically by the main axon_recorder application.
#
# Plugins:
#   - ROS1 (Noetic): libaxon_ros1_plugin.so
#   - ROS2 (Humble/Jazzy/Rolling): libaxon_ros2_plugin.so
#   - Zenoh: libaxon_zenoh_plugin.so
#   - UDP JSON: libaxon_udp.so
#
# All plugins are installed to: lib/axon/plugins/
# =============================================================================

cmake_minimum_required(VERSION 3.14)

# =============================================================================
# Path Definitions
# =============================================================================
# AXON_MIDDLEWARES_DIR is already defined by parent CMakeLists.txt
# No need to redefine it here

# Make variables available to child CMakeLists
set(AXON_MIDDLEWARES_DIR "${AXON_MIDDLEWARES_DIR}" CACHE INTERNAL "")

# =============================================================================
# Plugin Build Options
# =============================================================================

# Detect ROS environment
# Auto-detect ROS1_DIR from environment if not already set
if(NOT DEFINED ROS1_DIR OR ROS1_DIR STREQUAL "")
    if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
        set(ROS1_DIR "/opt/ros/$ENV{ROS_DISTRO}" CACHE PATH
            "ROS1 installation directory (auto-detected from ROS_DISTRO)"
        )
    else()
        set(ROS1_DIR "/opt/ros/noetic" CACHE PATH "ROS1 installation directory")
    endif()
endif()

# Auto-detect ROS2_DIR from environment if not already set
if(NOT DEFINED ROS2_DIR OR ROS2_DIR STREQUAL "")
    if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
        set(ROS2_DIR "/opt/ros/$ENV{ROS_DISTRO}" CACHE PATH
            "ROS2 installation directory (auto-detected from ROS_DISTRO)"
        )
    else()
        set(ROS2_DIR "/opt/ros/humble" CACHE PATH
            "ROS2 installation directory (humble/jazzy/rolling)"
        )
    endif()
endif()

# Auto-detect ROS1
# Check for ROS1-specific files (not just ROS_DISTRO which exists for ROS2 too)
if(
    EXISTS "${ROS1_DIR}/setup.bash"
    AND EXISTS "${ROS1_DIR}/lib/python3/dist-packages/ros"
)
    set(AXON_BUILD_ROS1_DEFAULT ON)
else()
    set(AXON_BUILD_ROS1_DEFAULT OFF)
endif()

# Auto-detect ROS2
if(
    EXISTS "${ROS2_DIR}/setup.bash"
    OR EXISTS "/opt/ros/jazzy/setup.bash"
    OR EXISTS "/opt/ros/rolling/setup.bash"
)
    set(AXON_BUILD_ROS2_DEFAULT ON)
else()
    set(AXON_BUILD_ROS2_DEFAULT OFF)
endif()

option(
    AXON_BUILD_ROS1_PLUGIN
    "Build ROS1 (Noetic) plugin"
    ${AXON_BUILD_ROS1_DEFAULT}
)
option(
    AXON_BUILD_ROS2_PLUGIN
    "Build ROS2 (Humble/Jazzy/Rolling) plugin"
    ${AXON_BUILD_ROS2_DEFAULT}
)
option(AXON_BUILD_ZENOH_PLUGIN "Build Zenoh plugin" OFF)
option(AXON_BUILD_MOCK_PLUGIN "Build Mock plugin (for testing)" OFF)
option(AXON_BUILD_UDP_PLUGIN "Build UDP JSON plugin" ON)

# =============================================================================
# Plugin Subdirectories
# =============================================================================

# ROS1 Plugin (pure CMake)
if(AXON_BUILD_ROS1_PLUGIN)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ros1/CMakeLists.txt")
        add_subdirectory(ros1 ${CMAKE_CURRENT_BINARY_DIR}/ros1_plugin)
        message(STATUS "ROS1 plugin: ENABLED")
    else()
        message(WARNING "ROS1 plugin requested but CMakeLists.txt not found")
    endif()
else()
    message(
        STATUS
        "ROS1 plugin: DISABLED (enable with -DAXON_BUILD_ROS1_PLUGIN=ON)"
    )
endif()

# ROS2 Plugin (pure CMake)
if(AXON_BUILD_ROS2_PLUGIN)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ros2/CMakeLists.txt")
        add_subdirectory(ros2 ${CMAKE_CURRENT_BINARY_DIR}/ros2_plugin)
        message(STATUS "ROS2 plugin: ENABLED")
    else()
        message(WARNING "ROS2 plugin requested but CMakeLists.txt not found")
    endif()
else()
    message(
        STATUS
        "ROS2 plugin: DISABLED (enable with -DAXON_BUILD_ROS2_PLUGIN=ON)"
    )
endif()

# Zenoh Plugin (standalone CMake, already pure CMake)
if(AXON_BUILD_ZENOH_PLUGIN)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/zenoh/CMakeLists.txt")
        add_subdirectory(zenoh ${CMAKE_CURRENT_BINARY_DIR}/zenoh_plugin)
        message(STATUS "Zenoh plugin: ENABLED")
    else()
        message(WARNING "Zenoh plugin requested but CMakeLists.txt not found")
    endif()
else()
    message(
        STATUS
        "Zenoh plugin: DISABLED (enable with -DAXON_BUILD_ZENOH_PLUGIN=ON)"
    )
endif()

# Mock Plugin (for testing, pure CMake)
if(AXON_BUILD_MOCK_PLUGIN)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mock/CMakeLists.txt")
        add_subdirectory(mock ${CMAKE_CURRENT_BINARY_DIR}/mock_plugin)
        message(STATUS "Mock plugin: ENABLED")
    else()
        message(WARNING "Mock plugin requested but CMakeLists.txt not found")
    endif()
else()
    message(
        STATUS
        "Mock plugin: DISABLED (enable with -DAXON_BUILD_MOCK_PLUGIN=ON)"
    )
endif()

# UDP Plugin (for JSON over UDP recording)
if(AXON_BUILD_UDP_PLUGIN)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/udp/CMakeLists.txt")
        add_subdirectory(udp ${CMAKE_CURRENT_BINARY_DIR}/udp_plugin)
        message(STATUS "UDP plugin: ENABLED")
    else()
        message(WARNING "UDP plugin requested but CMakeLists.txt not found")
    endif()
else()
    message(
        STATUS
        "UDP plugin: DISABLED (enable with -DAXON_BUILD_UDP_PLUGIN=ON)"
    )
endif()

# =============================================================================
# Summary
# =============================================================================

message(STATUS "")
message(STATUS "=== Axon Middlewares Configuration ===")
message(STATUS "  Middlewares Root: ${AXON_MIDDLEWARES_DIR}")
message(STATUS "  ROS1 Plugin:      ${AXON_BUILD_ROS1_PLUGIN}")
message(STATUS "  ROS2 Plugin:      ${AXON_BUILD_ROS2_PLUGIN}")
message(STATUS "  Zenoh Plugin:     ${AXON_BUILD_ZENOH_PLUGIN}")
message(STATUS "  Mock Plugin:      ${AXON_BUILD_MOCK_PLUGIN}")
message(STATUS "  UDP Plugin:       ${AXON_BUILD_UDP_PLUGIN}")
message(STATUS "  Coverage:         ${AXON_ENABLE_COVERAGE}")
message(STATUS "  Tests:            ${AXON_BUILD_TESTS}")
message(STATUS "=====================================")
message(STATUS "")
