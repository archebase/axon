# =============================================================================
# Axon ROS2 Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(axon_ros2_plugin VERSION 1.0.0 LANGUAGES CXX)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Options
# =============================================================================
option(
    AXON_ENABLE_DEPTH_COMPRESSION
    "Enable depth image compression (requires private DepthLiteZ)"
    OFF
)

if(AXON_ENABLE_DEPTH_COMPRESSION)
    message(STATUS "ROS2 Plugin: Depth compression ENABLED")
else()
    message(STATUS "ROS2 Plugin: Depth compression DISABLED")
endif()

# =============================================================================
# Find ROS2
# =============================================================================

# Auto-detect ROS2 installation from environment
# Priority: CMake variable > Environment variable > Default
if(NOT DEFINED ROS2_DIR OR ROS2_DIR STREQUAL "")
    if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
        set(ROS2_DIR
            "/opt/ros/$ENV{ROS_DISTRO}"
            CACHE PATH
            "ROS2 installation directory (auto-detected from ROS_DISTRO)"
        )
        message(STATUS "ROS2: Auto-detected from ROS_DISTRO=$ENV{ROS_DISTRO}")
    else()
        set(ROS2_DIR "/opt/ros/humble" CACHE PATH "ROS2 installation directory")
        message(STATUS "ROS2: Using default path (ROS_DISTRO not set)")
    endif()
endif()

if(NOT EXISTS "${ROS2_DIR}")
    message(
        FATAL_ERROR
        "ROS2 not found at ${ROS2_DIR}. "
        "Set ROS_DISTRO environment variable or pass -DROS2_DIR=/path/to/ros."
    )
endif()

# Find ROS2 libraries
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# ROS2 include directory
set(ROS2_INCLUDE_DIR "${ROS2_DIR}/include")

# =============================================================================
# Build Plugin Library
# =============================================================================

# Core sources (always built)
set(CORE_SOURCES
    src/ros2_plugin.cpp
    src/ros2_subscription_wrapper.cpp
    src/ros2_plugin_export.cpp
)

# Depth compression sources (conditional)
set(DEPTH_SOURCES "")
if(AXON_ENABLE_DEPTH_COMPRESSION)
    list(APPEND DEPTH_SOURCES src/depth_compression_filter.cpp)
endif()

add_library(axon_ros2_plugin SHARED ${CORE_SOURCES} ${DEPTH_SOURCES})

target_include_directories(
    axon_ros2_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${ROS2_INCLUDE_DIR}
        ${AXON_LOGGING_DIR}/include
)

# Add filters include directory for depth compression (conditional)
if(AXON_ENABLE_DEPTH_COMPRESSION)
    target_include_directories(
        axon_ros2_plugin
        PRIVATE
            ${AXON_MIDDLEWARES_DIR}/filters/include
            ${AXON_MIDDLEWARES_DIR}/filters/depthlitez/include
    )
    target_compile_definitions(
        axon_ros2_plugin
        PRIVATE AXON_ENABLE_DEPTH_COMPRESSION
    )
endif()

# Set library properties
set_target_properties(
    axon_ros2_plugin
    PROPERTIES VERSION 1.0.0 SOVERSION 1 VISIBILITY_INLINES_HIDDEN ON
)

# Link ROS2 libraries
target_link_libraries(
    axon_ros2_plugin
    PUBLIC
        rclcpp::rclcpp
        Boost::system
        nlohmann_json::nlohmann_json
        axon_logging
)

# Link filters library for depth compression (conditional)
if(AXON_ENABLE_DEPTH_COMPRESSION)
    target_link_libraries(axon_ros2_plugin PRIVATE axon_filters)
endif()

# Export all symbols for dynamic loading
target_compile_options(axon_ros2_plugin PRIVATE -fvisibility=default)

# Define AXON_ROS2 for conditional compilation
target_compile_definitions(axon_ros2_plugin PRIVATE AXON_ROS2)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_ros2_plugin)
endif()

# =============================================================================
# Tests
# =============================================================================
option(AXON_BUILD_TESTS "Build tests" OFF)

if(AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # Discover test files automatically
    file(
        GLOB TEST_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp"
    )

    foreach(test_source ${TEST_SOURCES})
        get_filename_component(test_name ${test_source} NAME_WE)

        add_executable(${test_name} ${test_source})

        target_include_directories(
            ${test_name}
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${ROS2_INCLUDE_DIR}
        )

        target_link_libraries(
            ${test_name}
            axon_ros2_plugin
            rclcpp::rclcpp
            GTest::gtest
            GTest::gtest_main
            pthread
        )

        # Add test labels
        set_target_properties(${test_name} PROPERTIES LABELS "unit;ros2_plugin")

        # Apply coverage flags
        if(COMMAND axon_add_coverage)
            axon_add_coverage(${test_name})
        endif()

        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# =============================================================================
# Install
# =============================================================================
install(
    TARGETS axon_ros2_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
)

install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon ROS2 Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  ROS2 Root: ${ROS2_DIR}")
message(STATUS "  Depth Compression: ${AXON_ENABLE_DEPTH_COMPRESSION}")
message(STATUS "  Tests Enabled: ${AXON_BUILD_TESTS}")
message(STATUS "")
