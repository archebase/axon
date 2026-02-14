# =============================================================================
# Axon ROS1 Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(axon_ros1_plugin VERSION 1.0.0 LANGUAGES CXX)

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
    message(STATUS "ROS1 Plugin: Depth compression ENABLED")
else()
    message(STATUS "ROS1 Plugin: Depth compression DISABLED")
endif()

# =============================================================================
# Find ROS1
# =============================================================================

# Auto-detect ROS1 installation from environment
# Priority: CMake variable > Environment variable > Default
if(NOT DEFINED ROS1_DIR OR ROS1_DIR STREQUAL "")
    if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
        set(ROS1_DIR
            "/opt/ros/$ENV{ROS_DISTRO}"
            CACHE PATH
            "ROS1 installation directory (auto-detected from ROS_DISTRO)"
        )
        message(STATUS "ROS1: Auto-detected from ROS_DISTRO=$ENV{ROS_DISTRO}")
    else()
        set(ROS1_DIR "/opt/ros/noetic" CACHE PATH "ROS1 installation directory")
        message(STATUS "ROS1: Using default path (ROS_DISTRO not set)")
    endif()
endif()

if(NOT EXISTS "${ROS1_DIR}")
    message(
        FATAL_ERROR
        "ROS1 not found at ${ROS1_DIR}. "
        "Set ROS_DISTRO environment variable or pass -DROS1_DIR=/path/to/ros."
    )
endif()

# Find ROS1 CMake libraries
find_package(Boost REQUIRED COMPONENTS system)

# ROS1 include directory
set(ROS1_INCLUDE_DIR "${ROS1_DIR}/include")

# Find ROS1 libraries dynamically
find_library(ROSCPP_LIB roscpp PATHS "${ROS1_DIR}/lib" NO_DEFAULT_PATH)
find_library(ROSLIB_LIB roslib PATHS "${ROS1_DIR}/lib" NO_DEFAULT_PATH)
find_library(
    TOPIC_TOOLS_LIB
    topic_tools
    PATHS "${ROS1_DIR}/lib"
    NO_DEFAULT_PATH
)

# Check that all libraries were found
if(NOT ROSCPP_LIB OR NOT ROSLIB_LIB OR NOT TOPIC_TOOLS_LIB)
    message(FATAL_ERROR "Required ROS1 libraries not found in ${ROS1_DIR}/lib")
endif()

# =============================================================================
# Build Plugin Library
# =============================================================================

# Core sources (always built)
set(CORE_SOURCES
    src/ros1_plugin.cpp
    src/ros1_subscription_wrapper.cpp
    src/ros1_plugin_export.cpp
)

# Depth compression sources (conditional)
set(DEPTH_SOURCES "")
if(AXON_ENABLE_DEPTH_COMPRESSION)
    list(APPEND DEPTH_SOURCES src/depth_compression_filter.cpp)
endif()

add_library(axon_ros1_plugin SHARED ${CORE_SOURCES} ${DEPTH_SOURCES})

target_include_directories(
    axon_ros1_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${ROS1_INCLUDE_DIR}
        ${AXON_LOGGING_DIR}/include
)

# Add filters include directory for depth compression (conditional)
if(AXON_ENABLE_DEPTH_COMPRESSION)
    target_include_directories(
        axon_ros1_plugin
        PRIVATE
            ${AXON_MIDDLEWARES_DIR}/filters/include
            ${AXON_MIDDLEWARES_DIR}/filters/depthlitez/include
    )
    target_compile_definitions(
        axon_ros1_plugin
        PRIVATE AXON_ENABLE_DEPTH_COMPRESSION
    )
endif()

# Find and link LZ4 and ZSTD for depth compression (conditional)
if(AXON_ENABLE_DEPTH_COMPRESSION)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LZ4 REQUIRED liblz4)
    pkg_check_modules(ZSTD REQUIRED libzstd)
endif()

# Link ROS1 libraries
target_link_libraries(
    axon_ros1_plugin
    PUBLIC
        ${ROSCPP_LIB}
        ${ROSLIB_LIB}
        ${TOPIC_TOOLS_LIB}
        Boost::system
        nlohmann_json::nlohmann_json
        axon_logging
)

# Link filters library for depth compression (conditional)
if(AXON_ENABLE_DEPTH_COMPRESSION)
    target_link_libraries(axon_ros1_plugin PRIVATE axon_filters)
    target_link_libraries(axon_ros1_plugin PRIVATE ${LZ4_LIBRARIES})
    target_link_libraries(axon_ros1_plugin PRIVATE ${ZSTD_LIBRARIES})
endif()

# Export all symbols for dynamic loading
target_compile_options(axon_ros1_plugin PRIVATE -fvisibility=default)

# Define AXON_ROS1 for conditional compilation
target_compile_definitions(axon_ros1_plugin PRIVATE AXON_ROS1)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_ros1_plugin)
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
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${ROS1_INCLUDE_DIR}
        )

        target_link_libraries(
            ${test_name}
            axon_ros1_plugin
            ${ROSCPP_LIB}
            ${ROSLIB_LIB}
            ${TOPIC_TOOLS_LIB}
            GTest::gtest
            GTest::gtest_main
            pthread
        )

        # Add test labels
        set_tests_properties(${test_name} PROPERTIES LABELS "unit;ros1_plugin")

        # Apply coverage flags
        if(COMMAND axon_add_coverage)
            axon_add_coverage(${test_name})
        endif()

        # Register with CTest
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# =============================================================================
# Install
# =============================================================================
install(
    TARGETS axon_ros1_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
)

install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon ROS1 Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  ROS1 Root: ${ROS1_DIR}")
message(STATUS "  Depth Compression: ${AXON_ENABLE_DEPTH_COMPRESSION}")
message(STATUS "  Tests Enabled: ${AXON_BUILD_TESTS}")
message(STATUS "")
