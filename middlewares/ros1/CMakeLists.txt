# =============================================================================
# Axon ROS1 Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(axon_ros1_plugin VERSION 1.0.0 LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Find ROS1
# =============================================================================

# Find ROS1 installation
set(ROS1_DIR "/opt/ros/noetic" CACHE PATH "ROS1 installation directory")

if(NOT EXISTS "${ROS1_DIR}")
    message(FATAL_ERROR "ROS1 not found at ${ROS1_DIR}. "
                        "Install ROS1 Noetic or set ROS1_DIR to your ROS installation.")
endif()

# Find ROS1 CMake libraries
find_package(Boost REQUIRED COMPONENTS system)

# ROS1 include directory
set(ROS1_INCLUDE_DIR "${ROS1_DIR}/include")

# Find ROS1 libraries dynamically
find_library(ROSCPP_LIB roscpp PATHS "${ROS1_DIR}/lib" NO_DEFAULT_PATH)
find_library(ROSLIB_LIB roslib PATHS "${ROS1_DIR}/lib" NO_DEFAULT_PATH)
find_library(TOPIC_TOOLS_LIB topic_tools PATHS "${ROS1_DIR}/lib" NO_DEFAULT_PATH)

# Check that all libraries were found
if(NOT ROSCPP_LIB OR NOT ROSLIB_LIB OR NOT TOPIC_TOOLS_LIB)
    message(FATAL_ERROR "Required ROS1 libraries not found in ${ROS1_DIR}/lib")
endif()

# =============================================================================
# Build the Plugin Library
# =============================================================================

add_library(axon_ros1_plugin SHARED
    src/ros1_plugin.cpp
    src/ros1_subscription_wrapper.cpp
    src/ros1_plugin_export.cpp
)

target_include_directories(axon_ros1_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ROS1_INCLUDE_DIR}
)

# Link ROS1 libraries
target_link_libraries(axon_ros1_plugin PUBLIC
    ${ROSCPP_LIB}
    ${ROSLIB_LIB}
    ${TOPIC_TOOLS_LIB}
    Boost::system
    nlohmann_json::nlohmann_json
)

# Export all symbols for dynamic loading
target_compile_options(axon_ros1_plugin PRIVATE -fvisibility=default)

# Define AXON_ROS1 for conditional compilation
target_compile_definitions(axon_ros1_plugin PRIVATE AXON_ROS1)

# =============================================================================
# Tests
# =============================================================================
option(AXON_BUILD_TESTS "Build tests" OFF)
if(AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # Discover test files automatically
    file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp")

    foreach(test_source ${TEST_SOURCES})
        get_filename_component(test_name ${test_source} NAME_WE)

        add_executable(${test_name} ${test_source})

        target_include_directories(${test_name} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${ROS1_INCLUDE_DIR}
        )

        target_link_libraries(${test_name}
            axon_ros1_plugin
            ${ROSCPP_LIB}
            ${ROSLIB_LIB}
            ${TOPIC_TOOLS_LIB}
            GTest::GTest
            pthread
        )

        # Add test labels
        set_target_properties(${test_name} PROPERTIES
            LABELS "unit;ros1_plugin"
        )

        # Register with CTest
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS axon_ros1_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon ROS1 Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  ROS1 Root: ${ROS1_DIR}")
message(STATUS "  Tests Enabled: ${AXON_BUILD_TESTS}")
message(STATUS "")
