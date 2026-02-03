# =============================================================================
# Axon ROS2 Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(axon_ros2_plugin VERSION 1.0.0 LANGUAGES CXX)

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
# Options
# =============================================================================
option(AXON_ENABLE_DEPTH_COMPRESSION "Enable depth image compression (requires private DepthLiteZ)" OFF)

if(AXON_ENABLE_DEPTH_COMPRESSION)
    message(STATUS "ROS2 Plugin: Depth compression ENABLED")
else()
    message(STATUS "ROS2 Plugin: Depth compression DISABLED")
endif()

# =============================================================================
# Find ROS2
# =============================================================================

# Try to detect ROS2 automatically
set(ROS2_DIR "/opt/ros/humble" CACHE PATH "ROS2 installation directory")

# Try alternative ROS2 distributions if humble not found
if(NOT EXISTS "${ROS2_DIR}")
    if(EXISTS "/opt/ros/jazzy")
        set(ROS2_DIR "/opt/ros/jazzy" CACHE PATH "ROS2 installation directory" FORCE)
    elseif(EXISTS "/opt/ros/rolling")
        set(ROS2_DIR "/opt/ros/rolling" CACHE PATH "ROS2 installation directory" FORCE)
    endif()
endif()

if(NOT EXISTS "${ROS2_DIR}")
    message(FATAL_ERROR "ROS2 not found. "
                        "Install ROS2 (Humble/Jazzy/Rolling) or set ROS2_DIR to your ROS installation.")
endif()

# ROS2 library and include directories
set(ROS2_LIB_DIR "${ROS2_DIR}/lib")
set(ROS2_INCLUDE_DIR "${ROS2_DIR}/include")

# Find ROS2 CMake config files
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${ROS2_DIR}")

# Find required ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Check for required ROS2 libraries
set(ROS2_REQUIRED_LIBS
    "${ROS2_LIB_DIR}/librclcpp.so"
)
foreach(lib ${ROS2_REQUIRED_LIBS})
    if(NOT EXISTS "${lib}")
        message(FATAL_ERROR "ROS2 library not found: ${lib}")
    endif()
endforeach()

# =============================================================================
# Build the Plugin Library
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

add_library(axon_ros2_plugin SHARED
    ${CORE_SOURCES}
    ${DEPTH_SOURCES}
)

target_include_directories(axon_ros2_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ROS2_INCLUDE_DIR}
)

# Link ROS2 libraries
target_link_libraries(axon_ros2_plugin PUBLIC
    rclcpp::rclcpp
    nlohmann_json::nlohmann_json
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

# Link filters library for depth compression (conditional)
if(AXON_ENABLE_DEPTH_COMPRESSION)
    if(TARGET axon::filters)
        target_link_libraries(axon_ros2_plugin PRIVATE axon::filters)
        target_include_directories(axon_ros2_plugin PRIVATE
            ${AXON_MIDDLEWARES_DIR}/filters/include
            ${AXON_MIDDLEWARES_DIR}/filters/depthlitez/include
        )
        target_compile_definitions(axon_ros2_plugin PRIVATE AXON_ENABLE_DEPTH_COMPRESSION)
    else()
        message(FATAL_ERROR
            "AXON_ENABLE_DEPTH_COMPRESSION is ON but axon::filters target not found. "
            "Build filters first with AXON_ENABLE_DEPTH_COMPRESSION=ON."
        )
    endif()
endif()

# Export all symbols for dynamic loading
target_compile_options(axon_ros2_plugin PRIVATE -fvisibility=default)

# Define AXON_ROS2 for conditional compilation
target_compile_definitions(axon_ros2_plugin PRIVATE AXON_ROS2)

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

        target_link_libraries(${test_name}
            axon_ros2_plugin
            rclcpp::rclcpp
            GTest::GTest
        )

        # Use ament_target_dependencies for ROS2 dependencies in tests
        ament_target_dependencies(${test_name}
            rclcpp
            sensor_msgs
            std_msgs
        )

        # Add filters include directory for tests (conditional)
        if(AXON_ENABLE_DEPTH_COMPRESSION AND TARGET axon::filters)
            target_include_directories(${test_name} PRIVATE
                ${AXON_MIDDLEWARES_DIR}/filters/include
                ${AXON_MIDDLEWARES_DIR}/filters/depthlitez/include
            )
        endif()

        # Add test labels
        set_target_properties(${test_name} PROPERTIES
            LABELS "unit;ros2_plugin"
        )

        # Register with CTest
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS axon_ros2_plugin
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
message(STATUS "Axon ROS2 Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  ROS2 Root: ${ROS2_DIR}")
message(STATUS "  Depth Compression: ${AXON_ENABLE_DEPTH_COMPRESSION}")
message(STATUS "  Tests Enabled: ${AXON_BUILD_TESTS}")
message(STATUS "")
