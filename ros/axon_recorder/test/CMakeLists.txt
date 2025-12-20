cmake_minimum_required(VERSION 3.15)

# ============================================================================
# Axon Recorder Tests
# ============================================================================
# This CMakeLists.txt is for ROS-specific integration tests only.
# ============================================================================

# Find GoogleTest (for any future ROS-specific unit tests)
find_package(GTest QUIET)
find_package(Threads REQUIRED)

enable_testing()

# ============================================================================
# Integration Tests
# ============================================================================
# Integration tests are shell scripts that test the full system
# Note: Rust bridge tests removed as the Rust/C FFI bridge is no longer used

# ============================================================================
# Performance Tests
# ============================================================================
# Performance tests for measuring recording throughput and resource usage

add_subdirectory(perf)

# ============================================================================
# Future: ROS-specific unit tests can be added here
# ============================================================================
# Example:
# if(GTest_FOUND)
#     add_executable(test_ros_interface test_ros_interface.cpp)
#     target_link_libraries(test_ros_interface
#         axon_recorder_lib
#         GTest::gtest
#         GTest::gtest_main
#         Threads::Threads
#     )
#     add_test(NAME RosInterfaceTest COMMAND test_ros_interface)
# endif()
