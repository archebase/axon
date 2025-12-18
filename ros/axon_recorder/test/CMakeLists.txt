cmake_minimum_required(VERSION 3.15)

# ============================================================================
# Axon Recorder Tests
# ============================================================================
# Note: Core C++ library tests (arrow_builder, batch_manager, config_parser)
# have been moved to cpp/axon_arrow/test/ and are run via cpp/Makefile.
#
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

# Add integration test for Rust bridge
add_test(
    NAME RustBridgeIntegrationTest
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/integration/test_rust_bridge.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set_tests_properties(RustBridgeIntegrationTest
    PROPERTIES
        TIMEOUT 60
        LABELS "integration"
)

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
