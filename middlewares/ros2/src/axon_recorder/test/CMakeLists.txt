cmake_minimum_required(VERSION 3.15)

if(NOT DEFINED PROJECT_ROOT)
    message(FATAL_ERROR "PROJECT_ROOT not set")
endif()
if(NOT DEFINED CPP_LOGGING_DIR)
    message(FATAL_ERROR "CPP_LOGGING_DIR not set")
endif()
if(NOT DEFINED CPP_MCAP_DIR)
    message(FATAL_ERROR "CPP_MCAP_DIR not set")
endif()

if(NOT DEFINED AXON_CMAKE_MODULES_DIR)
    set(AXON_CMAKE_MODULES_DIR "${PROJECT_ROOT}/cmake/Modules")
endif()

list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
include(AxonStdlib REQUIRED)
include(AxonCoverage REQUIRED)

set(AXON_ROS2 TRUE)
set(AXON_ROS_VERSION 2)
message(STATUS "Building tests with ROS 2 integration")

# Note: ament_add_gtest handles GTest discovery automatically
# We don't call find_package(GTest) here to avoid conflicts
find_package(ament_cmake_gtest REQUIRED)

if(NOT TARGET axon_logging)
    add_subdirectory(${CPP_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
endif()

enable_testing()
set(AXON_TEST_TIMEOUT 60 CACHE STRING "Default test timeout in seconds")

# =============================================================================
# Helper: Set RPATH for test executables
# =============================================================================
# This ensures tests can find shared libraries (like axon_logging) in the build tree
macro(axon_set_test_rpath TARGET)
    # Add build directory to RPATH so tests find libraries before install
    set_target_properties(${TARGET} PROPERTIES
        BUILD_RPATH "${CMAKE_BINARY_DIR}/axon_mcap:${CMAKE_BINARY_DIR}/axon_logging"
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    )
endmacro()

# =============================================================================
# ROS 2 Test Support
# =============================================================================
macro(axon_add_ros2_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCES} ${TEST_OPTIONS})
    target_compile_definitions(${TEST_NAME} PRIVATE AXON_ROS2)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${TEST_NAME} ${TEST_LIBS})
    axon_add_coverage(${TEST_NAME})
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${TEST_NAME} "${cpp_typesupport_target}")
    set_tests_properties(${TEST_NAME} PROPERTIES LABELS "integration" TIMEOUT ${AXON_TEST_TIMEOUT})
endmacro()

# =============================================================================
# Unit Tests
# =============================================================================
# Test support libraries (must be defined before unit tests)
add_library(axon_recorder_test_support STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/config_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/topic_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/worker_thread_pool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_service_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/logging/axon_ros_sink.cpp
)
target_include_directories(axon_recorder_test_support PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(axon_recorder_test_support axon_logging yaml-cpp OpenSSL::SSL OpenSSL::Crypto Threads::Threads ${Boost_LIBRARIES})

# For ROS 2, we rely on ament_add_gtest to auto-discover tests
# or we can explicitly list each test file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/unit")
    file(GLOB _UNIT_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/unit/test_*.cpp")
    foreach(_TEST_SOURCE ${_UNIT_TEST_SOURCES})
        get_filename_component(_TEST_NAME ${_TEST_SOURCE} NAME_WE)
        ament_add_gtest(${_TEST_NAME} ${_TEST_SOURCE})
        target_compile_definitions(${_TEST_NAME} PRIVATE AXON_ROS2)
        target_include_directories(${_TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
        target_link_libraries(${_TEST_NAME} Threads::Threads axon_logging axon_recorder_test_support)
        axon_add_coverage(${_TEST_NAME})
        axon_set_test_rpath(${_TEST_NAME})
    endforeach()
endif()

# =============================================================================
# Integration Tests
# =============================================================================
set(ROS2_TEST_LIBS Threads::Threads rclcpp::rclcpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS})

# MCAP test support library
add_library(axon_recorder_test_support_mcap STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
)
target_include_directories(axon_recorder_test_support_mcap PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_BINARY_DIR}/../generated)
target_link_libraries(axon_recorder_test_support_mcap axon_mcap axon_logging nlohmann_json::nlohmann_json OpenSSL::SSL OpenSSL::Crypto ${Boost_LIBRARIES})

# Unit tests
ament_add_gtest(test_recording_workflow integration/test_recording_workflow.cpp)
target_compile_definitions(test_recording_workflow PRIVATE AXON_ROS2)
target_include_directories(test_recording_workflow PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(test_recording_workflow axon_recorder_test_support)
axon_add_coverage(test_recording_workflow)
axon_set_test_rpath(test_recording_workflow)

ament_add_gtest(test_recording_session integration/test_recording_session.cpp)
target_compile_definitions(test_recording_session PRIVATE AXON_ROS2)
target_include_directories(test_recording_session PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(test_recording_session axon_recorder_test_support_mcap)
axon_add_coverage(test_recording_session)
axon_set_test_rpath(test_recording_session)

ament_add_gtest(test_metadata_injector integration/test_metadata_injector.cpp)
target_compile_definitions(test_metadata_injector PRIVATE AXON_ROS2)
target_include_directories(test_metadata_injector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(test_metadata_injector axon_recorder_test_support_mcap)
axon_add_coverage(test_metadata_injector)
axon_set_test_rpath(test_metadata_injector)

# ROS 2 specific tests
set(ROS2_INT_LIBS axon_recorder_lib Threads::Threads ${Boost_LIBRARIES} rclcpp::rclcpp ${std_msgs_TARGETS})

ament_add_gtest(test_ros_introspection_ros2 integration/test_ros_introspection.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/ros_introspection.cpp)
target_compile_definitions(test_ros_introspection_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_ros_introspection_ros2 ${ROS2_TEST_LIBS})
axon_add_coverage(test_ros_introspection_ros2)
axon_set_test_rpath(test_ros_introspection_ros2)

ament_add_gtest(test_message_factory_ros2 integration/test_message_factory.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/message_factory.cpp)
target_compile_definitions(test_message_factory_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_message_factory_ros2 ${ROS2_TEST_LIBS})
axon_add_coverage(test_message_factory_ros2)
axon_set_test_rpath(test_message_factory_ros2)

ament_add_gtest(test_service_adapter_ros2 integration/test_service_adapter.cpp)
target_compile_definitions(test_service_adapter_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_service_adapter_ros2 ${ROS2_INT_LIBS})
axon_add_coverage(test_service_adapter_ros2)
axon_set_test_rpath(test_service_adapter_ros2)

ament_add_gtest(test_ros_interface_ros2 integration/test_ros_interface.cpp)
target_compile_definitions(test_ros_interface_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_ros_interface_ros2 ${ROS2_INT_LIBS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})
axon_add_coverage(test_ros_interface_ros2)
axon_set_test_rpath(test_ros_interface_ros2)

ament_add_gtest(test_recorder_node_lifecycle_ros2 integration/test_recorder_node_lifecycle.cpp)
target_compile_definitions(test_recorder_node_lifecycle_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_recorder_node_lifecycle_ros2 ${ROS2_INT_LIBS})
axon_add_coverage(test_recorder_node_lifecycle_ros2)
axon_set_test_rpath(test_recorder_node_lifecycle_ros2)

ament_add_gtest(test_recorder_node_recording_ros2 integration/test_recorder_node_recording.cpp)
target_compile_definitions(test_recorder_node_recording_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_recorder_node_recording_ros2 ${ROS2_INT_LIBS})
axon_add_coverage(test_recorder_node_recording_ros2)
axon_set_test_rpath(test_recorder_node_recording_ros2)

ament_add_gtest(test_recording_services_ros2 integration/test_recording_services.cpp)
target_compile_definitions(test_recording_services_ros2 PRIVATE AXON_ROS2)
target_link_libraries(test_recording_services_ros2 ${ROS2_INT_LIBS})
axon_add_coverage(test_recording_services_ros2)
axon_set_test_rpath(test_recording_services_ros2)

# Performance tests (optional)
add_subdirectory(perf EXCLUDE_FROM_ALL)

add_custom_target(run_recorder_tests COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)
