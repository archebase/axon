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

set(AXON_ROS1 TRUE)
set(AXON_ROS_VERSION 1)
message(STATUS "Building tests with ROS 1 integration")

find_package(GTest REQUIRED)
include(GoogleTest)
find_package(Threads REQUIRED)

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
# ROS 1 Test Support
# =============================================================================
macro(axon_add_ros1_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    add_executable(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE AXON_ROS1)
    target_include_directories(${TEST_NAME} PRIVATE ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${TEST_NAME} GTest::gtest GTest::gtest_main ${TEST_LIBS})
    axon_add_coverage(${TEST_NAME})
    axon_set_test_rpath(${TEST_NAME})
    add_dependencies(${TEST_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT ${AXON_TEST_TIMEOUT} LABELS "integration")
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

file(GLOB _UNIT_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/unit/test_*.cpp")
foreach(_TEST_SOURCE ${_UNIT_TEST_SOURCES})
    get_filename_component(_TEST_NAME ${_TEST_SOURCE} NAME_WE)
    add_executable(${_TEST_NAME} ${_TEST_SOURCE})
    target_compile_definitions(${_TEST_NAME} PRIVATE AXON_ROS1)
    axon_add_boost_log_dyn_link(${_TEST_NAME})
    axon_add_coverage(${_TEST_NAME})
    target_include_directories(${_TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${_TEST_NAME} GTest::gtest GTest::gtest_main Threads::Threads axon_logging axon_recorder_test_support)
    axon_set_test_rpath(${_TEST_NAME})
    gtest_discover_tests(${_TEST_NAME} PROPERTIES LABELS "unit" TIMEOUT ${AXON_TEST_TIMEOUT})
endforeach()

# =============================================================================
# Integration Tests
# =============================================================================
set(ROS1_TEST_LIBS GTest::gtest GTest::gtest_main Threads::Threads ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# MCAP test support library
add_library(axon_recorder_test_support_mcap STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
)
target_include_directories(axon_recorder_test_support_mcap PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_BINARY_DIR}/../generated)
target_link_libraries(axon_recorder_test_support_mcap axon_mcap axon_logging nlohmann_json::nlohmann_json OpenSSL::SSL OpenSSL::Crypto ${Boost_LIBRARIES})

# Unit tests
axon_add_ros1_test(NAME test_recording_workflow SOURCES integration/test_recording_workflow.cpp LIBS axon_recorder_test_support)
axon_add_ros1_test(NAME test_recording_session SOURCES integration/test_recording_session.cpp LIBS axon_recorder_test_support_mcap)
axon_add_ros1_test(NAME test_metadata_injector SOURCES integration/test_metadata_injector.cpp LIBS axon_recorder_test_support_mcap)

# ROS 1 specific tests
set(ROS1_INT_LIBS axon_recorder_lib Threads::Threads ${Boost_LIBRARIES} ${catkin_LIBRARIES} GTest::gtest GTest::gtest_main)

axon_add_ros1_test(NAME test_ros_introspection_ros1 SOURCES integration/test_ros_introspection.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/ros_introspection.cpp LIBS ${ROS1_TEST_LIBS})
axon_add_ros1_test(NAME test_message_factory_ros1 SOURCES integration/test_message_factory.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/message_factory.cpp LIBS ${ROS1_TEST_LIBS})
axon_add_ros1_test(NAME test_service_adapter_ros1 SOURCES integration/test_service_adapter.cpp LIBS ${ROS1_INT_LIBS})
axon_add_ros1_test(NAME test_ros_interface_ros1 SOURCES integration/test_ros_interface.cpp LIBS ${ROS1_INT_LIBS})
axon_add_ros1_test(NAME test_recorder_node_lifecycle_ros1 SOURCES integration/test_recorder_node_lifecycle.cpp LIBS ${ROS1_INT_LIBS})
axon_add_ros1_test(NAME test_recorder_node_recording_ros1 SOURCES integration/test_recorder_node_recording.cpp LIBS ${ROS1_INT_LIBS})
axon_add_ros1_test(NAME test_recording_services_ros1 SOURCES integration/test_recording_services.cpp LIBS ${ROS1_INT_LIBS})

# Performance tests (optional)
add_subdirectory(perf EXCLUDE_FROM_ALL)

add_custom_target(run_recorder_tests COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)
