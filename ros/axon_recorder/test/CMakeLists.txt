cmake_minimum_required(VERSION 3.15)

# ============================================================================
# Axon Recorder Tests
# ============================================================================
# Supports two modes:
# 1. Standalone build (cmake ..) - Unit tests only
# 2. ROS-integrated build (via main CMakeLists.txt) - Full tests with services
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect build mode
if(DEFINED AXON_ROS1 OR DEFINED AXON_ROS2)
    set(ROS_INTEGRATED_BUILD TRUE)
    message(STATUS "Building tests with ROS integration")
else()
    set(ROS_INTEGRATED_BUILD FALSE)
    message(STATUS "Building standalone tests (no ROS)")
    project(axon_recorder_tests)
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(GTest REQUIRED)
include(GoogleTest)  # Required for gtest_discover_tests()
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system log log_setup thread filesystem)
find_package(yaml-cpp REQUIRED)

# Path to axon_logging
if(NOT DEFINED CPP_LOGGING_DIR)
    set(CPP_LOGGING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/axon_logging)
endif()

enable_testing()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${GTEST_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CPP_LOGGING_DIR}
)

# Build axon_logging if not already available
if(NOT TARGET axon_logging)
    add_subdirectory(${CPP_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
endif()

# Default test timeout
set(AXON_TEST_TIMEOUT 60 CACHE STRING "Default test timeout in seconds")

# ============================================================================
# Macro: axon_add_test - Reduces boilerplate for test definitions
# ============================================================================
# Usage:
#   axon_add_test(NAME test_name SOURCES file.cpp [file2.cpp ...] [LIBS lib1 lib2 ...])
#
macro(axon_add_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    
    add_executable(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(${TEST_NAME}
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        ${TEST_LIBS}
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT ${AXON_TEST_TIMEOUT})
endmacro()

# ============================================================================
# Macro: axon_add_ros1_test - For ROS 1 tests with catkin
# ============================================================================
# All ROS 1 tests are labeled as "integration" for ctest filtering
macro(axon_add_ros1_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    
    add_executable(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE AXON_ROS1)
    target_include_directories(${TEST_NAME} PRIVATE ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${TEST_NAME} ${TEST_LIBS})
    add_dependencies(${TEST_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    # Add "integration" label so tests are run by integration-tests.yml workflow
    set_tests_properties(${TEST_NAME} PROPERTIES 
        TIMEOUT ${AXON_TEST_TIMEOUT}
        LABELS "integration"
    )
endmacro()

# ============================================================================
# Macro: axon_add_ros2_test - For ROS 2 tests with proper environment setup
# ============================================================================
# All ROS 2 tests are labeled as "integration" for ctest filtering
macro(axon_add_ros2_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE AXON_ROS2)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${TEST_NAME} ${TEST_LIBS})
    
    # Link to generated service types
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${TEST_NAME} "${cpp_typesupport_target}")
    
    # Ensure type support libraries can be found at runtime
    # Add "integration" label so tests are run by integration-tests.yml workflow
    set_tests_properties(${TEST_NAME} PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH};AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
        LABELS "integration"
    )
endmacro()

# ============================================================================
# Macro: axon_discover_unit_tests - Auto-discover and register unit tests
# ============================================================================
# Usage:
#   axon_discover_unit_tests(DIR unit LIBS lib1 lib2 LABEL unit EXCLUDE "pattern")
#
# This macro auto-discovers all test_*.cpp files in a directory and registers
# them as tests. Use EXCLUDE to skip files that need special handling.
#
macro(axon_discover_unit_tests)
    cmake_parse_arguments(DISCOVER "" "DIR;LABEL" "LIBS;EXCLUDE" ${ARGN})
    
    # Find all test_*.cpp files in the directory
    file(GLOB _DISCOVER_TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/${DISCOVER_DIR}/test_*.cpp"
    )
    
    # Exclude specific files if requested
    if(DISCOVER_EXCLUDE)
        foreach(_EXCLUDE_PATTERN ${DISCOVER_EXCLUDE})
            list(FILTER _DISCOVER_TEST_SOURCES EXCLUDE REGEX "${_EXCLUDE_PATTERN}")
        endforeach()
    endif()
    
    # Count for status message
    list(LENGTH _DISCOVER_TEST_SOURCES _DISCOVER_COUNT)
    
    # Register each test
    foreach(_TEST_SOURCE ${_DISCOVER_TEST_SOURCES})
        get_filename_component(_TEST_NAME ${_TEST_SOURCE} NAME_WE)
        
        add_executable(${_TEST_NAME} ${_TEST_SOURCE})
        target_compile_definitions(${_TEST_NAME} PRIVATE BOOST_LOG_DYN_LINK)
        target_include_directories(${_TEST_NAME} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../src
            ${CMAKE_CURRENT_SOURCE_DIR}/${DISCOVER_DIR}
        )
        target_link_libraries(${_TEST_NAME}
            GTest::gtest
            GTest::gtest_main
            Threads::Threads
            ${DISCOVER_LIBS}
        )
        
        # Use gtest_discover_tests for automatic test case discovery
        gtest_discover_tests(${_TEST_NAME}
            PROPERTIES LABELS "${DISCOVER_LABEL}" TIMEOUT ${AXON_TEST_TIMEOUT}
        )
    endforeach()
    
    message(STATUS "Auto-discovered ${_DISCOVER_COUNT} tests in ${DISCOVER_DIR}/")
endmacro()

# ============================================================================
# Test Support Library (standalone sources compiled once)
# ============================================================================
# These sources don't require ROS and are used by multiple unit tests.
# Compile once here instead of in each test executable.
# Include as many source files as possible for better coverage.
add_library(axon_recorder_test_support STATIC
    # Core state management
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
    # Configuration
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/config_parser.cpp
    # HTTP/network
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    # Topic management
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/topic_manager.cpp
    # Threading
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/worker_thread_pool.cpp
    # Service implementation
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_service_impl.cpp
    # Logging sink (no ROS dependency, uses Boost.Log)
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/logging/axon_ros_sink.cpp
)
target_compile_definitions(axon_recorder_test_support PRIVATE BOOST_LOG_DYN_LINK)
target_include_directories(axon_recorder_test_support PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(axon_recorder_test_support
    axon_logging
    yaml-cpp
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
    ${Boost_LIBRARIES}
)

# ============================================================================
# Unit Tests (No ROS required) - Auto-discovered
# ============================================================================
# Tests in unit/ are automatically discovered. Just create a test_*.cpp file
# and it will be registered automatically on the next CMake configure.
#
# To add a new test: touch unit/test_my_feature.cpp
# No CMakeLists.txt edit needed!
#
# Excluded tests are handled explicitly below due to special requirements.
# ============================================================================

axon_discover_unit_tests(
    DIR unit
    LIBS axon_recorder_test_support axon_logging
    LABEL unit
    EXCLUDE "test_ros_introspection"  # Needs ROS compile definitions (handled in ROS section)
            "test_recording_session"  # Needs extra sources (handled in ROS section)
            "test_metadata_injector"  # Needs extra sources (handled in ROS section)
            "test_message_factory"    # Requires ROS (test_message_factory_standalone is the non-ROS version)
)

# ============================================================================
# Integration Tests (No full ROS, but test component interactions)
# ============================================================================

axon_add_test(NAME test_recording_workflow
    SOURCES integration/test_recording_workflow.cpp
    LIBS axon_recorder_test_support axon_logging
)
set_tests_properties(test_recording_workflow PROPERTIES LABELS "integration")

# ============================================================================
# Regression Tests
# ============================================================================

axon_add_test(NAME test_error_recovery
    SOURCES regression/test_error_recovery.cpp
    LIBS axon_recorder_test_support axon_logging
)

# ============================================================================
# MCAP Test Support Library (ROS-integrated build only)
# ============================================================================
# Consolidates MCAP-dependent source files to reduce duplication and improve
# coverage. These files require axon_mcap library.
# ============================================================================
if(ROS_INTEGRATED_BUILD)
    add_library(axon_recorder_test_support_mcap STATIC
        # Recording session (MCAP file lifecycle)
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
        # Metadata injection (JSON sidecar generation)
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
    )
    target_compile_definitions(axon_recorder_test_support_mcap PRIVATE BOOST_LOG_DYN_LINK)
    target_include_directories(axon_recorder_test_support_mcap PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
    )
    target_link_libraries(axon_recorder_test_support_mcap
        axon_mcap
        axon_logging
        nlohmann_json::nlohmann_json
        OpenSSL::SSL
        OpenSSL::Crypto
        ${Boost_LIBRARIES}
    )
    message(STATUS "MCAP test support library enabled")
endif()

# ============================================================================
# Tests Requiring axon_mcap (ROS-integrated build only)
# ============================================================================
if(ROS_INTEGRATED_BUILD)
    axon_add_test(NAME test_recording_session
        SOURCES unit/test_recording_session.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_recording_session PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)

    axon_add_test(NAME test_metadata_injector
        SOURCES unit/test_metadata_injector.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_metadata_injector PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)

    axon_add_test(NAME test_metadata_injection
        SOURCES integration/test_metadata_injection.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_metadata_injection PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)
    set_tests_properties(test_metadata_injection PROPERTIES LABELS "integration")

    # ROS Introspection test
    add_executable(test_ros_introspection
        unit/test_ros_introspection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/ros_introspection.cpp
    )
    if(DEFINED AXON_ROS1)
        target_compile_definitions(test_ros_introspection PRIVATE AXON_ROS1)
        target_include_directories(test_ros_introspection PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(test_ros_introspection GTest::gtest GTest::gtest_main Threads::Threads ${catkin_LIBRARIES})
    elseif(DEFINED AXON_ROS2)
        target_compile_definitions(test_ros_introspection PRIVATE AXON_ROS2)
        target_link_libraries(test_ros_introspection GTest::gtest GTest::gtest_main Threads::Threads rclcpp::rclcpp)
    endif()
    add_test(NAME test_ros_introspection COMMAND test_ros_introspection)
    set_tests_properties(test_ros_introspection PROPERTIES TIMEOUT ${AXON_TEST_TIMEOUT})
endif()

# ============================================================================
# ROS-Integrated Tests (require full ROS environment)
# ============================================================================
if(ROS_INTEGRATED_BUILD AND Boost_FOUND)
    if(DEFINED AXON_ROS1)
        find_package(rostest QUIET)
        if(rostest_FOUND)
            # Common libraries for ROS 1 tests
            set(ROS1_TEST_LIBS
                axon_recorder_lib
                GTest::gtest
                GTest::gtest_main
                Threads::Threads
                ${catkin_LIBRARIES}
                ${Boost_LIBRARIES}
            )

            axon_add_ros1_test(NAME test_service_adapter_ros1
                SOURCES integration/test_service_adapter.cpp
                LIBS ${ROS1_TEST_LIBS}
            )

            axon_add_ros1_test(NAME test_ros_interface_ros1
                SOURCES integration/test_ros_interface.cpp
                LIBS ${ROS1_TEST_LIBS}
            )

            axon_add_ros1_test(NAME test_recorder_node_lifecycle_ros1
                SOURCES integration/test_recorder_node_lifecycle.cpp
                LIBS ${ROS1_TEST_LIBS}
            )

            axon_add_ros1_test(NAME test_recorder_node_recording_ros1
                SOURCES integration/test_recorder_node_recording.cpp
                LIBS ${ROS1_TEST_LIBS}
            )
        endif()

    elseif(DEFINED AXON_ROS2)
        find_package(ament_cmake_gtest QUIET)
        if(ament_cmake_gtest_FOUND)
            # Common libraries for ROS 2 tests
            set(ROS2_TEST_LIBS
                axon_recorder_lib
                Threads::Threads
                ${Boost_LIBRARIES}
                rclcpp::rclcpp
                ${std_msgs_TARGETS}
            )

            axon_add_ros2_test(NAME test_service_adapter_ros2
                SOURCES integration/test_service_adapter.cpp
                LIBS ${ROS2_TEST_LIBS}
            )

            axon_add_ros2_test(NAME test_ros_interface_ros2
                SOURCES integration/test_ros_interface.cpp
                LIBS ${ROS2_TEST_LIBS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
            )

            axon_add_ros2_test(NAME test_recorder_node_lifecycle_ros2
                SOURCES integration/test_recorder_node_lifecycle.cpp
                LIBS ${ROS2_TEST_LIBS}
            )

            axon_add_ros2_test(NAME test_recorder_node_recording_ros2
                SOURCES integration/test_recorder_node_recording.cpp
                LIBS ${ROS2_TEST_LIBS}
            )
        endif()
    endif()
endif()

# ============================================================================
# Stress Tests
# ============================================================================
# Stress tests are longer-running tests that verify stability under load.
# Run separately from unit tests: ctest -L stress
# ============================================================================
if(ROS_INTEGRATED_BUILD)
    # Stress test timeout (longer than unit tests)
    set(AXON_STRESS_TEST_TIMEOUT 300 CACHE STRING "Stress test timeout in seconds")

    # test_recording_session_stress - requires axon_mcap
    axon_add_test(NAME test_recording_session_stress
        SOURCES stress/test_recording_session_stress.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
        LIBS axon_mcap axon_logging nlohmann_json::nlohmann_json OpenSSL::SSL OpenSSL::Crypto ${Boost_LIBRARIES}
    )
    target_include_directories(test_recording_session_stress PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)
    set_tests_properties(test_recording_session_stress PROPERTIES
        LABELS "stress"
        TIMEOUT ${AXON_STRESS_TEST_TIMEOUT}
    )

    message(STATUS "Stress tests enabled (run with: ctest -L stress)")
endif()

# ============================================================================
# Edge Uploader Integration Tests (conditional)
# ============================================================================
# These tests require AXON_HAS_UPLOADER to be defined and axon_uploader library.
# They test the integration between RecorderNode and EdgeUploader.
# ============================================================================
if(AXON_HAS_UPLOADER AND ROS_INTEGRATED_BUILD)
    axon_add_test(NAME test_recorder_node_uploader
        SOURCES integration/test_recorder_node_uploader.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
        LIBS axon_mcap axon_uploader axon_logging nlohmann_json::nlohmann_json OpenSSL::SSL OpenSSL::Crypto ${Boost_LIBRARIES}
    )
    target_compile_definitions(test_recorder_node_uploader PRIVATE AXON_HAS_UPLOADER)
    target_include_directories(test_recorder_node_uploader PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/axon_uploader
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/axon_mcap
    )
    set_tests_properties(test_recorder_node_uploader PROPERTIES
        LABELS "integration"
        TIMEOUT ${AXON_TEST_TIMEOUT}
    )
    message(STATUS "Edge Uploader integration tests enabled")
else()
    message(STATUS "Edge Uploader integration tests disabled (AXON_HAS_UPLOADER not defined)")
endif()

# ============================================================================
# Performance Tests (optional subdirectory)
# ============================================================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/perf/CMakeLists.txt)
    add_subdirectory(perf)
endif()

# ============================================================================
# Custom Target: Run All Tests
# ============================================================================
add_custom_target(run_recorder_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMENT "Running all recorder tests"
)
