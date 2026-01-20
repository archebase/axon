cmake_minimum_required(VERSION 3.12)

# =============================================================================
# Path Configuration
# =============================================================================
if(DEFINED AXON_REPO_ROOT)
    set(PROJECT_ROOT ${AXON_REPO_ROOT})
else()
    set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
endif()

message(STATUS "Test project root: ${PROJECT_ROOT}")

# =============================================================================
# Testing Options
# =============================================================================
option(AXON_BUILD_TESTS "Build unit tests" ON)
option(AXON_BUILD_INTEGRATION_TESTS "Build integration tests" ON)
option(AXON_ENABLE_COVERAGE "Enable code coverage reporting" OFF)

if(NOT AXON_BUILD_TESTS)
    return()
endif()

# =============================================================================
# Dependencies
# =============================================================================
# Try to find system GTest first, fallback to FetchContent
find_package(GTest QUIET)

# Include GoogleTest
include(FetchContent)

if(NOT GTest_FOUND)
    message(STATUS "GTest not found, downloading via FetchContent...")
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
            GIT_SHALLOW TRUE
            GIT_PROGRESS TRUE
        )
    else()
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
            GIT_SHALLOW TRUE
        )
    endif()

    # For Windows: Prevent overriding parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
else()
    message(STATUS "Found system GTest: ${GTest_VERSION}")
endif()

# =============================================================================
# Core Libraries (Required for tests)
# =============================================================================
# Always try to build with core libraries for testing
set(CPP_MCAP_DIR "${PROJECT_ROOT}/core/axon_mcap")
set(CPP_LOGGING_DIR "${PROJECT_ROOT}/core/axon_logging")

if(EXISTS "${CPP_MCAP_DIR}/CMakeLists.txt" AND NOT TARGET axon_mcap)
    message(STATUS "Building tests with axon_mcap at: ${CPP_MCAP_DIR}")
    add_subdirectory(${CPP_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
elseif(NOT TARGET axon_mcap)
    message(FATAL_ERROR "axon_mcap required for tests but not found at: ${CPP_MCAP_DIR}")
endif()

if(EXISTS "${CPP_LOGGING_DIR}/CMakeLists.txt" AND NOT TARGET axon_logging)
    message(STATUS "Building tests with axon_logging at: ${CPP_LOGGING_DIR}")
    add_subdirectory(${CPP_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
elseif(NOT TARGET axon_logging)
    message(FATAL_ERROR "axon_logging required for tests but not found at: ${CPP_LOGGING_DIR}")
endif()

# =============================================================================
# Common Test Configuration
# =============================================================================
set(TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
set(TEST_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${PROJECT_ROOT}/core/axon_mcap/include
    ${PROJECT_ROOT}/core/axon_logging/include
    ${nlohmann_json_SOURCE_DIR}/single_include
)

function(add_axon_test test_name)
    cmake_parse_arguments(ARG
        ""
        ""
        "SOURCES"
        ${ARGN}
    )

    add_executable(${test_name} ${ARG_SOURCES})

    target_include_directories(${test_name} PRIVATE ${TEST_INCLUDE_DIRS})

    target_link_libraries(${test_name}
        gtest
        gtest_main
        axon_mcap
        axon_logging
        pthread
    )

    target_compile_features(${test_name} PRIVATE cxx_std_17)

    # Coverage flags
    if(AXON_ENABLE_COVERAGE)
        target_compile_options(${test_name} PRIVATE
            --coverage
        )
        target_link_options(${test_name} PRIVATE
            --coverage
        )
    endif()

    # Discover tests automatically
    gtest_discover_tests(${test_name})

    # Add to test set
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# =============================================================================
# Unit Tests
# =============================================================================
message(STATUS "Configuring unit tests...")

# Test helpers (header-only library)
add_library(test_helper INTERFACE)
target_include_directories(test_helper INTERFACE ${TEST_INCLUDE_DIRS})

# test_state_machine.cpp
add_axon_test(test_state_machine
    SOURCES
        ${TEST_SOURCE_DIR}/test_state_machine.cpp
        ../state_machine.cpp
)

# test_state_transaction_guard.cpp
add_axon_test(test_state_transaction_guard
    SOURCES
        ${TEST_SOURCE_DIR}/test_state_transaction_guard.cpp
        ../state_machine.cpp
)

# test_task_config.cpp
add_axon_test(test_task_config
    SOURCES
        ${TEST_SOURCE_DIR}/test_task_config.cpp
)

# test_config_parser.cpp
add_axon_test(test_config_parser
    SOURCES
        ${TEST_SOURCE_DIR}/test_config_parser.cpp
        # Add required source files
        ../config_parser.cpp
)
target_link_libraries(test_config_parser yaml-cpp)

# test_http_callback_client.cpp
add_axon_test(test_http_callback_client
    SOURCES
        ${TEST_SOURCE_DIR}/test_http_callback_client.cpp
        # Add required source files
        ../http_callback_client.cpp
)
target_link_libraries(test_http_callback_client
    OpenSSL::Crypto
    OpenSSL::SSL
    Boost::system
    ${CMAKE_DL_LIBS}
)

# test_spsc_queue.cpp
add_axon_test(test_spsc_queue
    SOURCES
        ${TEST_SOURCE_DIR}/test_spsc_queue.cpp
)

# test_worker_thread_pool.cpp
add_axon_test(test_worker_thread_pool
    SOURCES
        ${TEST_SOURCE_DIR}/test_worker_thread_pool.cpp
        ../worker_thread_pool.cpp
)

# test_message_factory_standalone.cpp (standalone, no dependencies)
add_axon_test(test_message_factory_standalone
    SOURCES
        ${TEST_SOURCE_DIR}/test_message_factory_standalone.cpp
)

# test_metadata_injector.cpp
add_axon_test(test_metadata_injector
    SOURCES
        ${TEST_SOURCE_DIR}/test_metadata_injector.cpp
        # Add required source files
        ../metadata_injector.cpp
)
target_include_directories(test_metadata_injector PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)
target_link_libraries(test_metadata_injector OpenSSL::Crypto)

# test_recording_session.cpp
add_axon_test(test_recording_session
    SOURCES
        ${TEST_SOURCE_DIR}/test_recording_session.cpp
        # Add required source files
        ../recording_session.cpp
        ../metadata_injector.cpp
)
target_include_directories(test_recording_session PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)
target_link_libraries(test_recording_session OpenSSL::Crypto)

# test_topic_manager.cpp - DISABLED (TopicManager removed in new architecture)
# This file depends on the old RosInterface which has been replaced by plugins
# To enable: remove from exclusion list and implement mock plugin interface
list(APPEND EXCLUDED_TEST_FILES ${TEST_SOURCE_DIR}/test_topic_manager.cpp)

# =============================================================================
# Thread Safety and Type Validation Tests (Added for reviewer fixes)
# =============================================================================

# test_plugin_loader_thread_safety.cpp
add_axon_test(test_plugin_loader_thread_safety
    SOURCES
        ${TEST_SOURCE_DIR}/test_plugin_loader_thread_safety.cpp
        # Add required source files
        ../plugin_loader.cpp
)
target_link_libraries(test_plugin_loader_thread_safety ${CMAKE_DL_LIBS})



message(STATUS "Unit tests configured successfully")

# =============================================================================
# Integration Tests
# =============================================================================
# NOTE: Integration tests for the OLD architecture have been archived to
# integration/archive/. These tests depend on RosInterface and ROS services,
# which have been replaced by the new plugin-based architecture with HTTP RPC API.
#
# To enable integration testing for the new architecture, create test_recorder_integration.cpp
# that tests:
# 1. Plugin loading and ABI interface
# 2. HTTP RPC endpoints (start, stop, pause, resume, status)
# 3. Full recording workflow with mock middleware plugins
# =============================================================================
if(AXON_BUILD_INTEGRATION_TESTS)
    message(STATUS "Configuring integration tests...")

    # Add integration test directory
    set(INTEGRATION_TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/integration)

    # test_recorder_integration.cpp - AxonRecorder integration tests
    # Requires the full recorder implementation
    if(EXISTS "${INTEGRATION_TEST_SOURCE_DIR}/test_recorder_integration.cpp")
        add_executable(test_recorder_integration
            ${INTEGRATION_TEST_SOURCE_DIR}/test_recorder_integration.cpp
            # Add all required source files
            ../recorder.cpp
            ../state_machine.cpp
            ../recording_session.cpp
            ../metadata_injector.cpp
            ../worker_thread_pool.cpp
            ../plugin_loader.cpp
            ../config_parser.cpp
            ../http_callback_client.cpp
            ../http_server.cpp
        )

        target_include_directories(test_recorder_integration PRIVATE
            ${TEST_INCLUDE_DIRS}
            ${CMAKE_CURRENT_BINARY_DIR}/..  # For generated version.hpp from parent CMakeLists
        )

        target_link_libraries(test_recorder_integration
            gtest
            gtest_main
            axon_mcap
            axon_logging
            yaml-cpp
            OpenSSL::Crypto
            OpenSSL::SSL
            Boost::system
            Boost::thread
            pthread
            ${CMAKE_DL_LIBS}
        )

        target_compile_features(test_recorder_integration PRIVATE cxx_std_17)

        if(AXON_ENABLE_COVERAGE)
            target_compile_options(test_recorder_integration PRIVATE --coverage)
            target_link_options(test_recorder_integration PRIVATE --coverage)
        endif()

        gtest_discover_tests(test_recorder_integration)
        add_test(NAME test_recorder_integration COMMAND test_recorder_integration)
    endif()

    message(STATUS "Integration tests configured successfully")
endif()

# =============================================================================
# E2E Tests
# =============================================================================
# E2E tests now use the unified mock middleware from middlewares/mock/
# Build with: make build-mock from project root
option(AXON_BUILD_E2E_TESTS "Build E2E test support" ON)

if(AXON_BUILD_E2E_TESTS)
    message(STATUS "E2E tests configured to use mock middleware from middlewares/mock/")
    message(STATUS "  Build mock plugin with: make build-mock")
    message(STATUS "  Mock plugin location: middlewares/mock/src/mock_plugin/build/libmock_plugin.so")
endif()

# =============================================================================
# Coverage Reporting
# =============================================================================
if(AXON_ENABLE_COVERAGE)
    find_program(LCOV lcov)
    find_program(GENHTML genhtml)

    if(LCOV AND GENHTML)
        message(STATUS "Coverage reporting enabled")

        # Create coverage target
        add_custom_target(coverage
            COMMAND ${LCOV} --capture --directory . --output-file coverage.info
            COMMAND ${LCOV} --remove coverage.info
                '/usr/*' '*/test/*' '*/build/*' '*/_deps/*' '*/single_include/*'
                --output-file coverage.info
            COMMAND ${LCOV} --list coverage.info
            COMMAND ${GENHTML} coverage.info --output-directory coverage_html
            COMMAND ${CMAKE_COMMAND} -E echo "Coverage report generated in coverage_html/index.html"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating code coverage report"
        )

        add_custom_target(coverage-clean
            COMMAND ${CMAKE_COMMAND} -E remove coverage.info coverage.info.base
            COMMAND ${CMAKE_COMMAND} -E remove_directory coverage_html
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Cleaning coverage files"
        )
    else()
        message(WARNING "lcov and genhtml not found - coverage reporting disabled")
    endif()
endif()

# =============================================================================
# Test Labels
# =============================================================================
# Add labels to tests for easier filtering
set_property(TEST test_state_machine PROPERTY LABELS "unit;state")
set_property(TEST test_state_transaction_guard PROPERTY LABELS "unit;state")
set_property(TEST test_task_config PROPERTY LABELS "unit;config")
set_property(TEST test_config_parser PROPERTY LABELS "unit;config")
set_property(TEST test_http_callback_client PROPERTY LABELS "unit;http")
set_property(TEST test_spsc_queue PROPERTY LABELS "unit;queue")
set_property(TEST test_worker_thread_pool PROPERTY LABELS "unit;thread")
set_property(TEST test_message_factory_standalone PROPERTY LABELS "unit;factory")
set_property(TEST test_metadata_injector PROPERTY LABELS "unit;metadata")
set_property(TEST test_recording_session PROPERTY LABELS "unit;session")

if(TARGET test_recorder_integration)
    set_property(TEST test_recorder_integration PROPERTY LABELS "integration;recorder")
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Axon Recorder Test Configuration Summary ===")
message(STATUS "  Unit Tests:          ON")
message(STATUS "  Integration Tests:  ${AXON_BUILD_INTEGRATION_TESTS}")
message(STATUS "  E2E Tests:          ${AXON_BUILD_E2E_TESTS}")
message(STATUS "  Coverage:           ${AXON_ENABLE_COVERAGE}")
message(STATUS "===================================================")
message(STATUS "")
