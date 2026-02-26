# =============================================================================
# Axon Recorder Tests
# =============================================================================

cmake_minimum_required(VERSION 3.12)

# =============================================================================
# Testing Options
# =============================================================================
# Note: These options can be set from parent CMake, but defining them here
# allows standalone test builds
if(NOT DEFINED AXON_BUILD_TESTS)
    option(AXON_BUILD_TESTS "Build unit tests" ON)
endif()
if(NOT DEFINED AXON_BUILD_INTEGRATION_TESTS)
    option(AXON_BUILD_INTEGRATION_TESTS "Build integration tests" ON)
endif()
if(NOT DEFINED AXON_ENABLE_COVERAGE)
    option(AXON_ENABLE_COVERAGE "Enable code coverage reporting" OFF)
endif()

if(NOT AXON_BUILD_TESTS)
    return()
endif()

message(STATUS "Axon Recorder Tests Configuration:")
message(STATUS "  Unit Tests:          ${AXON_BUILD_TESTS}")
message(STATUS "  Integration Tests:  ${AXON_BUILD_INTEGRATION_TESTS}")
message(STATUS "  Coverage:           ${AXON_ENABLE_COVERAGE}")
message(STATUS "")

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
        )
    endif()
endif()

# =============================================================================
# Core Libraries (Required for tests)
# =============================================================================

# Use centralized path variables from parent (AXON_* prefix) if available
# Note: These targets are created by root CMakeLists.txt, so we just link to them
if(NOT DEFINED AXON_MCAP_DIR)
    set(AXON_MCAP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/axon_mcap")
endif()
if(NOT DEFINED AXON_LOGGING_DIR)
    set(AXON_LOGGING_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/axon_logging"
    )
endif()

# Check that core libraries exist (targets created by parent)
if(TARGET axon_mcap)
    message(STATUS "Using axon_mcap target from parent")
elseif(EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
    message(STATUS "Using axon_mcap at: ${AXON_MCAP_DIR}")
else()
    message(WARNING "axon_mcap not found - tests may fail to build")
endif()

if(TARGET axon_logging)
    message(STATUS "Using axon_logging target from parent")
elseif(EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
    message(STATUS "Using axon_logging at: ${AXON_LOGGING_DIR}")
else()
    message(WARNING "axon_logging not found - tests may fail to build")
endif()

# =============================================================================
# Common Test Configuration
# =============================================================================
set(TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
set(TEST_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_BINARY_DIR}/..
    ${nlohmann_json_SOURCE_DIR}/single_include
)

function(add_axon_test test_name)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})

    add_executable(${test_name} ${ARG_SOURCES})

    target_include_directories(${test_name} PRIVATE ${TEST_INCLUDE_DIRS})

    target_link_libraries(
        ${test_name}
        GTest::gtest
        GTest::gtest_main
        axon_mcap
        axon_logging
        pthread
    )

    target_compile_features(${test_name} PRIVATE cxx_std_17)

    # Coverage flags
    if(AXON_ENABLE_COVERAGE)
        target_compile_options(
            ${test_name}
            PRIVATE --coverage -fprofile-arcs -ftest-coverage
        )
        target_link_options(${test_name} PRIVATE --coverage)
    endif()

    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# =============================================================================
# Unit Tests
# =============================================================================

message(STATUS "Configuring unit tests...")

# test_helper.cpp - Header-only library for test utilities
add_library(test_helper INTERFACE unit/test_helpers.hpp)

# test_state_machine.cpp
add_axon_test(test_state_machine
    SOURCES unit/test_state_machine.cpp
            ../state_machine.cpp
)

# test_task_config.cpp
add_axon_test(test_task_config
    SOURCES unit/test_task_config.cpp
            ../config_parser.cpp
)
target_link_libraries(test_task_config yaml-cpp)

# test_config_injector.cpp
add_axon_test(test_config_injector
    SOURCES unit/test_config_injector.cpp
            ../config_injector.cpp
)
target_link_libraries(test_config_injector axon_mcap)

# test_metadata_injector.cpp
add_axon_test(test_metadata_injector
    SOURCES unit/test_metadata_injector.cpp
            ../metadata_injector.cpp
)

# test_worker_thread_pool.cpp
add_axon_test(test_worker_thread_pool
    SOURCES unit/test_worker_thread_pool.cpp
            ../worker_thread_pool.cpp
)
target_link_libraries(test_worker_thread_pool axon_mcap pthread)

# test_plugin_loader.cpp - Test the plugin loader ABI
add_axon_test(test_plugin_loader
    SOURCES unit/test_plugin_loader_thread_safety.cpp
            ../plugin_loader.cpp
)
target_link_libraries(test_plugin_loader ${CMAKE_DL_LIBS})

# test_spsc_queue.cpp - Uses mock plugin
add_axon_test(test_spsc_queue
    SOURCES unit/test_spsc_queue.cpp
            ../spsc_queue.hpp
)
target_link_libraries(test_spsc_queue axon_mcap)

# test_websocket_server.cpp - WebSocket server and session tests
add_axon_test(test_websocket_server
    SOURCES unit/test_websocket_server.cpp
            ../websocket_server.cpp
            ../websocket_session.cpp
)
target_link_libraries(
    test_websocket_server
    OpenSSL::Crypto
    OpenSSL::SSL
    pthread
)

# =============================================================================
# Integration Tests
# =============================================================================

if(AXON_BUILD_INTEGRATION_TESTS)
    message(STATUS "Configuring integration tests...")

    # test_http_callback_client.cpp
    add_axon_test(test_http_callback_client
        SOURCES unit/test_http_callback_client.cpp
                ../http_callback_client.cpp
    )
    target_link_libraries(
        test_http_callback_client
        ${CMAKE_DL_LIBS}
        OpenSSL::Crypto
        OpenSSL::SSL
        pthread
    )

    # test_psc_queue.cpp - Integration tests for S3 uploader
    add_axon_test(test_psc_queue
        SOURCES unit/test_spsc_queue.cpp
                ../spsc_queue.hpp
    )
    target_link_libraries(test_psc_queue axon_mcap pthread)

    # test_recording_session.cpp
    add_axon_test(test_recording_session
        SOURCES unit/test_recording_session.cpp
                ../recording_session.cpp
                ../metadata_injector.cpp
                ../config_injector.cpp
                ../plugin_loader.cpp
    )
    target_link_libraries(
        test_recording_session
        axon_mcap
        axon_logging
        yaml-cpp
        pthread
        ${CMAKE_DL_LIBS}
    )

    # test_config_parser_integration.cpp
    add_axon_test(test_config_parser_integration
        SOURCES unit/test_config_parser.cpp
                ../config_parser.cpp
                ../state_machine.cpp
                ../plugin_loader.cpp
    )
    target_link_libraries(
        test_config_parser_integration
        axon_mcap
        axon_logging
        yaml-cpp
        pthread
        ${CMAKE_DL_LIBS}
    )

    # test_worker_thread_pool_integration.cpp - With full recorder context
    add_axon_test(test_worker_thread_pool_integration
        SOURCES unit/test_worker_thread_pool.cpp
                ../worker_thread_pool.cpp
                ../recording_session.cpp
                ../metadata_injector.cpp
                ../config_injector.cpp
                ../plugin_loader.cpp
    )
    target_link_libraries(
        test_worker_thread_pool_integration
        axon_mcap
        axon_logging
        yaml-cpp
        pthread
        ${CMAKE_DL_LIBS}
    )

    # test_recorder_integration.cpp - Full workflow with mock plugin
    add_axon_test(test_recorder_integration
        SOURCES integration/test_recorder_integration.cpp
                ../recorder.cpp
                ../state_machine.cpp
                ../recording_session.cpp
                ../metadata_injector.cpp
                ../config_injector.cpp
                ../plugin_loader.cpp
    )
    target_link_libraries(
        test_recorder_integration
        axon_mcap
        axon_logging
        yaml-cpp
        pthread
        ${CMAKE_DL_LIBS}
    )

    # Add integration test label
    set_property(
        TEST test_recorder_integration
        PROPERTY LABELS "integration;recorder"
    )
endif()

# =============================================================================
# E2E Tests (using mock plugin)
# =============================================================================
if(NOT DEFINED AXON_BUILD_E2E_TESTS)
    option(AXON_BUILD_E2E_TESTS "Build E2E tests" ON)
endif()

if(
    AXON_BUILD_E2E_TESTS
    AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/e2e/test_e2e_full_workflow.cpp"
)
    message(STATUS "Configuring E2E tests...")

    # test_e2e_full_workflow.cpp
    add_axon_test(test_e2e_full_workflow
        SOURCES e2e/test_e2e_full_workflow.cpp
                ../recorder.cpp
                ../state_machine.cpp
                ../recording_session.cpp
                ../metadata_injector.cpp
                ../config_injector.cpp
                ../plugin_loader.cpp
    )
    target_link_libraries(
        test_e2e_full_workflow
        axon_mcap
        axon_logging
        yaml-cpp
        pthread
        ${CMAKE_DL_LIBS}
    )

    # Add E2E test label
    set_property(TEST test_e2e_full_workflow PROPERTY LABELS "e2e;recorder")
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
        add_custom_target(
            coverage
            COMMAND ${LCOV} --capture --directory . --output-file coverage.info
            COMMAND
                ${CMAKE_COMMAND} -E echo
                "Coverage report generated in coverage_html/index.html"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating code coverage report"
        )

        add_custom_target(
            coverage-clean
            COMMAND ${CMAKE_COMMAND} -E rm -f coverage.info coverage.info.base
            COMMAND ${CMAKE_COMMAND} -E remove_directory coverage_html
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Cleaning coverage files"
        )
    else()
        message(
            WARNING
            "lcov and genhtml not found - coverage reporting disabled"
        )
    endif()
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Axon Recorder Test Configuration Summary ===")
message(STATUS "  Unit Tests:          ${AXON_BUILD_TESTS}")
message(STATUS "  Integration Tests:  ${AXON_BUILD_INTEGRATION_TESTS}")
message(STATUS "  E2E Tests:          ${AXON_BUILD_E2E_TESTS}")
message(STATUS "  Coverage:           ${AXON_ENABLE_COVERAGE}")
message(STATUS "===================================================")
message(STATUS "")
