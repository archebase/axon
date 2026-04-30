# =============================================================================
# Axon Mock Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
#
# This plugin is used for testing without ROS dependencies.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(axon_mock_plugin VERSION 1.0.0 LANGUAGES CXX)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Dependencies
# =============================================================================
# Verify required variables are set (like ROS1 plugin does)
if(NOT DEFINED AXON_LOGGING_DIR)
    # Try to auto-detect from relative path
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_logging")
        set(AXON_LOGGING_DIR
            "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_logging"
        )
    else()
        message(
            FATAL_ERROR
            "AXON_LOGGING_DIR must be set. "
            "Please build via parent CMakeLists.txt or set -DAXON_LOGGING_DIR=/path/to/axon_logging"
        )
    endif()
endif()

find_package(
    Boost
    REQUIRED
    COMPONENTS log log_setup thread filesystem system
)
find_package(Threads REQUIRED)

# Try to find nlohmann_json (works when built from parent project)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
    message(
        STATUS
        "nlohmann_json not found via find_package, assuming provided by parent project"
    )
endif()

# =============================================================================
# Find axon_logging library
# =============================================================================
# The library can be in different locations depending on how core was built:
# - Project root build: ../../build/axon_logging/
# - Core build: ../../core/build/axon_logging/
find_library(
    AXON_LOGGING_LIBRARY
    NAMES axon_logging
    PATHS
        "${CMAKE_CURRENT_SOURCE_DIR}/../../build/axon_logging"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../build"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../core/build/axon_logging"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../core/build"
    NO_DEFAULT_PATH
)

if(NOT AXON_LOGGING_LIBRARY)
    message(
        STATUS
        "axon_logging library not found, assuming provided by parent project"
    )
    set(AXON_LOGGING_LIBRARY "axon_logging")
else()
    # Get the directory containing the library for linker path
    get_filename_component(
        AXON_LOGGING_LIB_DIR
        "${AXON_LOGGING_LIBRARY}"
        DIRECTORY
    )
    link_directories("${AXON_LOGGING_LIB_DIR}")
    message(STATUS "Found axon_logging library: ${AXON_LOGGING_LIBRARY}")
endif()

# =============================================================================
# Build Plugin Library
# =============================================================================

set(CORE_SOURCES src/mock_plugin.cpp src/mock_plugin_export.cpp)

add_library(axon_mock_plugin SHARED ${CORE_SOURCES})

# Set output name to match expected plugin name
set_target_properties(
    axon_mock_plugin
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION 0
        VISIBILITY_INLINES_HIDDEN ON
        OUTPUT_NAME
            "mock_plugin" # Produces libmock_plugin.so
)

target_include_directories(
    axon_mock_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${AXON_LOGGING_DIR}
)

# Link libraries
target_link_libraries(
    axon_mock_plugin
    PUBLIC
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::filesystem
        Boost::system
        Threads::Threads
        ${AXON_LOGGING_LIBRARY}
)

# Export all symbols for dynamic loading
target_compile_options(axon_mock_plugin PRIVATE -fvisibility=default)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_mock_plugin)
endif()

# =============================================================================
# Install
# =============================================================================
install(
    TARGETS axon_mock_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
)

install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")

# =============================================================================
# Tests
# =============================================================================
option(AXON_BUILD_TESTS "Build tests" OFF)

# Compute apps directory for test dependencies
if(NOT DEFINED AXON_APPS_DIR)
    set(AXON_APPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../apps")
endif()

if(AXON_BUILD_TESTS)
    enable_testing()

    # Simple plugin load test
    add_executable(
        test_mock_plugin_load
        test/test_mock_plugin_load.cpp
        ${AXON_APPS_DIR}/axon_recorder/src/plugin/plugin_loader.cpp
    )

    target_include_directories(
        test_mock_plugin_load
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${AXON_APPS_DIR}/axon_recorder
            ${AXON_APPS_DIR}/axon_recorder/src
    )

    target_link_libraries(test_mock_plugin_load ${CMAKE_DL_LIBS})

    # E2E test with mock plugin
    add_executable(test_mock_plugin_e2e test/test_mock_plugin_e2e.cpp)

    target_include_directories(
        test_mock_plugin_e2e
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(
        test_mock_plugin_e2e
        axon_mock_plugin
        ${CMAKE_DL_LIBS}
        Threads::Threads
        ${AXON_LOGGING_LIBRARY}
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::filesystem
    )

    # Apply coverage flags
    if(COMMAND axon_add_coverage)
        axon_add_coverage(test_mock_plugin_load)
        axon_add_coverage(test_mock_plugin_e2e)
    endif()

    # Register tests
    add_test(NAME test_mock_plugin_load COMMAND test_mock_plugin_load)
    add_test(NAME test_mock_plugin_e2e COMMAND test_mock_plugin_e2e)
    set_tests_properties(
        test_mock_plugin_load
        PROPERTIES LABELS "unit;mock_plugin"
    )
    set_tests_properties(
        test_mock_plugin_e2e
        PROPERTIES LABELS "e2e;mock_plugin"
    )
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon Mock Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  axon_logging: ${AXON_LOGGING_LIBRARY}")
message(STATUS "  Tests Enabled: ${AXON_BUILD_TESTS}")
message(STATUS "")
