cmake_minimum_required(VERSION 3.12)
project(mock_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Boost
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup thread filesystem)

# Find threading library
find_package(Threads REQUIRED)

# AXON_LOGGING_DIR is set by parent CMake when building as part of main project
# When building standalone (via make build-mock), it will be empty and we need to set it
if(NOT DEFINED AXON_LOGGING_DIR OR "${AXON_LOGGING_DIR}" STREQUAL "")
    # Standalone build: compute path relative to mock plugin CMakeLists.txt
    set(AXON_LOGGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_logging" CACHE PATH "Path to axon_logging")
endif()

# Create axon_logging as a static library within this project
# Only when building standalone (AXON_LOGGING_DIR not set by parent CMake)
# This avoids add_subdirectory issues while building standalone
if(NOT DEFINED AXON_LOGGING_DIR OR NOT AXON_LOGGING_DIR)
    set(AXON_LOGGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_logging" CACHE PATH "Path to axon_logging")
    add_library(axon_logging_static STATIC
        ${AXON_LOGGING_DIR}/src/axon_log_init.cpp
        ${AXON_LOGGING_DIR}/src/axon_console_sink.cpp
        ${AXON_LOGGING_DIR}/src/axon_file_sink.cpp
)

    target_include_directories(axon_logging_static PUBLIC
        ${AXON_LOGGING_DIR}/include
)
endif()

# Build the mock plugin as a shared library
add_library(${PROJECT_NAME} SHARED
    src/mock_plugin.cpp
    src/mock_plugin_export.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_logging
)

# Set library properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN ON
)

# Link pthread, Threads, and Boost
# Use axon_logging if available (when building from parent), otherwise axon_logging_static
if(TARGET axon_logging)
    target_link_libraries(${PROJECT_NAME}
        pthread
        Threads::Threads
        axon_logging
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::filesystem
    )
else()
    target_link_libraries(${PROJECT_NAME}
        pthread
        Threads::Threads
        axon_logging_static
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::filesystem
    )
endif()


# Also create an object library for testing
add_library(${PROJECT_NAME}_obj OBJECT
    src/mock_plugin.cpp
)

target_include_directories(${PROJECT_NAME}_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_logging
)

# Install
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES include/mock_plugin.hpp
    DESTINATION include
)

# =============================================================================
# Tests
# =============================================================================

# Simple plugin load test (requires building core libs first)
add_executable(test_mock_plugin_load
    test/test_mock_plugin_load.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../apps/axon_recorder/plugin_loader.cpp
)

target_include_directories(test_mock_plugin_load PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../apps/axon_recorder
)

target_link_libraries(test_mock_plugin_load
    ${CMAKE_DL_LIBS}
)

# E2E test with mock plugin
add_executable(test_mock_plugin_e2e
    test/test_mock_plugin_e2e.cpp
)

target_include_directories(test_mock_plugin_e2e PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link test executable - use axon_logging if available, otherwise axon_logging_static
if(TARGET axon_logging)
    target_link_libraries(test_mock_plugin_e2e
        ${PROJECT_NAME}  # Link against full mock_plugin library, not _obj
        pthread
        axon_logging
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::filesystem
    )
else()
    target_link_libraries(test_mock_plugin_e2e
        ${PROJECT_NAME}  # Link against full mock_plugin library, not _obj
        pthread
        axon_logging_static
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::filesystem
    )
endif()

# Install tests
install(TARGETS test_mock_plugin_load test_mock_plugin_e2e
    RUNTIME DESTINATION bin
)
