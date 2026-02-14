cmake_minimum_required(VERSION 3.12)
project(mock_plugin)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Centralized CMake Modules
# =============================================================================
# All Axon modules must use the centralized cmake/Modules directory.
# This ensures consistent configuration across the codebase.
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    # Compute relative path to cmake/Modules
    set(_AXON_CMAKE_MODULES_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules"
    )
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

# Include required modules (AxonStdLib provides standard helpers)
include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)

# =============================================================================
# Find Boost
# =============================================================================
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup thread filesystem)

# Find threading library
find_package(Threads REQUIRED)

# =============================================================================
# Build mock plugin as a shared library
# =============================================================================
add_library(
    ${PROJECT_NAME}
    SHARED
    src/mock_plugin.cpp
    src/mock_plugin_export.cpp
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Set library properties
set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES VERSION 1.0.0 SOVERSION 1 VISIBILITY_INLINES_HIDDEN ON
)

# Link pthread, Threads, Boost, and axon_logging
target_link_libraries(
    ${PROJECT_NAME}
    pthread
    Threads::Threads
    axon_logging
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::filesystem
)

# =============================================================================
# Also create an object library for testing
# =============================================================================
add_library(${PROJECT_NAME}_obj OBJECT src/mock_plugin.cpp)

target_include_directories(
    ${PROJECT_NAME}_obj
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link axon_logging for include paths (needed for logging headers)
target_link_libraries(${PROJECT_NAME}_obj PUBLIC axon_logging)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(${PROJECT_NAME})
    axon_add_coverage(${PROJECT_NAME}_obj)
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

install(FILES include/mock_plugin.hpp DESTINATION include)

# =============================================================================
# Tests
# =============================================================================

# Simple plugin load test (requires building core libs first)
add_executable(
    test_mock_plugin_load
    test/test_mock_plugin_load.cpp
    ${AXON_APPS_DIR}/axon_recorder/plugin_loader.cpp
)

target_include_directories(
    test_mock_plugin_load
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../apps/axon_recorder
)

target_link_libraries(test_mock_plugin_load ${CMAKE_DL_LIBS})

# E2E test with mock plugin
add_executable(test_mock_plugin_e2e test/test_mock_plugin_e2e.cpp)

target_include_directories(
    test_mock_plugin_e2e
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link test executable with axon_logging
target_link_libraries(
    test_mock_plugin_e2e
    ${PROJECT_NAME}
    ${CMAKE_DL_LIBS}
    pthread
    axon_logging
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::filesystem
)

# Install tests
install(
    TARGETS test_mock_plugin_load test_mock_plugin_e2e
    RUNTIME DESTINATION bin
)
