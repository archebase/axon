cmake_minimum_required(VERSION 3.14)
project(axon_logging)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Centralized CMake Modules
# =============================================================================
# All Axon modules must use the centralized cmake/Modules directory.
# This ensures consistent configuration across the codebase.
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
else()
    # Use AXON_REPO_ROOT (set by root CMakeLists.txt)
    set(_AXON_CMAKE_MODULES_DIR "${AXON_REPO_ROOT}/cmake/Modules")
    if(NOT EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdlib.cmake")
        message(FATAL_ERROR
            "Axon CMake modules not found at: ${_AXON_CMAKE_MODULES_DIR}\n"
            "Please ensure you're building from the Axon repository root."
        )
    endif()
    list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
endif()

# Include required modules (fail if not found)
include(AxonStdlib REQUIRED)
include(AxonCoverage REQUIRED)

# =============================================================================
# Find Boost.Log and dependencies
# =============================================================================
# Boost 1.71+ required (Ubuntu 20.04/ROS Noetic minimum)
# Ubuntu 22.04+ ships Boost 1.74+

find_package(Boost 1.71 REQUIRED COMPONENTS
    log
    log_setup
    thread
    filesystem
)

if(Boost_FOUND)
    message(STATUS "axon_logging: Found Boost ${Boost_VERSION}")
endif()

# =============================================================================
# axon_logging Library
# =============================================================================
add_library(axon_logging
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_log_init.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_console_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_file_sink.cpp
)

# Enable position-independent code
axon_set_position_independent_code(axon_logging)

target_include_directories(axon_logging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Add BOOST_LOG_DYN_LINK definition
axon_add_boost_log_dyn_link(axon_logging)

target_link_libraries(axon_logging PUBLIC
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::filesystem
)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_logging)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_LOGGING_BUILD_TESTS "Build logging tests" OFF)

if(AXON_LOGGING_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
