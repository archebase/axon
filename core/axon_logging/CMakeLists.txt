cmake_minimum_required(VERSION 3.14)
project(axon_logging)

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
# Find Boost.Log and dependencies
# =============================================================================
# Boost 1.71+ required (Ubuntu 20.04/ROS Noetic minimum)
# Ubuntu 22.04+ ships Boost 1.74+
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup thread filesystem)

if(Boost_FOUND)
    message(STATUS "axon_logging: Found Boost ${Boost_VERSION}")
endif()

# =============================================================================
# axon_logging Library
# =============================================================================
add_library(
    axon_logging
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_log_init.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_console_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_file_sink.cpp
)

# Enable position-independent code
axon_set_position_independent_code(axon_logging)

target_include_directories(
    axon_logging
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

# Add BOOST_LOG_DYN_LINK definition
axon_add_boost_log_dyn_link(axon_logging)

target_link_libraries(
    axon_logging
    PUBLIC Boost::log Boost::log_setup Boost::thread Boost::filesystem
)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_logging)
endif()

# =============================================================================
# Install
# =============================================================================
include(GNUInstallDirs)

install(
    TARGETS axon_logging
    EXPORT axon_logging_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/axon_logging
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
)

# Export library for use by other packages
install(
    EXPORT axon_logging_targets
    FILE axon_logging_targets.cmake
    NAMESPACE axon::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axon_logging
)

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_LOGGING_BUILD_TESTS "Build logging tests" OFF)

if(AXON_LOGGING_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
