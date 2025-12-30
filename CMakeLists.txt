# =============================================================================
# Axon by ArcheBase - Root CMake Configuration
# =============================================================================
# This file provides centralized configuration for all Axon modules.
# Individual modules can be built standalone (for faster iteration) or
# together from the repository root.
#
# Build from root:
#   cmake -B build -S . -DAXON_BUILD_TESTS=ON
#   cmake --build build
#
# Build individual modules (still works):
#   cmake -B build -S cpp/axon_mcap
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(Axon VERSION 1.0.0 LANGUAGES CXX)

# =============================================================================
# Centralized Path Definitions
# =============================================================================
# All modules use these variables instead of computing relative paths.
# This ensures consistency across the codebase.

get_filename_component(AXON_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(AXON_CPP_DIR "${AXON_REPO_ROOT}/cpp")
set(AXON_LOGGING_DIR "${AXON_CPP_DIR}/axon_logging")
set(AXON_MCAP_DIR "${AXON_CPP_DIR}/axon_mcap")
set(AXON_UPLOADER_DIR "${AXON_CPP_DIR}/axon_uploader")
set(AXON_CMAKE_MODULES_DIR "${AXON_REPO_ROOT}/cmake/Modules")

# Make variables available to child CMakeLists
set(AXON_REPO_ROOT "${AXON_REPO_ROOT}" CACHE INTERNAL "")
set(AXON_CPP_DIR "${AXON_CPP_DIR}" CACHE INTERNAL "")
set(AXON_LOGGING_DIR "${AXON_LOGGING_DIR}" CACHE INTERNAL "")
set(AXON_MCAP_DIR "${AXON_MCAP_DIR}" CACHE INTERNAL "")
set(AXON_UPLOADER_DIR "${AXON_UPLOADER_DIR}" CACHE INTERNAL "")
set(AXON_CMAKE_MODULES_DIR "${AXON_CMAKE_MODULES_DIR}" CACHE INTERNAL "")

# =============================================================================
# Common CMake Policies
# =============================================================================
# These policies apply to all Axon modules.

# Suppress CMP0167 warning (FindBoost module removed in CMake 3.30+)
# Use the new BoostConfig.cmake instead of FindBoost.cmake
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# =============================================================================
# Shared Options
# =============================================================================
# These options can be set from the command line and propagate to all modules.

option(AXON_ENABLE_COVERAGE "Enable code coverage for all Axon modules" OFF)
option(AXON_BUILD_TESTS "Build tests for all Axon modules" OFF)
option(AXON_BUILD_UPLOADER "Build axon_uploader (requires AWS SDK)" OFF)

# =============================================================================
# Include CMake Modules
# =============================================================================
list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")

include(AxonStdlib OPTIONAL)
include(AxonCoverage OPTIONAL)

# =============================================================================
# Module Subdirectories
# =============================================================================
# Add C++ libraries in dependency order.

# Logging (no dependencies)
if(EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
    add_subdirectory(${AXON_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
endif()

# MCAP (depends on logging)
if(EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
    add_subdirectory(${AXON_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
endif()

# Uploader (depends on MCAP, logging, AWS SDK)
if(AXON_BUILD_UPLOADER AND EXISTS "${AXON_UPLOADER_DIR}/CMakeLists.txt")
    add_subdirectory(${AXON_UPLOADER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_uploader)
endif()

# =============================================================================
# Summary
# =============================================================================

message(STATUS "")
message(STATUS "=== Axon Configuration ===")
message(STATUS "  Repository Root: ${AXON_REPO_ROOT}")
message(STATUS "  C++ Libraries:    ${AXON_CPP_DIR}")
message(STATUS "  Coverage:         ${AXON_ENABLE_COVERAGE}")
message(STATUS "  Tests:            ${AXON_BUILD_TESTS}")
message(STATUS "  Uploader:         ${AXON_BUILD_UPLOADER}")
message(STATUS "========================")
message(STATUS "")
