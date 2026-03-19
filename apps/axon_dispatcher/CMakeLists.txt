# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

cmake_minimum_required(VERSION 3.14)
project(axon_dispatcher VERSION 0.3.0 LANGUAGES CXX)

# =============================================================================
# Installation Configuration
# =============================================================================
# Set install prefix for packaging (defaults to /opt/axon)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/opt/axon" CACHE PATH "Installation prefix" FORCE)
endif()

# =============================================================================
# Centralized CMake Modules
# =============================================================================
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    set(_AXON_CMAKE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules")
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

include(AxonStdLib REQUIRED)

# =============================================================================
# Dispatcher Executable
# =============================================================================

add_executable(axon axon.cpp)

# Set C++17
target_compile_features(axon PUBLIC cxx_std_17)

# =============================================================================
# Installation
# =============================================================================
install(TARGETS axon RUNTIME DESTINATION bin)

# Install bash completion
install(FILES completion.bash DESTINATION share/bash-completion/completions
        RENAME axon)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Axon Dispatcher Configuration ===")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Output Binary: axon")
message(STATUS "=================================")
message(STATUS "")
