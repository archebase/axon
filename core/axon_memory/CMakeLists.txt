cmake_minimum_required(VERSION 3.14)
project(axon_memory)

# =============================================================================
# Centralized CMake Modules
# =============================================================================
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    set(_AXON_CMAKE_MODULES_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules"
    )
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)

# =============================================================================
# axon_memory Library
# =============================================================================
add_library(
    axon_memory
    ${CMAKE_CURRENT_SOURCE_DIR}/buffer_pool.cpp
)

axon_set_position_independent_code(axon_memory)

target_include_directories(
    axon_memory
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_memory)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_MEMORY_BUILD_TESTS "Build axon_memory tests" OFF)

if(AXON_MEMORY_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# =============================================================================
# Benchmarks (optional, off by default — zero-dependency micro-bench for
# quickly sanity-checking the pool vs std::vector allocation path on a given
# host. Enable with -DAXON_MEMORY_BUILD_BENCH=ON.)
# =============================================================================
option(AXON_MEMORY_BUILD_BENCH "Build axon_memory micro-benchmarks" OFF)

if(AXON_MEMORY_BUILD_BENCH OR AXON_BUILD_BENCH)
    find_package(Threads REQUIRED)
    add_subdirectory(bench)
endif()
