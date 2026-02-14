# =============================================================================
# AxonCoverage.cmake
# =============================================================================
# Unified coverage support for all Axon modules.
#
# Usage in module CMakeLists.txt:
#   include(AxonCoverage)
#   axon_add_coverage(my_target)
#
# Or enable via environment variable:
#   export AXON_ENABLE_COVERAGE=1
# =============================================================================

# Check environment variable (CI often uses this)
if(
    DEFINED ENV{AXON_ENABLE_COVERAGE}
    AND "$ENV{AXON_ENABLE_COVERAGE}" STREQUAL "1"
)
    set(AXON_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage" FORCE)
endif()

# =============================================================================
# axon_add_coverage(target)
# =============================================================================
# Add coverage instrumentation to a target.
# Only works with GCC or Clang compilers.

function(axon_add_coverage TARGET)
    if(NOT AXON_ENABLE_COVERAGE)
        return()
    endif()

    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(
            WARNING
            "Coverage requested but compiler is ${CMAKE_CXX_COMPILER_ID} (only GCC/Clang supported)"
        )
        return()
    endif()

    message(STATUS "Adding coverage instrumentation to ${TARGET}")

    # Compile flags
    target_compile_options(
        ${TARGET}
        PRIVATE --coverage -fprofile-arcs -ftest-coverage
    )

    # Link flags
    target_link_options(${TARGET} PRIVATE --coverage)
endfunction()

# =============================================================================
# axon_enable_coverage_for_directory()
# =============================================================================
# Enable coverage for all targets in the current directory and subdirectories.
# Useful for test directories.

macro(axon_enable_coverage_for_directory)
    if(AXON_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "${PROJECT_NAME}: Coverage instrumentation enabled")

        # Set compile flags for all targets in this directory
        set(CMAKE_CXX_FLAGS
            "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage"
        )
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
    endif()
endmacro()
