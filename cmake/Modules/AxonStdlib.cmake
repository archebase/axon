# =============================================================================
# AxonStdlib.cmake
# =============================================================================
# Standard C++ library configuration for all Axon modules.
#
# Usage in module CMakeLists.txt:
#   include(AxonStdlib)
# =============================================================================

# =============================================================================
# C++ Standard
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# Common Build Policies
# =============================================================================

# Suppress CMP0167 warning (FindBoost module removed in CMake 3.30+)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# =============================================================================
# Helper: axon_set_position_independent_code(target)
# =============================================================================
# Enable PIC for linking into shared libraries (required for ROS 2).

function(axon_set_position_independent_code TARGET)
    set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endfunction()

# =============================================================================
# Helper: axon_enable_warnings(target)
# =============================================================================
# Enable standard warning flags for a target.

function(axon_enable_warnings TARGET)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${TARGET} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
        )
    elseif(MSVC)
        target_compile_options(${TARGET} PRIVATE /W4)
    endif()
endfunction()

# =============================================================================
# Helper: axon_add_boost_log_dyn_link(target)
# =============================================================================
# Add BOOST_LOG_DYN_LINK definition for shared Boost libraries.

function(axon_add_boost_log_dyn_link TARGET)
    target_compile_definitions(${TARGET} PUBLIC BOOST_LOG_DYN_LINK)
endfunction()
