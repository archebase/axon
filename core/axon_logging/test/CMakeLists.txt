# =============================================================================
# Axon Logging Tests
# =============================================================================

cmake_minimum_required(VERSION 3.12)

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
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/Modules"
    )
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

# Include required modules (AxonStdLib provides standard helpers)
include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)

# =============================================================================
# Find Dependencies
# =============================================================================
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup thread filesystem)

if(Boost_FOUND)
    message(STATUS "axon_logging tests: Found Boost ${Boost_VERSION}")
endif()

# =============================================================================
# Test Executables (Auto-Discovery)
# =============================================================================
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/*.cpp)

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_SOURCE})

    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    target_link_libraries(
        ${TEST_NAME}
        axon_logging
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
    )

    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

    # Coverage flags
    if(COMMAND axon_add_coverage)
        axon_add_coverage(${TEST_NAME})
    endif()
endforeach()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Axon Logging Test Configuration ===")
message(STATUS "  Tests: ${TEST_SOURCES}")
message(STATUS "  Coverage: ${AXON_ENABLE_COVERAGE}")
message(STATUS "====================================")
message(STATUS "")
