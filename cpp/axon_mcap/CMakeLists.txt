cmake_minimum_required(VERSION 3.14)  # FetchContent requires 3.14+
project(axon_mcap)

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
# Fetch MCAP Library from Foxglove (Header-only)
# =============================================================================
# Source: https://github.com/foxglove/mcap

include(FetchContent)

# Pin to a specific commit for reproducibility
# releases/cpp/v2.1.1 -> commit b2953496735e7b89d5b2ea58be73abed85317c5f
set(MCAP_COMMIT "b2953496735e7b89d5b2ea58be73abed85317c5f" CACHE STRING "MCAP commit hash")

# Use URL-based FetchContent (more reliable in CI than GIT_REPOSITORY)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    FetchContent_Declare(
        mcap
        URL "https://github.com/foxglove/mcap/archive/${MCAP_COMMIT}.tar.gz"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
else()
    FetchContent_Declare(
        mcap
        URL "https://github.com/foxglove/mcap/archive/${MCAP_COMMIT}.tar.gz"
    )
endif()

FetchContent_MakeAvailable(mcap)

message(STATUS "MCAP library located at: ${mcap_SOURCE_DIR}")

# =============================================================================
# Find compression libraries (optional but recommended)
# =============================================================================

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZSTD QUIET IMPORTED_TARGET libzstd)
    pkg_check_modules(LZ4 QUIET IMPORTED_TARGET liblz4)
endif()

# =============================================================================
# MCAP Headers Interface Library
# =============================================================================

add_library(mcap_headers INTERFACE)
target_include_directories(mcap_headers SYSTEM INTERFACE
    $<BUILD_INTERFACE:${mcap_SOURCE_DIR}/cpp/mcap/include>
    $<INSTALL_INTERFACE:include>
)

# Configure compression support
if(ZSTD_FOUND)
    target_link_libraries(mcap_headers INTERFACE PkgConfig::ZSTD)
    message(STATUS "MCAP: Zstd compression enabled")
else()
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_NO_ZSTD)
    message(STATUS "MCAP: Zstd compression disabled (libzstd not found)")
endif()

if(LZ4_FOUND)
    target_link_libraries(mcap_headers INTERFACE PkgConfig::LZ4)
    message(STATUS "MCAP: LZ4 compression enabled")
else()
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_NO_LZ4)
    message(STATUS "MCAP: LZ4 compression disabled (liblz4 not found)")
endif()

# =============================================================================
# axon_logging Library (for structured logging)
# =============================================================================
# Use centralized AXON_LOGGING_DIR if available, otherwise fall back to sibling

if(DEFINED AXON_LOGGING_DIR AND EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
    # Use root's path (built from repo root)
    set(LOGGING_BUILD_DIR "${AXON_LOGGING_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../axon_logging/CMakeLists.txt")
    # Fallback to sibling directory
    set(LOGGING_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../axon_logging")
else()
    message(STATUS "axon_logging not found - logging support disabled for axon_mcap")
    set(LOGGING_BUILD_DIR "")
endif()

if(LOGGING_BUILD_DIR)
    message(STATUS "Found axon_logging at: ${LOGGING_BUILD_DIR}")
    # Propagate coverage flag to axon_logging subdirectory
    # Must explicitly set OFF to clear cached values from previous builds
    if(AXON_ENABLE_COVERAGE)
        set(AXON_LOGGING_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage for axon_logging" FORCE)
    else()
        set(AXON_LOGGING_ENABLE_COVERAGE OFF CACHE BOOL "Enable coverage for axon_logging" FORCE)
    endif()
    add_subdirectory(${LOGGING_BUILD_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
    # Set for later use
    set(AXON_LOGGING_DIR "${LOGGING_BUILD_DIR}" CACHE INTERNAL "")
endif()

# =============================================================================
# MCAP Writer Wrapper Library
# =============================================================================

add_library(axon_mcap
    ${CMAKE_CURRENT_SOURCE_DIR}/mcap_writer_wrapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mcap_validator.cpp
)

axon_set_position_independent_code(axon_mcap)

target_include_directories(axon_mcap PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(axon_mcap PUBLIC mcap_headers)

# Link axon_logging if available
if(TARGET axon_logging)
    target_link_libraries(axon_mcap PUBLIC axon_logging)
    target_compile_definitions(axon_mcap PRIVATE AXON_HAS_LOGGING)
endif()

# Link compression libraries
if(ZSTD_FOUND)
    target_link_libraries(axon_mcap PUBLIC PkgConfig::ZSTD)
endif()

if(LZ4_FOUND)
    target_link_libraries(axon_mcap PUBLIC PkgConfig::LZ4)
endif()

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_mcap)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_MCAP_BUILD_TESTS "Build MCAP tests" OFF)

if(AXON_MCAP_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # Auto-discover all test files
    file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp"
    )

    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

        target_link_libraries(${TEST_NAME}
            axon_mcap
            GTest::gtest
            GTest::gtest_main
        )

        # Mocked tests need GoogleMock
        if(TEST_SOURCE MATCHES ".*_mocked\\.cpp$")
            target_link_libraries(${TEST_NAME} GTest::gmock)
        endif()

        # Apply coverage flags
        if(COMMAND axon_add_coverage)
            axon_add_coverage(${TEST_NAME})
        elseif(AXON_MCAP_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${TEST_NAME} PRIVATE -O0 -g --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif()

        # Link compression libraries
        if(ZSTD_FOUND)
            target_link_libraries(${TEST_NAME} PkgConfig::ZSTD)
        endif()
        if(LZ4_FOUND)
            target_link_libraries(${TEST_NAME} PkgConfig::LZ4)
        endif()

        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
endif()
