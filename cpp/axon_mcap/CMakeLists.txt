cmake_minimum_required(VERSION 3.14)  # FetchContent requires 3.14+
project(axon_mcap)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Fetch MCAP Library from Foxglove (Header-only)
# Source: https://github.com/foxglove/mcap
# =============================================================================

include(FetchContent)

# Pin to a specific commit for reproducibility
# releases/cpp/v2.1.1 -> commit b2953496735e7b89d5b2ea58be73abed85317c5f
# Check https://github.com/foxglove/mcap/releases for latest C++ releases
set(MCAP_COMMIT "b2953496735e7b89d5b2ea58be73abed85317c5f" CACHE STRING "MCAP commit hash")

# Use URL-based FetchContent (more reliable in CI Docker environments than GIT_REPOSITORY)
# GIT_REPOSITORY can hang waiting for SSH auth or GitHub rate limits
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
    # Use IMPORTED_TARGET to create proper CMake targets with full library paths
    pkg_check_modules(ZSTD QUIET IMPORTED_TARGET libzstd)
    pkg_check_modules(LZ4 QUIET IMPORTED_TARGET liblz4)
endif()

# =============================================================================
# MCAP Headers Interface Library
# =============================================================================

add_library(mcap_headers INTERFACE)
# Use SYSTEM to suppress warnings from external MCAP headers
target_include_directories(mcap_headers SYSTEM INTERFACE
    $<BUILD_INTERFACE:${mcap_SOURCE_DIR}/cpp/mcap/include>
    $<INSTALL_INTERFACE:include>
)

# Configure compression support
# The MCAP library uses MCAP_COMPRESSION_NO_ZSTD and MCAP_COMPRESSION_NO_LZ4
# to DISABLE compression. By default, compression is enabled, so we need to
# define these macros when the libraries are NOT available.
if(ZSTD_FOUND)
    # Use the imported target which includes full library paths
    target_link_libraries(mcap_headers INTERFACE PkgConfig::ZSTD)
    message(STATUS "MCAP: Zstd compression enabled")
else()
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_NO_ZSTD)
    message(STATUS "MCAP: Zstd compression disabled (libzstd not found)")
endif()

if(LZ4_FOUND)
    # Use the imported target which includes full library paths
    target_link_libraries(mcap_headers INTERFACE PkgConfig::LZ4)
    message(STATUS "MCAP: LZ4 compression enabled")
else()
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_NO_LZ4)
    message(STATUS "MCAP: LZ4 compression disabled (liblz4 not found)")
endif()

# =============================================================================
# axon_logging Library (for structured logging)
# =============================================================================

# Find axon_logging directory - check multiple possible locations
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../axon_logging/CMakeLists.txt")
    set(AXON_LOGGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../axon_logging")
elseif(EXISTS "${PROJECT_ROOT}/cpp/axon_logging/CMakeLists.txt")
    set(AXON_LOGGING_DIR "${PROJECT_ROOT}/cpp/axon_logging")
else()
    message(STATUS "axon_logging not found - logging support disabled for axon_mcap")
    set(AXON_LOGGING_DIR "")
endif()

if(AXON_LOGGING_DIR)
    message(STATUS "Found axon_logging at: ${AXON_LOGGING_DIR}")
    # Propagate coverage flag to axon_logging subdirectory
    if(AXON_MCAP_ENABLE_COVERAGE)
        set(AXON_LOGGING_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage for axon_logging" FORCE)
    endif()
    add_subdirectory(${AXON_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
endif()

# =============================================================================
# MCAP Writer Wrapper Library
# =============================================================================

add_library(axon_mcap
    ${CMAKE_CURRENT_SOURCE_DIR}/mcap_writer_wrapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mcap_validator.cpp
)

# Enable position-independent code for linking into shared libraries (e.g., ROS 2 ament packages)
set_target_properties(axon_mcap PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(axon_mcap PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(axon_mcap PUBLIC
    mcap_headers
)

# Link axon_logging if available
if(AXON_LOGGING_DIR)
    target_link_libraries(axon_mcap PUBLIC axon_logging)
    target_compile_definitions(axon_mcap PRIVATE AXON_HAS_LOGGING)
endif()

# Link compression libraries if available (using imported targets for full paths)
if(ZSTD_FOUND)
    target_link_libraries(axon_mcap PUBLIC PkgConfig::ZSTD)
endif()

if(LZ4_FOUND)
    target_link_libraries(axon_mcap PUBLIC PkgConfig::LZ4)
endif()

# =============================================================================
# Coverage Support
# =============================================================================

option(AXON_MCAP_ENABLE_COVERAGE "Enable code coverage" OFF)

if(AXON_MCAP_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "axon_mcap: Coverage instrumentation enabled")
    # Use -O0 to prevent compiler from optimizing away branches, which affects coverage accuracy
    target_compile_options(axon_mcap PRIVATE -O0 -g --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(axon_mcap PRIVATE --coverage)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================

option(AXON_MCAP_BUILD_TESTS "Build MCAP tests" OFF)

if(AXON_MCAP_BUILD_TESTS)
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
        
        # Base libraries for all tests
        target_link_libraries(${TEST_NAME}
            axon_mcap
            GTest::gtest
            GTest::gtest_main
        )
        
        # Mocked tests need GoogleMock
        if(TEST_SOURCE MATCHES ".*_mocked\\.cpp$")
            target_link_libraries(${TEST_NAME} GTest::gmock)
        endif()
        
        # Apply coverage flags to all tests (including mocked tests)
        if(AXON_MCAP_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            # Use -O0 to prevent compiler from optimizing away branches, which affects coverage accuracy
            target_compile_options(${TEST_NAME} PRIVATE -O0 -g --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif()
        
        # Link compression libraries (if available)
        if(ZSTD_FOUND)
            target_link_libraries(${TEST_NAME} PkgConfig::ZSTD)
        endif()
        if(LZ4_FOUND)
            target_link_libraries(${TEST_NAME} PkgConfig::LZ4)
        endif()
        
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
endif()
