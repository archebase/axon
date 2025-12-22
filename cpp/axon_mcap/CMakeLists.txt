cmake_minimum_required(VERSION 3.14)  # FetchContent requires 3.14+
project(axon_mcap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Fetch MCAP Library from Foxglove (Header-only)
# Source: https://github.com/foxglove/mcap
# =============================================================================

include(FetchContent)

# Pin to a specific release for reproducibility
# releases/cpp/v2.1.1 -> commit b2953496735e7b89d5b2ea58be73abed85317c5f
# Check https://github.com/foxglove/mcap/releases for latest C++ releases
set(MCAP_COMMIT_HASH "b2953496735e7b89d5b2ea58be73abed85317c5f" CACHE STRING "MCAP commit hash for releases/cpp/v2.1.1")
set(MCAP_DOWNLOAD_URL "https://github.com/foxglove/mcap/archive/${MCAP_COMMIT_HASH}.tar.gz")
set(MCAP_TARBALL "${CMAKE_CURRENT_BINARY_DIR}/mcap-${MCAP_COMMIT_HASH}.tar.gz")
set(mcap_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/mcap-${MCAP_COMMIT_HASH}")

# Download if not already present
if(NOT EXISTS "${mcap_SOURCE_DIR}")
    message(STATUS "Downloading MCAP library from ${MCAP_DOWNLOAD_URL}")
    file(DOWNLOAD "${MCAP_DOWNLOAD_URL}" "${MCAP_TARBALL}" 
         STATUS DOWNLOAD_STATUS
         SHOW_PROGRESS)
    list(GET DOWNLOAD_STATUS 0 DOWNLOAD_ERROR)
    if(DOWNLOAD_ERROR)
        message(FATAL_ERROR "Failed to download MCAP: ${DOWNLOAD_STATUS}")
    endif()
    
    # Extract
    message(STATUS "Extracting MCAP library to ${CMAKE_CURRENT_BINARY_DIR}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${MCAP_TARBALL}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        RESULT_VARIABLE EXTRACT_RESULT
    )
    if(EXTRACT_RESULT)
        message(FATAL_ERROR "Failed to extract MCAP: ${EXTRACT_RESULT}")
    endif()
    
    # Clean up tarball
    file(REMOVE "${MCAP_TARBALL}")
endif()

message(STATUS "MCAP library located at: ${mcap_SOURCE_DIR}")

# =============================================================================
# Find compression libraries (optional but recommended)
# =============================================================================

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZSTD QUIET libzstd)
    pkg_check_modules(LZ4 QUIET liblz4)
endif()

# =============================================================================
# MCAP Headers Interface Library
# =============================================================================

add_library(mcap_headers INTERFACE)
target_include_directories(mcap_headers INTERFACE
    $<BUILD_INTERFACE:${mcap_SOURCE_DIR}/cpp/mcap/include>
    $<INSTALL_INTERFACE:include>
)

# Configure compression support
# The MCAP library uses MCAP_COMPRESSION_NO_ZSTD and MCAP_COMPRESSION_NO_LZ4
# to DISABLE compression. By default, compression is enabled, so we need to
# define these macros when the libraries are NOT available.
if(ZSTD_FOUND)
    target_link_libraries(mcap_headers INTERFACE ${ZSTD_LIBRARIES})
    target_include_directories(mcap_headers INTERFACE ${ZSTD_INCLUDE_DIRS})
    message(STATUS "MCAP: Zstd compression enabled")
else()
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_NO_ZSTD)
    message(STATUS "MCAP: Zstd compression disabled (libzstd not found)")
endif()

if(LZ4_FOUND)
    target_link_libraries(mcap_headers INTERFACE ${LZ4_LIBRARIES})
    target_include_directories(mcap_headers INTERFACE ${LZ4_INCLUDE_DIRS})
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

# Link compression libraries if available
if(ZSTD_FOUND)
    target_link_libraries(axon_mcap PUBLIC ${ZSTD_LIBRARIES})
endif()

if(LZ4_FOUND)
    target_link_libraries(axon_mcap PUBLIC ${LZ4_LIBRARIES})
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_MCAP_BUILD_TESTS "Build MCAP tests" OFF)

if(AXON_MCAP_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    
    add_executable(test_mcap_writer
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mcap_writer.cpp
    )
    
    target_include_directories(test_mcap_writer PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
    target_link_libraries(test_mcap_writer
        axon_mcap
        GTest::gtest
        GTest::gtest_main
    )
    
    # Link compression libraries for tests
    if(ZSTD_FOUND)
        target_link_libraries(test_mcap_writer ${ZSTD_LIBRARIES})
    endif()
    if(LZ4_FOUND)
        target_link_libraries(test_mcap_writer ${LZ4_LIBRARIES})
    endif()
    
    add_test(NAME test_mcap_writer COMMAND test_mcap_writer)
endif()
