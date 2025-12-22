cmake_minimum_required(VERSION 3.14)
project(axon_uploader)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Check Dependencies (optional build)
# =============================================================================

# The uploader requires CURL, OpenSSL, and SQLite3. If these are not available,
# the module will be skipped. Install them with:
#   Ubuntu/Debian: sudo apt-get install libcurl4-openssl-dev libssl-dev libsqlite3-dev
#   macOS: brew install curl openssl sqlite3

find_package(CURL QUIET)
find_package(OpenSSL QUIET)
find_package(SQLite3 QUIET)

if(NOT CURL_FOUND)
    message(WARNING "axon_uploader: CURL not found - skipping build. Install with: sudo apt-get install libcurl4-openssl-dev")
    return()
endif()

if(NOT OPENSSL_FOUND)
    message(WARNING "axon_uploader: OpenSSL not found - skipping build. Install with: sudo apt-get install libssl-dev")
    return()
endif()

if(NOT SQLite3_FOUND)
    message(WARNING "axon_uploader: SQLite3 not found - skipping build. Install with: sudo apt-get install libsqlite3-dev")
    return()
endif()

message(STATUS "axon_uploader: All dependencies found, building uploader")

# =============================================================================
# Fetch minio-cpp Library
# Source: https://github.com/minio/minio-cpp
# =============================================================================

include(FetchContent)

# Fetch minio-cpp using URL (more reliable in CI than GIT_REPOSITORY)
# GIT_REPOSITORY can hang waiting for SSH auth or GitHub rate limits
set(MINIO_CPP_VERSION "v0.3.0" CACHE STRING "minio-cpp version")
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    FetchContent_Declare(
        miniocpp
        URL "https://github.com/minio/minio-cpp/archive/refs/tags/${MINIO_CPP_VERSION}.tar.gz"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
else()
FetchContent_Declare(
    miniocpp
        URL "https://github.com/minio/minio-cpp/archive/refs/tags/${MINIO_CPP_VERSION}.tar.gz"
)
endif()

# Set minio-cpp build options before FetchContent_MakeAvailable
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(miniocpp)

message(STATUS "axon_uploader: minio-cpp fetched")
message(STATUS "axon_uploader: Found SQLite3 ${SQLite3_VERSION}")

# =============================================================================
# Find axon_mcap (sibling module - for MCAP validation)
# =============================================================================

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../axon_mcap/CMakeLists.txt")
    set(AXON_MCAP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../axon_mcap")
elseif(EXISTS "${PROJECT_ROOT}/cpp/axon_mcap/CMakeLists.txt")
    set(AXON_MCAP_DIR "${PROJECT_ROOT}/cpp/axon_mcap")
else()
    message(STATUS "axon_mcap not found - MCAP validation will not be available")
    set(AXON_MCAP_DIR "")
endif()

if(AXON_MCAP_DIR)
    message(STATUS "Found axon_mcap at: ${AXON_MCAP_DIR}")
    # Only add subdirectory if not already added
    if(NOT TARGET axon_mcap)
        add_subdirectory(${AXON_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
    endif()
endif()

# =============================================================================
# axon_uploader Library
# =============================================================================

add_library(axon_uploader
    ${CMAKE_CURRENT_SOURCE_DIR}/upload_queue.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/s3_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/upload_state_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/edge_uploader.cpp
)

# Enable position-independent code for linking into shared libraries
set_target_properties(axon_uploader PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(axon_uploader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(axon_uploader PUBLIC
    miniocpp::miniocpp
    SQLite::SQLite3
    CURL::libcurl
    OpenSSL::SSL
    OpenSSL::Crypto
)

# Link axon_mcap if available (for validation)
if(AXON_MCAP_DIR AND TARGET axon_mcap)
    target_link_libraries(axon_uploader PUBLIC axon_mcap)
    target_compile_definitions(axon_uploader PRIVATE AXON_HAS_MCAP)
endif()

# =============================================================================
# Coverage support
# =============================================================================

option(AXON_UPLOADER_ENABLE_COVERAGE "Enable code coverage" OFF)

if(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "axon_uploader: Coverage instrumentation enabled")
    target_compile_options(axon_uploader PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(axon_uploader PRIVATE --coverage)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================

option(AXON_UPLOADER_BUILD_TESTS "Build uploader tests" OFF)

if(AXON_UPLOADER_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    
    # Helper function to create test with coverage
    function(add_uploader_test TEST_NAME TEST_SOURCE)
        add_executable(${TEST_NAME}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/${TEST_SOURCE}
    )
        target_link_libraries(${TEST_NAME}
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
        # Apply coverage flags to test executables
        if(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${TEST_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif()
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endfunction()
    
    # Test upload queue
    add_uploader_test(test_upload_queue test_upload_queue.cpp)
    
    # Test retry handler
    add_uploader_test(test_retry_handler test_retry_handler.cpp)
    
    # Test state manager
    add_uploader_test(test_state_manager test_state_manager.cpp)
    
    # Integration test (requires MinIO)
    add_executable(test_edge_uploader
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_edge_uploader.cpp
    )
    target_link_libraries(test_edge_uploader
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
    if(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_edge_uploader PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(test_edge_uploader PRIVATE --coverage)
    endif()
    # Note: Integration test is not added to CTest by default
    # It requires MinIO to be running. Run manually:
    #   MINIO_ENDPOINT=http://localhost:9000 ./test_edge_uploader
endif()

