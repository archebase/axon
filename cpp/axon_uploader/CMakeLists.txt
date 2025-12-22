cmake_minimum_required(VERSION 3.14)
project(axon_uploader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Fetch minio-cpp Library
# Source: https://github.com/minio/minio-cpp
# =============================================================================

include(FetchContent)

# minio-cpp requires these dependencies
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)

# Fetch minio-cpp
FetchContent_Declare(
    miniocpp
    GIT_REPOSITORY https://github.com/minio/minio-cpp.git
    GIT_TAG        v0.3.0
)

# Set minio-cpp build options
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(miniocpp)

message(STATUS "axon_uploader: minio-cpp fetched")

# =============================================================================
# Find SQLite3 (system library)
# =============================================================================

find_package(SQLite3 REQUIRED)
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
# Tests (optional)
# =============================================================================

option(AXON_UPLOADER_BUILD_TESTS "Build uploader tests" OFF)

if(AXON_UPLOADER_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    
    # Test upload queue
    add_executable(test_upload_queue
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_upload_queue.cpp
    )
    target_link_libraries(test_upload_queue
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME test_upload_queue COMMAND test_upload_queue)
    
    # Test retry handler
    add_executable(test_retry_handler
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_retry_handler.cpp
    )
    target_link_libraries(test_retry_handler
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME test_retry_handler COMMAND test_retry_handler)
    
    # Test state manager
    add_executable(test_state_manager
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_state_manager.cpp
    )
    target_link_libraries(test_state_manager
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME test_state_manager COMMAND test_state_manager)
    
    # Integration test (requires MinIO)
    add_executable(test_edge_uploader
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_edge_uploader.cpp
    )
    target_link_libraries(test_edge_uploader
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
    # Note: Integration test is not added to CTest by default
    # It requires MinIO to be running. Run manually:
    #   MINIO_ENDPOINT=http://localhost:9000 ./test_edge_uploader
endif()

# =============================================================================
# Coverage support
# =============================================================================

option(AXON_UPLOADER_ENABLE_COVERAGE "Enable code coverage" OFF)

if(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(axon_uploader PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(axon_uploader PRIVATE --coverage)
endif()

