cmake_minimum_required(VERSION 3.14)
project(axon_uploader)

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
        "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules"
    )
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

# Include required modules (AxonStdLib provides standard helpers)
include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)

# =============================================================================
# Check Dependencies
# =============================================================================
# The uploader requires AWS SDK (S3), OpenSSL, and SQLite3.
# If AWS SDK is not available, the uploader is skipped gracefully.
#
# AWS SDK installation (Ubuntu 22.04):
#   # Build from source (no apt packages available)
#   git clone --recurse-submodules https://github.com/aws/aws-sdk-cpp
#   cd aws-sdk-cpp && cmake -B build -DBUILD_ONLY="s3;transfer" -DBUILD_SHARED_LIBS=ON
#   cmake --build build -j$(nproc) && sudo cmake --install build
#   sudo ldconfig
#
# Other dependencies:
#   Ubuntu/Debian: sudo apt-get install libssl-dev libsqlite3-dev libcurl4-openssl-dev
#   macOS: brew install openssl sqlite3 curl

find_package(OpenSSL QUIET)
find_package(SQLite3 QUIET)
find_package(ZLIB QUIET)
find_package(AWSSDK QUIET COMPONENTS s3 transfer)

# Check if all required dependencies are available
set(_AXON_UPLOADER_DEPS_MET TRUE)

if(NOT OpenSSL_FOUND)
    message(STATUS "axon_uploader: OpenSSL not found, skipping uploader build")
    set(_AXON_UPLOADER_DEPS_MET FALSE)
endif()

if(NOT SQLite3_FOUND)
    message(STATUS "axon_uploader: SQLite3 not found, skipping uploader build")
    set(_AXON_UPLOADER_DEPS_MET FALSE)
endif()

if(NOT AWSSDK_FOUND)
    message(STATUS "axon_uploader: AWS SDK not found, skipping uploader build")
    message(
        STATUS
        "axon_uploader: To enable, install AWS SDK from https://github.com/aws/aws-sdk-cpp"
    )
    set(_AXON_UPLOADER_DEPS_MET FALSE)
endif()

# Exit early if dependencies are not met
if(NOT _AXON_UPLOADER_DEPS_MET)
    return()
endif()

message(STATUS "axon_uploader: All dependencies found, building uploader")
message(STATUS "axon_uploader: Found SQLite3 ${SQLite3_VERSION}")
message(STATUS "axon_uploader: Found AWS SDK")

# =============================================================================
# axon_uploader Library
# =============================================================================

add_library(
    axon_uploader
    ${CMAKE_CURRENT_SOURCE_DIR}/upload_queue.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/s3_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/upload_state_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/edge_uploader.cpp
)

axon_set_position_independent_code(axon_uploader)

target_include_directories(
    axon_uploader
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    axon_uploader
    PUBLIC ${AWSSDK_LINK_LIBRARIES} SQLite::SQLite3 OpenSSL::Crypto OpenSSL::SSL
)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_uploader)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_UPLOADER_BUILD_TESTS "Build uploader tests" OFF)

if(AXON_UPLOADER_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # Auto-discover unit test files in test/ (integration/ tests are built separately)
    file(
        GLOB TEST_SOURCES
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp"
    )

    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

        add_executable(${TEST_NAME} ${TEST_SOURCE})

        target_include_directories(
            ${TEST_NAME}
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        )

        target_link_libraries(
            ${TEST_NAME}
            axon_uploader
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
        elseif(
            AXON_UPLOADER_ENABLE_COVERAGE
            AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"
        )
            target_compile_options(
                ${TEST_NAME}
                PRIVATE --coverage -fprofile-arcs -ftest-coverage
            )
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif()

        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()

    # Integration test (requires MinIO) - not added to CTest by default
    add_executable(
        test_edge_uploader
        ${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_edge_uploader.cpp
    )

    target_include_directories(
        test_edge_uploader
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_link_libraries(
        test_edge_uploader
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )

    if(COMMAND axon_add_coverage)
        axon_add_coverage(test_edge_uploader)
    elseif(
        AXON_UPLOADER_ENABLE_COVERAGE
        AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"
    )
        target_compile_options(
            test_edge_uploader
            PRIVATE --coverage -fprofile-arcs -ftest-coverage
        )
        target_link_options(test_edge_uploader PRIVATE --coverage)
    endif()

    # Note: Integration test is not added to CTest by default
    # It requires MinIO to be running. Run manually:
    #   MINIO_ENDPOINT=http://localhost:9000 ./test_edge_uploader
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Axon Uploader Configuration ===")
message(STATUS "  SQLite3: ${SQLite3_VERSION}")
message(STATUS "  AWS SDK: Found")
message(STATUS "  Tests: ${AXON_UPLOADER_BUILD_TESTS}")
message(STATUS "=====================================")
message(STATUS "")
