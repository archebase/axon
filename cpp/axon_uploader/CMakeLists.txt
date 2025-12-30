cmake_minimum_required(VERSION 3.14)
project(axon_uploader)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Use Axon Root Configuration (if available)
# =============================================================================
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
    include(AxonStdlib OPTIONAL)
    include(AxonCoverage OPTIONAL)
else()
    # Fallback for standalone builds
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    # Define fallback helper functions
    function(axon_set_position_independent_code TARGET)
        set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endfunction()
    function(axon_add_coverage TARGET)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${TARGET} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(${TARGET} PRIVATE --coverage)
        endif()
    endfunction()
endif()

# =============================================================================
# Check Dependencies (optional build)
# =============================================================================
# The uploader requires AWS SDK (S3), OpenSSL, and SQLite3.
# If these are not available, the module will be skipped.
#
# AWS SDK must be built from source (no apt packages available):
#   git clone --recurse-submodules https://github.com/aws/aws-sdk-cpp
#   cd aws-sdk-cpp && cmake -B build -DBUILD_ONLY="s3;transfer" && cmake --build build
#   sudo cmake --install build
#
# Other dependencies:
#   Ubuntu/Debian: sudo apt-get install libssl-dev libsqlite3-dev libcurl4-openssl-dev
#   macOS: brew install openssl sqlite3 curl

find_package(OpenSSL QUIET)
find_package(SQLite3 QUIET)
find_package(AWSSDK QUIET COMPONENTS s3 transfer)

if(NOT OPENSSL_FOUND)
    message(WARNING "axon_uploader: OpenSSL not found - skipping build. Install with: sudo apt-get install libssl-dev")
    return()
endif()

if(NOT SQLite3_FOUND)
    message(WARNING "axon_uploader: SQLite3 not found - skipping build. Install with: sudo apt-get install libsqlite3-dev")
    return()
endif()

if(NOT AWSSDK_FOUND)
    message(WARNING "axon_uploader: AWS SDK not found - skipping build. AWS SDK must be built from source (see CMakeLists.txt comments)")
    return()
endif()

message(STATUS "axon_uploader: All dependencies found, building uploader")
message(STATUS "axon_uploader: Found SQLite3 ${SQLite3_VERSION}")
message(STATUS "axon_uploader: Found AWS SDK")

# =============================================================================
# Find axon_mcap (sibling module - for MCAP validation)
# =============================================================================
# Use centralized AXON_MCAP_DIR if available, otherwise fall back to sibling

if(DEFINED AXON_MCAP_DIR AND EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
    # Use root's path (built from repo root)
    set(MCAP_BUILD_DIR "${AXON_MCAP_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../axon_mcap/CMakeLists.txt")
    # Fallback to sibling directory
    set(MCAP_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../axon_mcap")
else()
    message(STATUS "axon_mcap not found - MCAP validation will not be available")
    set(MCAP_BUILD_DIR "")
endif()

if(MCAP_BUILD_DIR)
    message(STATUS "Found axon_mcap at: ${MCAP_BUILD_DIR}")
    # Only add subdirectory if not already added
    if(NOT TARGET axon_mcap)
        # Propagate coverage flag
        # Must explicitly set OFF to clear cached values from previous builds
        if(AXON_ENABLE_COVERAGE)
            set(AXON_MCAP_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage for axon_mcap" FORCE)
        else()
            set(AXON_MCAP_ENABLE_COVERAGE OFF CACHE BOOL "Enable coverage for axon_mcap" FORCE)
        endif()
        add_subdirectory(${MCAP_BUILD_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
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

axon_set_position_independent_code(axon_uploader)

target_include_directories(axon_uploader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(axon_uploader PUBLIC
    ${AWSSDK_LINK_LIBRARIES}
    SQLite::SQLite3
    OpenSSL::SSL
    OpenSSL::Crypto
)

# Link axon_mcap if available (for validation)
if(TARGET axon_mcap)
    target_link_libraries(axon_uploader PUBLIC axon_mcap)
    target_compile_definitions(axon_uploader PRIVATE AXON_HAS_MCAP)
endif()

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_uploader)
elseif(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Fallback for legacy builds
    message(STATUS "axon_uploader: Coverage instrumentation enabled")
    target_compile_options(axon_uploader PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(axon_uploader PRIVATE --coverage)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_UPLOADER_BUILD_TESTS "Build uploader tests" OFF)

if(AXON_UPLOADER_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    # Auto-discover unit test files in test/ (integration/ tests are built separately)
    file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp"
    )

    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

        target_link_libraries(${TEST_NAME}
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
        elseif(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${TEST_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(${TEST_NAME} PRIVATE --coverage)
        endif()

        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()

    # Integration test (requires MinIO) - not added to CTest by default
    add_executable(test_edge_uploader
        ${CMAKE_CURRENT_SOURCE_DIR}/test/integration/test_edge_uploader.cpp
    )
    target_link_libraries(test_edge_uploader
        axon_uploader
        GTest::gtest
        GTest::gtest_main
    )
    if(COMMAND axon_add_coverage)
        axon_add_coverage(test_edge_uploader)
    elseif(AXON_UPLOADER_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_edge_uploader PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(test_edge_uploader PRIVATE --coverage)
    endif()
    # Note: Integration test is not added to CTest by default
    # It requires MinIO to be running. Run manually:
    #   MINIO_ENDPOINT=http://localhost:9000 ./test_edge_uploader
endif()
