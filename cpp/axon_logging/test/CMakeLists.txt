cmake_minimum_required(VERSION 3.14)
project(axon_logging_tests)

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
    if(POLICY CMP0167)
        cmake_policy(SET CMP0167 NEW)
    endif()
endif()

# =============================================================================
# Find Dependencies
# =============================================================================
find_package(GTest REQUIRED)
include(GoogleTest)
find_package(Threads REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup thread filesystem)

# Include parent for axon_logging headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# =============================================================================
# Build axon_logging if not already available
# =============================================================================
if(NOT TARGET axon_logging)
    # If built from root, use root's AXON_LOGGING_DIR
    if(DEFINED AXON_LOGGING_DIR)
        add_subdirectory(${AXON_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging_lib)
    else()
        # Fallback to parent directory
        add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/axon_logging_lib)
    endif()
endif()

enable_testing()

# =============================================================================
# Test Executables (Auto-Discovery)
# =============================================================================
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp"
)

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})

    axon_add_boost_log_dyn_link(${TEST_NAME})

    target_link_libraries(${TEST_NAME}
        axon_logging
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        ${Boost_LIBRARIES}
    )

    # Apply coverage flags
    if(COMMAND axon_add_coverage)
        axon_add_coverage(${TEST_NAME})
    elseif(AXON_LOGGING_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # Fallback for legacy builds
        target_compile_options(${TEST_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(${TEST_NAME} PRIVATE --coverage)
    endif()

    # Use gtest_discover_tests for automatic test case discovery
    gtest_discover_tests(${TEST_NAME} PROPERTIES LABELS "unit")
endforeach()
