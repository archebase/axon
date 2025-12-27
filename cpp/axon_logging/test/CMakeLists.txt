cmake_minimum_required(VERSION 3.14)
project(axon_logging_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suppress CMP0167 warning (FindBoost module is removed in newer CMake)
# Use the new Boost Config package instead when available
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Find dependencies
find_package(GTest REQUIRED)
include(GoogleTest)  # Required for gtest_discover_tests()
find_package(Threads REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS log log_setup thread filesystem)

# Include parent for axon_logging headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# Build axon_logging if not already available
if(NOT TARGET axon_logging)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/axon_logging_lib)
endif()

enable_testing()

# =============================================================================
# Test Executables (Auto-Discovery)
# =============================================================================

# Auto-discover all test files
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp"
)

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_compile_definitions(${TEST_NAME} PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(${TEST_NAME}
        axon_logging
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        ${Boost_LIBRARIES}
    )
    # Apply coverage flags
    if(AXON_LOGGING_ENABLE_COVERAGE AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
        target_compile_options(${TEST_NAME} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(${TEST_NAME} PRIVATE --coverage)
    endif()
    # Use gtest_discover_tests for automatic test case discovery
    gtest_discover_tests(${TEST_NAME} PROPERTIES LABELS "unit")
endforeach()

