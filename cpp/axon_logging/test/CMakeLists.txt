cmake_minimum_required(VERSION 3.14)
project(axon_logging_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(GTest REQUIRED)
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
# Test Executables
# =============================================================================

# Test severity level parsing and logging initialization
add_executable(test_log_init
    test_log_init.cpp
)
target_compile_definitions(test_log_init PRIVATE BOOST_LOG_DYN_LINK)
target_link_libraries(test_log_init
    axon_logging
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
    ${Boost_LIBRARIES}
)
add_test(NAME LogInitTest COMMAND test_log_init)

# Test console sink
add_executable(test_console_sink
    test_console_sink.cpp
)
target_compile_definitions(test_console_sink PRIVATE BOOST_LOG_DYN_LINK)
target_link_libraries(test_console_sink
    axon_logging
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
    ${Boost_LIBRARIES}
)
add_test(NAME ConsoleSinkTest COMMAND test_console_sink)

# Test file sink
add_executable(test_file_sink
    test_file_sink.cpp
)
target_compile_definitions(test_file_sink PRIVATE BOOST_LOG_DYN_LINK)
target_link_libraries(test_file_sink
    axon_logging
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
    ${Boost_LIBRARIES}
)
add_test(NAME FileSinkTest COMMAND test_file_sink)

# =============================================================================
# Coverage Support
# =============================================================================

if(AXON_LOGGING_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_log_init PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(test_log_init PRIVATE --coverage)
    
    target_compile_options(test_console_sink PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(test_console_sink PRIVATE --coverage)
    
    target_compile_options(test_file_sink PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(test_file_sink PRIVATE --coverage)
endif()

