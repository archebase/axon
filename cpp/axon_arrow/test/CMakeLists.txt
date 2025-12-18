cmake_minimum_required(VERSION 3.15)

# GoogleTest - must be installed before building
# macOS:  brew install googletest
# Ubuntu: sudo apt install libgtest-dev
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# Google Benchmark (optional for performance tests)
# macOS:  brew install google-benchmark
# Ubuntu: sudo apt install libbenchmark-dev
find_package(benchmark QUIET)

# Test executables
add_executable(test_arrow_builder test_arrow_builder.cpp)
add_executable(test_batch_manager test_batch_manager.cpp)
add_executable(test_config_parser test_config_parser.cpp)
add_executable(test_schema_merger test_schema_merger.cpp)

# Link libraries - all tests link against axon_arrow library
target_link_libraries(test_arrow_builder
    PRIVATE
        axon_arrow
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
)

target_link_libraries(test_batch_manager
    PRIVATE
        axon_arrow
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
)

target_link_libraries(test_config_parser
    PRIVATE
        axon_arrow
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
)

target_link_libraries(test_schema_merger
    PRIVATE
        axon_arrow
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
)

# Add tests to CTest
add_test(NAME ArrowBuilderTest COMMAND test_arrow_builder)
add_test(NAME BatchManagerTest COMMAND test_batch_manager)
add_test(NAME ConfigParserTest COMMAND test_config_parser)
add_test(NAME SchemaMergerTest COMMAND test_schema_merger)

# Set test properties
set_tests_properties(ArrowBuilderTest BatchManagerTest ConfigParserTest SchemaMergerTest
    PROPERTIES
        TIMEOUT 30
)

# =============================================================================
# Performance Benchmarks (requires Google Benchmark)
# =============================================================================
if(benchmark_FOUND)
    message(STATUS "Google Benchmark found - building performance tests")
    
    add_executable(perf_batch_manager perf_batch_manager.cpp)
    
    target_include_directories(perf_batch_manager
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    
    target_link_libraries(perf_batch_manager
        PRIVATE
            axon_arrow
            axon_compression
            benchmark::benchmark
            benchmark::benchmark_main
            Threads::Threads
    )
    
    # Don't add to CTest - benchmarks should be run manually
    set_target_properties(perf_batch_manager PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/benchmarks
    )
else()
    message(STATUS "Google Benchmark not found - skipping performance tests")
    message(STATUS "  To enable: brew install google-benchmark (macOS) or apt install libbenchmark-dev (Ubuntu)")
endif()
