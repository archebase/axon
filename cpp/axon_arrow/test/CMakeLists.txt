cmake_minimum_required(VERSION 3.15)

# GoogleTest - must be installed before building
# macOS:  brew install googletest
# Ubuntu: sudo apt install libgtest-dev
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

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
