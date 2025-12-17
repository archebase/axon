cmake_minimum_required(VERSION 3.15)

# Find GoogleTest
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# Find dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(ARROW arrow)
find_package(yaml-cpp REQUIRED)

# Project root directory (parent of test directory)
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Include directories
include_directories(
    ${PROJECT_ROOT}/cpp/src
    ${PROJECT_ROOT}/cpp/src/core
    ${PROJECT_ROOT}/ros/axon_common/src
    ${GTEST_INCLUDE_DIRS}
    ${ARROW_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
)

# Test executables
add_executable(test_config_parser
    cpp/test_config_parser.cpp
    ../cpp/src/core/config_parser.cpp
)

add_executable(test_batch_manager
    cpp/test_batch_manager.cpp
    ../cpp/src/core/batch_manager.cpp
)

add_executable(test_arrow_builder
    cpp/test_arrow_builder.cpp
    ../cpp/src/core/arrow_builder.cpp
)

# Link libraries
target_link_libraries(test_config_parser
    GTest::gtest
    GTest::gtest_main
    yaml-cpp
    Threads::Threads
)

target_link_libraries(test_batch_manager
    GTest::gtest
    GTest::gtest_main
    ${ARROW_LIBRARIES}
    Threads::Threads
)

target_link_libraries(test_arrow_builder
    GTest::gtest
    GTest::gtest_main
    ${ARROW_LIBRARIES}
    Threads::Threads
)

# Add test to CTest
enable_testing()
add_test(NAME ConfigParserTest COMMAND test_config_parser)
add_test(NAME BatchManagerTest COMMAND test_batch_manager)
add_test(NAME ArrowBuilderTest COMMAND test_arrow_builder)
