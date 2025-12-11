cmake_minimum_required(VERSION 3.15)

# Find GoogleTest
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

# Find dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(ARROW arrow)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/src/cpp
    ${CMAKE_SOURCE_DIR}/src/cpp/core
    ${CMAKE_SOURCE_DIR}/src/cpp/common
    ${GTEST_INCLUDE_DIRS}
    ${ARROW_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
)

# Test executables
add_executable(test_config_parser
    cpp/test_config_parser.cpp
    ../src/cpp/core/config_parser.cpp
)

add_executable(test_batch_manager
    cpp/test_batch_manager.cpp
    ../src/cpp/core/batch_manager.cpp
)

add_executable(test_arrow_builder
    cpp/test_arrow_builder.cpp
    ../src/cpp/core/arrow_builder.cpp
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

