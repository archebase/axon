cmake_minimum_required(VERSION 3.10)

# ============================================================================
# Axon Utils Tests
# ============================================================================

project(axon_utils_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(OpenSSL REQUIRED)
find_package(GTest REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${YAML_CPP_LIBRARY_DIRS}
)

# ============================================================================
# Test executables
# ============================================================================

# Test: Config Parser
add_executable(test_config_parser
  test_config_parser.cpp
)
target_link_libraries(test_config_parser
  axon_utils
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
  GTest::gtest
  GTest::gtest_main
  pthread
)
add_test(NAME test_config_parser COMMAND test_config_parser)

# Test: Common Types
add_executable(test_common_types
  test_common_types.cpp
)
target_link_libraries(test_common_types
  axon_utils
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
  GTest::gtest
  GTest::gtest_main
  pthread
)
add_test(NAME test_common_types COMMAND test_common_types)

# Enable testing
enable_testing()
