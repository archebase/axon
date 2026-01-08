cmake_minimum_required(VERSION 3.10)

# ============================================================================
# Axon Recorder Integration Tests
# ============================================================================

project(axon_recorder_tests LANGUAGES CXX)

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
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../core/axon_utils
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${YAML_CPP_LIBRARY_DIRS}
)

# Find axon_utils library
# Try to find it in the build directory
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/core_utils/libaxon_utils.a")
  set(AXON_UTILS_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/core_utils/libaxon_utils.a")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/core/libaxon_utils.a")
  set(AXON_UTILS_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/core/libaxon_utils.a")
else()
  message(WARNING "axon_utils library not found in expected build directories")
  set(AXON_UTILS_LIB "")
endif()

# ============================================================================
# Test executables
# ============================================================================

# Test: Recorder Services (State machine, configs, payloads)
add_executable(test_recorder_services
  test_recorder_services.cpp
)
target_link_libraries(test_recorder_services
  ${AXON_UTILS_LIB}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
  GTest::gtest
  GTest::gtest_main
  pthread
)
add_test(NAME test_recorder_services COMMAND test_recorder_services)

# Enable testing
enable_testing()
