cmake_minimum_required(VERSION 3.10)

# ============================================================================
# Axon Utils Tests
# ============================================================================

project(axon_utils_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(GTest REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${Boost_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

# ============================================================================
# Test executables
# ============================================================================
# Note: common_types tests have been moved to apps/axon_recorder

# Enable testing
enable_testing()
