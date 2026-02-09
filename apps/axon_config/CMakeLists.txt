cmake_minimum_required(VERSION 3.14)  # FetchContent requires 3.14+
project(axon_config LANGUAGES CXX)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Centralized CMake Modules
# =============================================================================
# Use the centralized cmake/Modules directory from the Axon repository.
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    # Compute relative path to cmake/Modules
    set(_AXON_CMAKE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules")
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdlib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

# Include required modules (optional if building standalone)
include(AxonStdlib OPTIONAL)
include(AxonCoverage OPTIONAL)

# =============================================================================
# Use axon_mcap for mcap_headers interface
# =============================================================================

if(DEFINED AXON_MCAP_DIR AND EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
    # Use root's path (built from repo root)
    set(MCAP_BUILD_DIR "${AXON_MCAP_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_mcap/CMakeLists.txt")
    # Fallback to relative path
    set(MCAP_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_mcap")
else()
    message(FATAL_ERROR
        "axon_mcap not found. Please build from the Axon repository root."
    )
endif()

if(NOT TARGET mcap_headers)
    message(STATUS "Found axon_mcap at: ${MCAP_BUILD_DIR}")
    add_subdirectory(${MCAP_BUILD_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
endif()

# =============================================================================
# Source files
# =============================================================================

set(SOURCES
  axon_config.cpp
  commands.cpp
  config_cache.cpp
)

# =============================================================================
# Create executable
# =============================================================================

add_executable(axon_config ${SOURCES})

axon_set_position_independent_code(axon_config)

# Link against axon_mcap which includes MCAP implementation
target_link_libraries(axon_config PRIVATE axon_mcap)

# Include directories
target_include_directories(axon_config PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Compiler warnings
target_compile_options(axon_config PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

# =============================================================================
# Installation
# =============================================================================

install(TARGETS axon_config DESTINATION bin)

# =============================================================================
# Tests
# =============================================================================

option(AXON_CONFIG_BUILD_TESTS "Build axon_config tests" OFF)

if(AXON_CONFIG_BUILD_TESTS OR AXON_BUILD_TESTS)
  enable_testing()

  # Find Google Test first
  find_package(GTest REQUIRED)

  # Test sources
  set(TEST_SOURCES
    test/test_config_cache.cpp
    test/test_commands.cpp
    test/test_main.cpp
    commands.cpp
    config_cache.cpp
  )

  # Create test executable
  add_executable(test_axon_config ${TEST_SOURCES})

  # Define testing flag for accessing private members
  target_compile_definitions(test_axon_config PRIVATE AXON_CONFIG_TESTING)

  axon_set_position_independent_code(test_axon_config)

  # Link against axon_mcap and Google Test
  target_link_libraries(test_axon_config PRIVATE
    axon_mcap
    GTest::gtest
    GTest::gtest_main
  )

  target_include_directories(test_axon_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Apply coverage flags
  if(COMMAND axon_add_coverage)
      axon_add_coverage(test_axon_config)
  endif()

  # Register tests with CTest
  include(GoogleTest)

  gtest_discover_tests(test_axon_config)

  # Set test properties for discovered tests
  set_tests_properties(PROPERTIES
    LABELS "axon_config"
  )
endif()
