# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

cmake_minimum_required(VERSION 3.12)
project(axon_zenoh_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Coverage support
option(AXON_ENABLE_COVERAGE "Enable coverage instrumentation" OFF)
if(AXON_ENABLE_COVERAGE)
  add_compile_options(--coverage -g -O0)
  add_link_options(--coverage)
endif()

# Find Zenoh C++ library
# First find zenohc (C library), then zenohcxx (C++ wrapper)
list(APPEND CMAKE_PREFIX_PATH /usr/local /usr/local/lib/cmake /usr/local/lib/x86_64-linux-gnu/cmake /usr/lib/cmake)

# Find zenohc first (required by zenohcxx)
find_package(zenohc CONFIG QUIET)

# Try CONFIG first (works if installed with cmake)
find_package(zenohcxx CONFIG QUIET)

if(NOT zenohcxx_FOUND)
  message(STATUS "zenohcxx not found via CONFIG, trying manual search...")
  # Try to find manually in /usr/local
  if(EXISTS "/usr/local/lib/cmake/zenohcxx/zenohcxxConfig.cmake" OR
     EXISTS "/usr/local/lib/x86_64-linux-gnu/cmake/zenohcxx/zenohcxxConfig.cmake")
    # Load the config file directly
    if(EXISTS "/usr/local/lib/cmake/zenohcxx/zenohcxxConfig.cmake")
      set(zenohcxx_DIR /usr/local/lib/cmake/zenohcxx)
    else()
      set(zenohcxx_DIR /usr/local/lib/x86_64-linux-gnu/cmake/zenohcxx)
    endif()
    include(${zenohcxx_DIR}/zenohcxxConfig.cmake)
    message(STATUS "Found zenohcxx cmake config at ${zenohcxx_DIR}")
    set(zenohcxx_FOUND TRUE)
  elseif(EXISTS "/usr/local/lib/libzenohcxx.a" OR EXISTS "/usr/local/lib/libzenohcxx.so")
    # Create imported target if library exists but no cmake config
    add_library(zenohcxx_zenohc INTERFACE IMPORTED)
    set_target_properties(zenohcxx_zenohc PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"
    )
    add_library(zenohcxx::zenohc ALIAS zenohcxx_zenohc)
    set(zenohcxx_FOUND TRUE)
    message(STATUS "Found zenohcxx library in /usr/local/lib, created imported target")
  else()
    set(zenohcxx_FOUND FALSE)
    message(
      WARNING
      "Zenoh C++ library not found. Please install zenoh-cpp or set CMAKE_PREFIX_PATH."
    )
  endif()
endif()

# Build the plugin library (direct C interface, no external ABI header)
add_library(axon_zenoh_plugin SHARED
  src/zenoh_plugin.cpp
  src/zenoh_subscription_wrapper.cpp
  src/zenoh_plugin_export.cpp
)

target_include_directories(axon_zenoh_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link nlohmann_json
target_link_libraries(axon_zenoh_plugin nlohmann_json::nlohmann_json)

# Link Zenoh if found
if(zenohcxx_FOUND)
  target_link_libraries(axon_zenoh_plugin zenohcxx::zenohc)
  target_compile_definitions(axon_zenoh_plugin PRIVATE
    ZENOHCPP_FOUND
    ZENOHCXX_ZENOHC  # Required for zenoh-cpp to include zenoh.h
  )
endif()

# Export all symbols for dynamic loading
target_compile_options(axon_zenoh_plugin PRIVATE -fvisibility=default)

# =============================================================================
# Tests
# =============================================================================
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest QUIET)

  if(GTest_FOUND)
    # Discover test files automatically
    file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp")

    foreach(test_source ${TEST_SOURCES})
      get_filename_component(test_name ${test_source} NAME_WE)

      add_executable(${test_name} ${test_source})

      target_link_libraries(${test_name} axon_zenoh_plugin GTest::GTest)

      if(zenohcxx_FOUND)
        target_link_libraries(${test_name} zenohcxx::zenohc)
        target_compile_definitions(${test_name} PRIVATE ZENOHCXX_ZENOHC)
      endif()

      # Add test labels
      set_target_properties(${test_name} PROPERTIES LABELS "unit;zenoh_plugin")

      # Register with CTest
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  else()
    message(STATUS "GTest not found, skipping Zenoh plugin tests")
  endif()
endif()

# =============================================================================
# Examples (Integration Test Programs)
# =============================================================================
if(zenohcxx_FOUND)
  # Test Publisher
  add_executable(test_publisher examples/test_publisher.cpp)
  target_link_libraries(test_publisher zenohcxx::zenohc)
  target_compile_definitions(test_publisher PRIVATE ZENOHCXX_ZENOHC)

  # Test Subscriber
  add_executable(test_subscriber examples/test_subscriber.cpp)
  target_link_libraries(test_subscriber zenohcxx::zenohc)
  target_compile_definitions(test_subscriber PRIVATE ZENOHCXX_ZENOHC)

  message(STATUS "Building integration test programs (test_publisher, test_subscriber)")
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS axon_zenoh_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/axon/plugins
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/zenoh_plugin
  FILES_MATCHING PATTERN "*.hpp"
)

# Print summary
message(STATUS "")
message(STATUS "Axon Zenoh Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Zenoh Found: ${zenohcxx_FOUND}")
message(STATUS "  Tests Enabled: ${BUILD_TESTING}")
message(STATUS "")
