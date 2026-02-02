# Depth Filters Library
# Provides depth image compression using DepthLiteZ

cmake_minimum_required(VERSION 3.16)

project(axon_filters
  VERSION 0.1.0
  LANGUAGES CXX
)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_TESTING "Build tests" ON)
option(BUILD_SHARED "Build shared library" OFF)

# Find dependencies
find_package(PkgConfig REQUIRED)

# Find LZ4
pkg_check_modules(LZ4 REQUIRED liblz4)

# Find ZSTD
pkg_check_modules(ZSTD REQUIRED libzstd)

# DepthLiteZ submodule
set(DEPTHLITEZ_DIR ${CMAKE_CURRENT_SOURCE_DIR}/depthlitez)
if(NOT EXISTS ${DEPTHLITEZ_DIR}/include/dlz.hpp)
  message(WARNING "DepthLiteZ submodule not found. Run: git submodule update --init --recursive")
endif()

# Sources
set(SOURCES
  src/depth_compressor.cpp
)

set(HEADERS
  include/depth_compressor.hpp
)

# Create library
if(BUILD_SHARED)
  add_library(axon_filters SHARED ${SOURCES})
else()
  add_library(axon_filters STATIC ${SOURCES})
endif()

target_include_directories(axon_filters
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${DEPTHLITEZ_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(axon_filters
  PUBLIC
    ${LZ4_LIBRARIES}
    ${ZSTD_LIBRARIES}
)

# Ensure PIC is used for static library (required when linking into shared library)
set_target_properties(axon_filters PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_compile_options(axon_filters
  PRIVATE
    -Wall -Wextra -Wpedantic
)

# Alias for consistent naming
add_library(axon::filters ALIAS axon_filters)

# Tests
if(BUILD_TESTING)
  enable_testing()

  find_package(GTest REQUIRED)

  # Auto-discover test files
  file(GLOB TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp
  )

  foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
      PRIVATE
        axon::filters
        GTest::gtest_main
    )
    target_include_directories(${TEST_NAME} PRIVATE ${DEPTHLITEZ_DIR}/include)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  endforeach()
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS axon_filters
  EXPORT axon_filters_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES ${HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/axon/filters
)

install(EXPORT axon_filters_targets
  FILE axon_filters_targets.cmake
  NAMESPACE axon::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axon_filters
)
