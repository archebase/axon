# Depth Filters Library
# Provides depth image compression using DepthLiteZ
#
# NOTE: Depth compression requires the private DepthLiteZ submodule.
# By default, depth compression is DISABLED for open-source builds.
# To enable: cmake -DAXON_ENABLE_DEPTH_COMPRESSION=ON

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

# =============================================================================
# Options
# =============================================================================
option(BUILD_TESTING "Build tests" ON)
option(BUILD_SHARED "Build shared library" OFF)

# Depth compression option (default OFF for open-source builds)
option(AXON_ENABLE_DEPTH_COMPRESSION "Enable depth image compression (requires private DepthLiteZ submodule)" OFF)

if(AXON_ENABLE_DEPTH_COMPRESSION)
  message(STATUS "Depth compression: ENABLED")
else()
  message(STATUS "Depth compression: DISABLED (default for open-source builds)")
endif()

# =============================================================================
# Find dependencies
# =============================================================================
find_package(PkgConfig REQUIRED)

# LZ4 and ZSTD are only needed if depth compression is enabled
if(AXON_ENABLE_DEPTH_COMPRESSION)
  pkg_check_modules(LZ4 REQUIRED liblz4)
  pkg_check_modules(ZSTD REQUIRED libzstd)

  # DepthLiteZ submodule
  set(DEPTHLITEZ_DIR ${CMAKE_CURRENT_SOURCE_DIR}/depthlitez)
  if(NOT EXISTS ${DEPTHLITEZ_DIR}/include/dlz.hpp)
    message(FATAL_ERROR
      "AXON_ENABLE_DEPTH_COMPRESSION is ON but DepthLiteZ submodule not found.\n"
      "  DepthLiteZ is a private repository. To enable depth compression:\n"
      "  1. Ensure you have access to the DepthLiteZ repository\n"
      "  2. Run: git submodule update --init --recursive middlewares/filters/depthlitez\n"
      "  3. Or rebuild with: cmake -DAXON_ENABLE_DEPTH_COMPRESSION=OFF"
    )
  endif()
else()
  message(STATUS "Skipping LZ4/ZSTD dependency checks (depth compression disabled)")
endif()

# =============================================================================
# Sources
# =============================================================================
set(SOURCES "")
set(HEADERS "")

if(AXON_ENABLE_DEPTH_COMPRESSION)
  list(APPEND SOURCES src/depth_compressor.cpp)
  list(APPEND HEADERS include/depth_compressor.hpp)
else()
  # Use stub source when depth compression is disabled
  list(APPEND SOURCES src/stub.cpp)
endif()

# Create library (even if empty, for dependency consistency)
if(BUILD_SHARED)
  add_library(axon_filters SHARED ${SOURCES})
else()
  add_library(axon_filters STATIC ${SOURCES})
endif()

target_include_directories(axon_filters
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Add DepthLiteZ include directory only if enabled
if(AXON_ENABLE_DEPTH_COMPRESSION)
  target_include_directories(axon_filters
    PUBLIC
      $<BUILD_INTERFACE:${DEPTHLITEZ_DIR}/include>
  )
  target_link_libraries(axon_filters
    PUBLIC
      ${LZ4_LIBRARIES}
      ${ZSTD_LIBRARIES}
  )
  target_compile_definitions(axon_filters PRIVATE AXON_ENABLE_DEPTH_COMPRESSION)
endif()

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

# =============================================================================
# Tests
# =============================================================================
if(BUILD_TESTING AND AXON_ENABLE_DEPTH_COMPRESSION)
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
elseif(BUILD_TESTING)
  message(STATUS "Tests disabled (require AXON_ENABLE_DEPTH_COMPRESSION=ON)")
endif()

# =============================================================================
# Installation
# =============================================================================
include(GNUInstallDirs)

install(TARGETS axon_filters
  EXPORT axon_filters_targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(AXON_ENABLE_DEPTH_COMPRESSION)
  install(FILES ${HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/axon/filters
  )
endif()

install(EXPORT axon_filters_targets
  FILE axon_filters_targets.cmake
  NAMESPACE axon::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axon_filters
)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon Filters Configuration:")
message(STATUS "  Depth Compression: ${AXON_ENABLE_DEPTH_COMPRESSION}")
message(STATUS "  Build Shared: ${BUILD_SHARED}")
message(STATUS "")
