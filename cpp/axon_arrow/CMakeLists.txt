cmake_minimum_required(VERSION 3.15)
project(axon_arrow VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable position-independent code for static libraries
# Required on ARM64 when linking static libs into shared libs
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(AXON_ARROW_BUILD_TESTS "Build tests" ON)
option(AXON_ARROW_ENABLE_COVERAGE "Enable code coverage" OFF)

# =============================================================================
# Address Sanitizer (ASAN) Support
# Can be enabled via AXON_ARROW_ASAN_FLAGS from parent project or ENABLE_ASAN
# =============================================================================
option(ENABLE_ASAN "Enable Address Sanitizer for debugging memory issues" OFF)

# Check environment variable
if(DEFINED ENV{ENABLE_ASAN} AND "$ENV{ENABLE_ASAN}" STREQUAL "1")
    set(ENABLE_ASAN ON)
endif()

# Apply ASAN flags (either from parent project or local option)
if(DEFINED AXON_ARROW_ASAN_FLAGS OR ENABLE_ASAN)
    if(DEFINED AXON_ARROW_ASAN_FLAGS)
        set(ASAN_FLAGS "${AXON_ARROW_ASAN_FLAGS}")
    else()
        set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer -g -O1")
    endif()
    
    message(STATUS "axon_arrow: ADDRESS SANITIZER ENABLED")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
endif()

# =============================================================================
# Code Coverage Configuration
# =============================================================================
if(AXON_ARROW_ENABLE_COVERAGE)
    message(STATUS "Code coverage enabled")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

# =============================================================================
# Dependencies - Must be installed before building
# =============================================================================
# macOS:  brew install apache-arrow yaml-cpp googletest
# Ubuntu: sudo apt install libarrow-dev libyaml-cpp-dev libgtest-dev
# =============================================================================

# Apache Arrow
find_package(Arrow REQUIRED)

# yaml-cpp (with platform-specific hints)
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
elseif(UNIX)
    list(APPEND CMAKE_PREFIX_PATH "/usr" "/usr/local")
endif()
find_package(yaml-cpp REQUIRED)

# Collect source files
set(AXON_ARROW_SOURCES
    arrow_builder.cpp
    batch_manager.cpp
    config_parser.cpp
    message_converter.cpp
    message_introspection.cpp
    schema_merger.cpp
)

set(AXON_ARROW_HEADERS
    arrow_builder.hpp
    batch_manager.hpp
    config_parser.hpp
    message_converter.hpp
    message_introspection.hpp
    schema_merger.hpp
)

# Create library
add_library(axon_arrow ${AXON_ARROW_SOURCES})

target_include_directories(axon_arrow
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/axon>
    PRIVATE
        ${Arrow_INCLUDE_DIRS}
)

# Link yaml-cpp - handle both old and new cmake target names
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

target_link_libraries(axon_arrow
    PUBLIC
        Arrow::arrow_shared
    PRIVATE
        ${YAML_CPP_TARGET}
)

# Set library properties
set_target_properties(axon_arrow PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    PUBLIC_HEADER "${AXON_ARROW_HEADERS}"
)

# Build compression module
add_subdirectory(compression)

# Link compression to main library
target_link_libraries(axon_arrow
    PUBLIC
        axon_compression
)

# Build tests if enabled
if(AXON_ARROW_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Install targets
install(TARGETS axon_arrow
    EXPORT axon_arrow-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/axon
)

# Install CMake config files
install(EXPORT axon_arrow-targets
    FILE axon_arrow-targets.cmake
    NAMESPACE axon::
    DESTINATION lib/cmake/axon_arrow
)

# Generate and install package config
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/axon_arrow-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/axon_arrow-config.cmake
    INSTALL_DESTINATION lib/cmake/axon_arrow
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/axon_arrow-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/axon_arrow-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/axon_arrow-config-version.cmake
    DESTINATION lib/cmake/axon_arrow
)
