cmake_minimum_required(VERSION 3.15)
project(axon_arrow VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(AXON_ARROW_BUILD_TESTS "Build tests" ON)

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
