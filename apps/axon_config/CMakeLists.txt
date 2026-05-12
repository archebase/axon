cmake_minimum_required(VERSION 3.14)
project(axon_config LANGUAGES CXX)

# =============================================================================
# Installation Configuration
# =============================================================================
# Set install prefix for packaging (defaults to /opt/axon)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/opt/axon" CACHE PATH "Installation prefix" FORCE)
endif()

# =============================================================================
# Centralized CMake Modules
# =============================================================================
# All Axon modules must use the centralized cmake/Modules directory.
# This ensures consistent configuration across the codebase.
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    # Compute relative path to cmake/Modules
    set(_AXON_CMAKE_MODULES_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules"
    )
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

# Include required modules (AxonStdLib provides standard helpers)
include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)

find_package(CURL REQUIRED)

# =============================================================================
# Use axon_mcap for mcap_headers interface
# =============================================================================
# When building for packaging with AXON_REPO_ROOT set, set MCAP path
if(DEFINED AXON_REPO_ROOT AND NOT DEFINED AXON_MCAP_DIR)
    set(AXON_MCAP_DIR
        "${AXON_REPO_ROOT}/core/axon_mcap"
        CACHE PATH
        "Axon MCAP directory"
    )
endif()

# Use centralized AXON_MCAP_DIR if set by parent, otherwise use AXON_CORE_DIR
if(DEFINED AXON_MCAP_DIR AND EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
    # Use parent's path (built from repo root)
    set(MCAP_BUILD_DIR "${AXON_MCAP_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_mcap/CMakeLists.txt")
    # Fallback to relative path
    set(MCAP_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../core/axon_mcap")
elseif(
    DEFINED AXON_CORE_DIR
    AND EXISTS "${AXON_CORE_DIR}/axon_mcap/CMakeLists.txt"
)
    # Use AXON_CORE_DIR if available
    set(MCAP_BUILD_DIR "${AXON_CORE_DIR}/axon_mcap")
else()
    message(
        FATAL_ERROR
        "axon_mcap not found. Please build from Axon repository root."
    )
endif()

if(NOT TARGET mcap_headers)
    message(STATUS "Found axon_mcap at: ${MCAP_BUILD_DIR}")
    add_subdirectory(${MCAP_BUILD_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
endif()

# =============================================================================
# Source files
# =============================================================================

set(SOURCES axon_config.cpp commands.cpp config_cache.cpp)

# =============================================================================
# Create executable
# =============================================================================
add_executable(axon_config ${SOURCES})

axon_set_position_independent_code(axon_config)

# Link against axon_mcap which includes MCAP implementation
target_link_libraries(axon_config PRIVATE axon_mcap CURL::libcurl)

if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(axon_config PRIVATE nlohmann_json::nlohmann_json)
elseif(DEFINED nlohmann_json_SOURCE_DIR)
    target_include_directories(
        axon_config
        PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include
    )
endif()

# Apply coverage flags if enabled
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_config)
endif()

# =============================================================================
# Include directories
# =============================================================================
target_include_directories(axon_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# =============================================================================
# Compiler warnings
# =============================================================================
target_compile_options(axon_config PRIVATE -Wall -Wextra -Wpedantic)

# Set output name to use hyphen (axon-config)
set_target_properties(axon_config PROPERTIES OUTPUT_NAME "axon-config")

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
    target_link_libraries(
        test_axon_config
        PRIVATE axon_mcap CURL::libcurl GTest::gtest GTest::gtest_main
    )

    if(TARGET nlohmann_json::nlohmann_json)
        target_link_libraries(
            test_axon_config
            PRIVATE nlohmann_json::nlohmann_json
        )
    elseif(DEFINED nlohmann_json_SOURCE_DIR)
        target_include_directories(
            test_axon_config
            PRIVATE ${nlohmann_json_SOURCE_DIR}/single_include
        )
    endif()

    target_include_directories(
        test_axon_config
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # Apply coverage flags
    if(COMMAND axon_add_coverage)
        axon_add_coverage(test_axon_config)
    endif()

    # Register test and set properties
    add_test(NAME test_axon_config COMMAND test_axon_config)
    set_tests_properties(test_axon_config PROPERTIES LABELS "axon_config")
endif()
