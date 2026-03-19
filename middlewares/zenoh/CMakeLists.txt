# =============================================================================
# Axon Zenoh Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
# =============================================================================

cmake_minimum_required(VERSION 3.12)
project(axon_zenoh_plugin VERSION 0.3.0 LANGUAGES CXX)

# =============================================================================
# Installation Configuration
# =============================================================================
# Set install prefix for packaging (defaults to /opt/axon)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/opt/axon" CACHE PATH "Installation prefix" FORCE)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
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

# =============================================================================
# Find nlohmann/json
# =============================================================================
# Try modern CMake config first (Ubuntu 22.04+), fallback to header-only
find_package(nlohmann_json 3.10.0 QUIET)
if(NOT nlohmann_json_FOUND)
    # Fallback for older systems
    message(
        STATUS
        "nlohmann_json CMake config not found, using header-only fallback"
    )
    # Check if FetchContent already created the target
    if(NOT TARGET nlohmann_json::nlohmann_json)
        # Check FetchContent source directory first (when built from parent project)
        if(DEFINED nlohmann_json_SOURCE_DIR)
            set(NLOHMANN_JSON_INCLUDE_DIR
                "${nlohmann_json_SOURCE_DIR}/single_include"
            )
        else()
            find_path(
                NLOHMANN_JSON_INCLUDE_DIR
                nlohmann/json.hpp
                PATHS /usr/include /usr/local/include
                NO_DEFAULT_PATH
            )
        endif()
        if(NOT NLOHMANN_JSON_INCLUDE_DIR)
            message(
                FATAL_ERROR
                "nlohmann_json not found. Please install nlohmann-json3-dev or build from parent project."
            )
        endif()
        # Create interface target for compatibility
        add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
        set_target_properties(
            nlohmann_json::nlohmann_json
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}"
        )
    endif()
endif()

# =============================================================================
# Find Zenoh C++ library
# =============================================================================
# First find zenohc (C library), then zenohcxx (C++ wrapper)

# Try CONFIG first (works if installed with cmake)
find_package(zenohc CONFIG QUIET)

# Try CONFIG first (works if installed with cmake)
find_package(zenohcxx CONFIG QUIET)

if(NOT zenohcxx_FOUND)
    message(
        STATUS
        "zenohcxx not found via CONFIG, trying manual search in /usr/local..."
    )
    # Try to find manually in /usr/local
    if(
        EXISTS "/usr/local/lib/cmake/zenohcxx/zenohcxxConfig.cmake"
        OR EXISTS
            "/usr/local/lib/x86_64-linux-gnu/cmake/zenohcxx/zenohcxxConfig.cmake"
    )
        set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")
    elseif(
        EXISTS "/usr/local/lib/zenohcxx.a"
        OR EXISTS "/usr/local/lib/x86_64-linux-gnu/lib/zenohcxx.a"
    )
        # Create imported target if library exists but no cmake config
        if(NOT TARGET zenohcxx::zenohc)
            add_library(zenohcxx::zenohc INTERFACE IMPORTED)
            set_target_properties(
                zenohcxx::zenohc
                PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"
            )
        endif()
        set(zenohcxx_FOUND TRUE)
        message(
            STATUS
            "Found zenohcxx library in /usr/local/lib, created imported target"
        )
    else()
        set(zenohcxx_FOUND FALSE)
        message(
            WARNING
            "Zenoh C++ library not found. Please install zenoh-cpp or set CMAKE_PREFIX_PATH."
        )
    endif()
endif()

# =============================================================================
# Find axon_logging (for standalone packaging)
# =============================================================================
if(NOT TARGET axon_logging)
    if(DEFINED AXON_REPO_ROOT AND EXISTS "${AXON_REPO_ROOT}/core/axon_logging")
        # Build axon_logging from source when AXON_REPO_ROOT is set (packaging use case)
        message(
            STATUS
            "Building axon_logging from source at ${AXON_REPO_ROOT}/core/axon_logging"
        )
        add_subdirectory(
            "${AXON_REPO_ROOT}/core/axon_logging"
            "${CMAKE_CURRENT_BINARY_DIR}/axon_logging"
        )
    elseif(
        DEFINED CMAKE_SOURCE_DIR
        AND EXISTS "${CMAKE_SOURCE_DIR}/../../core/axon_logging"
    )
        # Try relative path if built from middlewares directory
        set(_LOGGING_DIR "${CMAKE_SOURCE_DIR}/../../core/axon_logging")
        if(EXISTS "${_LOGGING_DIR}/CMakeLists.txt")
            message(
                STATUS
                "Building axon_logging from source at ${_LOGGING_DIR}"
            )
            add_subdirectory(
                "${_LOGGING_DIR}"
                "${CMAKE_CURRENT_BINARY_DIR}/axon_logging"
            )
        endif()
    else()
        message(
            FATAL_ERROR
            "axon_logging not found. Set AXON_LOGGING_DIR or AXON_REPO_ROOT, or build from parent project."
        )
    endif()
endif()

# =============================================================================
# Build Plugin Library
# =============================================================================

add_library(
    axon_zenoh_plugin
    SHARED
    src/zenoh_plugin.cpp
    src/zenoh_subscription_wrapper.cpp
    src/zenoh_plugin_export.cpp
)

target_include_directories(
    axon_zenoh_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${AXON_LOGGING_DIR}/include
)

target_link_libraries(
    axon_zenoh_plugin
    PUBLIC nlohmann_json::nlohmann_json axon_logging
)

if(zenohcxx_FOUND)
    target_link_libraries(axon_zenoh_plugin PUBLIC zenohcxx::zenohc)
endif()

# Export all symbols for dynamic loading
target_compile_options(axon_zenoh_plugin PRIVATE -fvisibility=default)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_zenoh_plugin)
endif()

# =============================================================================
# Tests
# =============================================================================
if(AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest QUIET)

    if(GTest_FOUND)
        # Discover test files automatically
        file(
            GLOB TEST_SOURCES
            CONFIGURE_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp"
        )

        foreach(test_source ${TEST_SOURCES})
            get_filename_component(test_name ${test_source} NAME_WE)

            add_executable(${test_name} ${test_source})

            target_include_directories(
                ${test_name}
                PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
            )

            target_link_libraries(
                ${test_name}
                axon_zenoh_plugin
                GTest::gtest
                GTest::gtest_main
            )

            if(zenohcxx_FOUND)
                target_link_libraries(${test_name} zenohcxx::zenohc)
            endif()

            # Apply coverage flags
            if(COMMAND axon_add_coverage)
                axon_add_coverage(${test_name})
            endif()

            # Register with CTest first, then set properties
            add_test(NAME ${test_name} COMMAND ${test_name})
            set_tests_properties(
                ${test_name}
                PROPERTIES LABELS "unit;zenoh_plugin"
            )
        endforeach()
    else()
        message(STATUS "GTest not found, skipping Zenoh plugin tests")
    endif()
endif()

# =============================================================================
# Examples (Integration Test Tools)
# =============================================================================
# These are standalone executables for manual testing and CI integration tests.
# They are NOT unit tests and are not registered with CTest.
if(zenohcxx_FOUND)
    # Build test_publisher example
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/test_publisher.cpp")
        add_executable(
            test_publisher
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/test_publisher.cpp"
        )
        target_link_libraries(
            test_publisher
            PRIVATE axon_zenoh_plugin zenohcxx::zenohc
        )
        message(STATUS "Zenoh examples: test_publisher ENABLED")
    endif()

    # Build test_subscriber example
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/test_subscriber.cpp")
        add_executable(
            test_subscriber
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/test_subscriber.cpp"
        )
        target_link_libraries(
            test_subscriber
            PRIVATE axon_zenoh_plugin zenohcxx::zenohc
        )
        message(STATUS "Zenoh examples: test_subscriber ENABLED")
    endif()
else()
    message(STATUS "Zenoh examples: DISABLED (zenohcxx not found)")
endif()

# =============================================================================
# Install
# =============================================================================
install(
    TARGETS axon_zenoh_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION plugins
    RUNTIME DESTINATION plugins
)

install(
    DIRECTORY include/
    DESTINATION include/axon
    FILES_MATCHING
    PATTERN "*.hpp"
)

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon Zenoh Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Zenoh Found: ${zenohcxx_FOUND}")
message(STATUS "  Tests Enabled: ${BUILD_TESTING}")
message(STATUS "")
