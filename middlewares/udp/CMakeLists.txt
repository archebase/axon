# =============================================================================
# Axon UDP Plugin
# =============================================================================
# The plugin is built as a shared library that can be dynamically loaded
# by the main axon_recorder application.
# =============================================================================

cmake_minimum_required(VERSION 3.14)
project(axon_udp_plugin VERSION 0.1.0 LANGUAGES CXX)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# Dependencies
# =============================================================================
# nlohmann_json is provided by parent project via FetchContent

# Verify required variables are set
if(NOT DEFINED AXON_LOGGING_DIR)
    message(
        FATAL_ERROR
        "AXON_LOGGING_DIR must be set by parent project. "
        "Please include this project via the parent CMakeLists.txt"
    )
endif()

find_package(Boost REQUIRED COMPONENTS system)

# Try to find nlohmann_json (works when built from parent project)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
    # nlohmann_json should be provided by parent project
    message(
        STATUS
        "nlohmann_json not found via find_package, assuming provided by parent project"
    )
endif()

# =============================================================================
# Build Plugin Library
# =============================================================================

set(PLUGIN_SOURCES
    src/udp_server.cpp
    src/udp_plugin.cpp
    src/udp_plugin_export.cpp
)

add_library(axon_udp_plugin SHARED ${PLUGIN_SOURCES})

target_include_directories(
    axon_udp_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${AXON_LOGGING_DIR}/include
)

# Set library properties
set_target_properties(
    axon_udp_plugin
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION 0
        VISIBILITY_INLINES_HIDDEN ON
        OUTPUT_NAME
            "axon_udp" # Produces libaxon_udp.so
)

# Link libraries
target_link_libraries(
    axon_udp_plugin
    PUBLIC Boost::system nlohmann_json::nlohmann_json axon_logging
)

# Export all symbols for dynamic loading
target_compile_options(axon_udp_plugin PRIVATE -fvisibility=default)

# C++17 standard
target_compile_features(axon_udp_plugin PUBLIC cxx_std_17)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_add_coverage)
    axon_add_coverage(axon_udp_plugin)
endif()

# =============================================================================
# Tests
# =============================================================================
option(AXON_BUILD_TESTS "Build tests" OFF)

if(AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

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
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

        target_link_libraries(
            ${test_name}
            axon_udp_plugin
            GTest::gtest
            GTest::gtest_main
            pthread
        )

        # Apply coverage flags
        if(COMMAND axon_add_coverage)
            axon_add_coverage(${test_name})
        endif()

        # Register with CTest
        add_test(NAME ${test_name} COMMAND ${test_name})
        set_tests_properties(${test_name} PROPERTIES LABELS "unit;udp_plugin")
    endforeach()
endif()

# =============================================================================
# Install
# =============================================================================
install(
    TARGETS axon_udp_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
)

install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")

# =============================================================================
# Examples
# =============================================================================
option(AXON_BUILD_EXAMPLES "Build example programs" OFF)

if(AXON_BUILD_EXAMPLES)
    # UDP Publisher example
    add_executable(udp_publisher examples/udp_publisher.cpp)

    target_include_directories(
        udp_publisher
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(
        udp_publisher
        PRIVATE Boost::system nlohmann_json::nlohmann_json
    )

    message(STATUS "  Examples:       ON")
else()
    message(STATUS "  Examples:       OFF")
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "Axon UDP Plugin Configuration:")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Tests Enabled: ${AXON_BUILD_TESTS}")
message(STATUS "")
