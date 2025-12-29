cmake_minimum_required(VERSION 3.14)
project(axon_logging)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Use Axon Root Configuration (if available)
# =============================================================================
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
    include(AxonStdlib OPTIONAL)
    include(AxonCoverage OPTIONAL)
else()
    # Fallback for standalone builds
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    if(POLICY CMP0167)
        cmake_policy(SET CMP0167 NEW)
    endif()
    # Define fallback helper functions
    function(axon_set_position_independent_code TARGET)
        set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endfunction()
    function(axon_add_boost_log_dyn_link TARGET)
        target_compile_definitions(${TARGET} PUBLIC BOOST_LOG_DYN_LINK)
    endfunction()
    function(axon_add_coverage TARGET)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${TARGET} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(${TARGET} PRIVATE --coverage)
        endif()
    endfunction()
endif()

# =============================================================================
# Find Boost.Log and dependencies
# =============================================================================
# Boost 1.71+ required (Ubuntu 20.04/ROS Noetic minimum)
# Ubuntu 22.04+ ships Boost 1.74+

find_package(Boost 1.71 REQUIRED COMPONENTS
    log
    log_setup
    thread
    filesystem
)

if(Boost_FOUND)
    message(STATUS "axon_logging: Found Boost ${Boost_VERSION}")
endif()

# =============================================================================
# axon_logging Library
# =============================================================================
add_library(axon_logging
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_log_init.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_console_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_file_sink.cpp
)

# Enable position-independent code
axon_set_position_independent_code(axon_logging)

target_include_directories(axon_logging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALLINTERFACE:include>
)

# Add BOOST_LOG_DYN_LINK definition
axon_add_boost_log_dyn_link(axon_logging)

target_link_libraries(axon_logging PUBLIC
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::filesystem
)

# =============================================================================
# Coverage Support
# =============================================================================
if(COMMAND axon_enable_coverage_for_directory)
    axon_enable_coverage_for_directory()
elseif(AXON_LOGGING_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Fallback for legacy builds
    message(STATUS "axon_logging: Coverage instrumentation enabled")
    target_compile_options(axon_logging PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(axon_logging PRIVATE --coverage)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_LOGGING_BUILD_TESTS "Build logging tests" OFF)

if(AXON_LOGGING_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
