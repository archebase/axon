cmake_minimum_required(VERSION 3.14)
project(axon_logging)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Find Boost.Log and dependencies
# Boost 1.71+ required (Ubuntu 20.04/ROS Noetic minimum)
# Ubuntu 22.04+ ships Boost 1.74+
# =============================================================================
find_package(Boost 1.71 REQUIRED COMPONENTS 
    log 
    log_setup 
    thread 
    filesystem
)

if(Boost_FOUND)
    message(STATUS "axon_logging: Found Boost ${Boost_VERSION}")
    message(STATUS "  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
endif()

# =============================================================================
# axon_logging Library
# =============================================================================
add_library(axon_logging
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_log_init.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_console_sink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/axon_file_sink.cpp
)

# Enable position-independent code for linking into shared libraries
set_target_properties(axon_logging PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(axon_logging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Define BOOST_LOG_DYN_LINK if using shared Boost libraries
target_compile_definitions(axon_logging PUBLIC
    BOOST_LOG_DYN_LINK
)

target_link_libraries(axon_logging PUBLIC
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::filesystem
)

# =============================================================================
# Coverage Support
# =============================================================================

option(AXON_LOGGING_ENABLE_COVERAGE "Enable code coverage" OFF)

if(AXON_LOGGING_ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "axon_logging: Coverage instrumentation enabled")
    target_compile_options(axon_logging PRIVATE --coverage -fprofile-arcs -ftest-coverage)
    target_link_options(axon_logging PRIVATE --coverage)
endif()

# =============================================================================
# Tests (optional)
# =============================================================================

option(AXON_LOGGING_BUILD_TESTS "Build logging tests" OFF)

if(AXON_LOGGING_BUILD_TESTS)
    add_subdirectory(test)
endif()

