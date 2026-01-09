cmake_minimum_required(VERSION 3.10)
project(axon_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system filesystem thread log log_setup)

# =============================================================================
# axon_logging Library (for structured logging)
# =============================================================================
# Only add subdirectory if axon_logging target doesn't already exist
# (it may have been added by root CMakeLists.txt or another module)

if(NOT TARGET axon_logging)
    # Use centralized AXON_LOGGING_DIR if available, otherwise fall back to sibling
    if(DEFINED AXON_LOGGING_DIR AND EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
        # Use root's path (built from repo root)
        set(LOGGING_BUILD_DIR "${AXON_LOGGING_DIR}")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../axon_logging/CMakeLists.txt")
        # Fallback to sibling directory
        set(LOGGING_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../axon_logging")
    else()
        message(STATUS "axon_logging not found - logging support disabled for axon_utils")
        set(LOGGING_BUILD_DIR "")
    endif()

    if(LOGGING_BUILD_DIR)
        message(STATUS "Found axon_logging at: ${LOGGING_BUILD_DIR}")
        # Propagate coverage flag to axon_logging subdirectory
        if(DEFINED AXON_ENABLE_COVERAGE)
            set(AXON_LOGGING_ENABLE_COVERAGE ${AXON_ENABLE_COVERAGE} CACHE BOOL "Enable coverage for axon_logging" FORCE)
        else()
            set(AXON_LOGGING_ENABLE_COVERAGE OFF CACHE BOOL "Enable coverage for axon_logging" FORCE)
        endif()
        add_subdirectory(${LOGGING_BUILD_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
        # Set for later use
        set(AXON_LOGGING_DIR "${LOGGING_BUILD_DIR}" CACHE INTERNAL "")
    endif()
else()
    message(STATUS "axon_logging target already exists, skipping add_subdirectory")
endif()

# Create the axon_utils library
# Note: common_types has been moved to apps/axon_recorder
add_library(axon_utils STATIC
  worker_thread_pool.cpp
)

# Set include directories (PUBLIC so linking targets can find headers)
target_include_directories(axon_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(axon_utils PUBLIC
  ${Boost_LIBRARIES}
)

# Link axon_logging if available
if(TARGET axon_logging)
    target_link_libraries(axon_utils PUBLIC axon_logging)
    # Add BOOST_LOG_DYN_LINK definition
    target_compile_definitions(axon_utils PRIVATE BOOST_LOG_DYN_LINK)
endif()

# Installation
install(TARGETS axon_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install headers
file(GLOB_RECURSE AXON_UTILS_HEADERS "*.hpp")
install(FILES ${AXON_UTILS_HEADERS} DESTINATION include)

# Enable testing
enable_testing()

# Add test subdirectory
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()
