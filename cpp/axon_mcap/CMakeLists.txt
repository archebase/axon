cmake_minimum_required(VERSION 3.12)
project(axon_mcap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Dependencies
# =============================================================================

# Find yaml-cpp for config parser
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
elseif(UNIX)
    list(APPEND CMAKE_PREFIX_PATH "/usr" "/usr/local")
endif()
find_package(yaml-cpp REQUIRED)

# Handle both old and new cmake target names for yaml-cpp
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

# =============================================================================
# MCAP Library (Header-only from Foxglove)
# =============================================================================

# Find compression libraries (optional but recommended)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(ZSTD QUIET libzstd)
    pkg_check_modules(LZ4 QUIET liblz4)
endif()

# Create interface library for MCAP headers
add_library(mcap_headers INTERFACE)
target_include_directories(mcap_headers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Add compression support if available
if(ZSTD_FOUND)
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_ZSTD)
    target_link_libraries(mcap_headers INTERFACE ${ZSTD_LIBRARIES})
    target_include_directories(mcap_headers INTERFACE ${ZSTD_INCLUDE_DIRS})
    message(STATUS "MCAP: Zstd compression enabled")
endif()

if(LZ4_FOUND)
    target_compile_definitions(mcap_headers INTERFACE MCAP_COMPRESSION_LZ4)
    target_link_libraries(mcap_headers INTERFACE ${LZ4_LIBRARIES})
    target_include_directories(mcap_headers INTERFACE ${LZ4_INCLUDE_DIRS})
    message(STATUS "MCAP: LZ4 compression enabled")
endif()

# =============================================================================
# MCAP Writer Wrapper Library (includes config parser)
# =============================================================================

add_library(axon_mcap
    ${CMAKE_CURRENT_SOURCE_DIR}/mcap_writer_wrapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/config_parser.cpp
)

target_include_directories(axon_mcap PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(axon_mcap PUBLIC
    mcap_headers
    ${YAML_CPP_TARGET}
)

# Link compression libraries if available
if(ZSTD_FOUND)
    target_link_libraries(axon_mcap PUBLIC ${ZSTD_LIBRARIES})
endif()

if(LZ4_FOUND)
    target_link_libraries(axon_mcap PUBLIC ${LZ4_LIBRARIES})
endif()

# =============================================================================
# Tests (optional)
# =============================================================================
option(AXON_MCAP_BUILD_TESTS "Build MCAP tests" OFF)

if(AXON_MCAP_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    
    add_executable(test_mcap_writer
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mcap_writer.cpp
    )
    
    target_include_directories(test_mcap_writer PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    
    target_link_libraries(test_mcap_writer
        axon_mcap
        GTest::gtest
        GTest::gtest_main
    )
    
    # Link compression libraries for tests
    if(ZSTD_FOUND)
        target_link_libraries(test_mcap_writer ${ZSTD_LIBRARIES})
    endif()
    if(LZ4_FOUND)
        target_link_libraries(test_mcap_writer ${LZ4_LIBRARIES})
    endif()
    
    add_test(NAME test_mcap_writer COMMAND test_mcap_writer)
endif()
