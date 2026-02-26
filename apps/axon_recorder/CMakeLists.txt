cmake_minimum_required(VERSION 3.12)
project(axon_recorder VERSION 0.2.1)

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
# Dependencies
# =============================================================================
# Find required components first
find_package(
    Boost
    REQUIRED
    COMPONENTS log log_setup thread filesystem system
)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)

# Create yaml-cpp alias if using modern yaml-cpp target
if(NOT TARGET yaml-cpp AND TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp ALIAS yaml-cpp::yaml-cpp)
endif()

# Compression libraries
pkg_check_modules(ZSTD QUIET libzstd)
pkg_check_modules(LZ4 QUIET liblz4)

if(ZSTD_FOUND)
    message(STATUS "Found Zstd: ${ZSTD_VERSION}")
else()
    message(WARNING "Zstd not found - MCAP compression disabled")
endif()

if(LZ4_FOUND)
    message(STATUS "Found LZ4: ${LZ4_VERSION}")
else()
    message(WARNING "LZ4 not found - MCAP compression disabled")
endif()

# =============================================================================
# Plugin Loader Library
# =============================================================================

set(PLUGIN_LOADER_SOURCES plugin_loader.cpp)

set(PLUGIN_LOADER_HEADERS plugin_loader.hpp)

# Create plugin loader library
add_library(axon_plugin_loader ${PLUGIN_LOADER_SOURCES})

target_include_directories(
    axon_plugin_loader
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

# Enable C++17
target_compile_features(axon_plugin_loader PUBLIC cxx_std_17)

# Link dl for dynamic loading
target_link_libraries(axon_plugin_loader dl)

# =============================================================================
# Version Header
# =============================================================================
# Generate version.hpp from version.hpp.in
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.hpp
    @ONLY
)

# =============================================================================
# Recorder Main Executable
# =============================================================================

# Build main recorder application
add_executable(
    axon_recorder
    axon_recorder.cpp
    recorder.cpp
    state_machine.cpp
    recording_session.cpp
    metadata_injector.cpp
    config_injector.cpp
    worker_thread_pool.cpp
    plugin_loader.cpp
    config_parser.cpp
    http_server.cpp
    http_callback_client.cpp
    websocket_server.cpp
    websocket_session.cpp
    event_broadcaster.cpp
)

target_include_directories(
    axon_recorder
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR} # For generated version.hpp
        ${nlohmann_json_SOURCE_DIR}/single_include
)

target_link_libraries(
    axon_recorder
    PRIVATE
        axon_plugin_loader
        axon_logging
        axon_mcap
        yaml-cpp
        OpenSSL::Crypto
        OpenSSL::SSL
        Boost::thread
        Boost::system
        pthread
)

# Link Boost::filesystem if available
if(TARGET Boost::filesystem)
    target_link_libraries(axon_recorder PRIVATE Boost::filesystem)
endif()

# Enable C++17
target_compile_features(axon_recorder PRIVATE cxx_std_17)

# Coverage flags
if(AXON_ENABLE_COVERAGE)
    target_compile_options(axon_recorder PRIVATE --coverage)
    target_link_options(axon_recorder PRIVATE --coverage)
endif()

# Install executable
install(TARGETS axon_recorder RUNTIME DESTINATION bin)

# Install plugin loader header for external plugins
install(FILES plugin_loader.hpp DESTINATION include)

# Install WebSocket server headers
install(
    FILES websocket_server.hpp websocket_session.hpp event_broadcaster.hpp
    DESTINATION include
)

# =============================================================================
# Tests
# =============================================================================
if(AXON_BUILD_TESTS)
    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/test
        ${CMAKE_CURRENT_BINARY_DIR}/test
    )
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "=== Axon Recorder Configuration ===")
message(STATUS "  C++ Standard: 17")
message(STATUS "  Coverage: ${AXON_ENABLE_COVERAGE}")
message(STATUS "=================================")
message(STATUS "")
