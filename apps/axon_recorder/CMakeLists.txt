cmake_minimum_required(VERSION 3.12)
project(axon_recorder VERSION 0.1.0)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Path Configuration
# =============================================================================
if(DEFINED AXON_REPO_ROOT)
    set(PROJECT_ROOT ${AXON_REPO_ROOT})
else()
    set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
endif()

message(STATUS "Project root: ${PROJECT_ROOT}")

# =============================================================================
# Dependencies
# =============================================================================
find_package(Boost REQUIRED COMPONENTS system log log_setup thread filesystem)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)

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
endif()

# =============================================================================
# nlohmann/json (header-only)
# =============================================================================
include(FetchContent)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
else()
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    )
endif()
FetchContent_MakeAvailable(nlohmann_json)

# =============================================================================
# Core Libraries (Optional - for testing without full ROS setup)
# =============================================================================
option(AXON_BUILD_WITH_CORE "Build with core libraries" OFF)

if(AXON_BUILD_WITH_CORE)
    if(DEFINED AXON_MCAP_DIR AND EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
        set(CPP_MCAP_DIR "${AXON_MCAP_DIR}")
    else()
        set(CPP_MCAP_DIR "${PROJECT_ROOT}/core/axon_mcap")
    endif()

    if(DEFINED AXON_LOGGING_DIR AND EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
        set(CPP_LOGGING_DIR "${AXON_LOGGING_DIR}")
    else()
        set(CPP_LOGGING_DIR "${PROJECT_ROOT}/core/axon_logging")
    endif()

    if(EXISTS "${CPP_MCAP_DIR}/CMakeLists.txt" AND NOT TARGET axon_mcap)
        message(STATUS "Building with axon_mcap at: ${CPP_MCAP_DIR}")
        add_subdirectory(${CPP_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
    elseif(NOT TARGET axon_mcap)
        message(WARNING "axon_mcap not found at: ${CPP_MCAP_DIR}")
    endif()

    if(EXISTS "${CPP_LOGGING_DIR}/CMakeLists.txt" AND NOT TARGET axon_logging)
        message(STATUS "Building with axon_logging at: ${CPP_LOGGING_DIR}")
        add_subdirectory(${CPP_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
    elseif(NOT TARGET axon_logging)
        message(WARNING "axon_logging not found at: ${CPP_LOGGING_DIR}")
    endif()
endif()

# =============================================================================
# Plugin Loader Library
# =============================================================================
set(PLUGIN_LOADER_SOURCES
    plugin_loader.cpp
)

set(PLUGIN_LOADER_HEADERS
    plugin_loader.hpp
)

add_library(axon_plugin_loader ${PLUGIN_LOADER_SOURCES})
target_include_directories(axon_plugin_loader PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(axon_plugin_loader
    dl  # For dlopen/dlsym
)

# Enable C++17
target_compile_features(axon_plugin_loader PUBLIC cxx_std_17)

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
# Build the main recorder application if axon_mcap is available
if(TARGET axon_mcap)
    add_executable(axon_recorder
        axon_recorder.cpp
        recorder.cpp
        state_machine.cpp
        recording_session.cpp
        metadata_injector.cpp
        worker_thread_pool.cpp
        plugin_loader.cpp
        config_parser.cpp
        http_server.cpp
    )

    target_include_directories(axon_recorder PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}  # For generated version.hpp
        ${nlohmann_json_SOURCE_DIR}/single_include
    )

    target_link_libraries(axon_recorder
        axon_mcap
        axon_logging
        yaml-cpp
        OpenSSL::Crypto
        OpenSSL::SSL
        Boost::system
        Boost::thread
        dl
        pthread
    )

    # Enable C++17
    target_compile_features(axon_recorder PRIVATE cxx_std_17)

    # Install executable
    install(TARGETS axon_recorder
        RUNTIME DESTINATION bin
    )

    message(STATUS "Axon recorder executable will be built")
else()
    message(STATUS "axon_mcap not found - skipping axon_recorder executable")
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS axon_plugin_loader
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES plugin_loader.hpp
    DESTINATION include/axon
)
