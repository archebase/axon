cmake_minimum_required(VERSION 3.16)
project(axon_transfer LANGUAGES CXX)

# =============================================================================
# Installation Configuration
# =============================================================================
# Set install prefix for packaging (defaults to /opt/axon)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/opt/axon" CACHE PATH "Installation prefix" FORCE)
endif()

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

# =============================================================================
# Core Libraries (when built from repo root or for packaging)
# =============================================================================
# When AXON_REPO_ROOT is set (e.g., for Debian packaging), add core libraries
# NOTE: We only add these if axon_uploader will be available, otherwise skip entirely
if(DEFINED AXON_REPO_ROOT)
    set(AXON_LOGGING_DIR "${AXON_REPO_ROOT}/core/axon_logging")
    set(AXON_UPLOADER_DIR "${AXON_REPO_ROOT}/core/axon_uploader")
    set(AXON_MCAP_DIR "${AXON_REPO_ROOT}/core/axon_mcap")

    # Check if axon_uploader can be built (requires AWS SDK)
    # We need to check this before adding any subdirectories
    find_package(OpenSSL QUIET)
    find_package(SQLite3 QUIET)
    find_package(AWSSDK QUIET COMPONENTS s3 transfer)

    if(NOT OpenSSL_FOUND OR NOT SQLite3_FOUND OR NOT AWSSDK_FOUND)
        message(STATUS "axon_transfer: skipping build - missing dependencies")
        message(STATUS "axon_transfer: Requires OpenSSL, SQLite3, and AWS SDK")
        return()
    endif()

    # Add axon_logging if not already built
    if(NOT TARGET axon_logging AND EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
        add_subdirectory("${AXON_LOGGING_DIR}" ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
    endif()

    # Add axon_uploader if not already built
    if(NOT TARGET axon_uploader AND EXISTS "${AXON_UPLOADER_DIR}/CMakeLists.txt")
        add_subdirectory("${AXON_UPLOADER_DIR}" ${CMAKE_CURRENT_BINARY_DIR}/axon_uploader)
    endif()

    # Add axon_mcap if not already built (uploader may depend on it)
    if(NOT TARGET axon_mcap AND EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
        add_subdirectory("${AXON_MCAP_DIR}" ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
    endif()
endif()

if(NOT TARGET axon_uploader)
  message(STATUS "axon_transfer: skipping build because axon_uploader is not available")
  message(STATUS "axon_transfer: Install AWS SDK to enable this component")
  return()
endif()

set(AXON_TRANSFER_CORE_SOURCES
    src/transfer_config.cpp
    src/ws_client.cpp
    src/file_scanner.cpp
    src/upload_coordinator.cpp
    src/transfer_daemon.cpp
)

add_library(axon_transfer_core ${AXON_TRANSFER_CORE_SOURCES})

target_include_directories(
    axon_transfer_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(
    axon_transfer_core
    PUBLIC
        axon_uploader
        axon_logging
        yaml-cpp
        OpenSSL::Crypto
        OpenSSL::SSL
        Boost::system
        Boost::thread
        Boost::filesystem
        pthread
)

if(TARGET nlohmann_json::nlohmann_json)
  target_link_libraries(axon_transfer_core PUBLIC nlohmann_json::nlohmann_json)
elseif(DEFINED nlohmann_json_SOURCE_DIR)
  target_include_directories(
      axon_transfer_core
      PUBLIC ${nlohmann_json_SOURCE_DIR}/single_include
  )
endif()

target_compile_features(axon_transfer_core PUBLIC cxx_std_17)

add_executable(axon_transfer src/main.cpp)
target_link_libraries(axon_transfer PRIVATE axon_transfer_core)

# Set output name to use hyphen (axon-transfer)
set_target_properties(axon_transfer PROPERTIES OUTPUT_NAME "axon-transfer")

install(TARGETS axon_transfer RUNTIME DESTINATION bin)

if(AXON_BUILD_TESTS)
  add_subdirectory(test)
endif()
