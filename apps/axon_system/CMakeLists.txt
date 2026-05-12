# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

cmake_minimum_required(VERSION 3.14)
project(axon_system LANGUAGES CXX)

if(CMAKE_PROJECT_VERSION)
    set(AXON_SYSTEM_VERSION "${CMAKE_PROJECT_VERSION}")
else()
    set(AXON_SYSTEM_VERSION "0.4.0-dev")
endif()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/opt/axon" CACHE PATH "Installation prefix" FORCE)
endif()

if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    set(_AXON_CMAKE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules")
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

include(AxonStdLib REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(nlohmann_json QUIET)

add_library(
    axon_system_core
    src/http_server.cpp
    src/procfs_reader.cpp
    src/resource_collector.cpp
    src/system_service.cpp
)

target_include_directories(axon_system_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(axon_system_core PUBLIC cxx_std_17)
target_compile_definitions(axon_system_core PUBLIC AXON_SYSTEM_VERSION="${AXON_SYSTEM_VERSION}")
target_link_libraries(
    axon_system_core
    PUBLIC
        Boost::system
        Threads::Threads
)

if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(axon_system_core PUBLIC nlohmann_json::nlohmann_json)
elseif(DEFINED nlohmann_json_SOURCE_DIR)
    target_include_directories(
        axon_system_core
        PUBLIC ${nlohmann_json_SOURCE_DIR}/single_include
    )
else()
    message(FATAL_ERROR "axon_system requires nlohmann_json")
endif()

axon_enable_warnings(axon_system_core)

add_executable(axon_system src/main.cpp)
target_link_libraries(axon_system PRIVATE axon_system_core)
axon_enable_warnings(axon_system)
set_target_properties(axon_system PROPERTIES OUTPUT_NAME "axon-system")

install(TARGETS axon_system RUNTIME DESTINATION bin)

option(AXON_SYSTEM_BUILD_TESTS "Build axon_system tests" OFF)
if(AXON_SYSTEM_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

message(STATUS "")
message(STATUS "=== Axon System Configuration ===")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Output Binary: axon-system")
message(STATUS "=================================")
message(STATUS "")
