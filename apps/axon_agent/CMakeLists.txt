# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

cmake_minimum_required(VERSION 3.14)
project(axon_agent LANGUAGES CXX)

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
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(
    axon_agent_core
    src/adapter_loader.cpp
    src/agent_service.cpp
    src/http_server.cpp
    src/process_manager.cpp
    src/profile_manager.cpp
)

target_include_directories(axon_agent_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(axon_agent_core PUBLIC cxx_std_17)
target_compile_options(axon_agent_core PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(
    axon_agent_core
    PUBLIC
        yaml-cpp
        Boost::system
        Threads::Threads
        dl
)

if(TARGET nlohmann_json::nlohmann_json)
    target_link_libraries(axon_agent_core PUBLIC nlohmann_json::nlohmann_json)
elseif(DEFINED nlohmann_json_SOURCE_DIR)
    target_include_directories(
        axon_agent_core
        PUBLIC ${nlohmann_json_SOURCE_DIR}/single_include
    )
endif()

add_executable(axon_agent src/main.cpp)
target_link_libraries(axon_agent PRIVATE axon_agent_core)
target_compile_options(axon_agent PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(axon_agent PROPERTIES OUTPUT_NAME "axon-agent")

install(TARGETS axon_agent RUNTIME DESTINATION bin)
install(DIRECTORY examples/robots DESTINATION share/axon_agent/examples)

message(STATUS "")
message(STATUS "=== Axon Agent Configuration ===")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Output Binary: axon-agent")
message(STATUS "================================")
message(STATUS "")
