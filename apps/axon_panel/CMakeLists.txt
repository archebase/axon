# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED AXON_REPO_ROOT)
    get_filename_component(
        AXON_REPO_ROOT
        "${CMAKE_CURRENT_SOURCE_DIR}/../.."
        ABSOLUTE
    )
endif()

# Check for npm (required for frontend build)
find_program(NPM_EXECUTABLE npm)
if(NOT NPM_EXECUTABLE)
    message(
        WARNING
        "npm not found. Skipping axon_panel build. Install Node.js to build axon_panel."
    )
    return()
endif()

# Find cpp-httplib
find_package(httplib QUIET)
if(NOT httplib_FOUND)
    # Try to use FetchContent if not found
    include(FetchContent)
    # Disable httplib's own install rules (header-only, embedded in binary)
    set(HTTPLIB_INSTALL OFF CACHE BOOL "Disable httplib install" FORCE)
    FetchContent_Declare(
        httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG v0.15.3
        EXCLUDE_FROM_ALL
    )
    FetchContent_MakeAvailable(httplib)
endif()

if(NOT TARGET nlohmann_json::nlohmann_json)
    find_package(nlohmann_json QUIET)
    if(NOT nlohmann_json_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            nlohmann_json
            URL
                https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
            URL_HASH
                SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
        )
        FetchContent_MakeAvailable(nlohmann_json)
    endif()
endif()

if(NOT TARGET axon_mcap)
    add_subdirectory(
        "${AXON_REPO_ROOT}/core/axon_mcap"
        "${CMAKE_CURRENT_BINARY_DIR}/axon_mcap"
    )
endif()

# Build frontend assets
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/frontend_dist.stamp
    COMMAND npm run build
    COMMAND
        ${CMAKE_COMMAND} -E touch
        ${CMAKE_CURRENT_BINARY_DIR}/frontend_dist.stamp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building frontend assets with Vite"
    VERBATIM
)

add_custom_target(
    axon_panel_frontend
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/frontend_dist.stamp
)

# Generate embedded asset source file
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/embedded_assets_generated.cpp
    COMMAND
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/embed_assets.sh
        ${CMAKE_CURRENT_SOURCE_DIR}/dist
        ${CMAKE_CURRENT_BINARY_DIR}/embedded_assets_generated.cpp
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/frontend_dist.stamp
    COMMENT "Embedding frontend assets into C++ source"
    VERBATIM
)

# Build axon_panel binary
add_executable(
    axon_panel
    server/main.cpp
    server/panel_api.cpp
    server/embedded_assets.cpp
    ${AXON_REPO_ROOT}/apps/axon_config/config_cache.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_assets_generated.cpp
)

# Ensure frontend is built before compiling
add_dependencies(axon_panel axon_panel_frontend)

target_include_directories(
    axon_panel
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/server
        ${AXON_REPO_ROOT}/apps/axon_config
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(
    axon_panel
    PRIVATE httplib::httplib nlohmann_json::nlohmann_json axon_mcap
)

# C++17 for Ubuntu 20.04 compatibility (GCC 9.3)
target_compile_features(axon_panel PRIVATE cxx_std_17)

# Set output name to use hyphen (axon-panel)
set_target_properties(axon_panel PROPERTIES OUTPUT_NAME "axon-panel")

# Install target
install(TARGETS axon_panel RUNTIME DESTINATION bin)

# Tests
option(AXON_PANEL_BUILD_TESTS "Build axon_panel tests" OFF)

if(AXON_PANEL_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(
        test_panel_api
        test/test_panel_api.cpp
        server/panel_api.cpp
        ${AXON_REPO_ROOT}/apps/axon_config/config_cache.cpp
    )

    target_include_directories(
        test_panel_api
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/server
            ${AXON_REPO_ROOT}/apps/axon_config
    )

    target_link_libraries(
        test_panel_api
        PRIVATE
            httplib::httplib
            nlohmann_json::nlohmann_json
            axon_mcap
            GTest::gtest
            GTest::gtest_main
    )

    target_compile_features(test_panel_api PRIVATE cxx_std_17)
    add_test(NAME test_panel_api COMMAND test_panel_api)
    set_tests_properties(test_panel_api PROPERTIES LABELS "axon_panel")
endif()
