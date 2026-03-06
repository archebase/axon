# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

cmake_minimum_required(VERSION 3.16)

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
    FetchContent_Declare(
        httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG v0.15.3
    )
    FetchContent_MakeAvailable(httplib)
endif()

# Build frontend assets
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/frontend_dist.stamp
    COMMAND npm run build
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
    server/embedded_assets.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_assets_generated.cpp
)

# Ensure frontend is built before compiling
add_dependencies(axon_panel axon_panel_frontend)

target_include_directories(
    axon_panel
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/server ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(axon_panel PRIVATE httplib::httplib)

# C++17 for Ubuntu 20.04 compatibility (GCC 9.3)
target_compile_features(axon_panel PRIVATE cxx_std_17)

# Install target
install(TARGETS axon_panel RUNTIME DESTINATION bin)
