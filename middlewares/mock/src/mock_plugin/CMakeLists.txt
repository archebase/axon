cmake_minimum_required(VERSION 3.12)
project(mock_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build the mock plugin as a shared library
add_library(${PROJECT_NAME} SHARED
    src/mock_plugin.cpp
    src/mock_plugin_export.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set library properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN ON
)

# Link pthread (required for std::thread)
target_link_libraries(${PROJECT_NAME}
    pthread
)

# Also create an object library for testing
add_library(${PROJECT_NAME}_obj OBJECT
    src/mock_plugin.cpp
)

target_include_directories(${PROJECT_NAME}_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES include/mock_plugin.hpp
    DESTINATION include
)

# =============================================================================
# Tests
# =============================================================================

# Simple plugin load test (requires building core libs first)
add_executable(test_mock_plugin_load
    test/test_mock_plugin_load.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../apps/axon_recorder/plugin_loader.cpp
)

target_include_directories(test_mock_plugin_load PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../apps/axon_recorder
)

target_link_libraries(test_mock_plugin_load
    ${CMAKE_DL_LIBS}
)

# E2E test with mock plugin
add_executable(test_mock_plugin_e2e
    test/test_mock_plugin_e2e.cpp
)

target_include_directories(test_mock_plugin_e2e PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_mock_plugin_e2e
    ${PROJECT_NAME}_obj
    pthread
)

# Install tests
install(TARGETS test_mock_plugin_load test_mock_plugin_e2e
    RUNTIME DESTINATION bin
)
