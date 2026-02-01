cmake_minimum_required(VERSION 3.12)
project(axon_plugin_example)

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

# Build the simple load test executable
add_executable(simple_load_test
    simple_load_test.cpp
    ../axon_recorder/plugin_loader.cpp
)

target_include_directories(simple_load_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../axon_recorder
)

target_link_libraries(simple_load_test
    ${CMAKE_DL_LIBS}
)

# Build the full plugin loader test executable
add_executable(plugin_loader_test
    plugin_loader_test.cpp
    ../axon_recorder/plugin_loader.cpp
)

# Include directories
target_include_directories(plugin_loader_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../axon_recorder
)

# Link libraries
target_link_libraries(plugin_loader_test
    ${CMAKE_DL_LIBS}
    pthread
)

# Install
install(TARGETS simple_load_test plugin_loader_test
    RUNTIME DESTINATION bin
)
