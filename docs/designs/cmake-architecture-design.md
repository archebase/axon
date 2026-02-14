# Axon CMake Architecture Design

<!--
SPDX-FileCopyrightText: 2026 ArcheBase
SPDX-License-Identifier: MulanPSL-2.0
-->

This document describes the CMake build system architecture for the Axon project.

## Overview

Axon uses a **modular, hierarchical CMake build system** designed for:

1. **Standalone builds** - Each module can be built independently for faster iteration
2. **Unified builds** - All modules can be built together from the repository root
3. **Plugin architecture** - Middleware plugins are built as dynamic libraries
4. **Cross-platform ROS support** - Works with both ROS1 (Noetic) and ROS2 (Humble/Jazzy/Rolling)

## Directory Structure

```
axon/
├── CMakeLists.txt              # Root CMake configuration
├── cmake/
│   └── Modules/
│       ├── AxonStdLib.cmake    # Standard helpers (PIC, coverage)
│       └── AxonCoverage.cmake  # Coverage configuration
├── core/
│   ├── axon_logging/
│   │   └── CMakeLists.txt
│   ├── axon_mcap/
│   │   └── CMakeLists.txt
│   └── axon_uploader/
│       └── CMakeLists.txt
├── middlewares/
│   ├── CMakeLists.txt          # Plugin orchestration
│   ├── ros1/
│   │   └── CMakeLists.txt
│   ├── ros2/
│   │   └── CMakeLists.txt
│   ├── zenoh/
│   │   └── CMakeLists.txt
│   ├── mock/
│   │   └── CMakeLists.txt
│   └── filters/
│       └── CMakeLists.txt
└── apps/
    ├── axon_recorder/
    │   ├── CMakeLists.txt
    │   └── test/
    │       └── CMakeLists.txt
    └── axon_config/
        └── CMakeLists.txt
```

## Core Design Principles

### 1. Centralized Path Definitions

All path variables are defined in the root `CMakeLists.txt` and propagated to child modules:

```cmake
# Root CMakeLists.txt
set(AXON_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "")
set(AXON_CORE_DIR "${AXON_REPO_ROOT}/core" CACHE INTERNAL "")
set(AXON_LOGGING_DIR "${AXON_CORE_DIR}/axon_logging" CACHE INTERNAL "")
set(AXON_MCAP_DIR "${AXON_CORE_DIR}/axon_mcap" CACHE INTERNAL "")
set(AXON_MIDDLEWARES_DIR "${AXON_REPO_ROOT}/middlewares" CACHE INTERNAL "")
set(AXON_APPS_DIR "${AXON_REPO_ROOT}/apps" CACHE INTERNAL "")
set(AXON_CMAKE_MODULES_DIR "${AXON_REPO_ROOT}/cmake/Modules" CACHE INTERNAL "")
```

This ensures consistent path references across all modules.

### 2. CMake Module Discovery Pattern

Each module follows a consistent pattern to discover and include shared CMake modules:

```cmake
# Try parent-provided path first, then compute relative path
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    # Compute relative path to cmake/Modules
    set(_AXON_CMAKE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules")
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)
```

### 3. Shared Dependency Management

Common dependencies are fetched once at the root level and reused by all modules:

```cmake
# Root CMakeLists.txt
include(FetchContent)

# nlohmann/json - shared across all modules
FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(nlohmann_json)
```

### 4. Modern CMake Target-Based Linking

Always use modern target-based linking instead of variable-based approaches:

```cmake
# GOOD: Target-based linking (automatic include paths and compile definitions)
target_link_libraries(my_target
    PRIVATE
        axon_mcap
        rclcpp::rclcpp
        GTest::gtest
)

# BAD: Variable-based linking (manual include paths required)
target_include_directories(my_target PRIVATE ${RCLCPP_INCLUDE_DIRS})
target_link_libraries(my_target ${RCLCPP_LIBRARIES})
```

## Module Structure

### Core Libraries

Core libraries are middleware-agnostic with no ROS dependencies:

| Library | Purpose | Dependencies |
|---------|---------|--------------|
| `axon_logging` | Boost.Log-based logging infrastructure | Boost::log, Boost::thread |
| `axon_mcap` | MCAP file writer and validator | libzstd, liblz4, mcap (FetchContent) |
| `axon_uploader` | S3 multipart upload with state recovery | AWS SDK, SQLite3, OpenSSL |

### Middleware Plugins

Middleware plugins implement the C ABI interface defined in `apps/axon_recorder/plugin_loader.hpp`:

| Plugin | Output | ROS Version |
|--------|--------|-------------|
| `libaxon_ros1.so` | ROS1 Noetic support | ROS 1 Noetic |
| `libaxon_ros2.so` | ROS2 Humble/Jazzy/Rolling | ROS 2 |
| `libaxon_zenoh.so` | Zenoh middleware | None (standalone) |
| `libmock_plugin.so` | Testing without ROS | None (standalone) |

### Plugin Auto-Detection

The middlewares `CMakeLists.txt` auto-detects available ROS versions:

```cmake
# Auto-detect ROS1
if(EXISTS "${ROS1_DIR}/setup.bash" AND EXISTS "${ROS1_DIR}/lib/python3/dist-packages/ros")
    set(AXON_BUILD_ROS1_DEFAULT ON)
else()
    set(AXON_BUILD_ROS1_DEFAULT OFF)
endif()

# Auto-detect ROS2
if(EXISTS "${ROS2_DIR}/setup.bash" OR EXISTS "/opt/ros/jazzy/setup.bash" OR EXISTS "/opt/ros/rolling/setup.bash")
    set(AXON_BUILD_ROS2_DEFAULT ON)
else()
    set(AXON_BUILD_ROS2_DEFAULT OFF)
endif()
```

## Shared CMake Modules

### AxonStdLib.cmake

Provides standard helper functions:

```cmake
# Set position-independent code for shared libraries
function(axon_set_position_independent_code target)
    set_target_properties(${target} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )
endfunction()

# Add coverage flags (GNU/Clang only)
function(axon_add_coverage target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(${target} PRIVATE --coverage)
    endif()
endfunction()
```

### AxonCoverage.cmake

Configures code coverage with lcov/genhtml:

```cmake
# Enable coverage for target
if(COMMAND axon_add_coverage)
    axon_add_coverage(my_target)
endif()
```

## Test Auto-Discovery

Tests are automatically discovered using `GLOB_CONFIGURE_DEPENDS`:

```cmake
# Auto-discover all test files
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp"
)

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
        axon_mcap
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
```

## Build Options

| Option | Description | Default |
|--------|-------------|---------|
| `AXON_BUILD_TESTS` | Build unit tests | OFF |
| `AXON_ENABLE_COVERAGE` | Enable code coverage | OFF |
| `AXON_BUILD_UPLOADER` | Build axon_uploader (requires AWS SDK) | OFF |
| `AXON_BUILD_ROS1_PLUGIN` | Build ROS1 plugin | Auto-detect |
| `AXON_BUILD_ROS2_PLUGIN` | Build ROS2 plugin | Auto-detect |
| `AXON_BUILD_ZENOH_PLUGIN` | Build Zenoh plugin | OFF |
| `AXON_BUILD_MOCK_PLUGIN` | Build mock plugin | OFF |
| `AXON_ENABLE_DEPTH_COMPRESSION` | Enable depth compression | OFF |

## Common Patterns

### Adding a New Core Library

1. Create directory: `core/axon_newlib/`
2. Create `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.14)
project(axon_newlib)

# Include centralized modules
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
include(AxonCoverage OPTIONAL)

# Build library
add_library(axon_newlib
    src/implementation.cpp
)

target_include_directories(axon_newlib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Tests
option(AXON_NEWLIB_BUILD_TESTS "Build tests" OFF)
if(AXON_NEWLIB_BUILD_TESTS OR AXON_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "test/test_*.cpp")
    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_link_libraries(${TEST_NAME} axon_newlib GTest::gtest GTest::gtest_main)
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
endif()
```

3. Add to root `CMakeLists.txt`:

```cmake
if(EXISTS "${AXON_NEWLIB_DIR}/CMakeLists.txt")
    add_subdirectory(${AXON_NEWLIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_newlib)
endif()
```

### Adding a New Middleware Plugin

1. Create directory: `middlewares/new_plugin/`
2. Create `CMakeLists.txt` following the ROS2 plugin pattern
3. Export the C ABI interface:

```cpp
// src/new_plugin_export.cpp
#include "plugin_loader.hpp"

extern "C" {
AxonPluginDescriptor axon_get_plugin_descriptor() {
    return AxonPluginDescriptor{
        .name = "new_plugin",
        .version = "1.0.0",
        .abi_version_major = 1,
        .abi_version_minor = 0,
    };
}

const AxonPluginVtable* axon_get_plugin_vtable() {
    static AxonPluginVtable vtable = {
        .init = new_plugin_init,
        .start = new_plugin_start,
        .stop = new_plugin_stop,
        .subscribe = new_plugin_subscribe,
        .publish = new_plugin_publish,
    };
    return &vtable;
}
}
```

4. Add to `middlewares/CMakeLists.txt`:

```cmake
option(AXON_BUILD_NEW_PLUGIN "Build new plugin" OFF)
if(AXON_BUILD_NEW_PLUGIN)
    add_subdirectory(new_plugin ${CMAKE_CURRENT_BINARY_DIR}/new_plugin)
endif()
```

## Formatting

CMake files are formatted using [gersemi](https://github.com/BlankSpruce/gersemi):

```bash
# Format all CMake files
make format

# Or manually
gersemi --in-place CMakeLists.txt apps/*/CMakeLists.txt core/*/CMakeLists.txt
```

## Troubleshooting

### "AxonStdLib.cmake not found"

Ensure you're building from the repository root, or set `AXON_REPO_ROOT`:

```bash
cmake -DAXON_REPO_ROOT=/path/to/axon ..
```

### ROS headers not found

Use modern target-based linking:

```cmake
# Instead of ${ROSCPP_LIB}, use:
target_link_libraries(my_target rclcpp::rclcpp)
```

### Duplicate symbol errors

Ensure each library is defined in only one CMakeLists.txt. Use `add_subdirectory` to include dependencies.

## See Also

- [Middleware Plugin Architecture](middleware-plugin-architecture-design.md)
- [Project Structure](project-structure.md)
- [Logging Infrastructure](logging-infrastructure-design.md)
