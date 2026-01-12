# Middleware Plugin Architecture Design

**Date:** 2025-01-11
**Author:** ArcheBase
**Status:** Design Document

## Overview

This document describes the plugin-based middleware integration architecture for Axon. The design cleanly separates middleware-specific code (ROS1, ROS2, etc.) from core functionality, enabling:

- Middleware-agnostic core libraries
- Independent compilation and deployment of middleware plugins
- Runtime dynamic loading of middleware support
- Easy addition of new middleware integrations

## Motivation

### Problems with Monolithic Design

The original Axon architecture mixed middleware-specific code with core functionality:

1. **Tight Coupling**: Core libraries had direct dependencies on ROS headers and libraries
2. **Build Complexity**: Required ROS environment to build any part of the system
3. **Testing Difficulty**: Could not test core functionality without ROS installed
4. **Deployment Overhead**: All ROS dependencies required even for non-ROS deployments

### Goals

1. **Separation of Concerns**: Core libraries should be completely middleware-agnostic
2. **Independent Compilation**: Each middleware plugin compiles independently into a dynamic library
3. **Clean Interface**: Middleware integration through a well-defined C API
4. **Runtime Flexibility**: Load required middleware plugins at runtime
5. **Extensibility**: Easy to add new middleware support (e.g., ROS3, other robotics frameworks)

## Architecture

### High-Level Design

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Main Application                                │
│                    (Middleware-Agnostic Core)                          │
│                                                                         │
│  Application Layer                                                      │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                    Plugin Loader                                 │  │
│  │  - Discover plugins in configured directories                    │  │
│  │  - Load shared libraries (.so)                                   │  │
│  │  - Resolve middleware_get_descriptor symbol                      │  │
│  │  - Validate plugin compatibility                                │  │
│  └────────────────────┬────────────────────┬────────────────────────┘  │
│                       │                    │                             │
│  ┌────────────────────▼────────────────────▼────────────────────────┐  │
│  │              Middleware Abstraction Layer                        │  │
│  │         (Unified C API - middleware_abi.h)                       │  │
│  └────────────────────┬────────────────────┬────────────────────────┘  │
│                       │                    │                             │
│           ┌───────────▼──────┐   ┌───────▼──────────┐                   │
│           │  ROS1 Plugin     │   │  ROS2 Plugin     │                   │
│           │  libaxon_ros1.so │   │  libaxon_ros2.so │                   │
│           └──────────────────┘   └──────────────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         Core Libraries (Middleware-Agnostic)            │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────────────────┐  │
│  │ axon_mcap     │  │ axon_logging  │  │ axon_uploader              │  │
│  │ - MCAP I/O    │  │ - Logging     │  │ - S3 upload                │  │
│  │ - Compression │  │ - Sinks       │  │ - State management         │  │
│  │ - Validation  │  │ - Formatting  │  │ - Retry logic              │  │
│  └───────────────┘  └───────────────┘  └───────────────────────────┘  │
│                                                                         │
│  No ROS dependencies. Pure C++17 with standard library.                │
└─────────────────────────────────────────────────────────────────────────┘
```

## Plugin Interface

### C API Definition

The plugin interface is defined in `include/axon/middleware_abi.hpp`:

```cpp
#ifndef AXON_MIDDLEWARE_ABI_HPP
#define AXON_MIDDLEWARE_ABI_HPP

#ifdef __cplusplus
extern "C" {
#endif

// Version information
#define AXON_ABI_VERSION_MAJOR 1
#define AXON_ABI_VERSION_MINOR 0

// Error codes
typedef enum {
    AXON_SUCCESS = 0,
    AXON_ERROR_INVALID_ARGUMENT = -1,
    AXON_ERROR_NOT_INITIALIZED = -2,
    AXON_ERROR_ALREADY_INITIALIZED = -3,
    AXON_ERROR_NOT_STARTED = -4,
    AXON_ERROR_ALREADY_STARTED = -5,
    AXON_ERROR_INTERNAL = -100,
} axon_status_t;

// Plugin lifecycle callback types
typedef axon_status_t (*axon_init_fn)(const char* config_json);
typedef axon_status_t (*axon_start_fn)(void);
typedef axon_status_t (*axon_stop_fn)(void);
typedef axon_status_t (*axon_shutdown_fn)(void);

// Message callback types
typedef void (*axon_message_callback_fn)(
    const char* topic_name,
    const uint8_t* message_data,
    size_t message_size,
    const char* message_type,
    uint64_t timestamp,
    void* user_data
);

// Configuration functions
typedef axon_status_t (*axon_set_callback_fn)(
    axon_message_callback_fn callback,
    void* user_data
);

typedef axon_status_t (*axon_subscribe_fn)(
    const char* topic_name,
    const char* message_type
);

typedef axon_status_t (*axon_publish_fn)(
    const char* topic_name,
    const uint8_t* message_data,
    size_t message_size,
    const char* message_type
);

// Plugin virtual function table
typedef struct {
    // Lifecycle
    axon_init_fn init;
    axon_start_fn start;
    axon_stop_fn stop;
    axon_shutdown_fn shutdown;

    // Message handling
    axon_set_callback_fn set_message_callback;
    axon_subscribe_fn subscribe;
    axon_publish_fn publish;

    // Query (future extension)
    void* reserved[8];

} axon_plugin_vtable_t;

// Plugin descriptor (exported symbol)
typedef struct {
    uint32_t abi_version_major;
    uint32_t abi_version_minor;

    const char* middleware_name;        // e.g., "ROS1", "ROS2"
    const char* middleware_version;     // e.g., "1.0.0"
    const char* plugin_version;         // e.g., "1.0.0"

    axon_plugin_vtable_t* vtable;

    void* reserved[16];                 // Future expansion

} axon_plugin_descriptor_t;

// Each plugin must export this function
extern const axon_plugin_descriptor_t* axon_get_plugin_descriptor(void);

#ifdef __cplusplus
}
#endif

#endif // AXON_MIDDLEWARE_ABI_HPP
```

### Plugin Implementation Example (ROS2)

```cpp
// middlewares/ros2/src/ros2_plugin.cpp

#include <axon/middleware_abi.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ros2_node.hpp"

static Ros2Node* g_node = nullptr;
static axon_message_callback_fn g_message_callback = nullptr;
static void* g_callback_user_data = nullptr;

// Lifecycle implementation
static axon_status_t ros2_init(const char* config_json) {
    if (g_node != nullptr) {
        return AXON_ERROR_ALREADY_INITIALIZED;
    }

    try {
        g_node = new Ros2Node(config_json);
        return AXON_SUCCESS;
    } catch (...) {
        return AXON_ERROR_INTERNAL;
    }
}

static axon_status_t ros2_start(void) {
    if (g_node == nullptr) {
        return AXON_ERROR_NOT_INITIALIZED;
    }
    return g_node->start() ? AXON_SUCCESS : AXON_ERROR_INTERNAL;
}

static axon_status_t ros2_stop(void) {
    if (g_node == nullptr) {
        return AXON_ERROR_NOT_INITIALIZED;
    }
    return g_node->stop() ? AXON_SUCCESS : AXON_ERROR_INTERNAL;
}

static axon_status_t ros2_shutdown(void) {
    if (g_node != nullptr) {
        delete g_node;
        g_node = nullptr;
    }
    return AXON_SUCCESS;
}

// Message handling
static axon_status_t ros2_set_callback(
    axon_message_callback_fn callback,
    void* user_data
) {
    g_message_callback = callback;
    g_callback_user_data = user_data;

    if (g_node != nullptr) {
        g_node->set_message_callback(callback, user_data);
    }

    return AXON_SUCCESS;
}

static axon_status_t ros2_subscribe(
    const char* topic_name,
    const char* message_type
) {
    if (g_node == nullptr) {
        return AXON_ERROR_NOT_INITIALIZED;
    }
    return g_node->subscribe(topic_name, message_type)
           ? AXON_SUCCESS : AXON_ERROR_INTERNAL;
}

static axon_status_t ros2_publish(
    const char* topic_name,
    const uint8_t* message_data,
    size_t message_size,
    const char* message_type
) {
    if (g_node == nullptr) {
        return AXON_ERROR_NOT_INITIALIZED;
    }
    return g_node->publish(topic_name, message_data, message_size, message_type)
           ? AXON_SUCCESS : AXON_ERROR_INTERNAL;
}

// Static vtable
static axon_plugin_vtable_t ros2_vtable = {
    .init = ros2_init,
    .start = ros2_start,
    .stop = ros2_stop,
    .shutdown = ros2_shutdown,
    .set_message_callback = ros2_set_callback,
    .subscribe = ros2_subscribe,
    .publish = ros2_publish,
};

// Exported descriptor
extern "C" {
    const axon_plugin_descriptor_t* axon_get_plugin_descriptor(void) {
        static const axon_plugin_descriptor_t descriptor = {
            .abi_version_major = AXON_ABI_VERSION_MAJOR,
            .abi_version_minor = AXON_ABI_VERSION_MINOR,
            .middleware_name = "ROS2",
            .middleware_version = RCLCPP_VERSION_STR,
            .plugin_version = "1.0.0",
            .vtable = &ros2_vtable,
        };
        return &descriptor;
    }
}
```

## Plugin Loader

### Loading Process

```cpp
// src/plugin_loader.cpp

#include <axon/middleware_abi.hpp>
#include <dlfcn.h>

class PluginLoader {
public:
    struct Plugin {
        void* handle;
        const axon_plugin_descriptor_t* descriptor;
    };

    std::optional<Plugin> load(const std::string& plugin_path) {
        // Load shared library
        void* handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
        if (!handle) {
            std::cerr << "Failed to load plugin: " << dlerror() << std::endl;
            return std::nullopt;
        }

        // Resolve symbol
        auto get_descriptor = reinterpret_cast<decltype(&axon_get_plugin_descriptor)>(
            dlsym(handle, "axon_get_plugin_descriptor")
        );

        if (!get_descriptor) {
            std::cerr << "Plugin missing axon_get_plugin_descriptor symbol" << std::endl;
            dlclose(handle);
            return std::nullopt;
        }

        // Get descriptor
        const axon_plugin_descriptor_t* descriptor = get_descriptor();

        // Validate ABI version
        if (descriptor->abi_version_major != AXON_ABI_VERSION_MAJOR) {
            std::cerr << "ABI version mismatch" << std::endl;
            dlclose(handle);
            return std::nullopt;
        }

        return Plugin{handle, descriptor};
    }

    void unload(const Plugin& plugin) {
        if (plugin.handle) {
            dlclose(plugin.handle);
        }
    }
};
```

### Usage in Main Application

```cpp
// src/main.cpp

#include "plugin_loader.hpp"
#include "config_loader.hpp"
#include <axon/mcap_writer.hpp>
#include <axon/logging.hpp>

int main(int argc, char** argv) {
    // Parse command-line arguments
    std::string config_path = "/opt/axon/share/axon/config/default.yaml";
    // Override with --config flag if provided...

    // Load configuration (YAML)
    ConfigLoader config_loader;
    auto config = config_loader.load(config_path);

    if (!config) {
        std::cerr << "Failed to load configuration" << std::endl;
        return 1;
    }

    // Initialize core (no ROS dependencies)
    axon::Log::init(config->logging);
    axon::McapWriter writer(config->dataset);

    // Load middleware plugins
    PluginLoader plugin_loader;
    std::vector<LoadedPlugin> loaded_plugins;

    for (const auto& plugin_name : config->middleware.autoload) {
        auto it = config->middleware.plugins.find(plugin_name);
        if (it == config->middleware.plugins.end() || !it->second.enabled) {
            continue;
        }

        const auto& plugin_config = it->second;

        // Find plugin library in search paths
        std::string plugin_path;
        for (const auto& search_path : config->middleware.plugin_search_paths) {
            std::string test_path = search_path + "/" + plugin_config.library;
            if (std::filesystem::exists(test_path)) {
                plugin_path = test_path;
                break;
            }
        }

        if (plugin_path.empty()) {
            LOG_ERROR("Plugin library not found: {}", plugin_config.library);
            continue;
        }

        // Load plugin
        auto plugin = plugin_loader.load(plugin_path);
        if (!plugin) {
            LOG_ERROR("Failed to load plugin: {}", plugin_name);
            continue;
        }

        // Serialize plugin config to JSON for C API
        std::string plugin_config_json = serialize_yaml_to_json(plugin_config.config);

        // Initialize plugin
        auto status = plugin->descriptor->vtable->init(plugin_config_json.c_str());
        if (status != AXON_SUCCESS) {
            LOG_ERROR("Failed to initialize plugin: {}", plugin_name);
            continue;
        }

        // Set message callback
        plugin->descriptor->vtable->set_message_callback(
            [](const char* topic,
               const uint8_t* data,
               size_t size,
               const char* type,
               uint64_t timestamp,
               void* user_data) {
                auto* writer = static_cast<axon::McapWriter*>(user_data);
                writer->write(topic, data, size, type, timestamp);
            },
            &writer
        );

        // Start plugin
        status = plugin->descriptor->vtable->start();
        if (status != AXON_SUCCESS) {
            LOG_ERROR("Failed to start plugin: {}", plugin_name);
            continue;
        }

        loaded_plugins.push_back({plugin_name, plugin});
        LOG_INFO("Loaded and started plugin: {}", plugin_name);
    }

    if (loaded_plugins.empty()) {
        LOG_ERROR("No middleware plugins loaded");
        return 1;
    }

    LOG_INFO("Axon recorder running with {} plugin(s)", loaded_plugins.size());

    // Main loop
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    for (const auto& [name, plugin] : loaded_plugins) {
        plugin->descriptor->vtable->stop();
        plugin->descriptor->vtable->shutdown();
        plugin_loader.unload(*plugin);
        LOG_INFO("Stopped plugin: {}", name);
    }

    return 0;
}
```

#### Helper: YAML to JSON Conversion

```cpp
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::string serialize_yaml_to_json(const std::map<std::string, YAML::Node>& yaml_config) {
    json j;

    for (const auto& [key, node] : yaml_config) {
        switch (node.Type()) {
            case YAML::NodeType::Null:
                j[key] = nullptr;
                break;
            case YAML::NodeType::Scalar:
                // Try to parse as int, float, or string
                try {
                    j[key] = node.as<int>();
                } catch (...) {
                    try {
                        j[key] = node.as<double>();
                    } catch (...) {
                        j[key] = node.as<std::string>();
                    }
                }
                break;
            case YAML::NodeType::Sequence:
                j[key] = node.as<std::vector<int>>();
                break;
            case YAML::NodeType::Map:
                // Recursive conversion would go here
                j[key] = "[complex object]";
                break;
            default:
                j[key] = node.as<std::string>();
                break;
        }
    }

    return j.dump();
}
```

## Build Configuration

### Core Libraries (No ROS Dependencies)

```cmake
# core/axon_mcap/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)
project(axon_mcap)

# Pure C++ target - no ROS dependencies
add_library(axon_mcap STATIC
    src/mcap_writer_wrapper.cpp
    src/mcap_validator.cpp
)

target_include_directories(axon_mcap PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Only standard library and mcap (header-only)
target_link_libraries(axon_mcap PUBLIC
    mcap::mcap_headers
    Zstd::Zstd
    LZ4::LZ4
)
```

### ROS2 Plugin

```cmake
# middlewares/ros2/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)
project(axon_ros2_plugin)

# Find ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build as shared library (plugin)
add_library(axon_ros2_plugin SHARED
    src/ros2_plugin.cpp
    src/ros2_node.cpp
    src/ros2_subscriber.cpp
)

target_include_directories(axon_ros2_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_SOURCE_DIR}/include  # For middleware_abi.hpp
)

# Link ROS2 libraries
target_link_libraries(axon_ros2_plugin
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
)

# Export plugin symbol
target_compile_definitions(axon_ros2_plugin PRIVATE
    AXON_BUILDING_PLUGIN
)

ament_package()
```

### Main Application

```cmake
# CMakeLists.txt (root)

cmake_minimum_required(VERSION 3.15)
project(axon_recorder)

# Build core libraries
add_subdirectory(core/axon_mcap)
add_subdirectory(core/axon_logging)
add_subdirectory(core/axon_uploader)

# Build main application (NO ROS dependencies!)
add_executable(axon_recorder
    src/main.cpp
    src/plugin_loader.cpp
)

target_link_libraries(axon_recorder PRIVATE
    axon_mcap
    axon_logging
    axon_uploader
    dl  # For dlopen/dlsym
)

# Plugins are built separately and loaded at runtime
# Optional: add_subdirectory(middlewares/ros2)  # Only if building with ROS2
```

## Deployment

### Directory Layout

```
/opt/axon/
├── bin/
│   └── axon_recorder              # Main executable
├── lib/
│   ├── libaxon_mcap.a             # Core libraries (static)
│   ├── libaxon_logging.a
│   ├── libaxon_uploader.a
│   │
│   └── plugins/                   # Middleware plugins
│       ├── libaxon_ros1.so        # ROS1 plugin
│       └── libaxon_ros2.so        # ROS2 plugin
└── share/
    └── axon/
        └── config/
            └── default.yaml       # Unified configuration file
```

### Plugin Configuration (YAML)

Plugin configuration is integrated into the main YAML configuration file. The same configuration file controls both the application and middleware plugins.

`share/axon/config/default.yaml`:

```yaml
# =============================================================================
# Axon Recorder Configuration
# Unified configuration for core functionality and middleware plugins
# =============================================================================

# -----------------------------------------------------------------------------
# Dataset Configuration
# -----------------------------------------------------------------------------
dataset:
  path: /data/recordings
  mode: append                    # append | create
  stats_file_path: /data/recordings/recorder_stats.json

# -----------------------------------------------------------------------------
# Topics Configuration
# -----------------------------------------------------------------------------
topics:
  - name: /camera/image_raw
    message_type: sensor_msgs/Image
    batch_size: 100
    flush_interval_ms: 1000

  - name: /lidar/scan
    message_type: sensor_msgs/LaserScan
    batch_size: 50
    flush_interval_ms: 500

# -----------------------------------------------------------------------------
# Recording Configuration
# -----------------------------------------------------------------------------
recording:
  max_disk_usage_gb: 100
  auto_restart: false

# -----------------------------------------------------------------------------
# Middleware Plugin Configuration
# -----------------------------------------------------------------------------
# Controls which middleware plugins are loaded and their configuration
middleware:
  # Plugin search paths (searched in order)
  plugin_search_paths:
    - /opt/axon/lib/plugins
    - /usr/local/lib/axon/plugins
    - ~/.axon/plugins

  # Auto-load plugins on startup
  autoload:
    - ros2                        # Load ros2 plugin by default

  # Plugin-specific configurations
  plugins:
    # ROS1 Plugin Configuration
    ros1:
      enabled: false               # Enable to use ROS1

      # Plugin library (relative to plugin_search_paths)
      library: libaxon_ros1.so

      # ROS1-specific configuration
      config:
        # Node configuration
        node_name: axon_recorder

        # ROS master
        ros_master_uri: ~          # Default to $ROS_MASTER_URI

        # Topic remappings
        remappings: []

        # Queue sizes
        subscription_queue_size: 10
        publisher_queue_size: 10

        # Threading
        num_threads: ~              # Auto-detect (default: hardware concurrency)

    # ROS2 Plugin Configuration
    ros2:
      enabled: true                # Enable to use ROS2

      # Plugin library (relative to plugin_search_paths)
      library: libaxon_ros2.so

      # ROS2-specific configuration
      config:
        # Node configuration
        node_name: axon_recorder
        namespace: ~                # Empty string for global namespace

        # ROS domain ID
        ros_domain_id: ~            # Default to $ROS_DOMAIN_ID

        # RMW implementation
        rmw_implementation: ~       # Default to $RMW_IMPLEMENTATION

        # Topic remappings (ROS2 format)
        remappings: []

        # QoS profiles
        qos:
          default:                 # Default QoS for subscriptions
            history: keep_last     # keep_last | keep_all
            depth: 10
            reliability: reliable   # reliable | best_effort
            durability: volatile    # volatile | transient_local

        # Executor configuration
        executor:
          type: single_threaded    # single_threaded | static_executor
          num_threads: ~            # Auto-detect

    # Future plugins can be added here
    # cyclonedds:
    #   enabled: false
    #   library: libaxon_cyclonedds.so
    #   config:
    #     domain_id: 0

# -----------------------------------------------------------------------------
# Logging Configuration
# -----------------------------------------------------------------------------
logging:
  console:
    enabled: true
    level: info                    # debug | info | warn | error | fatal
    colors: true

  file:
    enabled: false
    level: debug
    directory: /var/log/axon
    pattern: "axon_%Y%m%d_%H%M%S.log"
    format: json                   # json | text
    rotation_size_mb: 100
    max_files: 10
    rotate_at_midnight: true

# -----------------------------------------------------------------------------
# Edge Upload Configuration
# -----------------------------------------------------------------------------
upload:
  enabled: false

  s3:
    endpoint_url: ""
    bucket: "axon-raw-data"
    region: "us-east-1"
    use_ssl: true
    verify_ssl: true

  num_workers: 2

  retry:
    max_retries: 5
    initial_delay_ms: 1000
    max_delay_ms: 300000
    exponential_base: 2.0
    jitter: true

  state_db_path: "/var/lib/axon/uploader_state.db"
  delete_after_upload: true
  failed_uploads_dir: "/data/failed_uploads/"

  warn_pending_gb: 8
  alert_pending_gb: 20
```

### Configuration Loading

Configuration is loaded with cascading priority (highest to lowest):

1. **Command-line arguments**: `--middleware.ros2.enabled=false`
2. **Environment variables**: `AXON_MIDDLEWARE_ROS2_ENABLED=false`
3. **YAML configuration file**: `middleware.ros2.enabled: false`
4. **Default values**: from code

#### Example: Loading Configuration with yaml-cpp

```cpp
#include <yaml-cpp/yaml.h>
#include <optional>

struct MiddlewareConfig {
    bool enabled;
    std::string library;
    std::map<std::string, YAML::Node> config;
};

struct MiddlewareSection {
    std::vector<std::string> plugin_search_paths;
    std::vector<std::string> autoload;
    std::map<std::string, MiddlewareConfig> plugins;
};

class ConfigLoader {
public:
    std::optional<MiddlewareSection> load_middleware_config(
        const std::string& config_path
    ) {
        try {
            YAML::Node config = YAML::LoadFile(config_path);

            if (!config["middleware"]) {
                return std::nullopt;
            }

            YAML::Node middleware = config["middleware"];
            MiddlewareSection section;

            // Load search paths
            if (middleware["plugin_search_paths"]) {
                for (const auto& path : middleware["plugin_search_paths"]) {
                    section.plugin_search_paths.push_back(path.as<std::string>());
                }
            }

            // Load autoload list
            if (middleware["autoload"]) {
                for (const auto& plugin : middleware["autoload"]) {
                    section.autoload.push_back(plugin.as<std::string>());
                }
            }

            // Load plugin configurations
            if (middleware["plugins"]) {
                for (const auto& kv : middleware["plugins"]) {
                    std::string name = kv.first.as<std::string>();
                    YAML::Node plugin = kv.second;

                    MiddlewareConfig plugin_config;
                    plugin_config.enabled = plugin["enabled"] ?
                        plugin["enabled"].as<bool>() : false;
                    plugin_config.library = plugin["library"] ?
                        plugin["library"].as<std::string>() : "";

                    if (plugin["config"]) {
                        for (const auto& ckv : plugin["config"]) {
                            plugin_config.config[ckv.first.as<std::string>()] = ckv.second;
                        }
                    }

                    section.plugins[name] = plugin_config;
                }
            }

            return section;

        } catch (const YAML::Exception& e) {
            std::cerr << "Failed to load config: " << e.what() << std::endl;
            return std::nullopt;
        }
    }
};
```

### Environment Variable Override

Environment variables follow the pattern: `AXON_<SECTION>_<KEY>`

```bash
# Enable ROS1 plugin
export AXON_MIDDLEWARE_ROS1_ENABLED=true

# Set ROS2 domain ID
export AXON_MIDDLEWARE_ROS2_CONFIG__ROS_DOMAIN_ID=42

# Set plugin search paths
export AXON_MIDDLEWARE_PLUGIN_SEARCH_PATHS="/opt/axon/lib/plugins:/custom/path"

# Disable auto-loading
export AXON_MIDDLEWARE_AUTOLOAD=""

# Set log level
export AXON_LOGGING_CONSOLE_LEVEL=debug
```

#### Example: Environment Variable Parser

```cpp
#include <cstdlib>
#include <string>
#include <algorithm>

std::string env_get(const std::string& key, const std::string& default_val = "") {
    const char* val = std::getenv(key.c_str());
    return val ? val : default_val;
}

bool env_get_bool(const std::string& key, bool default_val = false) {
    std::string val = env_get(key);
    if (val.empty()) return default_val;

    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    return val == "true" || val == "1" || val == "yes";
}

// Usage
bool ros2_enabled = env_get_bool("AXON_MIDDLEWARE_ROS2_ENABLED", true);
std::string ros_domain_id = env_get("AXON_MIDDLEWARE_ROS2_CONFIG__ROS_DOMAIN_ID", "0");
```

## Migration Strategy

### Phase 1: Preparation
1. Define `middleware_abi.hpp` interface
2. Create plugin loader infrastructure
3. Set up build system for plugin architecture

### Phase 2: Extract ROS Code
1. Move ROS-specific code from `core/` to `middlewares/ros1/` and `middlewares/ros2/`
2. Remove ROS dependencies from core library CMakeLists.txt files
3. Create plugin implementations for ROS1 and ROS2

### Phase 3: Refactor Core
1. Update main application to use plugin loader
2. Remove direct ROS dependencies from main executable
3. Test with both ROS1 and ROS2 plugins

### Phase 4: Cleanup
1. Remove legacy conditional compilation (`#if defined(AXON_ROS1)`)
2. Update documentation
3. Update CI/CD pipelines

## Testing

### Core Library Testing (No ROS Required)

```bash
# Can be run on any system, no ROS installation needed
cd core/axon_mcap
mkdir build && cd build
cmake .. -DAXON_MCAP_BUILD_TESTS=ON
make
ctest
```

### Plugin Testing

```bash
# Requires ROS environment
cd middlewares/ros2
colcon build
colcon test
```

### Integration Testing

```bash
# Test plugin loading and messaging
./bin/axon_recorder --plugin lib/libaxon_ros2.so --test-mode
```

## Benefits

### For Developers

1. **Faster Build Cycles**: Core libraries build without ROS dependencies
2. **Simplified Testing**: Test core functionality without ROS installed
3. **Clear Boundaries**: Plugin interface defines clear contract between layers
4. **Parallel Development**: Teams can work on different plugins independently

### For Deployers

1. **Flexible Deployment**: Install only required middleware plugins
2. **Smaller Footprint**: Core deployment has fewer dependencies
3. **Easier Updates**: Update plugins without recompiling core
4. **Vendor Independence**: Core not tied to specific ROS version

### For Users

1. **Middleware Choice**: Use ROS1, ROS2, or future middlewares interchangeably
2. **Stability**: Core API stable across middleware versions
3. **Extensibility**: Third-party middleware plugins possible

## Future Extensions

### Potential New Plugins

1. **ROS3**: Future ROS versions
2. **CycloneDDS**: Direct DDS integration
3. **LCM**: Lightweight Communications and Marshalling
4. **ZeroMQ**: Direct ZeroMQ integration
5. **Custom Protocols**: Company-specific messaging protocols

### Enhanced Features

1. **Hot-Reloading**: Load/unload plugins at runtime without restart
2. **Multiple Plugins**: Load multiple plugins simultaneously
3. **Plugin Bridging**: Route messages between plugins (e.g., ROS1 ↔ ROS2)
4. **Plugin Sandboxing**: Run plugins in separate processes for isolation

## Conclusion

The plugin-based middleware architecture provides a clean separation between core functionality and middleware-specific code. This design enables:

- Independent development and testing of core libraries
- Flexible deployment with only required middleware support
- Easy addition of new middleware integrations
- Long-term maintainability and extensibility

The architecture is inspired by similar patterns in:
- **PostgreSQL**: Extension system with dynamic loading
- **LLVM**: Plugin-based optimization passes
- **VS Code**: Extension marketplace architecture
- **Firefox/Chrome**: WebExtension API
