// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_PLUGIN_LOADER_HPP
#define AXON_PLUGIN_LOADER_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {

// Forward declare ABI structures (no header dependency)
extern "C" {

// Error codes
enum AxonStatus : int32_t {
  AXON_SUCCESS = 0,
  AXON_ERROR_INVALID_ARGUMENT = -1,
  AXON_ERROR_NOT_INITIALIZED = -2,
  AXON_ERROR_ALREADY_INITIALIZED = -3,
  AXON_ERROR_NOT_STARTED = -4,
  AXON_ERROR_ALREADY_STARTED = -5,
  AXON_ERROR_INTERNAL = -100,
};

// Message callback type
using AxonMessageCallback = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
);

// Plugin function types
using AxonInitFn = AxonStatus (*)(const char*);
using AxonStartFn = AxonStatus (*)();
using AxonStopFn = AxonStatus (*)();
using AxonSubscribeFn =
  AxonStatus (*)(const char*, const char*, const char*, AxonMessageCallback, void*);
using AxonPublishFn = AxonStatus (*)(const char*, const uint8_t*, size_t, const char*);

// Plugin vtable structure
struct AxonPluginVtable {
  AxonInitFn init;
  AxonStartFn start;
  AxonStopFn stop;
  AxonSubscribeFn subscribe;
  AxonPublishFn publish;
  void* reserved[9];
};

// Plugin descriptor structure
struct AxonPluginDescriptor {
  uint32_t abi_version_major;
  uint32_t abi_version_minor;
  const char* middleware_name;
  const char* middleware_version;
  const char* plugin_version;
  AxonPluginVtable* vtable;
  void* reserved[16];
};

// Each plugin must export this function
const AxonPluginDescriptor* axon_get_plugin_descriptor(void);

}  // extern "C"

class PluginLoader {
public:
  struct Plugin {
    void* handle;
    const AxonPluginDescriptor* descriptor;
    std::string path;
    bool initialized;
    bool running;

    Plugin();
    Plugin(void* h, const AxonPluginDescriptor* desc, const std::string& p);
  };

  PluginLoader();
  ~PluginLoader();

  // Prevent copying
  PluginLoader(const PluginLoader&) = delete;
  PluginLoader& operator=(const PluginLoader&) = delete;

  // Load a plugin from a shared library path
  std::optional<std::string> load(const std::string& plugin_path);

  // Unload a specific plugin by name
  bool unload(const std::string& plugin_name);

  // Unload all plugins
  void unload_all();

  // Get plugin descriptor by name
  const AxonPluginDescriptor* get_descriptor(const std::string& plugin_name) const;

  // Get plugin by name
  Plugin* get_plugin(const std::string& plugin_name);

  // Check if a plugin is loaded
  bool is_loaded(const std::string& plugin_name) const;

  // Get all loaded plugin names
  std::vector<std::string> loaded_plugins() const;

  // Get last error message
  std::string get_last_error() const;

private:
  mutable std::mutex plugins_mutex_;
  std::unordered_map<std::string, std::unique_ptr<Plugin>> plugins_;
  std::string last_error_;

  void set_error(const std::string& error);
};

}  // namespace axon

#endif  // AXON_PLUGIN_LOADER_HPP
