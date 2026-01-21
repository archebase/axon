// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#include "plugin_loader.hpp"

#include <cstring>
#include <dlfcn.h>
#include <iostream>

namespace axon {

PluginLoader::Plugin::Plugin()
    : handle(nullptr)
    , descriptor(nullptr)
    , initialized(false)
    , running(false) {}

PluginLoader::Plugin::Plugin(void* h, const AxonPluginDescriptor* desc, const std::string& p)
    : handle(h)
    , descriptor(desc)
    , path(p)
    , initialized(false)
    , running(false) {}

PluginLoader::PluginLoader() = default;

PluginLoader::~PluginLoader() {
  unload_all();
}

std::optional<std::string> PluginLoader::load(const std::string& plugin_path) {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  // Clear previous error
  last_error_.clear();

  // Load shared library
  void* handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
  if (!handle) {
    set_error(std::string("Failed to load plugin: ") + dlerror());
    return std::nullopt;
  }

  // Reset dlerror
  dlerror();

  // Resolve the descriptor function
  using GetDescriptorFn = const AxonPluginDescriptor* (*)();
  auto get_descriptor =
    reinterpret_cast<GetDescriptorFn>(dlsym(handle, "axon_get_plugin_descriptor"));

  char* error = dlerror();
  if (error != nullptr) {
    set_error(std::string("Plugin missing axon_get_plugin_descriptor symbol: ") + error);
    dlclose(handle);
    return std::nullopt;
  }

  // Get descriptor
  const AxonPluginDescriptor* descriptor = get_descriptor();
  if (!descriptor) {
    set_error("Plugin returned null descriptor");
    dlclose(handle);
    return std::nullopt;
  }

  // Validate ABI version (using v1.0 as expected)
  constexpr uint32_t EXPECTED_ABI_VERSION_MAJOR = 1;
  if (descriptor->abi_version_major != EXPECTED_ABI_VERSION_MAJOR) {
    set_error(
      "ABI version mismatch: expected " + std::to_string(EXPECTED_ABI_VERSION_MAJOR) + ", got " +
      std::to_string(descriptor->abi_version_major)
    );
    dlclose(handle);
    return std::nullopt;
  }

  // Validate vtable
  if (!descriptor->vtable) {
    set_error("Plugin has null vtable");
    dlclose(handle);
    return std::nullopt;
  }

  // Check required functions
  if (!descriptor->vtable->init) {
    set_error("Plugin missing init function");
    dlclose(handle);
    return std::nullopt;
  }

  if (!descriptor->vtable->start) {
    set_error("Plugin missing start function");
    dlclose(handle);
    return std::nullopt;
  }

  if (!descriptor->vtable->stop) {
    set_error("Plugin missing stop function");
    dlclose(handle);
    return std::nullopt;
  }

  // Get plugin name
  std::string plugin_name = descriptor->middleware_name ? descriptor->middleware_name : "unknown";

  // Check if already loaded
  if (plugins_.find(plugin_name) != plugins_.end()) {
    set_error("Plugin already loaded: " + plugin_name);
    dlclose(handle);
    return std::nullopt;
  }

  // Create plugin object
  auto plugin = std::make_unique<Plugin>(handle, descriptor, plugin_path);
  plugins_[plugin_name] = std::move(plugin);

  return plugin_name;
}

bool PluginLoader::unload(const std::string& plugin_name) {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  auto it = plugins_.find(plugin_name);
  if (it == plugins_.end()) {
    set_error("Plugin not loaded: " + plugin_name);
    return false;
  }

  auto& plugin = it->second;

  // Shutdown if running
  if (plugin->running && plugin->descriptor->vtable->stop) {
    plugin->descriptor->vtable->stop();
    plugin->running = false;
  }

  // Close the library
  if (plugin->handle) {
    dlclose(plugin->handle);
  }

  plugins_.erase(it);
  return true;
}

void PluginLoader::unload_all() {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  for (auto& [name, plugin] : plugins_) {
    if (plugin->running && plugin->descriptor->vtable->stop) {
      plugin->descriptor->vtable->stop();
    }
    if (plugin->handle) {
      dlclose(plugin->handle);
    }
  }
  plugins_.clear();
}

const AxonPluginDescriptor* PluginLoader::get_descriptor(const std::string& plugin_name) const {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  auto it = plugins_.find(plugin_name);
  if (it == plugins_.end()) {
    return nullptr;
  }
  return it->second->descriptor;
}

PluginLoader::Plugin* PluginLoader::get_plugin(const std::string& plugin_name) {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  auto it = plugins_.find(plugin_name);
  if (it == plugins_.end()) {
    return nullptr;
  }
  return it->second.get();
}

bool PluginLoader::is_loaded(const std::string& plugin_name) const {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  return plugins_.find(plugin_name) != plugins_.end();
}

std::vector<std::string> PluginLoader::loaded_plugins() const {
  std::lock_guard<std::mutex> lock(plugins_mutex_);

  std::vector<std::string> names;
  names.reserve(plugins_.size());
  for (const auto& [name, plugin] : plugins_) {
    names.push_back(name);
  }
  return names;
}

std::string PluginLoader::get_last_error() const {
  return last_error_;
}

void PluginLoader::set_error(const std::string& error) {
  last_error_ = error;
}

}  // namespace axon
