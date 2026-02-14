/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

// Mock Plugin C ABI Export
// Implements the Axon plugin C ABI interface for testing

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

// Define component name for logging
#define AXON_LOG_COMPONENT "mock_plugin_export"
#include <axon_log_macros.hpp>

using axon::logging::kv;

#include "mock_plugin.hpp"

using namespace mock_plugin;

// =============================================================================
// Error codes (matching the Axon plugin ABI)
// =============================================================================
enum AxonStatus : int32_t {
  AXON_SUCCESS = 0,
  AXON_ERROR_INVALID_ARGUMENT = -1,
  AXON_ERROR_NOT_INITIALIZED = -2,
  AXON_ERROR_ALREADY_INITIALIZED = -3,
  AXON_ERROR_NOT_STARTED = -4,
  AXON_ERROR_ALREADY_STARTED = -5,
  AXON_ERROR_INTERNAL = -100,
};

// =============================================================================
// Message callback type
// =============================================================================
extern "C" {

using AxonMessageCallback = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
);

// =============================================================================
// Global plugin state
// =============================================================================
static std::unique_ptr<MockPlugin> g_plugin = nullptr;
static std::mutex g_plugin_mutex;

// =============================================================================
// C API Implementation
// =============================================================================

// Initialize the mock plugin
static int32_t axon_init(const char* config_json) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (g_plugin) {
    AXON_LOG_ERROR("Mock plugin already initialized");
    return static_cast<int32_t>(AXON_ERROR_ALREADY_INITIALIZED);
  }

  try {
    g_plugin = std::make_unique<MockPlugin>();

    if (!g_plugin->init(config_json)) {
      g_plugin.reset();
      return static_cast<int32_t>(AXON_ERROR_INTERNAL);
    }

    AXON_LOG_INFO("Mock plugin initialized via C API");
    return static_cast<int32_t>(AXON_SUCCESS);

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to initialize mock plugin: " << kv("error", e.what()));
    g_plugin.reset();
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }
}

// Start the mock plugin
static int32_t axon_start(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("Mock plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  if (!g_plugin->start()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  AXON_LOG_INFO("Mock plugin started via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Stop the mock plugin
static int32_t axon_stop(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    return static_cast<int32_t>(AXON_SUCCESS);  // Already stopped
  }

  if (!g_plugin->stop()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  g_plugin.reset();

  AXON_LOG_INFO("Mock plugin stopped via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Subscribe to a topic with callback
static int32_t axon_subscribe(
  const char* topic_name, const char* message_type, const char* options_json,
  AxonMessageCallback callback, void* user_data
) {
  (void)options_json;  // Ignore options for mock plugin

  if (!topic_name || !message_type || !callback) {
    return static_cast<int32_t>(AXON_ERROR_INVALID_ARGUMENT);
  }

  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("Cannot subscribe: mock plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  // Create lambda wrapper for the callback
  MessageCallback wrapper = [callback, user_data](
                              const std::string& topic,
                              const std::vector<uint8_t>& data,
                              const std::string& type,
                              uint64_t timestamp
                            ) {
    callback(topic.c_str(), data.data(), data.size(), type.c_str(), timestamp, user_data);
  };

  if (!g_plugin->subscribe(
        std::string(topic_name), std::string(message_type), wrapper, user_data
      )) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  return static_cast<int32_t>(AXON_SUCCESS);
}

// Publish a message
static int32_t axon_publish(
  const char* topic_name, const uint8_t* message_data, size_t message_size, const char* message_type
) {
  if (!topic_name || !message_data || !message_type) {
    return static_cast<int32_t>(AXON_ERROR_INVALID_ARGUMENT);
  }

  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("Cannot publish: mock plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  std::vector<uint8_t> data(message_data, message_data + message_size);

  if (!g_plugin->publish(std::string(topic_name), data, std::string(message_type))) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  return static_cast<int32_t>(AXON_SUCCESS);
}

// =============================================================================
// Plugin descriptor export
// =============================================================================

// Version information
#define AXON_ABI_VERSION_MAJOR 1
#define AXON_ABI_VERSION_MINOR 0

// Plugin vtable structure (matching loader's expectation)
struct AxonPluginVtable {
  int32_t (*init)(const char*);
  int32_t (*start)(void);
  int32_t (*stop)(void);
  int32_t (*subscribe)(const char*, const char*, const char*, AxonMessageCallback, void*);
  int32_t (*publish)(const char*, const uint8_t*, size_t, const char*);
  void* reserved[9];
};

// Plugin descriptor structure (matching loader's expectation)
struct AxonPluginDescriptor {
  uint32_t abi_version_major;
  uint32_t abi_version_minor;
  const char* middleware_name;
  const char* middleware_version;
  const char* plugin_version;
  AxonPluginVtable* vtable;
  void* reserved[16];
};

// Static vtable
static AxonPluginVtable mock_vtable = {
  axon_init, axon_start, axon_stop, axon_subscribe, axon_publish, {nullptr}
};

// Exported plugin descriptor
__attribute__((visibility("default"))) const AxonPluginDescriptor* axon_get_plugin_descriptor(
  void
) {
  static const AxonPluginDescriptor descriptor = {
    AXON_ABI_VERSION_MAJOR,
    AXON_ABI_VERSION_MINOR,
    "Mock",
    "1.0.0",
    "1.0.0",
    &mock_vtable,
    {nullptr}
  };
  return &descriptor;
}

}  // extern "C"
