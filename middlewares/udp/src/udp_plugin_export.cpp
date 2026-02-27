// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// UDP Plugin C ABI Export
// Provides C interface for dynamic loading by axon_recorder

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

#include "axon_udp/udp_plugin.hpp"

#define AXON_LOG_COMPONENT "udp_plugin_export"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace {

// =============================================================================
// Error codes (matching what the loader expects)
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
using AxonMessageCallback = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
);

// =============================================================================
// Global plugin state
// =============================================================================
std::unique_ptr<axon::udp::UdpPlugin> g_plugin = nullptr;
std::mutex g_plugin_mutex;
AxonMessageCallback g_callback = nullptr;
void* g_user_data = nullptr;

// =============================================================================
// C API Implementation
// =============================================================================

int32_t axon_init(const char* config_json) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (g_plugin) {
    AXON_LOG_ERROR("UDP plugin already initialized");
    return static_cast<int32_t>(AXON_ERROR_ALREADY_INITIALIZED);
  }

  try {
    g_plugin = std::make_unique<axon::udp::UdpPlugin>();

    if (!g_plugin->init(config_json ? config_json : "{}")) {
      g_plugin.reset();
      return static_cast<int32_t>(AXON_ERROR_INTERNAL);
    }

    AXON_LOG_INFO("UDP plugin initialized via C API");
    return static_cast<int32_t>(AXON_SUCCESS);

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to initialize UDP plugin: " << kv("error", e.what()));
    g_plugin.reset();
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }
}

int32_t axon_start(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("UDP plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  // Set message callback wrapper
  if (g_callback) {
    g_plugin->set_message_callback([](
                                     const std::string& topic,
                                     const std::vector<uint8_t>& data,
                                     const std::string& message_type,
                                     uint64_t timestamp
                                   ) {
      if (g_callback) {
        g_callback(
          topic.c_str(), data.data(), data.size(), message_type.c_str(), timestamp, g_user_data
        );
      }
    });
  }

  if (!g_plugin->start()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  AXON_LOG_INFO("UDP plugin started via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

int32_t axon_stop(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    return static_cast<int32_t>(AXON_SUCCESS);  // Already stopped
  }

  if (!g_plugin->stop()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  g_plugin.reset();
  g_callback = nullptr;
  g_user_data = nullptr;

  AXON_LOG_INFO("UDP plugin stopped via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

int32_t axon_subscribe(
  const char* topic_name, const char* message_type, const char* options_json,
  AxonMessageCallback callback, void* user_data
) {
  // UDP plugin doesn't use subscribe - it receives messages via UDP sockets
  // The streams are configured during init()
  (void)topic_name;
  (void)message_type;
  (void)options_json;

  // Store callback for later use in start()
  std::lock_guard<std::mutex> lock(g_plugin_mutex);
  g_callback = callback;
  g_user_data = user_data;

  AXON_LOG_DEBUG("UDP plugin: subscribe callback registered");
  return static_cast<int32_t>(AXON_SUCCESS);
}

int32_t axon_publish(
  const char* topic_name, const uint8_t* message_data, size_t message_size, const char* message_type
) {
  // UDP plugin doesn't support publishing
  (void)topic_name;
  (void)message_data;
  (void)message_size;
  (void)message_type;

  AXON_LOG_WARN("Publish not supported for UDP plugin");
  return static_cast<int32_t>(AXON_ERROR_INTERNAL);
}

}  // anonymous namespace

// =============================================================================
// Plugin descriptor export
// =============================================================================

extern "C" {

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
static AxonPluginVtable udp_vtable = {
  axon_init,
  axon_start,
  axon_stop,
  axon_subscribe,
  axon_publish,
  {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr}
};

// Exported plugin descriptor
__attribute__((visibility("default"))) const AxonPluginDescriptor* axon_get_plugin_descriptor(
  void
) {
  static const AxonPluginDescriptor descriptor = {
    AXON_ABI_VERSION_MAJOR, AXON_ABI_VERSION_MINOR, "UDP", "1.0.0", "0.1.0", &udp_vtable, {nullptr}
  };
  return &descriptor;
}

}  // extern "C"
