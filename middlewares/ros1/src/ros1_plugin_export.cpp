// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// ROS1 Plugin C ABI Export
// Direct C interface without shared ABI header dependency

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>

// JSON library for parsing options_json passed by the host
#include <nlohmann/json.hpp>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros1_plugin_export"
#include <axon_log_macros.hpp>

using axon::logging::kv;

// Plugin implementation headers
#include "ros1_plugin.hpp"
#include "ros1_subscription_wrapper.hpp"

using namespace ros1_plugin;

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
extern "C" {

using AxonMessageCallback = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
);

// ABI v1.2 zero-copy callback. Keep this identical to the host's
// AxonMessageCallbackV2 in apps/axon_recorder/src/plugin/plugin_loader.hpp
// (message_data ownership transfers; plugin fills release_fn/release_opaque
// so the recorder can call release_fn(release_opaque) exactly once when
// done with the buffer).
using AxonMessageCallbackV2 = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp,
  void (*release_fn)(void*), void* release_opaque, void* user_data
);

// =============================================================================
// Global plugin state
// =============================================================================
static std::unique_ptr<Ros1Plugin> g_plugin = nullptr;
static std::mutex g_plugin_mutex;

// =============================================================================
// C API Implementation
// =============================================================================

// Initialize the ROS1 plugin
static int32_t axon_init(const char* config_json) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (g_plugin) {
    AXON_LOG_ERROR("ROS1 plugin already initialized");
    return static_cast<int32_t>(AXON_ERROR_ALREADY_INITIALIZED);
  }

  try {
    g_plugin = std::make_unique<Ros1Plugin>();

    if (!g_plugin->init(config_json)) {
      g_plugin.reset();
      return static_cast<int32_t>(AXON_ERROR_INTERNAL);
    }

    AXON_LOG_INFO("ROS1 plugin initialized via C API");
    return static_cast<int32_t>(AXON_SUCCESS);

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to initialize ROS1 plugin: " << kv("error", e.what()));
    g_plugin.reset();
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }
}

// Start the ROS1 async spinner
static int32_t axon_start(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("ROS1 plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  if (!g_plugin->start()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  AXON_LOG_INFO("ROS1 plugin spinning via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Stop the ROS1 plugin
static int32_t axon_stop(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    return static_cast<int32_t>(AXON_SUCCESS);  // Already stopped
  }

  if (!g_plugin->stop()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  g_plugin.reset();

  AXON_LOG_INFO("ROS1 plugin stopped via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Stop per-recording ROS1 producers without tearing down process-level ROS state.
static int32_t axon_session_stop(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    return static_cast<int32_t>(AXON_SUCCESS);
  }

  if (!g_plugin->stop_session()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  AXON_LOG_INFO("ROS1 plugin session stopped via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Shared options_json parser for both axon_subscribe and axon_subscribe_v2.
// Kept file-static so the v1.x and v1.2 entry points stay semantically
// identical — only the callback-dispatch side differs.
static Ros1Plugin::SubscribeOptions parse_subscribe_options(
  const char* topic_name, const char* options_json
) {
  Ros1Plugin::SubscribeOptions options;

  if (!options_json || std::strlen(options_json) == 0) {
    return options;
  }

  try {
    nlohmann::json opts = nlohmann::json::parse(options_json);

    if (opts.contains("queue_size")) {
      options.queue_size = opts.value("queue_size", static_cast<uint32_t>(10));
    }

    if (opts.contains("depth_compression")) {
      auto dc = opts["depth_compression"];
      DepthCompressionConfig dc_config;
      dc_config.enabled = dc.value("enabled", false);
      dc_config.level = dc.value("level", "medium");

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
      options.depth_compression = dc_config;
      if (dc_config.enabled) {
        AXON_LOG_INFO(
          "Depth compression config for " << kv("topic", topic_name) << ": enabled="
                                          << kv("enabled", "true")
                                          << ", level=" << kv("level", dc_config.level)
        );
      }
#else
      // Without depth compression support, still populate the field so
      // Ros1Plugin sees the request, but warn that it is inert.
      options.depth_compression = dc_config;
      if (dc_config.enabled) {
        AXON_LOG_WARN(
          "Depth compression requested for "
          << kv("topic", topic_name)
          << " but not enabled at build time. "
             "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable."
        );
      }
#endif
    }
  } catch (const std::exception& e) {
    AXON_LOG_WARN(
      "Failed to parse options JSON for " << kv("topic", topic_name) << ": "
                                          << kv("error", e.what())
    );
  }

  return options;
}

// Subscribe to a topic with callback and optional options (JSON string,
// may be nullptr). Keep signature in sync with the host's AxonSubscribeFn
// in apps/axon_recorder/src/plugin/plugin_loader.hpp — any drift will
// silently mis-marshal the callback pointer as options_json and crash on
// first message. See also middlewares/ros2/src/ros2_plugin_export.cpp
// for the equivalent ROS2 implementation.
static int32_t axon_subscribe(
  const char* topic_name, const char* message_type, const char* options_json,
  AxonMessageCallback callback, void* user_data
) {
  if (!topic_name || !message_type || !callback) {
    AXON_LOG_ERROR(
      "Invalid argument: topic=" << kv("ptr", (void*)topic_name)
                                 << ", type=" << kv("ptr", (void*)message_type)
                                 << ", callback=" << kv("ptr", (void*)callback)
    );
    return static_cast<int32_t>(AXON_ERROR_INVALID_ARGUMENT);
  }

  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("Cannot subscribe: plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  Ros1Plugin::SubscribeOptions options = parse_subscribe_options(topic_name, options_json);

  // Create lambda wrapper for the callback
  MessageCallback wrapper = [callback, user_data](
                              const std::string& topic,
                              const std::string& type,
                              const std::vector<uint8_t>& data,
                              uint64_t timestamp
                            ) {
    callback(topic.c_str(), data.data(), data.size(), type.c_str(), timestamp, user_data);
  };

  if (!g_plugin->subscribe(
        std::string(topic_name), std::string(message_type), options, wrapper
      )) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  return static_cast<int32_t>(AXON_SUCCESS);
}

// ABI v1.2 zero-copy subscribe. Exposed through vtable.reserved[0] per the
// host-side contract in plugin_loader.hpp:112 (plugin_subscribe_v2). The
// host probes `abi_version_minor >= 2 && reserved[0] != nullptr` and calls
// this directly instead of axon_subscribe above; on any error we fall back
// to returning a non-zero status so the host can log the failure.
static int32_t axon_subscribe_v2(
  const char* topic_name, const char* message_type, const char* options_json,
  AxonMessageCallbackV2 callback, void* user_data
) {
  if (!topic_name || !message_type || !callback) {
    AXON_LOG_ERROR(
      "Invalid argument (v2): topic=" << kv("ptr", (void*)topic_name)
                                      << ", type=" << kv("ptr", (void*)message_type)
                                      << ", callback=" << kv("ptr", (void*)callback)
    );
    return static_cast<int32_t>(AXON_ERROR_INVALID_ARGUMENT);
  }

  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("Cannot subscribe (v2): plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  Ros1Plugin::SubscribeOptions options = parse_subscribe_options(topic_name, options_json);

  // Wrapper adapts the plugin-internal MessageCallbackV2 (std::function
  // with C++ types) into the C callback the host expects, forwarding
  // ownership-transfer arguments unchanged.
  MessageCallbackV2 wrapper = [callback, user_data](
                                const std::string& topic,
                                const std::string& type,
                                const uint8_t* data, size_t size,
                                uint64_t timestamp,
                                void (*release_fn)(void*),
                                void* release_opaque
                              ) {
    callback(
      topic.c_str(), data, size, type.c_str(), timestamp,
      release_fn, release_opaque, user_data
    );
  };

  if (!g_plugin->subscribe_v2(
        std::string(topic_name), std::string(message_type), options, wrapper
      )) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  return static_cast<int32_t>(AXON_SUCCESS);
}

// Publish a message (placeholder for future implementation)
static int32_t axon_publish(
  const char* topic_name, const uint8_t* message_data, size_t message_size, const char* message_type
) {
  (void)topic_name;
  (void)message_data;
  (void)message_size;
  (void)message_type;

  AXON_LOG_WARN("Publish not yet implemented for ROS1 plugin");
  return static_cast<int32_t>(AXON_ERROR_INTERNAL);
}

// =============================================================================
// Plugin descriptor export
// =============================================================================

// Version information.
//
// Bumped to 1.2 because this plugin now exposes AxonSubscribeV2Fn via
// vtable.reserved[0]. The host's plugin_subscribe_v2() helper only
// probes reserved[0] when abi_version_minor >= 2; older hosts simply
// continue to use axon_subscribe (backward compatible).
//
// Bumped to 1.3 because reserved[1] now exposes AxonSessionStopFn. Hosts use
// this between recording sessions to keep ROS1 process state alive until final
// vtable.stop during recorder teardown.
#define AXON_ABI_VERSION_MAJOR 1
#define AXON_ABI_VERSION_MINOR 3

// Plugin vtable structure (matching loader's expectation).
// Keep this layout bit-identical with AxonPluginVtable in
// apps/axon_recorder/src/plugin/plugin_loader.hpp:78. The `subscribe`
// slot takes an extra `const char* options_json` (may be nullptr) between
// message_type and the callback pointer — see the ROS2 plugin and the
// host call in recorder.cpp setup_subscriptions().
struct AxonPluginVtable {
  int32_t (*init)(const char*);
  int32_t (*start)(void);
  int32_t (*stop)(void);
  int32_t (*subscribe)(
    const char*, const char*, const char*, AxonMessageCallback, void*
  );
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

// Static vtable.
//
// reserved[0] carries AxonSubscribeV2Fn (ABI v1.2 zero-copy). reserved[1]
// carries AxonSessionStopFn (ABI v1.3 per-session stop). The remaining
// reserved slots are intentionally left null for future ABI growth.
static AxonPluginVtable ros1_vtable = {
  axon_init,
  axon_start,
  axon_stop,
  axon_subscribe,
  axon_publish,
  {
    reinterpret_cast<void*>(&axon_subscribe_v2),  // reserved[0]
    reinterpret_cast<void*>(&axon_session_stop),  // reserved[1]
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
  }
};

// Exported plugin descriptor
__attribute__((visibility("default"))) const AxonPluginDescriptor* axon_get_plugin_descriptor(
  void
) {
  static const AxonPluginDescriptor descriptor = {
    AXON_ABI_VERSION_MAJOR,
    AXON_ABI_VERSION_MINOR,
    "ROS1",
    "Noetic",
    "1.0.0",
    &ros1_vtable,
    {nullptr}
  };
  return &descriptor;
}

}  // extern "C"
