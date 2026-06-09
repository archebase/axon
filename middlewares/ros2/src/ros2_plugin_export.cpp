// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// ROS2 Plugin C ABI Export
// Direct C interface without shared ABI header dependency

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// JSON library
#include <nlohmann/json.hpp>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros2_plugin_export"
#include <axon_log_macros.hpp>

using axon::logging::kv;

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
#include "depth_compressor.hpp"
#endif
#include "ros2_plugin.hpp"
#include "ros2_subscription_wrapper.hpp"

using namespace ros2_plugin;

namespace ros2_plugin {

std::string trim_ascii(std::string value) {
  auto not_space = [](unsigned char c) {
    return !std::isspace(c);
  };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string normalize_qos_policy_token(std::string value) {
  value = trim_ascii(std::move(value));
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    if (c == '-' || std::isspace(c)) {
      return '_';
    }
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool json_has_value(const nlohmann::json& object, const char* key) {
  if (!object.is_object() || !object.contains(key) || object[key].is_null()) {
    return false;
  }
  if (object[key].is_string()) {
    return !trim_ascii(object[key].get<std::string>()).empty();
  }
  return true;
}

bool json_field_is_auto(const nlohmann::json& object, const char* key) {
  return json_has_value(object, key) && object[key].is_string() &&
         normalize_qos_policy_token(object[key].get<std::string>()) == "auto";
}

std::optional<size_t> read_qos_depth(const nlohmann::json& object, const char* key) {
  if (!json_has_value(object, key)) {
    return std::nullopt;
  }
  if (json_field_is_auto(object, key)) {
    return std::nullopt;
  }
  long long depth = 0;
  if (object[key].is_string()) {
    depth = std::stoll(trim_ascii(object[key].get<std::string>()));
  } else {
    depth = object[key].get<long long>();
  }
  return depth > 0 ? std::optional<size_t>(static_cast<size_t>(depth)) : std::optional<size_t>(0);
}

std::optional<std::string> read_qos_policy(const nlohmann::json& object, const char* key) {
  if (!json_has_value(object, key)) {
    return std::nullopt;
  }
  return normalize_qos_policy_token(object[key].get<std::string>());
}

std::optional<bool> read_qos_reliable(const nlohmann::json& object, const char* key) {
  if (!json_has_value(object, key)) {
    return std::nullopt;
  }
  if (object[key].is_boolean()) {
    return object[key].get<bool>();
  }
  const std::string reliability = normalize_qos_policy_token(object[key].get<std::string>());
  if (reliability == "reliable") {
    return true;
  }
  if (reliability == "best_effort") {
    return false;
  }
  return std::nullopt;
}

void apply_subscribe_qos_options(SubscribeOptions& options, const nlohmann::json& opts) {
  size_t qos_depth = 10;
  const nlohmann::json* qos_opts = &opts;
  if (opts.contains("qos") && opts["qos"].is_object()) {
    qos_opts = &opts["qos"];
  }

  const bool auto_mode = read_qos_policy(*qos_opts, "mode").value_or(std::string{}) == "auto";
  if (auto_mode) {
    options.qos_depth_auto = true;
    options.qos_reliability_auto = true;
    options.qos_durability_auto = true;
    options.qos_history_auto = true;
  }

  if (json_field_is_auto(*qos_opts, "depth") || json_field_is_auto(*qos_opts, "qos_depth")) {
    options.qos_depth_auto = true;
  }
  if (auto depth = read_qos_depth(*qos_opts, "depth")) {
    options.qos_depth_auto = false;
    qos_depth = *depth;
  } else if (auto depth = read_qos_depth(*qos_opts, "qos_depth")) {
    options.qos_depth_auto = false;
    qos_depth = *depth;
  } else if (auto depth = read_qos_depth(opts, "qos_depth")) {
    qos_depth = *depth;
  } else if (auto depth = read_qos_depth(opts, "queue_size")) {
    qos_depth = *depth;
  }
  if (qos_depth == 0) {
    qos_depth = 10;
  }

  const std::string history = read_qos_policy(*qos_opts, "history").value_or("keep_last");
  if (history == "auto") {
    options.qos_history_auto = true;
  } else if (json_has_value(*qos_opts, "history")) {
    options.qos_history_auto = false;
  }
  if (history == "keep_all") {
    options.qos = rclcpp::QoS(rclcpp::KeepAll());
  } else {
    options.qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
  }

  bool reliable = true;
  if (json_field_is_auto(*qos_opts, "reliability") || json_field_is_auto(*qos_opts, "reliable")) {
    options.qos_reliability_auto = true;
  } else if (auto qos_reliable = read_qos_reliable(*qos_opts, "reliability")) {
    options.qos_reliability_auto = false;
    reliable = *qos_reliable;
  } else if (auto qos_reliable = read_qos_reliable(*qos_opts, "reliable")) {
    options.qos_reliability_auto = false;
    reliable = *qos_reliable;
  } else if (auto qos_reliable = read_qos_reliable(opts, "qos_reliable")) {
    reliable = *qos_reliable;
  } else if (auto legacy_reliable = read_qos_reliable(opts, "reliable")) {
    reliable = *legacy_reliable;
  }
  if (reliable) {
    options.qos.reliable();
  } else {
    options.qos.best_effort();
  }

  const std::string durability = read_qos_policy(*qos_opts, "durability").value_or("volatile");
  if (durability == "auto") {
    options.qos_durability_auto = true;
  } else if (json_has_value(*qos_opts, "durability")) {
    options.qos_durability_auto = false;
  }
  if (durability == "transient_local") {
    options.qos.transient_local();
  } else {
    options.qos.durability_volatile();
  }
}

}  // namespace ros2_plugin

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

// ABI v1.2 zero-copy callback
using AxonMessageCallbackV2 = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void (*release_fn)(void*), void* release_opaque,
  void* user_data
);

// =============================================================================
// Global plugin state
// =============================================================================
static std::unique_ptr<Ros2Plugin> g_plugin = nullptr;
static std::mutex g_plugin_mutex;

// =============================================================================
// C API Implementation
// =============================================================================

// Initialize the ROS2 plugin
static int32_t axon_init(const char* config_json) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (g_plugin) {
    AXON_LOG_ERROR("ROS2 plugin already initialized");
    return static_cast<int32_t>(AXON_ERROR_ALREADY_INITIALIZED);
  }

  try {
    g_plugin = std::make_unique<Ros2Plugin>();

    if (!g_plugin->init(config_json)) {
      g_plugin.reset();
      return static_cast<int32_t>(AXON_ERROR_INTERNAL);
    }

    AXON_LOG_INFO("ROS2 plugin initialized via C API");
    return static_cast<int32_t>(AXON_SUCCESS);

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to initialize ROS2 plugin: " << kv("error", e.what()));
    g_plugin.reset();
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }
}

// Start the ROS2 executor
static int32_t axon_start(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    AXON_LOG_ERROR("ROS2 plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  if (!g_plugin->start()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  AXON_LOG_INFO("ROS2 plugin spinning via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Stop the ROS2 plugin
static int32_t axon_stop(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    return static_cast<int32_t>(AXON_SUCCESS);  // Already stopped
  }

  if (!g_plugin->stop()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  g_plugin.reset();

  AXON_LOG_INFO("ROS2 plugin stopped via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// ABI v1.3: stop per-recording producers without destroying initialized plugin state.
static int32_t axon_session_stop(void) {
  std::lock_guard<std::mutex> lock(g_plugin_mutex);

  if (!g_plugin) {
    return static_cast<int32_t>(AXON_SUCCESS);
  }

  if (!g_plugin->stop_session()) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  AXON_LOG_INFO("ROS2 plugin session stopped via C API");
  return static_cast<int32_t>(AXON_SUCCESS);
}

// Subscribe to a topic with callback and optional options (JSON string, can be nullptr)
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

  // Parse options JSON if provided
  SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  options.qos.reliable();
  options.qos.durability_volatile();

  if (options_json && strlen(options_json) > 0) {
    try {
      nlohmann::json opts = nlohmann::json::parse(options_json);
      apply_subscribe_qos_options(options, opts);
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
      if (opts.contains("depth_compression")) {
        auto dc = opts["depth_compression"];
        DepthCompressionConfig dc_config;
        dc_config.enabled = dc.value("enabled", false);
        dc_config.level = dc.value("level", "medium");
        options.depth_compression = dc_config;
        AXON_LOG_INFO(
          "Depth compression config for " << kv("topic", topic_name) << ": enabled="
                                          << kv("enabled", dc_config.enabled ? "true" : "false")
                                          << ", level=" << kv("level", dc_config.level)
        );
      }
#else
      if (opts.contains("depth_compression") && opts["depth_compression"].value("enabled", false)) {
        AXON_LOG_WARN(
          "Depth compression requested for "
          << kv("topic", topic_name)
          << " but not enabled at build time. "
             "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable."
        );
      }
#endif
    } catch (const std::exception& e) {
      AXON_LOG_WARN(
        "Failed to parse options JSON for " << kv("topic", topic_name) << ": "
                                            << kv("error", e.what())
      );
    }
  }

  // Create lambda wrapper for the callback
  MessageCallback wrapper = [callback, user_data](
                              const std::string& topic,
                              const std::string& type,
                              const std::vector<uint8_t>& data,
                              rclcpp::Time timestamp
                            ) {
    callback(
      topic.c_str(), data.data(), data.size(), type.c_str(), timestamp.nanoseconds(), user_data
    );
  };

  if (!g_plugin->subscribe(std::string(topic_name), std::string(message_type), options, wrapper)) {
    return static_cast<int32_t>(AXON_ERROR_INTERNAL);
  }

  return static_cast<int32_t>(AXON_SUCCESS);
}

// ABI v1.2: zero-copy subscribe. Shares config parsing with axon_subscribe.
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
    AXON_LOG_ERROR("Cannot subscribe_v2: plugin not initialized");
    return static_cast<int32_t>(AXON_ERROR_NOT_INITIALIZED);
  }

  SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  options.qos.reliable();
  options.qos.durability_volatile();

  if (options_json && strlen(options_json) > 0) {
    try {
      nlohmann::json opts = nlohmann::json::parse(options_json);
      apply_subscribe_qos_options(options, opts);
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
      if (opts.contains("depth_compression")) {
        auto dc = opts["depth_compression"];
        DepthCompressionConfig dc_config;
        dc_config.enabled = dc.value("enabled", false);
        dc_config.level = dc.value("level", "medium");
        options.depth_compression = dc_config;
      }
#else
      if (opts.contains("depth_compression") && opts["depth_compression"].value("enabled", false)) {
        AXON_LOG_WARN(
          "Depth compression requested (v2) for "
          << kv("topic", topic_name)
          << " but not enabled at build time. "
             "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable."
        );
      }
#endif
    } catch (const std::exception& e) {
      AXON_LOG_WARN(
        "Failed to parse options JSON (v2) for " << kv("topic", topic_name) << ": "
                                                 << kv("error", e.what())
      );
    }
  }

  MessageCallbackV2 wrapper = [callback, user_data](
                                const std::string& topic,
                                const std::string& type,
                                const uint8_t* data,
                                std::size_t size,
                                rclcpp::Time timestamp,
                                void (*release_fn)(void*),
                                void* release_opaque
                              ) {
    callback(
      topic.c_str(),
      data,
      size,
      type.c_str(),
      timestamp.nanoseconds(),
      release_fn,
      release_opaque,
      user_data
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

  AXON_LOG_WARN("Publish not yet implemented for ROS2 plugin");
  return static_cast<int32_t>(AXON_ERROR_INTERNAL);
}

// =============================================================================
// Plugin descriptor export
// =============================================================================

// Version information.
// v1.1: subscribe(options_json, ...) signature change.
// v1.2: zero-copy AxonSubscribeV2Fn exposed via vtable.reserved[0].
// v1.3: AxonSessionStopFn exposed via vtable.reserved[1].
#define AXON_ABI_VERSION_MAJOR 1
#define AXON_ABI_VERSION_MINOR 3

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

// Static vtable.
// reserved[0] carries AxonSubscribeV2Fn (ABI v1.2 zero-copy) and reserved[1]
// carries AxonSessionStopFn (ABI v1.3 per-session stop). The recorder probes
// these slots by ABI minor before using them.
static AxonPluginVtable ros2_vtable = {
  axon_init,
  axon_start,
  axon_stop,
  axon_subscribe,
  axon_publish,
  {reinterpret_cast<void*>(&axon_subscribe_v2),
   reinterpret_cast<void*>(&axon_session_stop),
   nullptr,
   nullptr,
   nullptr,
   nullptr,
   nullptr,
   nullptr,
   nullptr}
};

// Exported plugin descriptor
__attribute__((visibility("default"))) const AxonPluginDescriptor* axon_get_plugin_descriptor(
  void
) {
  static const AxonPluginDescriptor descriptor = {
    AXON_ABI_VERSION_MAJOR,
    AXON_ABI_VERSION_MINOR,
    "ROS2",
    "Humble/Jazzy/Rolling",
    "1.0.0",
    &ros2_vtable,
    {nullptr}
  };
  return &descriptor;
}

}  // extern "C"
