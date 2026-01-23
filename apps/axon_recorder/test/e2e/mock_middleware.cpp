// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * @file mock_middleware.cpp
 * @brief Mock middleware plugin for E2E testing
 *
 * This is a minimal implementation of the middleware ABI for testing purposes.
 * It simulates message publishing without requiring a full ROS installation.
 *
 * Direct C interface without shared ABI header dependency (matches ROS2 plugin pattern).
 */

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

// ============================================================================
// ABI Definition (matching plugin_loader.hpp expectations)
// ============================================================================

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
using AxonSubscribeFn = AxonStatus (*)(
  const char* topic, const char* type, AxonMessageCallback callback, void* user_data
);
using AxonPublishFn =
  AxonStatus (*)(const char* topic, const uint8_t* data, size_t size, const char* type);

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

// ============================================================================
// Mock Plugin State
// ============================================================================

struct MockState {
  bool initialized;
  bool running;
  AxonMessageCallback message_callback;
  void* user_data;

  MockState()
      : initialized(false)
      , running(false)
      , message_callback(nullptr)
      , user_data(nullptr) {}
};

static MockState g_state;

// ============================================================================
// ABI Implementation
// ============================================================================

extern "C" {

AxonStatus axon_init(const char* config) {
  (void)config;  // Unused
  std::cout << "[MockMiddleware] Initialized" << std::endl;
  g_state.initialized = true;
  return AXON_SUCCESS;
}

AxonStatus axon_start() {
  if (!g_state.initialized) {
    std::cerr << "[MockMiddleware] Not initialized" << std::endl;
    return AXON_ERROR_NOT_INITIALIZED;
  }

  std::cout << "[MockMiddleware] Started" << std::endl;
  g_state.running = true;

  // Start a background thread to publish mock messages
  std::thread([&]() {
    int counter = 0;
    while (g_state.running) {
      if (g_state.message_callback) {
        // Publish mock message for /test/topic1
        std::string topic1 = "/test/topic1";
        std::string type1 = "std_msgs/String";
        std::string data1 = "test_message_" + std::to_string(counter++);

        uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch()
        )
                               .count();

        g_state.message_callback(
          topic1.c_str(),
          reinterpret_cast<const uint8_t*>(data1.c_str()),
          data1.size(),
          type1.c_str(),
          timestamp,
          g_state.user_data
        );
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }).detach();

  return AXON_SUCCESS;
}

AxonStatus axon_stop() {
  std::cout << "[MockMiddleware] Stopping..." << std::endl;
  g_state.running = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "[MockMiddleware] Stopped" << std::endl;
  return AXON_SUCCESS;
}

AxonStatus axon_subscribe(
  const char* topic, const char* type, AxonMessageCallback callback, void* user_data
) {
  std::cout << "[MockMiddleware] Subscribed to " << topic << " (" << type << ")" << std::endl;
  g_state.message_callback = callback;
  g_state.user_data = user_data;
  return AXON_SUCCESS;
}

AxonStatus axon_publish(const char* topic, const uint8_t* data, size_t size, const char* type) {
  (void)topic;
  (void)data;
  (void)size;
  (void)type;
  // Not used in recorder
  return AXON_SUCCESS;
}

// Plugin descriptor
static AxonPluginVtable g_vtable = {
  axon_init,
  axon_start,
  axon_stop,
  axon_subscribe,
  axon_publish,
};

static AxonPluginDescriptor g_descriptor = {
  1,                  // abi_version_major
  0,                  // abi_version_minor
  "mock_middleware",  // middleware_name
  "1.0.0",            // middleware_version
  "1.0.0",            // plugin_version
  &g_vtable,          // vtable
  {},                 // reserved
};

const AxonPluginDescriptor* axon_get_plugin_descriptor() {
  return &g_descriptor;
}

}  // extern "C"
