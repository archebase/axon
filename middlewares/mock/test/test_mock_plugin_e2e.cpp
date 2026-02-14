/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

/**
 * @file test_mock_plugin_e2e.cpp
 * @brief End-to-end test for mock plugin without PluginLoader
 * This tests plugin functionality by loading the prebuilt library via dlopen.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <dlfcn.h>
#include <iostream>
#include <thread>

// Mock plugin ABI types (must match plugin_export.cpp)
extern "C" {
using AxonMessageCallback = void (*)(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
);

struct AxonPluginVtable {
  int32_t (*init)(const char*);
  int32_t (*start)(void);
  int32_t (*stop)(void);
  int32_t (*subscribe)(const char*, const char*, const char*, AxonMessageCallback, void*);
  int32_t (*publish)(const char*, const uint8_t*, size_t, const char*);
  void* reserved[9];
};

struct AxonPluginDescriptor {
  uint32_t abi_version_major;
  uint32_t abi_version_minor;
  const char* middleware_name;
  const char* middleware_version;
  const char* plugin_version;
  AxonPluginVtable* vtable;
  void* reserved[16];
};

// Function to get plugin descriptor
const AxonPluginDescriptor* axon_get_plugin_descriptor(void);
}

// Plugin vtable (loaded via dlsym)
static AxonPluginVtable* g_vtable = nullptr;

// Wrapper functions for vtable calls
static int32_t plugin_init(const char* config_json) {
  if (g_vtable && g_vtable->init) {
    return g_vtable->init(config_json);
  }
  return -1;
}

static int32_t plugin_start(void) {
  if (g_vtable && g_vtable->start) {
    return g_vtable->start();
  }
  return -1;
}

static int32_t plugin_stop(void) {
  if (g_vtable && g_vtable->stop) {
    return g_vtable->stop();
  }
  return -1;
}

static int32_t plugin_subscribe(
  const char* topic, const char* type, AxonMessageCallback callback, void* user_data
) {
  if (g_vtable && g_vtable->subscribe) {
    return g_vtable->subscribe(topic, type, nullptr, callback, user_data);
  }
  return -1;
}

static int32_t plugin_publish(
  const char* topic, const uint8_t* data, size_t size, const char* type
) {
  if (g_vtable && g_vtable->publish) {
    return g_vtable->publish(topic, data, size, type);
  }
  return -1;
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  std::cout << "=== Mock Plugin E2E Test ===" << std::endl;

  // Test 1: Load plugin library
  std::cout << "\n[TEST 1] Loading mock plugin library..." << std::endl;
  void* handle = dlopen("./libmock_plugin.so", RTLD_NOW);
  if (!handle) {
    std::cerr << "[FAIL] Could not load ./libmock_plugin.so: " << dlerror() << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin library loaded" << std::endl;

  // Test 2: Get plugin descriptor
  std::cout << "\n[TEST 2] Getting plugin descriptor..." << std::endl;
  auto get_descriptor = reinterpret_cast<decltype(&axon_get_plugin_descriptor)>(
    dlsym(handle, "axon_get_plugin_descriptor")
  );
  if (!get_descriptor) {
    std::cerr << "[FAIL] Could not find axon_get_plugin_descriptor: " << dlerror() << std::endl;
    dlclose(handle);
    return 1;
  }

  const AxonPluginDescriptor* descriptor = get_descriptor();
  if (!descriptor) {
    std::cerr << "[FAIL] Plugin descriptor is null" << std::endl;
    dlclose(handle);
    return 1;
  }

  std::cout << "[PASS] Plugin descriptor retrieved" << std::endl;
  std::cout << "  Middleware: " << descriptor->middleware_name << std::endl;
  std::cout << "  Version: " << descriptor->plugin_version << std::endl;
  std::cout << "  ABI: " << descriptor->abi_version_major << "." << descriptor->abi_version_minor
            << std::endl;

  // Store vtable for later use
  g_vtable = descriptor->vtable;
  if (!g_vtable) {
    std::cerr << "[FAIL] Plugin vtable is null" << std::endl;
    dlclose(handle);
    return 1;
  }

  // Test 3: Initialize plugin
  std::cout << "\n[TEST 3] Initializing plugin..." << std::endl;
  if (plugin_init("{}") != 0) {
    std::cerr << "[FAIL] Could not initialize plugin" << std::endl;
    dlclose(handle);
    return 1;
  }
  std::cout << "[PASS] Plugin initialized" << std::endl;

  // Test 4: Start plugin
  std::cout << "\n[TEST 4] Starting plugin..." << std::endl;
  if (plugin_start() != 0) {
    std::cerr << "[FAIL] Could not start plugin" << std::endl;
    dlclose(handle);
    return 1;
  }
  std::cout << "[PASS] Plugin started" << std::endl;

  // Test 5: Subscribe to topics (without actual ROS - just test the interface)
  std::cout << "\n[TEST 5] Subscribing to topics..." << std::endl;

  // Track message counts
  std::atomic<int> string_count{0};
  std::atomic<int> int_count{0};

  // Define a mock callback for string messages
  auto string_callback = [](
                           const char* topic_name,
                           const uint8_t* message_data,
                           size_t message_size,
                           const char* message_type,
                           uint64_t timestamp,
                           void* user_data
                         ) -> void {
    auto* counter = static_cast<std::atomic<int>*>(user_data);
    counter->fetch_add(1, std::memory_order_relaxed);
    std::cout << "  [String #" << topic_name << "] (" << message_type << "): " << message_size
              << " bytes @ " << timestamp << std::endl;
  };

  // Test subscribe via plugin
  const char* test_topic = "/test/string";
  if (plugin_subscribe(test_topic, "std_msgs/String", string_callback, &string_count) != 0) {
    std::cerr << "[FAIL] Could not subscribe to " << test_topic << std::endl;
    plugin_stop();
    dlclose(handle);
    return 1;
  }
  std::cout << "[PASS] Subscribed to " << test_topic << " (std_msgs/String)" << std::endl;

  // Test 6: Subscribe to int32 topic
  std::cout << "\n[TEST 6] Subscribing to int topic..." << std::endl;

  // Define a mock callback for int32 messages
  auto int_callback = [](
                        const char* topic_name,
                        const uint8_t* message_data,
                        size_t message_size,
                        const char* message_type,
                        uint64_t timestamp,
                        void* user_data
                      ) -> void {
    auto* counter = static_cast<std::atomic<int>*>(user_data);
    counter->fetch_add(1, std::memory_order_relaxed);
    std::cout << "  [Int #" << topic_name << "] " << message_type << ": " << message_size
              << " bytes @ " << timestamp << std::endl;
  };

  const char* test_topic_int = "/test/int";
  if (plugin_subscribe(test_topic_int, "std_msgs/Int32", int_callback, &int_count) != 0) {
    std::cerr << "[FAIL] Could not subscribe to " << test_topic_int << std::endl;
    plugin_stop();
    dlclose(handle);
    return 1;
  }
  std::cout << "[PASS] Subscribed to " << test_topic_int << " (std_msgs/Int32)" << std::endl;

  // Test 7: Wait for messages
  std::cout << "\n[TEST 7] Waiting for messages (2 seconds)..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));

  int string_msgs = string_count.load(std::memory_order_relaxed);
  int int_msgs = int_count.load(std::memory_order_relaxed);

  std::cout << "[PASS] Messages received:" << std::endl;
  std::cout << "  String messages: " << string_msgs << std::endl;
  std::cout << "  Int messages: " << int_msgs << std::endl;

  // Test 8: Stop plugin
  std::cout << "\n[TEST 8] Stopping plugin..." << std::endl;
  if (plugin_stop() != 0) {
    std::cerr << "[FAIL] Could not stop plugin" << std::endl;
    dlclose(handle);
    return 1;
  }
  std::cout << "[PASS] Plugin stopped" << std::endl;

  // Unload library
  dlclose(handle);

  std::cout << "\n=== All Tests PASSED ===" << std::endl;
  return 0;
}
