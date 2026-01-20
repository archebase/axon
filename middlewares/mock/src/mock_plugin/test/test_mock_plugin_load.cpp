/**
 * @file test_mock_plugin_load.cpp
 * @brief Test loading the mock plugin using PluginLoader
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "plugin_loader.hpp"

using namespace axon;

int main(int argc, char* argv[]) {
  std::cout << "=== Mock Plugin Load Test ===" << std::endl;

  // Parse arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <plugin_path>" << std::endl;
    std::cerr << "Example: " << argv[0] << " /path/to/libmock_plugin.so" << std::endl;
    return 1;
  }

  std::string plugin_path = argv[1];
  std::cout << "Plugin path: " << plugin_path << std::endl << std::endl;

  // Create loader
  PluginLoader loader;

  // Test 1: Load plugin
  std::cout << "[TEST 1] Loading plugin..." << std::endl;
  auto plugin_name_opt = loader.load(plugin_path);
  if (!plugin_name_opt) {
    std::cerr << "[FAIL] Could not load plugin" << std::endl;
    std::cerr << "Error: " << loader.get_last_error() << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin loaded: " << *plugin_name_opt << std::endl << std::endl;

  // Test 2: Get descriptor
  std::cout << "[TEST 2] Getting descriptor..." << std::endl;
  const auto* descriptor = loader.get_descriptor(*plugin_name_opt);
  if (!descriptor) {
    std::cerr << "[FAIL] Could not get descriptor" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Descriptor found" << std::endl;

  // Print plugin info
  std::cout << "\nPlugin Information:" << std::endl;
  std::cout << "  ABI Version: " << descriptor->abi_version_major << "."
            << descriptor->abi_version_minor << std::endl;
  std::cout << "  Middleware: "
            << (descriptor->middleware_name ? descriptor->middleware_name : "N/A") << std::endl;
  std::cout << "  Middleware Version: "
            << (descriptor->middleware_version ? descriptor->middleware_version : "N/A")
            << std::endl;
  std::cout << "  Plugin Version: "
            << (descriptor->plugin_version ? descriptor->plugin_version : "N/A") << std::endl;

  // Test 3: Check vtable
  std::cout << "\n[TEST 3] Checking vtable..." << std::endl;
  if (!descriptor->vtable) {
    std::cerr << "[FAIL] Vtable is null" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Vtable exists" << std::endl;

  std::cout << "\nVtable Functions:" << std::endl;
  std::cout << "  init: " << (descriptor->vtable->init ? "✓" : "✗") << std::endl;
  std::cout << "  start: " << (descriptor->vtable->start ? "✓" : "✗") << std::endl;
  std::cout << "  stop: " << (descriptor->vtable->stop ? "✓" : "✗") << std::endl;
  std::cout << "  subscribe: " << (descriptor->vtable->subscribe ? "✓" : "✗") << std::endl;
  std::cout << "  publish: " << (descriptor->vtable->publish ? "✓" : "✗") << std::endl;

  // Test 4: Initialize plugin
  std::cout << "\n[TEST 4] Initializing plugin..." << std::endl;
  if (descriptor->vtable->init("{}") != 0) {
    std::cerr << "[FAIL] Could not initialize plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin initialized" << std::endl;

  // Test 5: Start plugin
  std::cout << "\n[TEST 5] Starting plugin..." << std::endl;
  if (descriptor->vtable->start() != 0) {
    std::cerr << "[FAIL] Could not start plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin started" << std::endl;

  // Test 6: Subscribe to a topic
  std::cout << "\n[TEST 6] Subscribing to topic..." << std::endl;
  int message_count = 0;

  auto callback = [](
                    const char* topic_name,
                    const uint8_t* data,
                    size_t size,
                    const char* type,
                    uint64_t timestamp,
                    void* user_data
                  ) {
    int* count = static_cast<int*>(user_data);
    (*count)++;

    std::cout << "  [Message #" << *count << "]" << std::endl;
    std::cout << "    Topic: " << topic_name << std::endl;
    std::cout << "    Type: " << type << std::endl;
    std::cout << "    Size: " << size << " bytes" << std::endl;
    std::cout << "    Timestamp: " << timestamp << std::endl;

    // Print first few bytes of data
    std::cout << "    Data: ";
    size_t preview_len = std::min(size_t(32), size);
    for (size_t i = 0; i < preview_len; i++) {
      printf("%02x ", data[i]);
    }
    if (size > 32) {
      std::cout << "...";
    }
    std::cout << std::endl;
  };

  if (descriptor->vtable->subscribe("/test_topic", "std_msgs/String", callback, &message_count) != 0) {
    std::cerr << "[FAIL] Could not subscribe to topic" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Subscribed to /test_topic" << std::endl;

  // Test 7: Receive messages
  std::cout << "\n[TEST 7] Receiving messages (2 seconds)..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "[PASS] Received " << message_count << " messages" << std::endl;

  if (message_count == 0) {
    std::cerr << "[FAIL] No messages received!" << std::endl;
    return 1;
  }

  // Test 8: Stop plugin
  std::cout << "\n[TEST 8] Stopping plugin..." << std::endl;
  if (descriptor->vtable->stop() != 0) {
    std::cerr << "[FAIL] Could not stop plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin stopped" << std::endl;

  // Test 9: Unload
  std::cout << "\n[TEST 9] Unloading plugin..." << std::endl;
  if (!loader.unload(*plugin_name_opt)) {
    std::cerr << "[FAIL] Could not unload plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin unloaded" << std::endl;

  std::cout << "\n=== All Tests PASSED ===" << std::endl;
  return 0;
}
