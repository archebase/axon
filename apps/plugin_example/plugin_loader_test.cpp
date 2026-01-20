// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * @file plugin_loader_test.cpp
 * @brief Plugin loader test example for Axon middleware plugins
 *
 * This example demonstrates how to:
 * 1. Load a ROS2 plugin dynamically
 * 2. Initialize the plugin with configuration
 * 3. Set message callback
 * 4. Subscribe to topics
 * 5. Spin the executor
 * 6. Cleanup and unload
 */

#include "../axon_recorder/plugin_loader.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

using namespace axon;

// Global flag for graceful shutdown
static std::sig_atomic_t g_running = 1;

// Signal handler for Ctrl+C
void signal_handler(int signal) {
  (void)signal;
  std::cout << "\n[INFO] Shutdown signal received..." << std::endl;
  g_running = 0;
}

// =============================================================================
// Message callback implementation
// =============================================================================
void message_callback(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
) {
  (void)user_data;

  // Print message received info
  std::cout << "[MSG] Topic: " << topic_name << " | Type: " << message_type
            << " | Size: " << message_size << " bytes"
            << " | Timestamp: " << timestamp << std::endl;

  // Print first 32 bytes of message data (hexdump style)
  constexpr size_t MAX_DUMP = 32;
  size_t dump_size = std::min(message_size, MAX_DUMP);

  std::cout << "      Data: ";
  for (size_t i = 0; i < dump_size; ++i) {
    printf("%02x ", message_data[i]);
  }
  if (message_size > MAX_DUMP) {
    std::cout << "...";
  }
  std::cout << std::endl;
}

// =============================================================================
// Test scenarios
// =============================================================================

/**
 * Test 1: Basic plugin loading and initialization
 */
bool test_load_and_init(PluginLoader& loader, const std::string& plugin_path) {
  std::cout << "\n=== Test 1: Load and Initialize Plugin ===" << std::endl;

  // Load the plugin
  auto plugin_name_opt = loader.load(plugin_path);
  if (!plugin_name_opt) {
    std::cerr << "[ERROR] Failed to load plugin: " << loader.get_last_error() << std::endl;
    return false;
  }

  std::string plugin_name = *plugin_name_opt;
  std::cout << "[OK] Loaded plugin: " << plugin_name << std::endl;

  // Get descriptor
  const auto* descriptor = loader.get_descriptor(plugin_name);
  if (!descriptor) {
    std::cerr << "[ERROR] Failed to get plugin descriptor" << std::endl;
    return false;
  }

  // Print plugin info
  std::cout << "[INFO] Plugin Details:" << std::endl;
  std::cout << "       - ABI Version: " << descriptor->abi_version_major << "."
            << descriptor->abi_version_minor << std::endl;
  std::cout << "       - Middleware: " << descriptor->middleware_name << std::endl;
  std::cout << "       - Version: " << descriptor->middleware_version << std::endl;
  std::cout << "       - Plugin: " << descriptor->plugin_version << std::endl;

  // Initialize plugin with JSON config
  const char* config_json = R"({
        "node_name": "axon_plugin_test",
        "use_sim_time": false
    })";

  auto* plugin = loader.get_plugin(plugin_name);
  AxonStatus status = descriptor->vtable->init(config_json);

  if (status != AXON_SUCCESS) {
    std::cerr << "[ERROR] Failed to initialize plugin, status: " << status << std::endl;
    return false;
  }

  plugin->initialized = true;
  std::cout << "[OK] Plugin initialized" << std::endl;

  return true;
}

/**
 * Test 2: Subscribe to topics
 */
bool test_subscribe(PluginLoader& loader, const std::string& plugin_name) {
  std::cout << "\n=== Test 2: Subscribe to Topics ===" << std::endl;

  const auto* descriptor = loader.get_descriptor(plugin_name);
  if (!descriptor || !descriptor->vtable->subscribe) {
    std::cerr << "[ERROR] Plugin does not support subscribe" << std::endl;
    return false;
  }

  // List of topics to subscribe
  std::vector<std::pair<std::string, std::string>> topics = {
    {"/imu/data", "sensor_msgs/msg/Imu"},
    {"/camera0/rgb", "sensor_msgs/msg/Image"},
  };

  for (const auto& [topic, type] : topics) {
    std::cout << "[INFO] Subscribing to: " << topic << " (" << type << ")" << std::endl;

    AxonStatus status = descriptor->vtable->subscribe(
      topic.c_str(),
      type.c_str(),
      nullptr,  // options_json (none for this test)
      message_callback,
      nullptr  // user_data
    );

    if (status != AXON_SUCCESS) {
      std::cout << "[WARN] Failed to subscribe to " << topic << ", status: " << status << std::endl;
      // Continue with other topics
    } else {
      std::cout << "[OK] Subscribed to: " << topic << std::endl;
    }
  }

  return true;
}

/**
 * Test 3: Spin the plugin
 */
bool test_spin(PluginLoader& loader, const std::string& plugin_name, int seconds) {
  std::cout << "\n=== Test 3: Spin Plugin (" << seconds << " seconds) ===" << std::endl;

  const auto* descriptor = loader.get_descriptor(plugin_name);
  if (!descriptor || !descriptor->vtable->start) {
    std::cerr << "[ERROR] Plugin does not support start" << std::endl;
    return false;
  }

  auto* plugin = loader.get_plugin(plugin_name);

  // Start spinning (this creates the executor thread)
  std::cout << "[INFO] Starting plugin executor... (Press Ctrl+C to stop early)" << std::endl;
  AxonStatus status = descriptor->vtable->start();

  if (status != AXON_SUCCESS) {
    std::cerr << "[ERROR] Failed to start spinning, status: " << status << std::endl;
    return false;
  }

  plugin->running = true;

  // Wait for the specified duration while the executor runs in background
  auto start_time = std::chrono::steady_clock::now();

  while (g_running) {
    // Check timeout
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                     std::chrono::steady_clock::now() - start_time
    )
                     .count();

    if (elapsed >= seconds) {
      break;
    }

    // Sleep for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "[INFO] Spin test completed" << std::endl;

  return true;
}

/**
 * Test 4: Cleanup and unload
 */
bool test_cleanup(PluginLoader& loader, const std::string& plugin_name) {
  std::cout << "\n=== Test 4: Cleanup and Unload ===" << std::endl;

  const auto* descriptor = loader.get_descriptor(plugin_name);
  auto* plugin = loader.get_plugin(plugin_name);

  // Shutdown plugin
  if (plugin && plugin->running && descriptor->vtable->stop) {
    std::cout << "[INFO] Shutting down plugin..." << std::endl;
    AxonStatus status = descriptor->vtable->stop();

    if (status != AXON_SUCCESS) {
      std::cerr << "[WARN] Shutdown returned error: " << status << std::endl;
    }
    plugin->running = false;
  }

  // Unload plugin
  if (loader.unload(plugin_name)) {
    std::cout << "[OK] Plugin unloaded" << std::endl;
    return true;
  } else {
    std::cerr << "[ERROR] Failed to unload: " << loader.get_last_error() << std::endl;
    return false;
  }
}

// =============================================================================
// Main entry point
// =============================================================================
int main(int argc, char* argv[]) {
  std::cout << "========================================" << std::endl;
  std::cout << "  Axon Plugin Loader Test" << std::endl;
  std::cout << "========================================" << std::endl;

  // Setup signal handler
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Parse command line arguments
  // Plugin path must be provided via command line argument or environment variable
  std::string plugin_path;
  const char* env_path = std::getenv("AXON_ROS2_PLUGIN_PATH");
  if (env_path) {
    plugin_path = env_path;
  } else if (argc > 1) {
    plugin_path = argv[1];
  } else {
    std::cerr << "[ERROR] Plugin path must be provided via:" << std::endl;
    std::cerr << "  1. Command line argument: ./plugin_loader_test <plugin_path>" << std::endl;
    std::cerr << "  2. Environment variable: export AXON_ROS2_PLUGIN_PATH=<path>" << std::endl;
    return 1;
  }
  int test_duration_seconds = 10;

  if (argc > 2) {
    test_duration_seconds = std::stoi(argv[2]);
  }

  std::cout << "[INFO] Plugin path: " << plugin_path << std::endl;
  std::cout << "[INFO] Test duration: " << test_duration_seconds << " seconds" << std::endl;

  // Create plugin loader
  PluginLoader loader;

  // Run tests - stop on first failure
  // Test 1: Load and initialize (critical - must pass to continue)
  if (!test_load_and_init(loader, plugin_path)) {
    std::cerr << "\n[FAIL] Test 1 failed" << std::endl;
    loader.unload_all();
    return 1;
  }

  // Test 2: Subscribe
  if (!test_subscribe(loader, "ROS2")) {
    std::cerr << "\n[FAIL] Test 2 failed" << std::endl;
    loader.unload_all();
    return 1;
  }

  // Test 3: Spin
  if (!test_spin(loader, "ROS2", test_duration_seconds)) {
    std::cerr << "\n[FAIL] Test 3 failed" << std::endl;
    loader.unload_all();
    return 1;
  }

  // Test 4: Cleanup
  if (!test_cleanup(loader, "ROS2")) {
    std::cerr << "\n[FAIL] Test 4 failed" << std::endl;
    loader.unload_all();
    return 1;
  }
  // Final cleanup
  loader.unload_all();

  std::cout << "\n========================================" << std::endl;
  std::cout << "  All tests PASSED" << std::endl;
  std::cout << "========================================" << std::endl;

  return 0;
}
