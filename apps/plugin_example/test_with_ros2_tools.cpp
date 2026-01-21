// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

/**
 * @file test_with_ros2_tools.cpp
 * @brief Test plugin by subscribing to messages published via ros2 topic pub CLI tool
 *
 * Usage:
 *   1. Run this program: ./test_with_ros2_tools <plugin_path>
 *   2. In another terminal, publish messages:
 *      ros2 topic pub /chatter std_msgs/String "data: 'Hello World'"
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "../axon_recorder/plugin_loader.hpp"

using namespace axon;

static std::sig_atomic_t g_running = 1;
static std::atomic<int> g_message_count{0};

void signal_handler(int signal) {
  (void)signal;
  std::cout << "\n[INFO] Shutdown signal received..." << std::endl;
  g_running = 0;
}

void message_callback(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
) {
  (void)user_data;
  g_message_count++;

  std::cout << "[MSG " << g_message_count << "] Topic: " << topic_name
            << " | Type: " << message_type << " | Size: " << message_size << " bytes"
            << " | Timestamp: " << timestamp << std::endl;

  // Print first 16 bytes
  constexpr size_t MAX_DUMP = 16;
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

int main(int argc, char* argv[]) {
  std::cout << "========================================" << std::endl;
  std::cout << "  ROS2 Plugin Test with CLI Tools" << std::endl;
  std::cout << "========================================" << std::endl;

  // Parse arguments
  std::string plugin_path =
    "/home/xlw/src/Axon/middlewares/ros2/src/ros2_plugin/install/axon_ros2_plugin/lib/axon/plugins/"
    "libaxon_ros2_plugin.so";
  int wait_seconds = 30;

  if (argc > 1) {
    plugin_path = argv[1];
  }
  if (argc > 2) {
    wait_seconds = std::atoi(argv[2]);
  }

  std::cout << "[INFO] Plugin path: " << plugin_path << std::endl;
  std::cout << "[INFO] Wait time: " << wait_seconds << " seconds" << std::endl;

  // Setup signal handler
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create loader
  PluginLoader loader;

  // Load plugin
  std::cout << "\n[INFO] Loading plugin..." << std::endl;
  auto plugin_name_opt = loader.load(plugin_path);

  if (!plugin_name_opt) {
    std::cerr << "[ERROR] Failed to load plugin: " << loader.get_last_error() << std::endl;
    return 1;
  }

  std::string plugin_name = *plugin_name_opt;
  std::cout << "[OK] Loaded plugin: " << plugin_name << std::endl;

  // Initialize plugin
  const auto* descriptor = loader.get_descriptor(plugin_name);
  auto* plugin = loader.get_plugin(plugin_name);

  const char* config_json = "{}";
  AxonStatus status = descriptor->vtable->init(config_json);

  if (status != AXON_SUCCESS) {
    std::cerr << "[ERROR] Failed to initialize plugin, status: " << status << std::endl;
    loader.unload_all();
    return 1;
  }

  plugin->initialized = true;
  std::cout << "[OK] Plugin initialized" << std::endl;

  // Subscribe to topic
  std::cout << "[INFO] Subscribing to /chatter..." << std::endl;
  status =
    descriptor->vtable->subscribe("/chatter", "std_msgs/msg/String", message_callback, nullptr);

  if (status != AXON_SUCCESS) {
    std::cerr << "[ERROR] Failed to subscribe, status: " << status << std::endl;
    loader.unload_all();
    return 1;
  }

  std::cout << "[OK] Subscribed to /chatter" << std::endl;

  // Start spinning
  std::cout << "[INFO] Starting plugin..." << std::endl;
  status = descriptor->vtable->start();

  if (status != AXON_SUCCESS) {
    std::cerr << "[ERROR] Failed to start plugin, status: " << status << std::endl;
    loader.unload_all();
    return 1;
  }

  plugin->running = true;
  std::cout << "[OK] Plugin started" << std::endl;

  // Wait for messages
  std::cout << "\n========================================" << std::endl;
  std::cout << "  Ready to receive messages" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "\nIn another terminal, run:" << std::endl;
  std::cout << "  ros2 topic pub /chatter std_msgs/String \"data: 'Hello World'\"" << std::endl;
  std::cout << "\nWaiting " << wait_seconds << " seconds (or press Ctrl+C to stop early)..."
            << std::endl;
  std::cout << std::endl;

  auto start_time = std::chrono::steady_clock::now();
  while (g_running) {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                     std::chrono::steady_clock::now() - start_time
    )
                     .count();

    if (elapsed >= wait_seconds) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Shutdown
  std::cout << "\n[INFO] Shutting down..." << std::endl;
  if (descriptor->vtable->stop) {
    descriptor->vtable->stop();
  }
  plugin->running = false;

  loader.unload_all();
  std::cout << "[OK] Plugin unloaded" << std::endl;

  std::cout << "\n========================================" << std::endl;
  std::cout << "  Test Complete" << std::endl;
  std::cout << "  Messages received: " << g_message_count << std::endl;
  std::cout << "========================================" << std::endl;

  return (g_message_count > 0) ? 0 : 1;
}
