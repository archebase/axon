/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

/**
 * @file test_mock_plugin_e2e.cpp
 * @brief End-to-end test for the mock plugin without PluginLoader
 * This tests the plugin functionality directly without loading via dlopen
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include "mock_plugin.hpp"

using namespace mock_plugin;

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  std::cout << "=== Mock Plugin E2E Test ===" << std::endl;

  // Test 1: Create plugin
  std::cout << "\n[TEST 1] Creating mock plugin..." << std::endl;
  MockPlugin plugin;
  std::cout << "[PASS] Plugin created" << std::endl;

  // Test 2: Initialize
  std::cout << "\n[TEST 2] Initializing plugin..." << std::endl;
  if (!plugin.init("{}")) {
    std::cerr << "[FAIL] Could not initialize plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin initialized" << std::endl;

  // Test 3: Start
  std::cout << "\n[TEST 3] Starting plugin..." << std::endl;
  if (!plugin.start()) {
    std::cerr << "[FAIL] Could not start plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin started" << std::endl;

  // Test 4: Check if running
  std::cout << "\n[TEST 4] Checking if running..." << std::endl;
  if (!plugin.is_running()) {
    std::cerr << "[FAIL] Plugin should be running" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin is running" << std::endl;

  // Test 5: Subscribe to topics
  std::cout << "\n[TEST 5] Subscribing to topics..." << std::endl;

  std::atomic<int> string_count{0};
  std::atomic<int> int_count{0};
  std::atomic<int> image_count{0};

  auto string_callback = [&string_count](
                           const std::string& topic,
                           const std::vector<uint8_t>& data,
                           const std::string& type,
                           uint64_t timestamp
                         ) {
    string_count++;
    std::string msg(data.begin(), data.end());
    std::cout << "  [String #" << string_count << "] " << topic << " (" << type << "): " << msg
              << " @ " << timestamp << std::endl;
  };

  auto int_callback = [&int_count](
                        const std::string& topic,
                        const std::vector<uint8_t>& data,
                        const std::string& type,
                        uint64_t timestamp
                      ) {
    int_count++;
    if (data.size() >= sizeof(int32_t)) {
      int32_t value;
      std::memcpy(&value, data.data(), sizeof(int32_t));
      std::cout << "  [Int #" << int_count << "] " << topic << " (" << type << "): " << value
                << " @ " << timestamp << std::endl;
    }
  };

  auto image_callback = [&image_count](
                          const std::string& topic,
                          const std::vector<uint8_t>& data,
                          const std::string& type,
                          uint64_t timestamp
                        ) {
    image_count++;
    std::cout << "  [Image #" << image_count << "] " << topic << " (" << type
              << "): " << data.size() << " bytes @ " << timestamp << std::endl;
  };

  if (!plugin.subscribe("/test/string", "std_msgs/String", string_callback, nullptr)) {
    std::cerr << "[FAIL] Could not subscribe to /test/string" << std::endl;
    return 1;
  }
  std::cout << "  ✓ Subscribed to /test/string (std_msgs/String)" << std::endl;

  if (!plugin.subscribe("/test/int", "std_msgs/Int32", int_callback, nullptr)) {
    std::cerr << "[FAIL] Could not subscribe to /test/int" << std::endl;
    return 1;
  }
  std::cout << "  ✓ Subscribed to /test/int (std_msgs/Int32)" << std::endl;

  if (!plugin.subscribe("/test/image", "sensor_msgs/Image", image_callback, nullptr)) {
    std::cerr << "[FAIL] Could not subscribe to /test/image" << std::endl;
    return 1;
  }
  std::cout << "  ✓ Subscribed to /test/image (sensor_msgs/Image)" << std::endl;

  // Test 6: Check subscription count
  std::cout << "\n[TEST 6] Checking subscription count..." << std::endl;
  size_t sub_count = plugin.get_subscription_count();
  if (sub_count != 3) {
    std::cerr << "[FAIL] Expected 3 subscriptions, got " << sub_count << std::endl;
    return 1;
  }
  std::cout << "[PASS] Subscription count: " << sub_count << std::endl;

  // Test 7: Receive messages
  std::cout << "\n[TEST 7] Receiving messages (2 seconds)..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "[PASS] Messages received:" << std::endl;
  std::cout << "  String messages: " << string_count << std::endl;
  std::cout << "  Int messages: " << int_count << std::endl;
  std::cout << "  Image messages: " << image_count << std::endl;

  if (string_count == 0 || int_count == 0 || image_count == 0) {
    std::cerr << "[FAIL] Not all message types received!" << std::endl;
    return 1;
  }

  // Test 8: Publish manually
  std::cout << "\n[TEST 8] Publishing messages manually..." << std::endl;

  std::string test_data = "Manual test message";
  std::vector<uint8_t> manual_data(test_data.begin(), test_data.end());

  if (!plugin.publish("/test/string", manual_data, "std_msgs/String")) {
    std::cerr << "[FAIL] Could not publish manual message" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Manual message published" << std::endl;

  // Wait for the manual message
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Test 9: Stop plugin
  std::cout << "\n[TEST 9] Stopping plugin..." << std::endl;
  if (!plugin.stop()) {
    std::cerr << "[FAIL] Could not stop plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin stopped" << std::endl;

  // Test 10: Check if stopped
  std::cout << "\n[TEST 10] Checking if stopped..." << std::endl;
  if (plugin.is_running()) {
    std::cerr << "[FAIL] Plugin should not be running" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin is stopped" << std::endl;

  std::cout << "\n=== All Tests PASSED ===" << std::endl;
  return 0;
}
