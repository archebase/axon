/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

#ifndef MOCK_PLUGIN_HPP
#define MOCK_PLUGIN_HPP

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace mock_plugin {

// Message callback signature
using MessageCallback = std::function<void(
  const std::string& topic_name, const std::vector<uint8_t>& message_data,
  const std::string& message_type, uint64_t timestamp
)>;

// Mock subscription info
struct MockSubscription {
  std::string topic_name;
  std::string message_type;
  MessageCallback callback;
  void* user_data;
  int publish_count;  // Track how many messages were published
};

// Mock plugin implementation
class MockPlugin {
public:
  MockPlugin();
  ~MockPlugin();

  // Initialize the plugin with config (JSON string, but we'll ignore it for mock)
  bool init(const char* config_json);

  // Start the mock plugin (spins a thread that generates fake messages)
  bool start();

  // Stop the plugin
  bool stop();

  // Subscribe to a topic
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, MessageCallback callback,
    void* user_data
  );

  // Publish a message (for testing purposes)
  bool publish(
    const std::string& topic_name, const std::vector<uint8_t>& data, const std::string& message_type
  );

  // Check if plugin is running
  bool is_running() const;

  // Get number of active subscriptions
  size_t get_subscription_count() const;

  // Publish mock messages to all subscriptions
  void publish_mock_messages();

private:
  mutable std::mutex mutex_;
  std::atomic<bool> running_;
  std::atomic<bool> initialized_;
  std::thread publisher_thread_;

  // Map of topic name to subscription info
  std::unordered_map<std::string, MockSubscription> subscriptions_;

  // Publisher thread function
  void publisher_loop();

  // Generate mock message data for a given message type
  std::vector<uint8_t> generate_mock_message(const std::string& message_type);
};

}  // namespace mock_plugin

#endif  // MOCK_PLUGIN_HPP
