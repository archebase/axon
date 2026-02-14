/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

#include "mock_plugin.hpp"

#include <chrono>
#include <cstring>
#include <random>

// Define component name for logging
#define AXON_LOG_COMPONENT "mock_plugin"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace mock_plugin {

MockPlugin::MockPlugin()
    : running_(false)
    , initialized_(false) {}

MockPlugin::~MockPlugin() {
  stop();
}

bool MockPlugin::init(const char* config_json) {
  (void)config_json;  // Ignore config for mock

  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    AXON_LOG_ERROR("Already initialized");
    return false;
  }

  initialized_ = true;
  AXON_LOG_INFO("Initialized");
  return true;
}

bool MockPlugin::start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    AXON_LOG_ERROR("Not initialized");
    return false;
  }

  if (running_) {
    AXON_LOG_ERROR("Already running");
    return false;
  }

  running_ = true;

  // Start publisher thread
  publisher_thread_ = std::thread(&MockPlugin::publisher_loop, this);

  AXON_LOG_INFO("Started");
  return true;
}

bool MockPlugin::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return true;  // Already stopped
    }
    running_ = false;
  }

  // Wait for publisher thread to finish
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }

  AXON_LOG_INFO("Stopped");
  return true;
}

bool MockPlugin::subscribe(
  const std::string& topic_name, const std::string& message_type, MessageCallback callback,
  void* user_data
) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    AXON_LOG_WARN("Already subscribed to " << kv("topic", topic_name));
    return false;
  }

  MockSubscription sub;
  sub.topic_name = topic_name;
  sub.message_type = message_type;
  sub.callback = callback;
  sub.user_data = user_data;
  sub.publish_count = 0;

  subscriptions_[topic_name] = sub;

  AXON_LOG_INFO(
    "Subscribed to " << kv("topic", topic_name) << " (" << kv("type", message_type) << ")"
  );
  return true;
}

bool MockPlugin::publish(
  const std::string& topic_name, const std::vector<uint8_t>& data, const std::string& message_type
) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    AXON_LOG_ERROR("No subscription for " << kv("topic", topic_name));
    return false;
  }

  // Get current timestamp
  auto now = std::chrono::system_clock::now();
  auto timestamp =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  // Call the callback
  it->second.callback(topic_name, data, message_type, timestamp);
  it->second.publish_count++;

  return true;
}

bool MockPlugin::is_running() const {
  return running_;
}

size_t MockPlugin::get_subscription_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return subscriptions_.size();
}

void MockPlugin::publish_mock_messages() {
  std::lock_guard<std::mutex> lock(mutex_);

  auto now = std::chrono::system_clock::now();
  auto timestamp =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  // Publish a mock message to each subscription
  for (auto& [topic_name, sub] : subscriptions_) {
    auto mock_data = generate_mock_message(sub.message_type);
    sub.callback(topic_name, mock_data, sub.message_type, timestamp);
    sub.publish_count++;
  }
}

void MockPlugin::publisher_loop() {
  AXON_LOG_INFO("Publisher thread started");

  while (running_) {
    // Publish mock messages every 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (running_) {
      publish_mock_messages();
    }
  }

  AXON_LOG_INFO("Publisher thread stopped");
}

std::vector<uint8_t> MockPlugin::generate_mock_message(const std::string& message_type) {
  // Generate simple mock data based on message type
  std::vector<uint8_t> data;

  if (message_type == "std_msgs/String") {
    // Mock string message: "Hello, Mock!"
    std::string msg = "Hello, Mock!";
    data.insert(data.end(), msg.begin(), msg.end());
  } else if (message_type == "std_msgs/Int32") {
    // Mock int32 message: 42
    int32_t value = 42;
    data.resize(sizeof(int32_t));
    std::memcpy(data.data(), &value, sizeof(int32_t));
  } else if (message_type == "sensor_msgs/Image") {
    // Mock image header
    std::string header = "MockImage";
    data.insert(data.end(), header.begin(), header.end());
    // Add some fake image data (100 bytes)
    for (int i = 0; i < 100; i++) {
      data.push_back(static_cast<uint8_t>(i % 256));
    }
  } else {
    // Generic mock data
    std::string msg = "Mock data for " + message_type;
    data.insert(data.end(), msg.begin(), msg.end());
  }

  return data;
}

}  // namespace mock_plugin
