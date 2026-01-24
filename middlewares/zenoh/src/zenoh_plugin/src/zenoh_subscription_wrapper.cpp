// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "zenoh_subscription_wrapper.hpp"

#include <chrono>
#include <iostream>
#include <zenoh.hxx>

namespace zenoh_plugin {

SubscriptionManager::SubscriptionManager(zenoh::Session* session)
    : session_(session) {}

SubscriptionManager::~SubscriptionManager() {
  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
}

bool SubscriptionManager::subscribe(
  const std::string& keyexpr, const std::string& message_type, MessageCallback callback
) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if already subscribed
  if (subscriptions_.find(keyexpr) != subscriptions_.end()) {
    std::cout << "[ZenohPlugin] Already subscribed to: " << keyexpr << std::endl;
    return true;
  }

  try {
    // Declare key expression (registers for bandwidth optimization)
    auto ze_keyexpr = std::make_unique<zenoh::KeyExpr>(keyexpr);

    // Capture callback by value for lambda
    std::string captured_type = message_type;
    MessageCallback captured_callback = callback;

    // Declare subscriber with callback lambda
    // zenoh-cpp 1.7.x requires on_sample and on_drop callbacks
    auto subscriber = session_->declare_subscriber(
      *ze_keyexpr,
      [captured_type, captured_callback, keyexpr](const zenoh::Sample& sample) {
        if (!captured_callback) {
          return;
        }

        try {
          // Extract raw payload as bytes using as_vector()
          const auto& payload = sample.get_payload();
          std::vector<uint8_t> data = payload.as_vector();

          // Extract timestamp from sample (if available)
          uint64_t timestamp_ns;

          if (sample.get_timestamp().has_value()) {
            // Use Zenoh timestamp if available
            timestamp_ns = sample.get_timestamp()->get_time();
          } else {
            // Fallback to current system time
            auto now = std::chrono::system_clock::now();
            timestamp_ns = static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()
            );
          }

          // Invoke callback with raw data
          captured_callback(keyexpr, captured_type, data, timestamp_ns);

        } catch (const std::exception& e) {
          std::cerr << "[ZenohPlugin] Failed to handle sample on " << keyexpr << ": " << e.what()
                    << std::endl;
        }
      },
      zenoh::closures::none  // on_drop callback (no-op)
    );

    // Store subscription info
    SubscriptionInfo info;
    info.subscriber = std::make_unique<zenoh::Subscriber<void>>(std::move(subscriber));
    info.keyexpr = std::move(ze_keyexpr);
    info.callback = std::move(callback);
    subscriptions_[keyexpr] = std::move(info);

    std::cout << "[ZenohPlugin] Subscribed to: " << keyexpr << " (" << message_type << ")"
              << std::endl;

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[ZenohPlugin] Exception subscribing to " << keyexpr << ": " << e.what()
              << std::endl;
    return false;
  }
}

bool SubscriptionManager::unsubscribe(const std::string& keyexpr) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(keyexpr);
  if (it == subscriptions_.end()) {
    std::cout << "[ZenohPlugin] Not subscribed to: " << keyexpr << std::endl;
    return false;
  }

  // Subscriber and keyexpr automatically cleaned up when erased
  subscriptions_.erase(it);
  std::cout << "[ZenohPlugin] Unsubscribed from: " << keyexpr << std::endl;
  return true;
}

std::vector<std::string> SubscriptionManager::get_subscribed_keyexprs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> keyexprs;
  keyexprs.reserve(subscriptions_.size());
  for (const auto& [keyexpr, _] : subscriptions_) {
    keyexprs.push_back(keyexpr);
  }
  return keyexprs;
}

}  // namespace zenoh_plugin
