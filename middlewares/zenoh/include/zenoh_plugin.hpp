// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_ZENOH_PLUGIN_HPP
#define AXON_ZENOH_PLUGIN_HPP

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "zenoh_subscription_wrapper.hpp"

namespace zenoh_plugin {

/**
 * @brief Zenoh plugin for Axon recorder
 *
 * Implements the plugin interface for recording Zenoh messages to MCAP.
 * Follows the same pattern as ROS2 plugin with Zenoh-specific adaptations.
 */
class ZenohPlugin {
public:
  ZenohPlugin();
  ~ZenohPlugin();

  // Non-copyable, non-movable
  ZenohPlugin(const ZenohPlugin&) = delete;
  ZenohPlugin& operator=(const ZenohPlugin&) = delete;

  /**
   * @brief Initialize the Zenoh plugin with JSON config
   *
   * Config format: {"mode": "peer|client", "locator": "tcp/addr:port"}
   *
   * @param config_json JSON configuration string
   * @return true if initialization succeeded
   */
  bool init(const char* config_json);

  /**
   * @brief Start the plugin (Zenoh session is already active after init)
   *
   * @return true if start succeeded
   */
  bool start();

  /**
   * @brief Stop the plugin and close session
   *
   * @return true if stop succeeded
   */
  bool stop();

  /**
   * @brief Check if plugin is initialized
   *
   * @return true if initialized
   */
  bool is_initialized() const {
    return initialized_.load();
  }

  /**
   * @brief Check if plugin is running
   *
   * @return true if running
   */
  bool is_running() const {
    return running_.load();
  }

  /**
   * @brief Subscribe to a key expression with callback
   *
   * @param keyexpr Zenoh key expression (e.g., "demo/example/simple")
   * @param message_type Message type for MCAP metadata
   * @param callback Callback function for received messages
   * @return true if subscription succeeded
   */
  bool subscribe(
    const std::string& keyexpr, const std::string& message_type, MessageCallback callback
  );

  /**
   * @brief Unsubscribe from a key expression
   *
   * @param keyexpr Key expression to unsubscribe from
   * @return true if unsubscription succeeded
   */
  bool unsubscribe(const std::string& keyexpr);

  /**
   * @brief Get list of subscribed key expressions
   *
   * @return Vector of subscribed key expressions
   */
  std::vector<std::string> get_subscribed_keyexprs() const;

private:
  std::unique_ptr<SubscriptionManager> subscription_manager_;
  std::atomic<bool> initialized_;
  std::atomic<bool> running_;
};

}  // namespace zenoh_plugin

#endif  // AXON_ZENOH_PLUGIN_HPP
