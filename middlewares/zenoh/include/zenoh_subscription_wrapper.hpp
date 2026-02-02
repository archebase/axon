// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_ZENOH_SUBSCRIPTION_WRAPPER_HPP
#define AXON_ZENOH_SUBSCRIPTION_WRAPPER_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declare Zenoh session to avoid including zenoh.hxx in header
namespace zenoh {
class Session;
template<typename T>
class Subscriber;
class KeyExpr;
class Sample;

}  // namespace zenoh

namespace zenoh_plugin {

/**
 * @brief Message callback type for Zenoh messages
 *
 * @param keyexpr The key expression of the message
 * @param message_type The message type (for MCAP metadata)
 * @param message_data Raw message bytes
 * @param timestamp_ns Unix timestamp in nanoseconds
 */
using MessageCallback = std::function<void(
  const std::string& keyexpr, const std::string& message_type,
  const std::vector<uint8_t>& message_data, uint64_t timestamp_ns
)>;

/**
 * @brief Manages Zenoh subscriptions with thread safety
 *
 * Handles subscription lifecycle and callback invocation.
 * Zenoh delivers messages in its own threads, so we use mutexes
 * to protect the subscription map.
 */
class SubscriptionManager {
public:
  /**
   * @brief Construct a SubscriptionManager
   *
   * @param session Zenoh session (non-owning pointer)
   */
  explicit SubscriptionManager(zenoh::Session* session);

  /**
   * @brief Destructor - unsubscribes from all key expressions
   */
  ~SubscriptionManager();

  /**
   * @brief Subscribe to a key expression with callback
   *
   * @param keyexpr Zenoh key expression to subscribe to
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
   * @brief Get all active subscriptions
   *
   * @return Vector of subscribed key expressions
   */
  std::vector<std::string> get_subscribed_keyexprs() const;

private:
  zenoh::Session* session_;  // Non-owning pointer

  /**
   * @brief Subscription information
   */
  struct SubscriptionInfo {
    std::unique_ptr<zenoh::Subscriber<void>> subscriber;
    std::unique_ptr<zenoh::KeyExpr> keyexpr;
    MessageCallback callback;
  };

  std::unordered_map<std::string, SubscriptionInfo> subscriptions_;
  mutable std::mutex mutex_;
};

}  // namespace zenoh_plugin

#endif  // AXON_ZENOH_SUBSCRIPTION_WRAPPER_HPP
