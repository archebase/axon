// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_subscription_wrapper.hpp"

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
#include "depth_compression_filter.hpp"
#endif

#include <algorithm>
#include <cctype>
#include <string>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros2_subscription"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace ros2_plugin {

namespace {

struct QosPolicySelection {
  size_t depth = 10;
  rmw_qos_reliability_policy_t reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_durability_policy_t durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  rmw_qos_history_policy_t history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
};

bool has_auto_qos_fields(const SubscribeOptions& options) {
  return options.qos_depth_auto || options.qos_reliability_auto || options.qos_durability_auto ||
         options.qos_history_auto;
}

std::string normalize_type_name(std::string value) {
  std::replace(value.begin(), value.end(), ':', '/');
  value.erase(
    std::unique(
      value.begin(),
      value.end(),
      [](char lhs, char rhs) {
        return lhs == '/' && rhs == '/';
      }
    ),
    value.end()
  );
  return value;
}

bool topic_type_matches(const rclcpp::TopicEndpointInfo& info, const std::string& message_type) {
  return message_type.empty() ||
         normalize_type_name(info.topic_type()) == normalize_type_name(message_type);
}

bool is_sensor_data_topic(const std::string& topic_name, const std::string& message_type) {
  const std::string type = normalize_type_name(message_type);
  if (type == "sensor_msgs/msg/Image" || type == "sensor_msgs/msg/CompressedImage" ||
      type == "sensor_msgs/msg/PointCloud2" || type == "sensor_msgs/msg/LaserScan" ||
      type == "sensor_msgs/msg/Imu") {
    return true;
  }
  return topic_name.find("/camera") != std::string::npos ||
         topic_name.find("/lidar") != std::string::npos ||
         topic_name.find("/scan") != std::string::npos ||
         topic_name.find("/imu") != std::string::npos;
}

QosPolicySelection selection_from_qos(const rclcpp::QoS& qos) {
  const auto& profile = qos.get_rmw_qos_profile();
  QosPolicySelection selection;
  selection.depth = profile.depth > 0 ? profile.depth : 10;
  selection.reliability = profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                            ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                            : RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  selection.durability = profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                           ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                           : RMW_QOS_POLICY_DURABILITY_VOLATILE;
  selection.history = profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL
                        ? RMW_QOS_POLICY_HISTORY_KEEP_ALL
                        : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  return selection;
}

rclcpp::QoS qos_from_selection(const QosPolicySelection& selection) {
  rclcpp::QoS qos = selection.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL
                      ? rclcpp::QoS(rclcpp::KeepAll())
                      : rclcpp::QoS(rclcpp::KeepLast(selection.depth > 0 ? selection.depth : 10));
  if (selection.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  if (selection.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  return qos;
}

std::optional<QosPolicySelection> selection_from_publishers(
  const rclcpp::Node::SharedPtr& node, const std::string& topic_name,
  const std::string& message_type
) {
  if (!node) {
    return std::nullopt;
  }

  const auto publishers = node->get_publishers_info_by_topic(topic_name);
  bool matched = false;
  bool any_best_effort = false;
  bool any_volatile = false;
  bool any_transient_local = false;
  bool any_keep_all = false;
  size_t max_depth = 0;

  for (const auto& publisher : publishers) {
    if (!topic_type_matches(publisher, message_type)) {
      continue;
    }
    matched = true;
    const auto& profile = publisher.qos_profile().get_rmw_qos_profile();
    any_best_effort =
      any_best_effort || profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    any_volatile = any_volatile || profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE;
    any_transient_local =
      any_transient_local || profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    any_keep_all = any_keep_all || profile.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    max_depth = std::max(max_depth, profile.depth);
  }

  if (!matched) {
    return std::nullopt;
  }

  QosPolicySelection selection;
  selection.depth = max_depth > 0 ? max_depth : 10;
  selection.reliability =
    any_best_effort ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT : RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  selection.durability = any_volatile || !any_transient_local
                           ? RMW_QOS_POLICY_DURABILITY_VOLATILE
                           : RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  selection.history =
    any_keep_all ? RMW_QOS_POLICY_HISTORY_KEEP_ALL : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  return selection;
}

QosPolicySelection fallback_selection_for_topic(
  const std::string& topic_name, const std::string& message_type
) {
  QosPolicySelection selection;
  if (topic_name == "/tf_static" || topic_name.rfind("/tf_static", 0) == 0) {
    selection.depth = 1;
    selection.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    selection.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    selection.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    return selection;
  }
  if (is_sensor_data_topic(topic_name, message_type)) {
    selection.depth = 10;
    selection.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    selection.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    selection.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  return selection;
}

}  // namespace

SubscribeOptions resolve_auto_qos_options(
  const rclcpp::Node::SharedPtr& node, const std::string& topic_name,
  const std::string& message_type, const SubscribeOptions& options
) {
  if (!has_auto_qos_fields(options)) {
    return options;
  }

  SubscribeOptions resolved = options;
  QosPolicySelection selection = selection_from_qos(options.qos);
  const auto detected = selection_from_publishers(node, topic_name, message_type);
  const QosPolicySelection automatic =
    detected.value_or(fallback_selection_for_topic(topic_name, message_type));

  if (options.qos_depth_auto) {
    selection.depth = automatic.depth;
  }
  if (options.qos_reliability_auto) {
    selection.reliability = automatic.reliability;
  }
  if (options.qos_durability_auto) {
    selection.durability = automatic.durability;
  }
  if (options.qos_history_auto) {
    selection.history = automatic.history;
  }
  resolved.qos = qos_from_selection(selection);

  AXON_LOG_INFO(
    "Resolved auto QoS for topic "
    << kv("topic", topic_name) << kv("type", message_type)
    << kv("source", detected.has_value() ? "publisher" : "fallback")
    << kv(
         "reliability",
         selection.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ? "best_effort"
                                                                         : "reliable"
       )
    << kv(
         "durability",
         selection.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "transient_local"
                                                                           : "volatile"
       )
    << kv(
         "history", selection.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL ? "keep_all" : "keep_last"
       )
    << kv("depth", selection.depth)
  );
  return resolved;
}

// =============================================================================
// SubscriptionManager Implementation
// =============================================================================

SubscriptionManager::SubscriptionManager(rclcpp::Node::SharedPtr node)
    : node_(node) {}

SubscriptionManager::~SubscriptionManager() {
  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
}

bool SubscriptionManager::subscribe(
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallback callback
) {
  std::lock_guard<std::mutex> lock(mutex_);
  const SubscribeOptions effective_options =
    resolve_auto_qos_options(node_, topic_name, message_type, options);

  // Check if already subscribed
  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    AXON_LOG_WARN("Already subscribed to topic: " << kv("topic", topic_name));
    return true;
  }

  try {
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    // Create compression filter (if configured)
    std::shared_ptr<DepthCompressionFilter> compression_filter;
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      compression_filter = std::make_shared<DepthCompressionFilter>(*options.depth_compression);
      AXON_LOG_INFO(
        "Depth compression enabled for topic "
        << kv("topic", topic_name) << ": level=" << kv("level", options.depth_compression->level)
      );
    }
#else
    // Depth compression not available at compile time
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      AXON_LOG_WARN(
        "Depth compression requested for topic "
        << kv("topic", topic_name)
        << " but not enabled at build time. "
           "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable."
      );
    }
#endif

    // Create generic subscription using ROS2's built-in API
    // Capture compression_filter by value to avoid race condition with subscriptions_ map
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    auto* captured_filter = compression_filter.get();
#else
    void* captured_filter = nullptr;  // No compression filter available
#endif

    auto callback_group =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group;

    auto subscription = node_->create_generic_subscription(
      topic_name,
      message_type,
      effective_options.qos,
      [this, topic_name, message_type, callback, captured_filter](
        std::shared_ptr<rclcpp::SerializedMessage> msg
      ) {
        if (!callback) {
          return;
        }

        try {
          // Extract raw bytes from serialized message
          std::vector<uint8_t> data(
            msg->get_rcl_serialized_message().buffer,
            msg->get_rcl_serialized_message().buffer +
              msg->get_rcl_serialized_message().buffer_length
          );

          // Get current time as timestamp
          rclcpp::Time timestamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
          // Use captured filter pointer instead of accessing subscriptions_ map
          if (captured_filter) {
            static_cast<DepthCompressionFilter*>(captured_filter)
              ->filter_and_process(
                topic_name,
                message_type,
                data,
                timestamp.nanoseconds(),
                [&](
                  const std::string& filtered_topic,
                  const std::string& filtered_type,
                  const std::vector<uint8_t>& filtered_data,
                  uint64_t filtered_timestamp_ns
                ) {
                  // Convert nanosecond timestamp back to rclcpp::Time
                  rclcpp::Time filtered_timestamp(filtered_timestamp_ns);
                  callback(filtered_topic, filtered_type, filtered_data, filtered_timestamp);
                }
              );
          } else {
            // Direct callback
            callback(topic_name, message_type, data, timestamp);
          }
#else
          // Direct callback (no depth compression support)
          callback(topic_name, message_type, data, timestamp);
#endif

        } catch (const std::exception& e) {
          AXON_LOG_ERROR(
            "Failed to handle message on topic " << kv("topic", topic_name) << ": "
                                                 << kv("error", e.what())
          );
        }
      },
      subscription_options
    );

    if (!subscription) {
      AXON_LOG_ERROR("Failed to create subscription for: " << kv("topic", topic_name));
      return false;
    }

    // Store subscription info with callback and filter
    SubscriptionInfo info;
    info.subscription = subscription;
    info.callback_group = callback_group;
    info.callback = callback;
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    info.compression_filter = compression_filter;
#endif
    subscriptions_[topic_name] = std::move(info);

    AXON_LOG_INFO(
      "Subscribed to topic " << kv("topic", topic_name) << " (" << kv("type", message_type) << ")"
    );

    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR(
      "Exception subscribing to " << kv("topic", topic_name) << ": " << kv("error", e.what())
    );
    return false;
  }
}

bool SubscriptionManager::subscribe_v2(
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallbackV2 callback
) {
  std::lock_guard<std::mutex> lock(mutex_);
  const SubscribeOptions effective_options =
    resolve_auto_qos_options(node_, topic_name, message_type, options);

  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    AXON_LOG_WARN("Already subscribed to topic: " << kv("topic", topic_name));
    return true;
  }

  try {
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    std::shared_ptr<DepthCompressionFilter> compression_filter;
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      compression_filter = std::make_shared<DepthCompressionFilter>(*options.depth_compression);
      AXON_LOG_INFO(
        "Depth compression enabled (v2) for topic "
        << kv("topic", topic_name) << ": level=" << kv("level", options.depth_compression->level)
      );
    }
    auto* captured_filter = compression_filter.get();
#else
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      AXON_LOG_WARN(
        "Depth compression requested (v2) for " << kv("topic", topic_name)
                                                << " but not enabled at build time."
      );
    }
    void* captured_filter = nullptr;
#endif

    auto callback_group =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group;

    auto subscription = node_->create_generic_subscription(
      topic_name,
      message_type,
      effective_options.qos,
      [topic_name, message_type, callback, captured_filter](
        std::shared_ptr<rclcpp::SerializedMessage> msg
      ) {
        if (!callback || !msg) {
          return;
        }

        try {
          rclcpp::Time timestamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
          if (captured_filter) {
            // Compression path: we must materialize the bytes into a std::vector
            // before handing to the filter. The filter's output is re-wrapped into
            // a heap-allocated vector and adopted by the recorder (one copy total,
            // matching the previous pipeline).
            std::vector<uint8_t> data(
              msg->get_rcl_serialized_message().buffer,
              msg->get_rcl_serialized_message().buffer +
                msg->get_rcl_serialized_message().buffer_length
            );
            static_cast<DepthCompressionFilter*>(captured_filter)
              ->filter_and_process(
                topic_name,
                message_type,
                data,
                timestamp.nanoseconds(),
                [&](
                  const std::string& filtered_topic,
                  const std::string& filtered_type,
                  const std::vector<uint8_t>& filtered_data,
                  uint64_t filtered_timestamp_ns
                ) {
                  auto* holder = new std::vector<uint8_t>(filtered_data);
                  rclcpp::Time filtered_timestamp(filtered_timestamp_ns);
                  callback(
                    filtered_topic,
                    filtered_type,
                    holder->data(),
                    holder->size(),
                    filtered_timestamp,
                    +[](void* p) {
                      delete static_cast<std::vector<uint8_t>*>(p);
                    },
                    holder
                  );
                }
              );
            return;
          }
#endif

          // Copy the serialized payload into plugin-owned storage before
          // returning from the ROS callback. Holding the rclcpp
          // SerializedMessage in Axon's queues can exhaust DDS/rclcpp receive
          // buffers when recorder batching or writer backlog retains many
          // messages, throttling high-rate topics before they reach Axon.
          const auto& raw = msg->get_rcl_serialized_message();
          auto* holder = new std::vector<uint8_t>();
          if (raw.buffer != nullptr && raw.buffer_length > 0) {
            holder->assign(raw.buffer, raw.buffer + raw.buffer_length);
          }
          if (holder->empty()) {
            holder->resize(1);
          }
          callback(
            topic_name,
            message_type,
            holder->data(),
            raw.buffer_length,
            timestamp,
            +[](void* p) {
              delete static_cast<std::vector<uint8_t>*>(p);
            },
            holder
          );
        } catch (const std::exception& e) {
          AXON_LOG_ERROR(
            "Failed to handle message (v2) on topic " << kv("topic", topic_name) << ": "
                                                      << kv("error", e.what())
          );
        }
      },
      subscription_options
    );

    if (!subscription) {
      AXON_LOG_ERROR("Failed to create subscription (v2) for: " << kv("topic", topic_name));
      return false;
    }

    SubscriptionInfo info;
    info.subscription = subscription;
    info.callback_group = callback_group;
    // v2 does not retain the v1 std::function callback; that slot is unused here.
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    info.compression_filter = compression_filter;
#endif
    subscriptions_[topic_name] = std::move(info);

    AXON_LOG_INFO(
      "Subscribed (v2) to topic " << kv("topic", topic_name) << " (" << kv("type", message_type)
                                  << ")"
    );
    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR(
      "Exception subscribing (v2) to " << kv("topic", topic_name) << ": " << kv("error", e.what())
    );
    return false;
  }
}

bool SubscriptionManager::unsubscribe(const std::string& topic_name) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    AXON_LOG_WARN("Not subscribed to topic: " << kv("topic", topic_name));
    return false;
  }

  subscriptions_.erase(it);
  AXON_LOG_INFO("Unsubscribed from topic: " << kv("topic", topic_name));
  return true;
}

std::vector<std::string> SubscriptionManager::get_subscribed_topics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> topics;
  topics.reserve(subscriptions_.size());
  for (const auto& [topic, _] : subscriptions_) {
    topics.push_back(topic);
  }
  return topics;
}

}  // namespace ros2_plugin
