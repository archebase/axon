// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros1_subscription_wrapper.hpp"

#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
#include "depth_compression_filter.hpp"
#endif

// Define component name for logging
#define AXON_LOG_COMPONENT "ros1_subscription"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace ros1_plugin {

// =============================================================================
// SubscriptionManager Implementation
// =============================================================================

SubscriptionManager::SubscriptionManager(ros::NodeHandlePtr node_handle)
    : node_handle_(node_handle) {}

SubscriptionManager::~SubscriptionManager() {
  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
}

bool SubscriptionManager::subscribe(
  const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
  MessageCallback callback
) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if already subscribed
  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    AXON_LOG_WARN("Already subscribed to topic: " << kv("topic", topic_name));
    return true;
  }

  try {
    // Use topic_tools::ShapeShifter to subscribe to any message type
    // This is the ROS1 equivalent of ROS2's GenericSubscription
    auto subscriber = node_handle_->subscribe<topic_tools::ShapeShifter>(
      topic_name,
      queue_size,
      [topic_name,
       message_type,
       callback](const typename topic_tools::ShapeShifter::ConstPtr& msg) {
        if (!callback) {
          return;
        }

        try {
          // Get the message definition (type) from the ShapeShifter
          std::string actual_type = msg->getDataType();

          // Serialize the message to a byte array
          uint32_t serial_size = ros::serialization::serializationLength(*msg);
          std::vector<uint8_t> data(serial_size);

          ros::serialization::OStream stream(data.data(), serial_size);
          ros::serialization::serialize(stream, *msg);

          // Get current time as timestamp (nanoseconds)
          uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec());

          // Invoke callback with serialized data
          callback(topic_name, actual_type, data, timestamp);

        } catch (const std::exception& e) {
          AXON_LOG_ERROR(
            "Failed to handle message on topic " << kv("topic", topic_name) << ": "
                                                 << kv("error", e.what())
          );
        }
      }
    );

    if (!subscriber) {
      AXON_LOG_ERROR("Failed to create subscription for: " << kv("topic", topic_name));
      return false;
    }

    // Store subscription info with callback
    SubscriptionInfo info;
    info.subscriber = subscriber;
    info.callback = callback;
    info.message_type = message_type;
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

#ifdef AXON_ENABLE_DEPTH_COMPRESSION

bool SubscriptionManager::subscribe(
  const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
  MessageCallback callback, const std::optional<DepthCompressionConfig>& depth_compression
) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if already subscribed
  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    AXON_LOG_WARN("Already subscribed to topic: " << kv("topic", topic_name));
    return true;
  }

  // Create depth compression filter if enabled
  std::shared_ptr<DepthCompressionFilter> depth_filter;
  if (depth_compression && depth_compression->enabled) {
    depth_filter = std::make_shared<DepthCompressionFilter>(*depth_compression);
    AXON_LOG_INFO(
      "Depth compression enabled for topic: " << kv("topic", topic_name) << " (level: "
                                              << kv("level", depth_compression->level) << ")"
    );
  }

  try {
    // Use topic_tools::ShapeShifter to subscribe to any message type
    auto subscriber = node_handle_->subscribe<topic_tools::ShapeShifter>(
      topic_name,
      queue_size,
      [topic_name, message_type, callback, depth_filter](
        const typename topic_tools::ShapeShifter::ConstPtr& msg
      ) {
        if (!callback) {
          return;
        }

        try {
          // Get the message definition (type) from the ShapeShifter
          std::string actual_type = msg->getDataType();

          // Serialize the message to a byte array
          uint32_t serial_size = ros::serialization::serializationLength(*msg);
          std::vector<uint8_t> data(serial_size);

          ros::serialization::OStream stream(data.data(), serial_size);
          ros::serialization::serialize(stream, *msg);

          // Get current time as timestamp (nanoseconds)
          uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec());

          // Apply depth compression filter if available
          if (depth_filter) {
            depth_filter->filter_and_process(topic_name, actual_type, data, timestamp, callback);
          } else {
            // Invoke callback directly with serialized data
            callback(topic_name, actual_type, data, timestamp);
          }

        } catch (const std::exception& e) {
          AXON_LOG_ERROR(
            "Failed to handle message on topic " << kv("topic", topic_name) << ": "
                                                 << kv("error", e.what())
          );
        }
      }
    );

    if (!subscriber) {
      AXON_LOG_ERROR("Failed to create subscription for: " << kv("topic", topic_name));
      return false;
    }

    // Store subscription info with callback
    SubscriptionInfo info;
    info.subscriber = subscriber;
    info.callback = callback;
    info.message_type = message_type;
    info.depth_filter = depth_filter;
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

#endif  // AXON_ENABLE_DEPTH_COMPRESSION

// =============================================================================
// ABI v1.2 zero-copy subscribe
// =============================================================================
//
// Implementation notes:
//   - ShapeShifter does not expose the raw wire bytes directly, so we must
//     call ros::serialization::serialize into a buffer we own. That's the
//     same cost the v1 path paid; the difference is we now heap-allocate
//     that buffer and transfer ownership to the recorder instead of
//     returning a const reference the recorder has to copy out of.
//   - The holder type is a plain `std::vector<uint8_t>*` — simple, does not
//     require shared_ptr overhead, and deallocation is a single `delete`.
//   - release_fn must be safe to call from any thread. The lambda is a
//     stateless function pointer (note the `+[]` conversion), so it's
//     callable without capturing wrapper state. That means late releases
//     after SubscriptionManager teardown are still well-defined (the heap
//     vector lifetime is decoupled from the wrapper).
//   - When depth compression is active and the incoming frame triggers
//     the filter, we still allocate a fresh buffer for the filter output
//     and hand that off; the recorder pipeline treats it identically.
bool SubscriptionManager::subscribe_v2(
  const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
  MessageCallbackV2 callback, const std::optional<DepthCompressionConfig>& depth_compression
) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    AXON_LOG_WARN("Already subscribed to topic: " << kv("topic", topic_name));
    return true;
  }

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
  std::shared_ptr<DepthCompressionFilter> depth_filter;
  if (depth_compression && depth_compression->enabled) {
    depth_filter = std::make_shared<DepthCompressionFilter>(*depth_compression);
    AXON_LOG_INFO(
      "Depth compression enabled (v2) for topic: " << kv("topic", topic_name) << " (level: "
                                                   << kv("level", depth_compression->level) << ")"
    );
  }
#else
  if (depth_compression && depth_compression->enabled) {
    AXON_LOG_WARN(
      "Depth compression requested (v2) for "
      << kv("topic", topic_name)
      << " but not enabled at build time. "
         "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable."
    );
  }
#endif

  try {
    auto subscriber = node_handle_->subscribe<topic_tools::ShapeShifter>(
      topic_name,
      queue_size,
      [topic_name,
       message_type,
       callback
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
       ,
       depth_filter
#endif
    ](const typename topic_tools::ShapeShifter::ConstPtr& msg) {
        if (!callback || !msg) {
          return;
        }

        try {
          std::string actual_type = msg->getDataType();

          // Serialize ShapeShifter payload into a heap-owned buffer. This
          // is the buffer whose ownership we hand off to the recorder.
          uint32_t serial_size = ros::serialization::serializationLength(*msg);
          auto* holder = new std::vector<uint8_t>(serial_size);
          ros::serialization::OStream stream(holder->data(), serial_size);
          ros::serialization::serialize(stream, *msg);

          uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec());

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
          if (depth_filter) {
            // Filter path: feed a const-ref view to the filter, let it emit
            // the compressed frame, then we transfer ownership of a fresh
            // buffer containing the filter output.
            depth_filter->filter_and_process(
              topic_name,
              actual_type,
              *holder,
              timestamp,
              [&callback](
                const std::string& out_topic,
                const std::string& out_type,
                const std::vector<uint8_t>& out_data,
                uint64_t out_ts
              ) {
                auto* out_holder = new std::vector<uint8_t>(out_data);
                callback(
                  out_topic,
                  out_type,
                  out_holder->data(),
                  out_holder->size(),
                  out_ts,
                  +[](void* p) {
                    delete static_cast<std::vector<uint8_t>*>(p);
                  },
                  out_holder
                );
              }
            );
            // We no longer need the original pre-filter buffer.
            delete holder;
            return;
          }
#endif

          // Zero-copy pass-through: hand the recorder raw bytes + a
          // release trampoline that frees the std::vector holder.
          callback(
            topic_name,
            actual_type,
            holder->data(),
            holder->size(),
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
      }
    );

    if (!subscriber) {
      AXON_LOG_ERROR("Failed to create subscription (v2) for: " << kv("topic", topic_name));
      return false;
    }

    SubscriptionInfo info;
    info.subscriber = subscriber;
    info.callback_v2 = callback;
    info.message_type = message_type;
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    info.depth_filter = depth_filter;
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

}  // namespace ros1_plugin
