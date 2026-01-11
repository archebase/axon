#ifndef AXON_ROS_INTERFACE_HPP
#define AXON_ROS_INTERFACE_HPP

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace axon {
namespace recorder {

// Forward declarations
class RosNodeHandle;

/**
 * Zero-copy serialized message data for high-performance recording
 *
 * This structure allows ownership transfer of serialized message bytes,
 * avoiding expensive memory copies in the subscription callback hot path.
 */
struct SerializedMessageData {
  std::vector<uint8_t> data;  // Raw serialized message bytes (owned)
  int64_t receive_time_ns;    // Receive timestamp in nanoseconds

  SerializedMessageData()
      : receive_time_ns(0) {}

  SerializedMessageData(std::vector<uint8_t>&& d, int64_t ts)
      : data(std::move(d))
      , receive_time_ns(ts) {}

  // Move-only semantics for zero-copy
  SerializedMessageData(SerializedMessageData&&) = default;
  SerializedMessageData& operator=(SerializedMessageData&&) = default;
  SerializedMessageData(const SerializedMessageData&) = delete;
  SerializedMessageData& operator=(const SerializedMessageData&) = delete;

  size_t size() const {
    return data.size();
  }
  bool empty() const {
    return data.empty();
  }
};

/**
 * QoS profile for subscriptions
 */
enum class QosProfile {
  Default,        // Default reliable QoS
  SensorData,     // Best-effort, small history - for high-frequency sensors
  SystemDefault,  // Use system/DDS defaults
  HighThroughput  // Optimized for high message rates
};

/**
 * Subscription configuration for fine-tuned performance
 */
struct SubscriptionConfig {
  QosProfile qos = QosProfile::HighThroughput;
  size_t history_depth = 2000;               // DDS history depth
  bool use_dedicated_callback_group = true;  // Enable per-subscription parallelism

  static SubscriptionConfig high_throughput() {
    SubscriptionConfig config;
    config.qos = QosProfile::HighThroughput;
    config.history_depth = 2000;
    config.use_dedicated_callback_group = true;
    return config;
  }

  static SubscriptionConfig sensor_data() {
    SubscriptionConfig config;
    config.qos = QosProfile::SensorData;
    config.history_depth = 100;
    config.use_dedicated_callback_group = true;
    return config;
  }
};

/**
 * Abstract interface for ROS operations (ROS 1 and ROS 2 compatible)
 *
 * This interface abstracts away ROS version differences, allowing the same
 * recorder code to work with both ROS 1 and ROS 2. The appropriate implementation
 * is selected at compile time via AXON_ROS1 or AXON_ROS2 macros.
 */
class RosInterface {
public:
  virtual ~RosInterface() = default;

  /**
   * Initialize ROS (equivalent to ros::init or rclcpp::init)
   */
  virtual bool init(int argc, char** argv, const std::string& node_name) = 0;

  /**
   * Shutdown ROS
   */
  virtual void shutdown() = 0;

  /**
   * Check if ROS is running
   */
  virtual bool ok() const = 0;

  /**
   * Get the node handle (type-erased)
   */
  virtual void* get_node_handle() = 0;

  /**
   * Subscribe to a topic (legacy callback - may copy data)
   * @param topic Topic name
   * @param message_type ROS message type (e.g., "sensor_msgs/Image")
   * @param callback Callback function (void* is the message pointer)
   * @return Subscription handle (can be used to unsubscribe)
   */
  virtual void* subscribe(
    const std::string& topic, const std::string& message_type,
    std::function<void(const void*)> callback
  ) = 0;

  /**
   * Subscribe to a topic with zero-copy callback (high-performance)
   *
   * This method provides ownership transfer of serialized message data,
   * avoiding expensive memory copies. Use this for high-throughput recording.
   *
   * @param topic Topic name
   * @param message_type ROS message type (e.g., "sensor_msgs/Image")
   * @param callback Callback receiving ownership of serialized data
   * @param config Subscription configuration for QoS and parallelism
   * @return Subscription handle (can be used to unsubscribe)
   */
  virtual void* subscribe_zero_copy(
    const std::string& topic, const std::string& message_type,
    std::function<void(SerializedMessageData&&)> callback,
    const SubscriptionConfig& config = SubscriptionConfig::high_throughput()
  ) = 0;

  /**
   * Unsubscribe from a topic
   */
  virtual void unsubscribe(void* subscription_handle) = 0;

  /**
   * Advertise a service
   * @param service_name Service name
   * @param service_type Service type (e.g., "std_srvs/Empty")
   * @param callback Service callback (request, response)
   * @return Service handle
   */
  virtual void* advertise_service(
    const std::string& service_name, const std::string& service_type,
    std::function<bool(const void*, void*)> callback
  ) = 0;

  /**
   * Spin once (process callbacks)
   */
  virtual void spin_once() = 0;

  /**
   * Spin (blocking)
   */
  virtual void spin() = 0;

  /**
   * Get current time in nanoseconds
   */
  virtual int64_t now_nsec() const = 0;

  /**
   * Log at different levels
   */
  virtual void log_info(const std::string& message) const = 0;
  virtual void log_warn(const std::string& message) const = 0;
  virtual void log_error(const std::string& message) const = 0;
  virtual void log_debug(const std::string& message) const = 0;

  /**
   * Get the message definition for a message type
   *
   * Returns the message definition string in ROS format (ros1msg or ros2msg).
   * This is used for MCAP schema registration.
   *
   * @param message_type Full message type name (e.g., "sensor_msgs/msg/Image")
   * @return Message definition string, or empty string if not found
   */
  virtual std::string get_message_definition(const std::string& message_type) const = 0;
};

/**
 * Factory for creating ROS interface instances
 */
class RosInterfaceFactory {
public:
  /**
   * Create ROS interface instance
   * Returns the appropriate implementation based on compile-time flags
   */
  static std::unique_ptr<RosInterface> create();
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_ROS_INTERFACE_HPP
