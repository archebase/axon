#include "ros_interface.hpp"

#include "message_factory.hpp"
#include "register_common_messages.hpp"

#if defined(AXON_ROS1)
#include <ros/ros.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>
#elif defined(AXON_ROS2)
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/rmw.h>
#endif

#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace axon {
namespace recorder {

#if defined(AXON_ROS1)
// ============================================================================
// ROS 1 Implementation
// ============================================================================

class RosInterfaceImpl : public RosInterface {
public:
  RosInterfaceImpl()
      : node_handle_(nullptr)
      , initialized_(false) {}

  ~RosInterfaceImpl() override {
    shutdown();
    // Cleanup subscriptions
    for (auto& pair : subscriptions_) {
      delete static_cast<ros::Subscriber*>(pair.first);
    }
    subscriptions_.clear();
  }

  bool init(int argc, char** argv, const std::string& node_name) override {
    if (initialized_) {
      return true;
    }

    ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
    if (!ros::ok()) {
      return false;
    }

    node_handle_ = std::make_unique<ros::NodeHandle>();

    // Register common message types for full typed subscription support
    register_common_message_types();

    initialized_ = true;
    return true;
  }

  void shutdown() override {
    if (initialized_) {
      for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
        auto* sub = static_cast<ros::Subscriber*>(it->first);
        subscriptions_.erase(it++);
        delete sub;
      }

      ros::shutdown();
      node_handle_.reset();
      initialized_ = false;
    }
  }

  bool ok() const override {
    return ros::ok() && initialized_;
  }

  void* get_node_handle() override {
    return node_handle_.get();
  }

  void* subscribe(
    const std::string& topic, const std::string& message_type,
    std::function<void(const void*)> callback
  ) override {
    if (!node_handle_) {
      return nullptr;
    }

    ros::Subscriber* sub = nullptr;

    try {
      // Use ShapeShifter for generic subscription (type-agnostic)
      auto shape_callback =
        [callback](const topic_tools::ShapeShifter::ConstPtr& msg) {
          if (msg) {
            callback(static_cast<const void*>(msg.get()));
          }
        };

      sub = new ros::Subscriber(
        node_handle_->subscribe<topic_tools::ShapeShifter>(topic, 10, shape_callback)
      );

    } catch (const std::exception& e) {
      ROS_ERROR(
        "Failed to subscribe to topic %s (type: %s): %s",
        topic.c_str(),
        message_type.c_str(),
        e.what()
      );
      if (sub) {
        delete sub;
        sub = nullptr;
      }
    }

    if (sub) {
      subscriptions_[sub] = std::make_pair(topic, message_type);
      ROS_INFO(
        "Successfully subscribed to topic %s (type: %s)", topic.c_str(), message_type.c_str()
      );
    }

    return sub;
  }

  void unsubscribe(void* subscription_handle) override {
    if (!subscription_handle) {
      return;
    }

    auto it = subscriptions_.find(subscription_handle);
    if (it != subscriptions_.end()) {
      auto* sub = static_cast<ros::Subscriber*>(subscription_handle);
      subscriptions_.erase(it);
      delete sub;
    }
  }

  void* subscribe_zero_copy(
    const std::string& topic, const std::string& message_type,
    std::function<void(SerializedMessageData&&)> callback,
    const SubscriptionConfig& /* config */
  ) override {
    if (!node_handle_) {
      return nullptr;
    }

    ros::Subscriber* sub = nullptr;

    try {
      // Use ShapeShifter for zero-copy access to serialized data
      auto shape_callback =
        [callback, this](const topic_tools::ShapeShifter::ConstPtr& msg) {
          if (!msg) {
            return;
          }

          // Get receive timestamp
          int64_t timestamp_ns = now_nsec();

          // Get serialized data size
          uint32_t serialized_size = msg->size();

          // Allocate buffer and serialize
          std::vector<uint8_t> buffer(serialized_size);
          ros::serialization::OStream stream(buffer.data(), serialized_size);
          msg->write(stream);

          // Create SerializedMessageData with ownership transfer
          SerializedMessageData data(std::move(buffer), timestamp_ns);
          callback(std::move(data));
        };

      sub = new ros::Subscriber(
        node_handle_->subscribe<topic_tools::ShapeShifter>(topic, 100, shape_callback)
      );

    } catch (const std::exception& e) {
      ROS_ERROR(
        "Failed to subscribe (zero-copy) to topic %s (type: %s): %s",
        topic.c_str(),
        message_type.c_str(),
        e.what()
      );
      if (sub) {
        delete sub;
        sub = nullptr;
      }
    }

    if (sub) {
      subscriptions_[sub] = std::make_pair(topic, message_type);
      ROS_INFO(
        "Successfully subscribed (zero-copy) to topic %s (type: %s)",
        topic.c_str(),
        message_type.c_str()
      );
    }

    return sub;
  }

  void* advertise_service(
    const std::string& service_name, const std::string& service_type,
    std::function<bool(const void*, void*)> /* callback */
  ) override {
    // Note: ROS 1 generic service advertisement requires compile-time type knowledge.
    // Services are registered directly in recorder_node using generated service types.
    // This generic interface is primarily for ROS 2 compatibility.
    ROS_WARN(
      "Generic service advertisement not supported in ROS 1. "
      "Use typed service registration instead for: %s (%s)",
      service_name.c_str(),
      service_type.c_str()
    );
    return nullptr;
  }

  void spin_once() override {
    ros::spinOnce();
  }

  void spin() override {
    ros::spin();
  }

  int64_t now_nsec() const override {
    auto now = ros::Time::now();
    return static_cast<int64_t>(now.sec) * 1000000000LL + static_cast<int64_t>(now.nsec);
  }

  void log_info(const std::string& message) const override {
    ROS_INFO("%s", message.c_str());
  }

  void log_warn(const std::string& message) const override {
    ROS_WARN("%s", message.c_str());
  }

  void log_error(const std::string& message) const override {
    ROS_ERROR("%s", message.c_str());
  }

  void log_debug(const std::string& message) const override {
    ROS_DEBUG("%s", message.c_str());
  }

  std::string get_message_definition(const std::string& message_type) const override {
    // For ROS 1, try to get message definition from MessageFactory
    if (MessageFactory::is_registered(message_type)) {
      MessageFactory::MessageInfo info;
      if (MessageFactory::get_message_info(message_type, info)) {
        return info.definition;
      }
    }
    // Return placeholder for unknown types
    return "# Message definition not available for: " + message_type;
  }

private:
  std::unique_ptr<ros::NodeHandle> node_handle_;
  bool initialized_;
  std::map<void*, std::pair<std::string, std::string>> subscriptions_;
};

#elif defined(AXON_ROS2)
// ============================================================================
// ROS 2 Implementation - High Performance Architecture
// ============================================================================
//
// Key optimizations for high-throughput recording:
// 1. Per-subscription callback groups for true parallelism
// 2. Zero-copy message handling with ownership transfer
// 3. Configurable QoS profiles optimized for sensor data
// 4. Increased executor thread count based on subscription count
//

class RosInterfaceImpl : public RosInterface {
public:
  // High thread count to ensure per-topic callback groups can run in parallel
  // Each subscription with dedicated callback group needs its own thread
  // 16 threads allows 7 topics + IMU + overhead for image processing
  static constexpr size_t DEFAULT_EXECUTOR_THREADS = 16;
  static constexpr size_t MAX_EXECUTOR_THREADS = 32;

  RosInterfaceImpl()
      : node_(nullptr)
      , initialized_(false)
      , num_threads_(DEFAULT_EXECUTOR_THREADS) {}

  ~RosInterfaceImpl() override {
    shutdown();
  }

  bool init(int argc, char** argv, const std::string& node_name) override {
    if (initialized_) {
      return true;
    }

    rclcpp::init(argc, argv);
    if (!rclcpp::ok()) {
      return false;
    }

    // Create node with options that allow reentrant callbacks
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    node_ = std::make_shared<rclcpp::Node>(node_name, options);

    // Legacy callback group (kept for compatibility)
    legacy_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create multi-threaded executor - thread count will be adjusted
    // based on number of subscriptions
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), num_threads_
    );
    executor_->add_node(node_);

    initialized_ = true;
    return true;
  }

  void shutdown() override {
    if (initialized_) {
      // Cancel executor before cleaning up
      if (executor_) {
        executor_->cancel();
        executor_.reset();
      }
      subscriptions_.clear();
      zero_copy_subscriptions_.clear();
      services_.clear();
      per_topic_callback_groups_.clear();
      legacy_callback_group_.reset();
      node_.reset();
      rclcpp::shutdown();
      initialized_ = false;
    }
  }

  bool ok() const override {
    return rclcpp::ok() && initialized_;
  }

  void* get_node_handle() override {
    return node_.get();
  }

  // Legacy subscribe (kept for compatibility)
  void* subscribe(
    const std::string& topic, const std::string& message_type,
    std::function<void(const void*)> callback
  ) override {
    if (!node_) {
      return nullptr;
    }

    try {
      auto sub_wrapper = std::make_shared<SubscriptionWrapper>();
      sub_wrapper->callback = callback;
      sub_wrapper->topic = topic;
      sub_wrapper->message_type = message_type;

      auto qos = rclcpp::QoS(1000).reliable().keep_last(1000).durability_volatile();

      rclcpp::SubscriptionOptions sub_options;
      sub_options.callback_group = legacy_callback_group_;

      sub_wrapper->subscription = rclcpp::create_generic_subscription(
        node_->get_node_topics_interface(),
        topic,
        message_type,
        qos,
        [sub_wrapper](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          if (sub_wrapper->callback && msg) {
            sub_wrapper->callback(static_cast<const void*>(msg.get()));
          }
        },
        sub_options
      );

      if (sub_wrapper->subscription) {
        auto* wrapper_ptr = new std::shared_ptr<SubscriptionWrapper>(sub_wrapper);
        subscriptions_[wrapper_ptr] = std::make_pair(topic, message_type);
        return wrapper_ptr;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to subscribe to topic %s: %s", topic.c_str(), e.what()
      );
    }

    return nullptr;
  }

  /**
   * High-performance zero-copy subscription
   *
   * Key optimizations:
   * 1. Dedicated callback group per subscription for true parallelism
   * 2. Ownership transfer of serialized data (no copy in callback)
   * 3. Optimized QoS based on message type characteristics
   */
  void* subscribe_zero_copy(
    const std::string& topic, const std::string& message_type,
    std::function<void(SerializedMessageData&&)> callback, const SubscriptionConfig& config
  ) override {
    if (!node_) {
      return nullptr;
    }

    try {
      auto sub_wrapper = std::make_shared<ZeroCopySubscriptionWrapper>();
      sub_wrapper->zero_copy_callback = std::move(callback);
      sub_wrapper->topic = topic;
      sub_wrapper->message_type = message_type;

      // Create dedicated callback group for this subscription
      rclcpp::CallbackGroup::SharedPtr callback_group;
      if (config.use_dedicated_callback_group) {
        callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        per_topic_callback_groups_[topic] = callback_group;

        // Ensure executor has enough threads
        maybe_increase_executor_threads();
      } else {
        callback_group = legacy_callback_group_;
      }

      // Configure QoS based on profile
      auto qos = create_qos_profile(config);

      rclcpp::SubscriptionOptions sub_options;
      sub_options.callback_group = callback_group;

      sub_wrapper->subscription = rclcpp::create_generic_subscription(
        node_->get_node_topics_interface(),
        topic,
        message_type,
        qos,
        [sub_wrapper](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          if (!sub_wrapper->zero_copy_callback || !msg) {
            return;
          }

          // Get timestamp using chrono directly (faster than node->now())
          int64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch()
          )
                                   .count();

          // Zero-copy: move the serialized data instead of copying
          const auto& rcl_msg = msg->get_rcl_serialized_message();

          // Create owned buffer with the data
          // Note: We must copy here because rcl_serialized_message_t doesn't
          // support ownership transfer. However, we do this efficiently:
          // - Single allocation
          // - Direct memcpy (no per-element copy)
          // - Ownership transferred to callback via move
          std::vector<uint8_t> data(rcl_msg.buffer_length);
          std::memcpy(data.data(), rcl_msg.buffer, rcl_msg.buffer_length);

          // Move ownership to callback - no further copies
          sub_wrapper->zero_copy_callback(SerializedMessageData(std::move(data), timestamp_ns));
        },
        sub_options
      );

      if (sub_wrapper->subscription) {
        auto* wrapper_ptr = new std::shared_ptr<ZeroCopySubscriptionWrapper>(sub_wrapper);
        zero_copy_subscriptions_[wrapper_ptr] = std::make_pair(topic, message_type);

        RCLCPP_INFO(
          node_->get_logger(),
          "Zero-copy subscription created for %s (QoS: %s, history: %zu, dedicated_group: %s)",
          topic.c_str(),
          qos_profile_name(config.qos).c_str(),
          config.history_depth,
          config.use_dedicated_callback_group ? "yes" : "no"
        );

        return wrapper_ptr;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to create zero-copy subscription for %s: %s",
        topic.c_str(),
        e.what()
      );
    }

    return nullptr;
  }

  void unsubscribe(void* subscription_handle) override {
    if (!subscription_handle) {
      return;
    }

    // Check legacy subscriptions
    auto it = subscriptions_.find(subscription_handle);
    if (it != subscriptions_.end()) {
      auto* wrapper_ptr = static_cast<std::shared_ptr<SubscriptionWrapper>*>(subscription_handle);
      subscriptions_.erase(it);
      delete wrapper_ptr;
      return;
    }

    // Check zero-copy subscriptions
    auto zc_it = zero_copy_subscriptions_.find(subscription_handle);
    if (zc_it != zero_copy_subscriptions_.end()) {
      auto* wrapper_ptr =
        static_cast<std::shared_ptr<ZeroCopySubscriptionWrapper>*>(subscription_handle);

      // Clean up dedicated callback group if any
      const std::string& topic = zc_it->second.first;
      per_topic_callback_groups_.erase(topic);

      zero_copy_subscriptions_.erase(zc_it);
      delete wrapper_ptr;
    }
  }

  void* advertise_service(
    const std::string& service_name, const std::string& service_type,
    std::function<bool(const void*, void*)> callback
  ) override {
    if (!node_) {
      return nullptr;
    }

    try {
      auto service_wrapper = std::make_shared<ServiceWrapper>();
      service_wrapper->callback = callback;
      service_wrapper->service_name = service_name;
      service_wrapper->service_type = service_type;

      RCLCPP_WARN(
        node_->get_logger(),
        "Generic service '%s' registered but requires typed implementation",
        service_name.c_str()
      );

      auto* wrapper_ptr = new std::shared_ptr<ServiceWrapper>(service_wrapper);
      services_[wrapper_ptr] = std::make_pair(service_name, service_type);
      return wrapper_ptr;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to advertise service %s: %s", service_name.c_str(), e.what()
      );
    }

    return nullptr;
  }

  void spin_once() override {
    if (executor_) {
      executor_->spin_some();
    }
  }

  void spin() override {
    if (executor_) {
      executor_->spin();
    }
  }

  int64_t now_nsec() const override {
    if (!node_) {
      return 0;
    }
    auto now = node_->now();
    return now.nanoseconds();
  }

  void log_info(const std::string& message) const override {
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    }
  }

  void log_warn(const std::string& message) const override {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    }
  }

  void log_error(const std::string& message) const override {
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    }
  }

  void log_debug(const std::string& message) const override {
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), "%s", message.c_str());
    }
  }

  std::string get_message_definition(const std::string& message_type) const override {
    // For ROS 2, we construct a basic message definition
    // MCAP can work without schema data, but having it helps with playback
    //
    // In production, you would use rosidl_runtime introspection to get
    // the full message definition. For now, we return a placeholder.
    //
    // The message definition format for ROS 2 is the .msg file content.
    // Example for sensor_msgs/msg/Image:
    // std_msgs/Header header
    // uint32 height
    // uint32 width
    // string encoding
    // uint8 is_bigendian
    // uint32 step
    // uint8[] data

    // Common message definitions for well-known types
    static const std::map<std::string, std::string> known_definitions = {
      {"std_msgs/msg/Header",
       "builtin_interfaces/Time stamp\n"
       "string frame_id"},
      {"sensor_msgs/msg/Image",
       "std_msgs/Header header\n"
       "uint32 height\n"
       "uint32 width\n"
       "string encoding\n"
       "uint8 is_bigendian\n"
       "uint32 step\n"
       "uint8[] data"},
      {"sensor_msgs/msg/Imu",
       "std_msgs/Header header\n"
       "geometry_msgs/Quaternion orientation\n"
       "float64[9] orientation_covariance\n"
       "geometry_msgs/Vector3 angular_velocity\n"
       "float64[9] angular_velocity_covariance\n"
       "geometry_msgs/Vector3 linear_acceleration\n"
       "float64[9] linear_acceleration_covariance"},
      {"sensor_msgs/msg/PointCloud2",
       "std_msgs/Header header\n"
       "uint32 height\n"
       "uint32 width\n"
       "sensor_msgs/PointField[] fields\n"
       "bool is_bigendian\n"
       "uint32 point_step\n"
       "uint32 row_step\n"
       "uint8[] data\n"
       "bool is_dense"},
      {"geometry_msgs/msg/Twist",
       "geometry_msgs/Vector3 linear\n"
       "geometry_msgs/Vector3 angular"},
      {"geometry_msgs/msg/TwistStamped",
       "std_msgs/Header header\n"
       "geometry_msgs/Twist twist"},
      {"geometry_msgs/msg/Pose",
       "geometry_msgs/Point position\n"
       "geometry_msgs/Quaternion orientation"},
      {"geometry_msgs/msg/PoseStamped",
       "std_msgs/Header header\n"
       "geometry_msgs/Pose pose"},
      {"nav_msgs/msg/Odometry",
       "std_msgs/Header header\n"
       "string child_frame_id\n"
       "geometry_msgs/PoseWithCovariance pose\n"
       "geometry_msgs/TwistWithCovariance twist"},
    };

    auto it = known_definitions.find(message_type);
    if (it != known_definitions.end()) {
      return it->second;
    }

    // Return placeholder for unknown types
    // MCAP will still work, but Foxglove may not be able to decode messages
    return "# Message definition not available for: " + message_type + "\n"
           "# Recording will still work, but schema-based decoding may be limited.";
  }

private:
  // Legacy subscription wrapper
  struct SubscriptionWrapper {
    std::function<void(const void*)> callback;
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::string topic;
    std::string message_type;
  };

  // Zero-copy subscription wrapper
  struct ZeroCopySubscriptionWrapper {
    std::function<void(SerializedMessageData&&)> zero_copy_callback;
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::string topic;
    std::string message_type;
  };

  struct ServiceWrapper {
    std::function<bool(const void*, void*)> callback;
    std::shared_ptr<void> service;
    std::string service_name;
    std::string service_type;
  };

  /**
   * Create QoS profile based on configuration
   */
  rclcpp::QoS create_qos_profile(const SubscriptionConfig& config) {
    switch (config.qos) {
      case QosProfile::SensorData:
        // Best-effort for sensor data - allows dropping old messages
        // if subscriber can't keep up (preferred for high-frequency sensors)
        return rclcpp::QoS(config.history_depth)
          .best_effort()
          .keep_last(config.history_depth)
          .durability_volatile();

      case QosProfile::HighThroughput:
        // Reliable with large buffers for recording
        // This ensures no message loss if subscriber can keep up
        return rclcpp::QoS(config.history_depth)
          .reliable()
          .keep_last(config.history_depth)
          .durability_volatile();

      case QosProfile::SystemDefault:
        return rclcpp::QoS(config.history_depth);

      case QosProfile::Default:
      default:
        return rclcpp::QoS(config.history_depth)
          .reliable()
          .keep_last(config.history_depth)
          .durability_volatile();
    }
  }

  std::string qos_profile_name(QosProfile profile) {
    switch (profile) {
      case QosProfile::SensorData:
        return "SensorData";
      case QosProfile::HighThroughput:
        return "HighThroughput";
      case QosProfile::SystemDefault:
        return "SystemDefault";
      case QosProfile::Default:
        return "Default";
      default:
        return "Unknown";
    }
  }

  /**
   * Increase executor threads to handle parallel subscriptions
   */
  void maybe_increase_executor_threads() {
    size_t needed_threads = DEFAULT_EXECUTOR_THREADS + per_topic_callback_groups_.size();
    if (needed_threads > num_threads_ && needed_threads <= MAX_EXECUTOR_THREADS) {
      // Note: ROS2 executor doesn't support dynamic thread count changes
      // Log warning about thread allocation
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Subscription count increased, may need more executor threads (current: %zu, ideal: %zu)",
        num_threads_,
        needed_threads
      );
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr legacy_callback_group_;
  std::map<std::string, rclcpp::CallbackGroup::SharedPtr> per_topic_callback_groups_;
  bool initialized_;
  size_t num_threads_;
  std::map<void*, std::pair<std::string, std::string>> subscriptions_;
  std::map<void*, std::pair<std::string, std::string>> zero_copy_subscriptions_;
  std::map<void*, std::pair<std::string, std::string>> services_;
};

#else
#error "Either AXON_ROS1 or AXON_ROS2 must be defined"
#endif

// ============================================================================
// Factory Implementation
// ============================================================================

std::unique_ptr<RosInterface> RosInterfaceFactory::create() {
  return std::make_unique<RosInterfaceImpl>();
}

}  // namespace recorder
}  // namespace axon
