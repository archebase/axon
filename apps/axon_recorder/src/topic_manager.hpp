#ifndef AXON_RECORDER_TOPIC_MANAGER_HPP
#define AXON_RECORDER_TOPIC_MANAGER_HPP

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros_interface.hpp"

namespace axon {
namespace recorder {

/**
 * TopicManager handles ROS topic subscriptions for the recorder.
 *
 * This class is responsible for:
 * - Creating and managing subscriptions to ROS topics
 * - Routing received messages to callbacks
 * - Tracking subscription metadata (message types)
 * - Proper cleanup of subscriptions
 *
 * Extracted from RecorderNode to follow Single Responsibility Principle.
 *
 * Thread Safety:
 * - Subscription creation/removal should be called from a single thread
 * - The message callback can be invoked from multiple ROS executor threads
 */
class TopicManager {
public:
  /**
   * Callback type for received messages.
   * Parameters:
   * - topic: The topic name
   * - timestamp_ns: Receive timestamp in nanoseconds
   * - data: Serialized message data (ownership transferred)
   */
  using MessageCallback = std::function<
    void(const std::string& topic, int64_t timestamp_ns, std::vector<uint8_t>&& data)>;

  /**
   * Topic subscription info.
   */
  struct TopicInfo {
    std::string topic;
    std::string message_type;
    void* subscription_handle = nullptr;
  };

  /**
   * Construct a TopicManager.
   * @param ros_interface Non-owning pointer to the ROS interface
   */
  explicit TopicManager(RosInterface* ros_interface);

  ~TopicManager();

  // Non-copyable, non-movable
  TopicManager(const TopicManager&) = delete;
  TopicManager& operator=(const TopicManager&) = delete;
  TopicManager(TopicManager&&) = delete;
  TopicManager& operator=(TopicManager&&) = delete;

  /**
   * Subscribe to a topic with zero-copy message delivery.
   *
   * @param topic Topic name (e.g., "/camera/image_raw")
   * @param message_type Full message type (e.g., "sensor_msgs/Image")
   * @param callback Callback to invoke when messages are received
   * @param config Optional subscription configuration
   * @return true if subscription was created successfully
   */
  bool subscribe(
    const std::string& topic, const std::string& message_type, MessageCallback callback,
    const SubscriptionConfig& config = SubscriptionConfig::high_throughput()
  );

  /**
   * Unsubscribe from a specific topic.
   *
   * @param topic Topic name to unsubscribe from
   */
  void unsubscribe(const std::string& topic);

  /**
   * Unsubscribe from all topics.
   */
  void unsubscribe_all();

  /**
   * Check if subscribed to a topic.
   *
   * @param topic Topic name
   * @return true if currently subscribed to the topic
   */
  bool is_subscribed(const std::string& topic) const;

  /**
   * Get the number of active subscriptions.
   */
  size_t topic_count() const;

  /**
   * Get list of all subscribed topics.
   */
  std::vector<std::string> get_topics() const;

  /**
   * Get info for all subscribed topics.
   */
  std::vector<TopicInfo> get_topic_info() const;

  /**
   * Get the message type for a topic.
   *
   * @param topic Topic name
   * @return Message type, or empty string if not subscribed
   */
  std::string get_message_type(const std::string& topic) const;

private:
  RosInterface* ros_interface_;  // Non-owning

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicInfo> subscriptions_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_TOPIC_MANAGER_HPP
