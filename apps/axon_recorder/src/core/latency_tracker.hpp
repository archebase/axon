// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_LATENCY_TRACKER_HPP
#define AXON_RECORDER_LATENCY_TRACKER_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "percentile_calculator.hpp"

namespace axon {
namespace recorder {

/**
 * LatencyRecord - Record of end-to-end latency for a single message
 */
struct LatencyRecord {
  std::string topic;
  uint64_t publish_time_ns = 0;
  uint64_t receive_time_ns = 0;
  uint64_t enqueue_time_ns = 0;
  uint64_t dequeue_time_ns = 0;
  uint64_t write_time_ns = 0;
  uint64_t record_time_ns = 0;

  uint64_t latency_total_ns() const {
    if (record_time_ns == 0 || publish_time_ns == 0) {
      return 0;
    }
    return record_time_ns - publish_time_ns;
  }

  uint64_t latency_queue_ns() const {
    if (dequeue_time_ns == 0 || enqueue_time_ns == 0) {
      return 0;
    }
    return dequeue_time_ns - enqueue_time_ns;
  }

  uint64_t latency_process_ns() const {
    if (write_time_ns == 0 || dequeue_time_ns == 0) {
      return 0;
    }
    return write_time_ns - dequeue_time_ns;
  }

  uint64_t latency_receive_ns() const {
    if (receive_time_ns == 0 || publish_time_ns == 0) {
      return 0;
    }
    return receive_time_ns - publish_time_ns;
  }
};

/**
 * TopicLatencyStats - Latency statistics for a specific topic
 */
struct TopicLatencyStats {
  std::string topic;
  uint64_t message_count = 0;
  uint64_t latency_min_ns = 0;
  uint64_t latency_p50_ns = 0;
  uint64_t latency_p90_ns = 0;
  uint64_t latency_p95_ns = 0;
  uint64_t latency_p99_ns = 0;
  uint64_t latency_p999_ns = 0;
  uint64_t latency_max_ns = 0;
  double latency_mean_ns = 0;
  double latency_std_ns = 0;
  uint64_t anomaly_count_p99 = 0;
  uint64_t anomaly_count_1ms = 0;
  uint64_t anomaly_count_10ms = 0;
};

/**
 * GlobalLatencyStats - Global latency statistics across all topics
 */
struct GlobalLatencyStats {
  uint64_t total_messages = 0;
  uint64_t global_p50_ns = 0;
  uint64_t global_p90_ns = 0;
  uint64_t global_p99_ns = 0;
  uint64_t global_p999_ns = 0;
};

/**
 * LatencyTracker - Tracks end-to-end latency for messages
 *
 * Records latency at each stage:
 * 1. publish_time: Message published by source (from ROS header)
 * 2. receive_time: Received by plugin callback
 * 3. enqueue_time: Pushed to SPSC queue
 * 4. dequeue_time: Popped from queue by worker
 * 5. write_time: Written to MCAP
 * 6. record_time: Final record time (same as write_time)
 */
class LatencyTracker {
public:
  LatencyTracker();

  void record(const LatencyRecord& record);

  TopicLatencyStats get_topic_stats(const std::string& topic) const;

  GlobalLatencyStats get_global_stats() const;

  std::vector<std::string> get_topics() const;

  void reset();

  void reset_topic(const std::string& topic);

  size_t topic_count() const;

  struct Config {
    size_t max_samples_per_topic = 100000;
    uint64_t p99_anomaly_threshold_ns = 10000000;
  };

  void set_config(const Config& config);

  Config get_config() const;

private:
  struct PerTopicData {
    PercentileCalculator calculator;
    std::atomic<uint64_t> message_count{0};
    std::atomic<uint64_t> anomaly_count_p99{0};
    std::atomic<uint64_t> anomaly_count_1ms{0};
    std::atomic<uint64_t> anomaly_count_10ms{0};

    PerTopicData(size_t max_samples)
        : calculator(max_samples) {}
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<PerTopicData>> topic_data_;
  PercentileCalculator global_calculator_;
  Config config_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_LATENCY_TRACKER_HPP
