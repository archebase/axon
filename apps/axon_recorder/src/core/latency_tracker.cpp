// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "latency_tracker.hpp"

#include <algorithm>

namespace axon {
namespace recorder {

LatencyTracker::LatencyTracker() : global_calculator_(100000) {}

void LatencyTracker::record(const LatencyRecord& record) {
  uint64_t total_latency = record.latency_total_ns();
  if (total_latency == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_data_.find(record.topic);
  if (it == topic_data_.end()) {
    auto data = std::make_shared<PerTopicData>(config_.max_samples_per_topic);
    it = topic_data_.emplace(record.topic, data).first;
  }

  auto& data = it->second;
  data->calculator.add(total_latency);
  data->message_count.fetch_add(1, std::memory_order_relaxed);
  global_calculator_.add(total_latency);

  if (total_latency > config_.p99_anomaly_threshold_ns) {
    data->anomaly_count_p99.fetch_add(1, std::memory_order_relaxed);
  }
  if (total_latency > 1000000) {
    data->anomaly_count_1ms.fetch_add(1, std::memory_order_relaxed);
  }
  if (total_latency > 10000000) {
    data->anomaly_count_10ms.fetch_add(1, std::memory_order_relaxed);
  }
}

TopicLatencyStats LatencyTracker::get_topic_stats(const std::string& topic) const {
  TopicLatencyStats stats;
  stats.topic = topic;

  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_data_.find(topic);
  if (it == topic_data_.end()) {
    return stats;
  }

  auto& data = it->second;
  stats.message_count = data->message_count.load(std::memory_order_relaxed);
  stats.anomaly_count_p99 = data->anomaly_count_p99.load(std::memory_order_relaxed);
  stats.anomaly_count_1ms = data->anomaly_count_1ms.load(std::memory_order_relaxed);
  stats.anomaly_count_10ms = data->anomaly_count_10ms.load(std::memory_order_relaxed);

  auto percentiles = data->calculator.get_percentiles();
  stats.latency_min_ns = percentiles.min_ns;
  stats.latency_p50_ns = percentiles.p50_ns;
  stats.latency_p90_ns = percentiles.p90_ns;
  stats.latency_p95_ns = percentiles.p95_ns;
  stats.latency_p99_ns = percentiles.p99_ns;
  stats.latency_p999_ns = percentiles.p999_ns;
  stats.latency_max_ns = percentiles.max_ns;
  stats.latency_mean_ns = percentiles.mean_ns;
  stats.latency_std_ns = percentiles.std_dev_ns;

  return stats;
}

GlobalLatencyStats LatencyTracker::get_global_stats() const {
  GlobalLatencyStats stats;

  std::lock_guard<std::mutex> lock(mutex_);

  stats.total_messages = static_cast<uint64_t>(global_calculator_.size());

  auto percentiles = global_calculator_.get_percentiles();
  stats.global_p50_ns = percentiles.p50_ns;
  stats.global_p90_ns = percentiles.p90_ns;
  stats.global_p99_ns = percentiles.p99_ns;
  stats.global_p999_ns = percentiles.p999_ns;

  return stats;
}

std::vector<std::string> LatencyTracker::get_topics() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> topics;
  topics.reserve(topic_data_.size());

  for (const auto& [topic, _] : topic_data_) {
    topics.push_back(topic);
  }

  return topics;
}

void LatencyTracker::reset() {
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto& [_, data] : topic_data_) {
    data->calculator.reset();
    data->message_count.store(0, std::memory_order_relaxed);
    data->anomaly_count_p99.store(0, std::memory_order_relaxed);
    data->anomaly_count_1ms.store(0, std::memory_order_relaxed);
    data->anomaly_count_10ms.store(0, std::memory_order_relaxed);
  }

  global_calculator_.reset();
}

void LatencyTracker::reset_topic(const std::string& topic) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_data_.find(topic);
  if (it != topic_data_.end()) {
    it->second->calculator.reset();
    it->second->message_count.store(0, std::memory_order_relaxed);
    it->second->anomaly_count_p99.store(0, std::memory_order_relaxed);
    it->second->anomaly_count_1ms.store(0, std::memory_order_relaxed);
    it->second->anomaly_count_10ms.store(0, std::memory_order_relaxed);
  }
}

size_t LatencyTracker::topic_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return topic_data_.size();
}

void LatencyTracker::set_config(const Config& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

LatencyTracker::Config LatencyTracker::get_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

}  // namespace recorder
}  // namespace axon
