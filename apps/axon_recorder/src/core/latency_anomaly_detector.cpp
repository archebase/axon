// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "latency_anomaly_detector.hpp"

#include <algorithm>
#include <cmath>

namespace axon {
namespace recorder {

void LatencyAnomalyDetector::WindowStats::add(uint64_t latency) {
  if (recent_latencies.size() >= 1000) {
    recent_latencies.pop_front();
  }
  recent_latencies.push_back(latency);
  update_statistics();
}

void LatencyAnomalyDetector::WindowStats::update_statistics() {
  if (recent_latencies.empty()) {
    mean = 0;
    variance = 0;
    return;
  }

  double sum = 0;
  for (uint64_t v : recent_latencies) {
    sum += static_cast<double>(v);
  }
  mean = sum / static_cast<double>(recent_latencies.size());

  if (recent_latencies.size() > 1) {
    double sum_sq = 0;
    for (uint64_t v : recent_latencies) {
      double diff = static_cast<double>(v) - mean;
      sum_sq += diff * diff;
    }
    variance = sum_sq / static_cast<double>(recent_latencies.size());
  } else {
    variance = 0;
  }
}

double LatencyAnomalyDetector::WindowStats::compute_std_dev() const {
  return std::sqrt(std::max(0.0, variance));
}

LatencyAnomalyDetector::LatencyAnomalyDetector() {}

void LatencyAnomalyDetector::set_alert_callback(AlertCallback callback) {
  alert_callback_ = std::move(callback);
}

LatencyAnomalyDetector::Severity LatencyAnomalyDetector::check_latency(
  const std::string& topic, uint64_t latency_ns
) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_windows_.find(topic);
  if (it == topic_windows_.end()) {
    WindowStats stats;
    stats.add(latency_ns);
    topic_windows_[topic] = stats;
    return Severity::Normal;
  }

  update_window(topic, latency_ns);
  auto& window = it->second;

  if (config_.enable_threshold_detection) {
    if (latency_ns >= config_.critical_threshold_ns) {
      window.status = Severity::Critical;
      return Severity::Critical;
    }
    if (latency_ns >= config_.warning_threshold_ns) {
      window.status = Severity::Warning;
      return Severity::Warning;
    }
  }

  if (config_.enable_statistical_detection && window.recent_latencies.size() >= 100) {
    double std_dev = window.compute_std_dev();
    if (std_dev > 0 && latency_ns > window.mean + config_.std_dev_multiplier * std_dev) {
      window.status = Severity::Warning;
      return Severity::Warning;
    }
  }

  window.status = Severity::Normal;
  return Severity::Normal;
}

std::optional<LatencyAnomalyDetector::Alert> LatencyAnomalyDetector::check_window_anomaly(
  const std::string& topic
) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_windows_.find(topic);
  if (it == topic_windows_.end()) {
    return std::nullopt;
  }

  const auto& window = it->second;
  if (window.recent_latencies.size() < 100) {
    return std::nullopt;
  }

  double std_dev = window.compute_std_dev();
  if (std_dev == 0) {
    return std::nullopt;
  }

  uint64_t latest = window.recent_latencies.back();
  double threshold = window.mean + config_.std_dev_multiplier * std_dev;

  if (latest > threshold) {
    Alert alert;
    alert.topic = topic;
    alert.latency_ns = latest;
    alert.threshold_ns = static_cast<uint64_t>(threshold);
    alert.timestamp_ns = 0;
    alert.detection_method = "statistical";

    if (latest >= config_.critical_threshold_ns) {
      alert.severity = Severity::Critical;
      alert.message = "Critical latency anomaly detected on " + topic;
    } else {
      alert.severity = Severity::Warning;
      alert.message = "Warning: latency spike detected on " + topic;
    }

    if (alert_callback_) {
      alert_callback_(alert);
    }

    return alert;
  }

  return std::nullopt;
}

LatencyAnomalyDetector::Severity LatencyAnomalyDetector::get_topic_status(
  const std::string& topic
) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = topic_windows_.find(topic);
  if (it == topic_windows_.end()) {
    return Severity::Normal;
  }

  return it->second.status;
}

void LatencyAnomalyDetector::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  topic_windows_.clear();
}

void LatencyAnomalyDetector::reset_topic(const std::string& topic) {
  std::lock_guard<std::mutex> lock(mutex_);
  topic_windows_.erase(topic);
}

void LatencyAnomalyDetector::set_config(const Config& config) {
  config_ = config;
}

LatencyAnomalyDetector::Config LatencyAnomalyDetector::get_config() const {
  return config_;
}

void LatencyAnomalyDetector::record_latency(const std::string& topic, uint64_t latency_ns) {
  Severity severity = check_latency(topic, latency_ns);

  if (severity != Severity::Normal && alert_callback_) {
    Alert alert;
    alert.topic = topic;
    alert.severity = severity;
    alert.latency_ns = latency_ns;
    alert.threshold_ns = (severity == Severity::Critical) ? config_.critical_threshold_ns
                                                          : config_.warning_threshold_ns;
    alert.timestamp_ns = 0;
    alert.detection_method = "threshold";
    alert.message = "Latency anomaly on " + topic + ": " + std::to_string(latency_ns / 1000000.0) +
                    "ms exceeds " + (severity == Severity::Critical ? "critical" : "warning") +
                    " threshold";

    alert_callback_(alert);
  }
}

void LatencyAnomalyDetector::update_window(const std::string& topic, uint64_t latency) {
  auto& window = topic_windows_[topic];
  window.add(latency);
}

}  // namespace recorder
}  // namespace axon
