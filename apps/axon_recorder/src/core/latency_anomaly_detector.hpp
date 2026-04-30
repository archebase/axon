// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_LATENCY_ANOMALY_DETECTOR_HPP
#define AXON_RECORDER_LATENCY_ANOMALY_DETECTOR_HPP

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace axon {
namespace recorder {

/**
 * LatencyAnomalyDetector - Detects latency anomalies using threshold and statistical methods
 *
 * Detection methods:
 * 1. Threshold-based: Flags messages exceeding absolute thresholds
 * 2. Statistical-based: Flags messages exceeding mean + N * std_dev
 * 3. Sliding window: Tracks recent latency distribution for dynamic baseline
 */
class LatencyAnomalyDetector {
public:
  enum class Severity { Normal, Warning, Critical };

  struct Config {
    uint64_t warning_threshold_ns = 5000000;
    uint64_t critical_threshold_ns = 10000000;
    size_t sliding_window_size = 1000;
    double std_dev_multiplier = 3.0;
    bool enable_threshold_detection = true;
    bool enable_statistical_detection = true;
  };

  struct Alert {
    std::string topic;
    Severity severity;
    uint64_t latency_ns;
    uint64_t threshold_ns;
    uint64_t timestamp_ns;
    std::string message;
    std::string detection_method;
  };

  using AlertCallback = std::function<void(const Alert&)>;

  LatencyAnomalyDetector();

  void set_alert_callback(AlertCallback callback);

  Severity check_latency(const std::string& topic, uint64_t latency_ns);

  std::optional<Alert> check_window_anomaly(const std::string& topic);

  Severity get_topic_status(const std::string& topic) const;

  void reset();

  void reset_topic(const std::string& topic);

  void set_config(const Config& config);

  Config get_config() const;

  void record_latency(const std::string& topic, uint64_t latency_ns);

  static const char* severity_to_string(Severity severity);

private:
  struct WindowStats {
    std::deque<uint64_t> recent_latencies;
    double mean = 0;
    double variance = 0;
    Severity status = Severity::Normal;
    uint64_t total_anomalies = 0;

    void add(uint64_t latency);
    void update_statistics();
    double compute_std_dev() const;
  };

  void update_window(const std::string& topic, uint64_t latency);

  Config config_;
  AlertCallback alert_callback_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, WindowStats> topic_windows_;
};

inline const char* LatencyAnomalyDetector::severity_to_string(Severity severity) {
  switch (severity) {
    case Severity::Normal:
      return "normal";
    case Severity::Warning:
      return "warning";
    case Severity::Critical:
      return "critical";
    default:
      return "unknown";
  }
}

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_LATENCY_ANOMALY_DETECTOR_HPP
