// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_LATENCY_HOTSPOT_ANALYZER_HPP
#define AXON_RECORDER_LATENCY_HOTSPOT_ANALYZER_HPP

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "latency_anomaly_detector.hpp"
#include "latency_tracker.hpp"

namespace axon {
namespace recorder {

/**
 * HotspotReport - Report for a latency hotspot
 */
struct HotspotReport {
  std::string topic;
  uint64_t message_count = 0;
  uint64_t avg_latency_ns = 0;
  uint64_t p99_latency_ns = 0;
  double anomaly_rate_bps = 0.0;
  bool is_hotspot = false;
  std::string severity;
  LatencyAnomalyDetector::Severity anomaly_status = LatencyAnomalyDetector::Severity::Normal;
};

/**
 * LatencyHeatmap - Histogram data for latency distribution
 */
struct LatencyHeatmap {
  std::string topic;
  std::vector<uint64_t> bucket_boundaries;
  std::vector<uint64_t> bucket_counts;
  size_t total_messages = 0;

  std::string to_ascii_string() const;
};

/**
 * LatencyHotspotAnalyzer - Analyzes latency patterns to identify hotspots
 *
 * Hotspot detection criteria:
 * - P99 > 10ms → elevated severity
 * - P99 > 50ms OR anomaly rate > 10% → critical severity
 */
class LatencyHotspotAnalyzer {
public:
  static constexpr uint64_t kElevatedP99Threshold = 10000000;
  static constexpr uint64_t kCriticalP99Threshold = 50000000;
  static constexpr double kElevatedAnomalyRate = 100.0;
  static constexpr double kCriticalAnomalyRate = 1000.0;

  LatencyHotspotAnalyzer();

  void update_from_tracker(const LatencyTracker& tracker);

  std::vector<HotspotReport> generate_report() const;

  std::vector<std::string> get_hotspot_topics() const;

  std::optional<HotspotReport> get_topic_report(const std::string& topic) const;

  std::vector<LatencyHeatmap> generate_heatmaps() const;

  void reset();

  void set_anomaly_detector(std::shared_ptr<LatencyAnomalyDetector> detector);

  std::shared_ptr<LatencyAnomalyDetector> get_anomaly_detector() const;

private:
  HotspotReport create_report(const std::string& topic, const TopicLatencyStats& stats) const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, HotspotReport> reports_;
  std::shared_ptr<LatencyAnomalyDetector> anomaly_detector_;
};

inline std::string LatencyHeatmap::to_ascii_string() const {
  if (bucket_counts.empty()) {
    return {};
  }

  uint64_t max_count = 0;
  for (uint64_t count : bucket_counts) {
    max_count = std::max(max_count, count);
  }

  std::string result = "Latency Distribution for " + topic + "\n";

  static const char* bars = " ▁▂▃▄▅▆▇█";

  for (size_t i = 0; i < bucket_counts.size(); ++i) {
    size_t bar_height = 0;
    if (max_count > 0) {
      bar_height = (bucket_counts[i] * 8) / max_count;
      if (bucket_counts[i] > 0 && bar_height == 0) {
        bar_height = 1;
      }
    }

    result += bars[bar_height];
  }
  result += "\n";

    for (size_t i = 0; i < bucket_counts.size(); ++i) {
    if (i % 5 == 0 && i < bucket_boundaries.size()) {
      result += std::to_string(bucket_boundaries[i] / 1000000) + "ms";
    } else {
      result += " ";
    }
  }
  result += "\n";

  return result;
}

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_LATENCY_HOTSPOT_ANALYZER_HPP
