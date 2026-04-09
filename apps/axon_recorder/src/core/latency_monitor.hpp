// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_LATENCY_MONITOR_HPP
#define AXON_RECORDER_LATENCY_MONITOR_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "latency_anomaly_detector.hpp"
#include "latency_hotspot_analyzer.hpp"
#include "latency_tracker.hpp"

namespace axon {
namespace recorder {

/**
 * LatencyMonitor - Main latency monitoring facade
 *
 * Integrates:
 * - LatencyTracker: Records and computes percentiles
 * - LatencyAnomalyDetector: Detects anomalies and triggers alerts
 * - LatencyHotspotAnalyzer: Identifies problematic topics
 *
 * Provides unified interface for latency monitoring.
 */
class LatencyMonitor {
public:
  using AlertCallback = std::function<void(const LatencyAnomalyDetector::Alert&)>;
  using HotspotCallback = std::function<void(const nlohmann::json&)>;

  LatencyMonitor();

  ~LatencyMonitor();

  void set_alert_callback(AlertCallback callback);

  void set_hotspot_callback(HotspotCallback callback);

  void start(uint32_t hotspot_report_interval_ms = 10000);

  void stop();

  void record(const LatencyRecord& record);

  TopicLatencyStats get_topic_stats(const std::string& topic) const;

  GlobalLatencyStats get_global_stats() const;

  nlohmann::json get_stats_json() const;

  std::vector<std::string> get_topics() const;

  void reset();

  void set_config(const LatencyTracker::Config& tracker_config,
                  const LatencyAnomalyDetector::Config& detector_config);

  LatencyTracker::Config get_tracker_config() const;

  LatencyAnomalyDetector::Config get_detector_config() const;

  std::shared_ptr<LatencyTracker> get_tracker() { return tracker_; }

  std::shared_ptr<LatencyAnomalyDetector> get_detector() { return detector_; }

  std::shared_ptr<LatencyHotspotAnalyzer> get_analyzer() { return analyzer_; }

private:
  void hotspot_timer_thread(uint32_t interval_ms);

  void on_anomaly_alert(const LatencyAnomalyDetector::Alert& alert);

  std::shared_ptr<LatencyTracker> tracker_;
  std::shared_ptr<LatencyAnomalyDetector> detector_;
  std::shared_ptr<LatencyHotspotAnalyzer> analyzer_;

  AlertCallback alert_callback_;
  HotspotCallback hotspot_callback_;

  std::atomic<bool> running_{false};
  std::thread hotspot_thread_;
  mutable std::mutex mutex_;
};

nlohmann::json latency_stats_to_json(const GlobalLatencyStats& stats);

nlohmann::json topic_stats_to_json(const TopicLatencyStats& stats);

nlohmann::json hotspot_report_to_json(const HotspotReport& report);

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_LATENCY_MONITOR_HPP
