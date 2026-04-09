// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "latency_hotspot_analyzer.hpp"

#include <algorithm>

namespace axon {
namespace recorder {

LatencyHotspotAnalyzer::LatencyHotspotAnalyzer() {}

void LatencyHotspotAnalyzer::update_from_tracker(const LatencyTracker& tracker) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto topics = tracker.get_topics();
  for (const auto& topic : topics) {
    auto stats = tracker.get_topic_stats(topic);
    reports_[topic] = create_report(topic, stats);
  }
}

HotspotReport LatencyHotspotAnalyzer::create_report(
    const std::string& topic, const TopicLatencyStats& stats) const {
  HotspotReport report;
  report.topic = topic;
  report.message_count = stats.message_count;
  report.avg_latency_ns = static_cast<uint64_t>(stats.latency_mean_ns);
  report.p99_latency_ns = stats.latency_p99_ns;

  if (stats.message_count > 0) {
    report.anomaly_rate_bps =
        (static_cast<double>(stats.anomaly_count_p99) / static_cast<double>(stats.message_count)) *
        10000.0;
  }

  bool is_hotspot = report.p99_latency_ns > kElevatedP99Threshold ||
                    report.anomaly_rate_bps > kElevatedAnomalyRate;

  if (report.p99_latency_ns > kCriticalP99Threshold ||
      report.anomaly_rate_bps > kCriticalAnomalyRate) {
    report.severity = "critical";
    report.anomaly_status = LatencyAnomalyDetector::Severity::Critical;
  } else if (is_hotspot) {
    report.severity = "elevated";
    report.anomaly_status = LatencyAnomalyDetector::Severity::Warning;
  } else {
    report.severity = "normal";
    report.anomaly_status = LatencyAnomalyDetector::Severity::Normal;
  }

  report.is_hotspot = is_hotspot;

  if (anomaly_detector_) {
    report.anomaly_status = anomaly_detector_->get_topic_status(topic);
    if (report.anomaly_status != LatencyAnomalyDetector::Severity::Normal) {
      report.is_hotspot = true;
    }
  }

  return report;
}

std::vector<HotspotReport> LatencyHotspotAnalyzer::generate_report() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<HotspotReport> result;
  result.reserve(reports_.size());

  for (const auto& [_, report] : reports_) {
    result.push_back(report);
  }

  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.anomaly_rate_bps > b.anomaly_rate_bps;
  });

  return result;
}

std::vector<std::string> LatencyHotspotAnalyzer::get_hotspot_topics() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> hotspots;
  for (const auto& [topic, report] : reports_) {
    if (report.is_hotspot) {
      hotspots.push_back(topic);
    }
  }

  return hotspots;
}

std::optional<HotspotReport> LatencyHotspotAnalyzer::get_topic_report(
    const std::string& topic) const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = reports_.find(topic);
  if (it == reports_.end()) {
    return std::nullopt;
  }

  return it->second;
}

std::vector<LatencyHeatmap> LatencyHotspotAnalyzer::generate_heatmaps() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<LatencyHeatmap> heatmaps;
  heatmaps.reserve(reports_.size());

  for (const auto& [topic, report] : reports_) {
    LatencyHeatmap hm;
    hm.topic = topic;
    hm.total_messages = report.message_count;

    std::vector<uint64_t> boundaries = {0,    100000,  500000,  1000000,  2000000,
                                        5000000, 10000000, 20000000, 50000000, 100000000};

    hm.bucket_boundaries = boundaries;
    hm.bucket_counts.assign(boundaries.size(), 0);

    heatmaps.push_back(hm);
  }

  return heatmaps;
}

void LatencyHotspotAnalyzer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  reports_.clear();
}

void LatencyHotspotAnalyzer::set_anomaly_detector(
    std::shared_ptr<LatencyAnomalyDetector> detector) {
  anomaly_detector_ = detector;
}

std::shared_ptr<LatencyAnomalyDetector> LatencyHotspotAnalyzer::get_anomaly_detector() const {
  return anomaly_detector_;
}

}  // namespace recorder
}  // namespace axon
