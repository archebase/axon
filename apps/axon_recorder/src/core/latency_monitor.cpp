// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "latency_monitor.hpp"

#include <algorithm>

namespace axon {
namespace recorder {

LatencyMonitor::LatencyMonitor()
    : tracker_(std::make_shared<LatencyTracker>()),
      detector_(std::make_shared<LatencyAnomalyDetector>()),
      analyzer_(std::make_shared<LatencyHotspotAnalyzer>()) {
  analyzer_->set_anomaly_detector(detector_);

  auto detector_config = detector_->get_config();
  detector_config.enable_threshold_detection = true;
  detector_config.enable_statistical_detection = true;
  detector_->set_config(detector_config);
}

LatencyMonitor::~LatencyMonitor() {
  stop();
}

void LatencyMonitor::set_alert_callback(AlertCallback callback) {
  alert_callback_ = std::move(callback);

  detector_->set_alert_callback([this](const LatencyAnomalyDetector::Alert& alert) {
    on_anomaly_alert(alert);
  });
}

void LatencyMonitor::set_hotspot_callback(HotspotCallback callback) {
  hotspot_callback_ = std::move(callback);
}

void LatencyMonitor::start(uint32_t hotspot_report_interval_ms) {
  if (running_.load(std::memory_order_acquire)) {
    return;
  }

  running_.store(true, std::memory_order_release);

  if (hotspot_report_interval_ms > 0) {
    hotspot_thread_ = std::thread(&LatencyMonitor::hotspot_timer_thread, this,
                                  hotspot_report_interval_ms);
  }
}

void LatencyMonitor::stop() {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }

  running_.store(false, std::memory_order_release);

  if (hotspot_thread_.joinable()) {
    hotspot_thread_.join();
  }
}

void LatencyMonitor::record(const LatencyRecord& record) {
  tracker_->record(record);
  detector_->record_latency(record.topic, record.latency_total_ns());
}

TopicLatencyStats LatencyMonitor::get_topic_stats(const std::string& topic) const {
  return tracker_->get_topic_stats(topic);
}

GlobalLatencyStats LatencyMonitor::get_global_stats() const {
  return tracker_->get_global_stats();
}

nlohmann::json LatencyMonitor::get_stats_json() const {
  nlohmann::json json;

  auto global_stats = tracker_->get_global_stats();
  json["global"] = latency_stats_to_json(global_stats);

  auto topics = tracker_->get_topics();
  nlohmann::json topics_json = nlohmann::json::object();

  for (const auto& topic : topics) {
    auto stats = tracker_->get_topic_stats(topic);
    topics_json[topic] = topic_stats_to_json(stats);
  }

  json["topics"] = topics_json;

  auto hotspot_reports = analyzer_->generate_report();
  nlohmann::json hotspots_json = nlohmann::json::array();

  for (const auto& report : hotspot_reports) {
    if (report.is_hotspot) {
      hotspots_json.push_back(hotspot_report_to_json(report));
    }
  }

  json["hotspots"] = hotspots_json;

  return json;
}

std::vector<std::string> LatencyMonitor::get_topics() const {
  return tracker_->get_topics();
}

void LatencyMonitor::reset() {
  tracker_->reset();
  detector_->reset();
  analyzer_->reset();
}

void LatencyMonitor::set_config(const LatencyTracker::Config& tracker_config,
                                 const LatencyAnomalyDetector::Config& detector_config) {
  tracker_->set_config(tracker_config);
  detector_->set_config(detector_config);
}

LatencyTracker::Config LatencyMonitor::get_tracker_config() const {
  return tracker_->get_config();
}

LatencyAnomalyDetector::Config LatencyMonitor::get_detector_config() const {
  return detector_->get_config();
}

void LatencyMonitor::hotspot_timer_thread(uint32_t interval_ms) {
  while (running_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));

    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    analyzer_->update_from_tracker(*tracker_);

    if (hotspot_callback_) {
      auto hotspots = analyzer_->get_hotspot_topics();
      if (!hotspots.empty()) {
        nlohmann::json json = nlohmann::json::array();
        for (const auto& topic : hotspots) {
          auto report = analyzer_->get_topic_report(topic);
          if (report && report->is_hotspot) {
            json.push_back(hotspot_report_to_json(*report));
          }
        }
        if (!json.empty()) {
          hotspot_callback_(json);
        }
      }
    }
  }
}

void LatencyMonitor::on_anomaly_alert(const LatencyAnomalyDetector::Alert& alert) {
  if (alert_callback_) {
    alert_callback_(alert);
  }
}

nlohmann::json latency_stats_to_json(const GlobalLatencyStats& stats) {
  nlohmann::json json;
  json["total_messages"] = stats.total_messages;
  json["global_p50_ns"] = stats.global_p50_ns;
  json["global_p90_ns"] = stats.global_p90_ns;
  json["global_p99_ns"] = stats.global_p99_ns;
  json["global_p999_ns"] = stats.global_p999_ns;
  return json;
}

nlohmann::json topic_stats_to_json(const TopicLatencyStats& stats) {
  nlohmann::json json;
  json["message_count"] = stats.message_count;
  json["latency_min_ns"] = stats.latency_min_ns;
  json["latency_p50_ns"] = stats.latency_p50_ns;
  json["latency_p90_ns"] = stats.latency_p90_ns;
  json["latency_p95_ns"] = stats.latency_p95_ns;
  json["latency_p99_ns"] = stats.latency_p99_ns;
  json["latency_p999_ns"] = stats.latency_p999_ns;
  json["latency_max_ns"] = stats.latency_max_ns;
  json["latency_mean_ns"] = stats.latency_mean_ns;
  json["latency_std_ns"] = stats.latency_std_ns;
  json["anomaly_count_p99"] = stats.anomaly_count_p99;
  json["anomaly_count_1ms"] = stats.anomaly_count_1ms;
  json["anomaly_count_10ms"] = stats.anomaly_count_10ms;
  return json;
}

nlohmann::json hotspot_report_to_json(const HotspotReport& report) {
  nlohmann::json json;
  json["topic"] = report.topic;
  json["message_count"] = report.message_count;
  json["avg_latency_ns"] = report.avg_latency_ns;
  json["p99_latency_ns"] = report.p99_latency_ns;
  json["anomaly_rate_bps"] = report.anomaly_rate_bps;
  json["is_hotspot"] = report.is_hotspot;
  json["severity"] = report.severity;
  return json;
}

}  // namespace recorder
}  // namespace axon
