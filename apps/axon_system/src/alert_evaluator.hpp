// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_ALERT_EVALUATOR_HPP
#define AXON_SYSTEM_ALERT_EVALUATOR_HPP

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "alert_sink.hpp"

namespace axon {
namespace system {

struct AlertRuleConfig {
  std::string id;
  std::string severity = "warning";
  std::string metric;
  std::string op;
  double threshold = 0.0;
  std::map<std::string, std::string> labels;
  std::string process_id;
  std::string status;
  int for_sec = 0;
};

struct AlertEvaluatorOptions {
  int evaluate_interval_ms = 5000;
  std::filesystem::path state_dir = "/var/lib/axon/system";
  std::vector<AlertRuleConfig> rules;
  std::vector<AlertSinkConfig> sinks;
};

class AlertEvaluator {
public:
  explicit AlertEvaluator(AlertEvaluatorOptions options = {});

  nlohmann::json evaluate(const nlohmann::json& snapshot);
  nlohmann::json current_state() const;
  nlohmann::json cadence_json() const;

private:
  struct RuleState {
    std::string status = "inactive";
    std::chrono::steady_clock::time_point condition_since;
    std::chrono::system_clock::time_point last_observed_at;
    nlohmann::json last_event;
    nlohmann::json last_values;
  };

  struct PendingDelivery {
    nlohmann::json event;
    int attempt_count = 0;
    std::chrono::steady_clock::time_point next_retry_at;
    std::string last_error;
  };

  struct EvaluationResult {
    bool matched = false;
    nlohmann::json values = nlohmann::json::object();
    nlohmann::json threshold = nlohmann::json::object();
    nlohmann::json subject = nlohmann::json::object();
    std::string summary;
  };

  EvaluationResult evaluate_rule(const AlertRuleConfig& rule, const nlohmann::json& snapshot) const;
  EvaluationResult evaluate_process_rule(
    const AlertRuleConfig& rule, const nlohmann::json& snapshot
  ) const;
  EvaluationResult evaluate_metric_rule(const AlertRuleConfig& rule, const nlohmann::json& snapshot)
    const;
  void update_rule_state(
    const AlertRuleConfig& rule, const EvaluationResult& result, const nlohmann::json& snapshot,
    std::chrono::steady_clock::time_point now
  );
  void emit_event(
    const AlertRuleConfig& rule, const EvaluationResult& result, const nlohmann::json& snapshot,
    const std::string& status
  );
  void enqueue_delivery(const nlohmann::json& event);
  void process_delivery_queue(std::chrono::steady_clock::time_point now);
  nlohmann::json build_summary() const;

  static bool compare(double lhs, const std::string& op, double rhs);
  static bool extract_metric_value(
    const nlohmann::json& snapshot, const AlertRuleConfig& rule, double* value
  );
  static std::string now_iso8601();
  static std::string time_to_iso8601(std::chrono::system_clock::time_point time);
  static std::string host_name();

  AlertEvaluatorOptions options_;
  std::vector<std::unique_ptr<AlertSink>> sinks_;
  std::map<std::string, RuleState> states_;
  std::vector<nlohmann::json> recent_events_;
  std::vector<PendingDelivery> delivery_queue_;
  std::uint64_t event_counter_ = 0;
  std::string last_delivery_error_;
};

std::vector<AlertRuleConfig> default_alert_rules();
std::vector<AlertSinkConfig> default_alert_sinks();

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_ALERT_EVALUATOR_HPP
