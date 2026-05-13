// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "alert_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <utility>

namespace axon {
namespace system {

namespace {

constexpr std::size_t kMaxRecentEvents = 50;

}  // namespace

AlertEvaluator::AlertEvaluator(AlertEvaluatorOptions options)
    : options_(std::move(options)) {
  if (options_.evaluate_interval_ms <= 0) {
    options_.evaluate_interval_ms = 5000;
  }
  for (const auto& sink_config : options_.sinks) {
    sinks_.push_back(make_alert_sink(sink_config, options_.state_dir));
  }
}

nlohmann::json AlertEvaluator::evaluate(const nlohmann::json& snapshot) {
  const auto now = std::chrono::steady_clock::now();
  for (const auto& rule : options_.rules) {
    const auto result = evaluate_rule(rule, snapshot);
    update_rule_state(rule, result, snapshot, now);
  }
  process_delivery_queue(now);
  return build_summary();
}

nlohmann::json AlertEvaluator::current_state() const {
  return build_summary();
}

nlohmann::json AlertEvaluator::cadence_json() const {
  return {
    {"alerts", options_.evaluate_interval_ms},
    {"unit", "milliseconds"},
  };
}

AlertEvaluator::EvaluationResult AlertEvaluator::evaluate_rule(
  const AlertRuleConfig& rule, const nlohmann::json& snapshot
) const {
  if (!rule.process_id.empty()) {
    return evaluate_process_rule(rule, snapshot);
  }
  return evaluate_metric_rule(rule, snapshot);
}

AlertEvaluator::EvaluationResult AlertEvaluator::evaluate_process_rule(
  const AlertRuleConfig& rule, const nlohmann::json& snapshot
) const {
  EvaluationResult result;
  const auto& processes = snapshot.value("processes", nlohmann::json::object());
  if (!processes.is_object() || !processes.contains(rule.process_id)) {
    result.values = {{"status", "unknown"}};
    result.threshold = {{"status", rule.status}};
    result.summary = rule.process_id + " status is unknown";
    return result;
  }

  const auto& process = processes.at(rule.process_id);
  const auto observed_status = process.value("status", "unknown");
  result.matched = observed_status == rule.status;
  result.values = {{"status", observed_status}};
  result.threshold = {{"status", rule.status}};
  result.subject = {
    {"process",
     {
       {"id", rule.process_id},
       {"binary", process.value("binary", "")},
       {"pid", process.contains("pid") ? process["pid"] : nlohmann::json(nullptr)},
     }},
  };
  result.summary = process.value("binary", rule.process_id) + " is " + observed_status;
  return result;
}

AlertEvaluator::EvaluationResult AlertEvaluator::evaluate_metric_rule(
  const AlertRuleConfig& rule, const nlohmann::json& snapshot
) const {
  EvaluationResult result;
  double value = 0.0;
  if (!extract_metric_value(snapshot, rule, &value)) {
    result.values = {{"available", false}};
    result.threshold = {{"op", rule.op}, {"value", rule.threshold}};
    result.summary = rule.metric + " is unavailable";
    return result;
  }

  result.matched = compare(value, rule.op, rule.threshold);
  result.values = {{"value", value}};
  result.threshold = {{"op", rule.op}, {"value", rule.threshold}};
  result.summary = rule.metric + " is " + std::to_string(value);
  if (!rule.labels.empty()) {
    result.subject["labels"] = rule.labels;
  }
  return result;
}

void AlertEvaluator::update_rule_state(
  const AlertRuleConfig& rule, const EvaluationResult& result, const nlohmann::json& snapshot,
  std::chrono::steady_clock::time_point now
) {
  auto& state = states_[rule.id];
  state.last_observed_at = std::chrono::system_clock::now();
  state.last_values = result.values;

  if (result.matched) {
    if (state.status == "inactive" || state.status == "resolved") {
      state.status = "pending";
      state.condition_since = now;
    }
    const auto elapsed =
      std::chrono::duration_cast<std::chrono::seconds>(now - state.condition_since);
    if (state.status == "pending" && elapsed.count() >= rule.for_sec) {
      state.status = "firing";
      emit_event(rule, result, snapshot, "firing");
      state.last_event = recent_events_.empty() ? nlohmann::json::object() : recent_events_.back();
    }
    return;
  }

  if (state.status == "firing") {
    state.status = "resolved";
    emit_event(rule, result, snapshot, "resolved");
    state.last_event = recent_events_.empty() ? nlohmann::json::object() : recent_events_.back();
    return;
  }

  state.status = "inactive";
}

void AlertEvaluator::emit_event(
  const AlertRuleConfig& rule, const EvaluationResult& result, const nlohmann::json& snapshot,
  const std::string& status
) {
  const auto& service = snapshot.value("service", nlohmann::json::object());
  const auto event_id = "axon-system-" + std::to_string(++event_counter_);
  nlohmann::json event = {
    {"event_id", event_id},
    {"rule_id", rule.id},
    {"severity", rule.severity},
    {"status", status},
    {"observed_at", now_iso8601()},
    {"host", {{"hostname", host_name()}}},
    {"source",
     {
       {"service", "axon-system"},
       {"pid", service.value("pid", 0)},
     }},
    {"summary", result.summary},
    {"values", result.values},
    {"threshold", result.threshold},
  };
  if (!rule.labels.empty()) {
    event["labels"] = rule.labels;
  }
  if (result.subject.is_object()) {
    for (const auto& item : result.subject.items()) {
      event[item.key()] = item.value();
    }
  }

  recent_events_.push_back(event);
  if (recent_events_.size() > kMaxRecentEvents) {
    recent_events_.erase(recent_events_.begin());
  }
  enqueue_delivery(event);
}

void AlertEvaluator::enqueue_delivery(const nlohmann::json& event) {
  if (sinks_.empty()) {
    return;
  }
  delivery_queue_.push_back({event, 0, std::chrono::steady_clock::time_point{}, ""});
}

void AlertEvaluator::process_delivery_queue(std::chrono::steady_clock::time_point now) {
  auto it = delivery_queue_.begin();
  while (it != delivery_queue_.end()) {
    if (it->next_retry_at > now) {
      ++it;
      continue;
    }

    bool delivered = true;
    std::string error;
    for (auto& sink : sinks_) {
      const auto result = sink->deliver(it->event);
      if (!result.success) {
        delivered = false;
        error = result.error;
        break;
      }
    }

    if (delivered) {
      it = delivery_queue_.erase(it);
      continue;
    }

    ++it->attempt_count;
    it->last_error = error;
    last_delivery_error_ = error;
    const auto retry_delay_sec = std::min(300, 1 << std::min(it->attempt_count, 8));
    it->next_retry_at = now + std::chrono::seconds(retry_delay_sec);
    ++it;
  }

  if (delivery_queue_.empty()) {
    last_delivery_error_.clear();
  }
}

nlohmann::json AlertEvaluator::build_summary() const {
  nlohmann::json rules = nlohmann::json::object();
  int firing_count = 0;
  int pending_count = 0;
  int resolved_count = 0;

  for (const auto& rule : options_.rules) {
    const auto state_it = states_.find(rule.id);
    const auto status = state_it == states_.end() ? "inactive" : state_it->second.status;
    if (status == "firing") {
      ++firing_count;
    } else if (status == "pending") {
      ++pending_count;
    } else if (status == "resolved") {
      ++resolved_count;
    }

    nlohmann::json rule_state = {
      {"id", rule.id},
      {"severity", rule.severity},
      {"status", status},
    };
    if (state_it != states_.end()) {
      rule_state["last_observed_at"] = time_to_iso8601(state_it->second.last_observed_at);
      rule_state["values"] = state_it->second.last_values;
      if (!state_it->second.last_event.is_null() && !state_it->second.last_event.empty()) {
        rule_state["last_event"] = state_it->second.last_event;
      }
    }
    rules[rule.id] = std::move(rule_state);
  }

  nlohmann::json delivery = {
    {"queued_count", delivery_queue_.size()},
    {"last_error", last_delivery_error_},
  };
  if (!delivery_queue_.empty()) {
    delivery["next_retry_attempt_count"] = delivery_queue_.front().attempt_count;
    delivery["next_retry_last_error"] = delivery_queue_.front().last_error;
  }

  return {
    {"firing_count", firing_count},
    {"pending_count", pending_count},
    {"resolved_count", resolved_count},
    {"last_delivery_error", last_delivery_error_},
    {"rules", rules},
    {"events", recent_events_},
    {"delivery", delivery},
  };
}

bool AlertEvaluator::compare(double lhs, const std::string& op, double rhs) {
  if (op == "lt") return lhs < rhs;
  if (op == "le") return lhs <= rhs;
  if (op == "gt") return lhs > rhs;
  if (op == "ge") return lhs >= rhs;
  if (op == "eq") return std::abs(lhs - rhs) < 0.000001;
  if (op == "ne") return std::abs(lhs - rhs) >= 0.000001;
  return false;
}

bool AlertEvaluator::extract_metric_value(
  const nlohmann::json& snapshot, const AlertRuleConfig& rule, double* value
) {
  if (value == nullptr) {
    return false;
  }
  const auto& resources = snapshot.value("resources", nlohmann::json::object());
  if (rule.metric.rfind("memory.", 0) == 0) {
    const auto key = rule.metric.substr(std::string("memory.").size());
    if (resources.contains("memory") && resources["memory"].contains(key)) {
      *value = resources["memory"][key].get<double>();
      return true;
    }
  }
  if (rule.metric.rfind("cpu.", 0) == 0) {
    const auto key = rule.metric.substr(std::string("cpu.").size());
    if (resources.contains("cpu") && resources["cpu"].contains(key)) {
      *value = resources["cpu"][key].get<double>();
      return true;
    }
  }
  if (rule.metric.rfind("disk.", 0) == 0) {
    const auto key = rule.metric.substr(std::string("disk.").size());
    const auto path_id = rule.labels.count("path_id") > 0 ? rule.labels.at("path_id") : "";
    const auto& disks = resources.value("disk", nlohmann::json::array());
    if (!disks.is_array()) {
      return false;
    }
    for (const auto& disk : disks) {
      if (!path_id.empty() && disk.value("id", "") != path_id) {
        continue;
      }
      if (disk.contains(key)) {
        *value = disk[key].get<double>();
        return true;
      }
    }
  }
  return false;
}

std::string AlertEvaluator::now_iso8601() {
  return time_to_iso8601(std::chrono::system_clock::now());
}

std::string AlertEvaluator::time_to_iso8601(std::chrono::system_clock::time_point time) {
  const std::time_t raw_time = std::chrono::system_clock::to_time_t(time);
  std::tm tm{};
  gmtime_r(&raw_time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

std::string AlertEvaluator::host_name() {
  char buffer[256] = {};
  if (gethostname(buffer, sizeof(buffer) - 1) != 0) {
    return "";
  }
  return buffer;
}

std::vector<AlertRuleConfig> default_alert_rules() {
  return {
    {"memory_high", "warning", "memory.used_percent", "gt", 90.0, {}, "", "", 120},
    {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 10},
    {"transfer_unavailable", "warning", "", "", 0.0, {}, "transfer", "unavailable", 10},
  };
}

std::vector<AlertSinkConfig> default_alert_sinks() {
  return {
    {"file", {}},
  };
}

}  // namespace system
}  // namespace axon
