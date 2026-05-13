// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "system_config.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <utility>
#include <vector>

namespace axon {
namespace system {

namespace {

bool parse_port(const YAML::Node& node, std::uint16_t* port, std::string* error) {
  try {
    const int value = node.as<int>();
    if (value <= 0 || value > std::numeric_limits<std::uint16_t>::max()) {
      if (error != nullptr) {
        *error = "server.port must be between 1 and 65535";
      }
      return false;
    }
    *port = static_cast<std::uint16_t>(value);
    return true;
  } catch (const YAML::Exception& ex) {
    if (error != nullptr) {
      *error = std::string("failed to parse server.port: ") + ex.what();
    }
    return false;
  }
}

bool parse_disk_paths(const YAML::Node& node, SystemConfig* config, std::string* error) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "disk_paths must be a sequence";
    }
    return false;
  }

  std::vector<DiskPathConfig> disk_paths;
  for (const auto& item : node) {
    if (!item["id"] || !item["path"]) {
      if (error != nullptr) {
        *error = "each disk_paths item must contain id and path";
      }
      return false;
    }
    disk_paths.push_back({item["id"].as<std::string>(), item["path"].as<std::string>()});
  }
  config->resource_options.disk_paths = std::move(disk_paths);
  return true;
}

bool parse_network_interfaces(const YAML::Node& node, SystemConfig* config, std::string* error) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "network.interfaces must be a sequence";
    }
    return false;
  }

  std::vector<std::string> interfaces;
  for (const auto& item : node) {
    interfaces.push_back(item.as<std::string>());
  }
  config->resource_options.network_interfaces = std::move(interfaces);
  return true;
}

bool parse_pid_file_candidates(
  const YAML::Node& node, ProcessTargetConfig* target, std::string* error
) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "pid_file_candidates must be a sequence";
    }
    return false;
  }
  for (const auto& item : node) {
    target->pid_file_candidates.push_back(item.as<std::string>());
  }
  return true;
}

bool parse_cmdline_contains(
  const YAML::Node& node, ProcessTargetConfig* target, std::string* error
) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "cmdline_contains must be a sequence";
    }
    return false;
  }
  for (const auto& item : node) {
    target->cmdline_contains.push_back(item.as<std::string>());
  }
  return true;
}

bool parse_process_rpc(const YAML::Node& node, ProcessTargetConfig* target, std::string* error) {
  if (!node.IsMap()) {
    if (error != nullptr) {
      *error = "monitored_processes.rpc must be a map";
    }
    return false;
  }

  ProcessHttpProbeConfig rpc;
  rpc.type = node["type"] ? node["type"].as<std::string>() : "http";
  if (node["url"]) {
    rpc.url = node["url"].as<std::string>();
  }
  if (node["timeout_ms"]) {
    rpc.timeout_ms = node["timeout_ms"].as<int>();
  }
  target->rpc = rpc;
  return true;
}

bool parse_monitored_processes(const YAML::Node& node, SystemConfig* config, std::string* error) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "monitored_processes must be a sequence";
    }
    return false;
  }

  std::vector<ProcessTargetConfig> targets;
  for (const auto& item : node) {
    if (!item["id"] || !item["executable"]) {
      if (error != nullptr) {
        *error = "each monitored_processes item must contain id and executable";
      }
      return false;
    }

    ProcessTargetConfig target;
    target.id = item["id"].as<std::string>();
    target.executable = item["executable"].as<std::string>();
    if (item["pid_file"]) {
      target.pid_file = item["pid_file"].as<std::string>();
    }
    if (item["pid_file_candidates"] &&
        !parse_pid_file_candidates(item["pid_file_candidates"], &target, error)) {
      return false;
    }
    if (item["cmdline_contains"] &&
        !parse_cmdline_contains(item["cmdline_contains"], &target, error)) {
      return false;
    }
    if (item["rpc"] && !parse_process_rpc(item["rpc"], &target, error)) {
      return false;
    }
    targets.push_back(std::move(target));
  }

  config->process_options.targets = std::move(targets);
  return true;
}

bool parse_alert_sinks(const YAML::Node& node, SystemConfig* config, std::string* error) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "alerts.sinks must be a sequence";
    }
    return false;
  }

  std::vector<AlertSinkConfig> sinks;
  for (const auto& item : node) {
    AlertSinkConfig sink;
    sink.type = item["type"] ? item["type"].as<std::string>() : "log";
    if (item["path"]) {
      sink.path = item["path"].as<std::string>();
    }
    sinks.push_back(std::move(sink));
  }
  config->alert_options.sinks = std::move(sinks);
  return true;
}

bool parse_alert_labels(const YAML::Node& node, AlertRuleConfig* rule, std::string* error) {
  if (!node.IsMap()) {
    if (error != nullptr) {
      *error = "alerts.rules.labels must be a map";
    }
    return false;
  }

  for (const auto& item : node) {
    rule->labels[item.first.as<std::string>()] = item.second.as<std::string>();
  }
  return true;
}

bool parse_alert_rules(const YAML::Node& node, SystemConfig* config, std::string* error) {
  if (!node.IsSequence()) {
    if (error != nullptr) {
      *error = "alerts.rules must be a sequence";
    }
    return false;
  }

  std::vector<AlertRuleConfig> rules;
  for (const auto& item : node) {
    if (!item["id"]) {
      if (error != nullptr) {
        *error = "each alerts.rules item must contain id";
      }
      return false;
    }

    AlertRuleConfig rule;
    rule.id = item["id"].as<std::string>();
    rule.severity = item["severity"] ? item["severity"].as<std::string>() : "warning";
    if (item["metric"]) {
      rule.metric = item["metric"].as<std::string>();
    }
    if (item["op"]) {
      rule.op = item["op"].as<std::string>();
    }
    if (item["threshold"]) {
      rule.threshold = item["threshold"].as<double>();
    }
    if (item["labels"] && !parse_alert_labels(item["labels"], &rule, error)) {
      return false;
    }
    if (item["process_id"]) {
      rule.process_id = item["process_id"].as<std::string>();
    }
    if (item["status"]) {
      rule.status = item["status"].as<std::string>();
    }
    if (item["for_sec"]) {
      rule.for_sec = item["for_sec"].as<int>();
    }
    rules.push_back(std::move(rule));
  }

  config->alert_options.rules = std::move(rules);
  return true;
}

bool parse_alerts(const YAML::Node& node, SystemConfig* config, std::string* error) {
  if (!node.IsMap()) {
    if (error != nullptr) {
      *error = "alerts must be a map";
    }
    return false;
  }

  if (node["evaluate_interval_ms"]) {
    config->alert_options.evaluate_interval_ms = node["evaluate_interval_ms"].as<int>();
  }
  if (node["sinks"] && !parse_alert_sinks(node["sinks"], config, error)) {
    return false;
  }
  if (node["rules"] && !parse_alert_rules(node["rules"], config, error)) {
    return false;
  }
  return true;
}

}  // namespace

SystemConfig default_system_config() {
  SystemConfig config;
  config.resource_options.proc_root = "/proc";
  config.resource_options.resource_sample_cadence_ms = 1000;
  config.resource_options.disk_sample_cadence_ms = 5000;
  config.process_options.proc_root = "/proc";
  config.process_options.targets = default_process_targets();
  config.process_options.process_sample_cadence_ms = 2000;
  config.alert_options.state_dir = config.state_dir;
  config.alert_options.evaluate_interval_ms = 5000;
  config.alert_options.sinks = default_alert_sinks();
  config.alert_options.rules = default_alert_rules();
  return config;
}

bool load_system_config(
  const std::filesystem::path& path, SystemConfig* config, std::string* error
) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "config output pointer is null";
    }
    return false;
  }

  try {
    const auto yaml = YAML::LoadFile(path.string());

    if (yaml["server"]) {
      const auto server = yaml["server"];
      if (server["host"]) {
        config->host = server["host"].as<std::string>();
      }
      if (server["port"] && !parse_port(server["port"], &config->port, error)) {
        return false;
      }
    }

    if (yaml["state_dir"]) {
      config->state_dir = yaml["state_dir"].as<std::string>();
      config->alert_options.state_dir = config->state_dir;
    }

    if (yaml["sampling"]) {
      const auto sampling = yaml["sampling"];
      if (sampling["resources_ms"]) {
        config->resource_options.resource_sample_cadence_ms = sampling["resources_ms"].as<int>();
      }
      if (sampling["disk_ms"]) {
        config->resource_options.disk_sample_cadence_ms = sampling["disk_ms"].as<int>();
      }
      if (sampling["processes_ms"]) {
        config->process_options.process_sample_cadence_ms = sampling["processes_ms"].as<int>();
      }
      if (sampling["alerts_ms"]) {
        config->alert_options.evaluate_interval_ms = sampling["alerts_ms"].as<int>();
      }
    }

    if (yaml["disk_paths"] && !parse_disk_paths(yaml["disk_paths"], config, error)) {
      return false;
    }

    if (yaml["network"] && yaml["network"]["interfaces"] &&
        !parse_network_interfaces(yaml["network"]["interfaces"], config, error)) {
      return false;
    }

    if (yaml["monitored_processes"] &&
        !parse_monitored_processes(yaml["monitored_processes"], config, error)) {
      return false;
    }

    if (yaml["alerts"] && !parse_alerts(yaml["alerts"], config, error)) {
      return false;
    }

    if (error != nullptr) {
      error->clear();
    }
    return true;
  } catch (const YAML::Exception& ex) {
    if (error != nullptr) {
      *error = "failed to load " + path.string() + ": " + ex.what();
    }
    return false;
  }
}

}  // namespace system
}  // namespace axon
