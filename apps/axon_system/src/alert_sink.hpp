// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_ALERT_SINK_HPP
#define AXON_SYSTEM_ALERT_SINK_HPP

#include <nlohmann/json.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace axon {
namespace system {

struct AlertSinkConfig {
  std::string type = "log";
  std::filesystem::path path;
};

struct AlertDeliveryResult {
  bool success = true;
  std::string error;
};

class AlertSink {
public:
  virtual ~AlertSink() = default;
  virtual AlertDeliveryResult deliver(const nlohmann::json& event) = 0;
};

std::unique_ptr<AlertSink> make_alert_sink(
  const AlertSinkConfig& config, const std::filesystem::path& state_dir
);

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_ALERT_SINK_HPP
