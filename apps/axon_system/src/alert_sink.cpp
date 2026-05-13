// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "alert_sink.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace axon {
namespace system {

namespace {

class LogAlertSink final : public AlertSink {
public:
  AlertDeliveryResult deliver(const nlohmann::json& event) override {
    std::cout << event.dump() << std::endl;
    return {};
  }
};

class FileAlertSink final : public AlertSink {
public:
  explicit FileAlertSink(std::filesystem::path path)
      : path_(std::move(path)) {}

  AlertDeliveryResult deliver(const nlohmann::json& event) override {
    std::error_code ec;
    if (!path_.parent_path().empty()) {
      std::filesystem::create_directories(path_.parent_path(), ec);
      if (ec) {
        return {false, "failed to create alert sink directory: " + ec.message()};
      }
    }

    std::ofstream output(path_, std::ios::app);
    if (!output) {
      return {false, "failed to open alert sink file: " + path_.string()};
    }
    output << event.dump() << '\n';
    if (!output) {
      return {false, "failed to write alert sink file: " + path_.string()};
    }
    return {};
  }

private:
  std::filesystem::path path_;
};

}  // namespace

std::unique_ptr<AlertSink> make_alert_sink(
  const AlertSinkConfig& config, const std::filesystem::path& state_dir
) {
  if (config.type == "file") {
    const auto path = config.path.empty() ? state_dir / "alerts.jsonl" : config.path;
    return std::make_unique<FileAlertSink>(path);
  }
  if (config.type == "log") {
    return std::make_unique<LogAlertSink>();
  }
  throw std::invalid_argument("unsupported alert sink type: " + config.type);
}

}  // namespace system
}  // namespace axon
