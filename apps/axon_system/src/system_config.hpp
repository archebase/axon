// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_SYSTEM_CONFIG_HPP
#define AXON_SYSTEM_SYSTEM_CONFIG_HPP

#include <cstdint>
#include <filesystem>
#include <string>

#include "process_monitor.hpp"
#include "resource_collector.hpp"

namespace axon {
namespace system {

struct SystemConfig {
  std::string host = "127.0.0.1";
  std::uint16_t port = 8091;
  std::filesystem::path state_dir = "/var/lib/axon/system";
  ResourceCollectorOptions resource_options;
  ProcessMonitorOptions process_options;
};

SystemConfig default_system_config();
bool load_system_config(
  const std::filesystem::path& path, SystemConfig* config, std::string* error
);

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_SYSTEM_CONFIG_HPP
