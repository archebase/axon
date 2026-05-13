// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "http_server.hpp"
#include "system_config.hpp"
#include "system_service.hpp"

#ifndef AXON_SYSTEM_VERSION
#define AXON_SYSTEM_VERSION "0.4.0-dev"
#endif

namespace {

volatile std::sig_atomic_t g_should_exit = 0;

void handle_signal(int) {
  g_should_exit = 1;
}

std::optional<std::string> get_arg(int argc, char** argv, const std::string& name) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) {
      if (i + 1 >= argc) {
        throw std::runtime_error(name + " requires a value");
      }
      return argv[i + 1];
    }
  }
  return std::nullopt;
}

bool has_flag(int argc, char** argv, const std::string& name) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) {
      return true;
    }
  }
  return false;
}

std::uint16_t parse_port(const std::string& value) {
  const int port = std::stoi(value);
  if (port <= 0 || port > 65535) {
    throw std::runtime_error("port must be between 1 and 65535");
  }
  return static_cast<std::uint16_t>(port);
}

void print_usage() {
  std::cout
    << "Usage: axon-system [options]\n"
    << "\nOptions:\n"
    << "  --host <host>        HTTP bind host (default: 127.0.0.1)\n"
    << "  --port <port>        HTTP bind port (default: 8091)\n"
    << "  --config <path>      YAML config path (default: /etc/axon/system.yaml if present)\n"
    << "  --state-dir <path>   Runtime state dir (default: /var/lib/axon/system)\n"
    << "  --version            Show version\n"
    << "  --help               Show this help\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (has_flag(argc, argv, "--help") || has_flag(argc, argv, "-h")) {
    print_usage();
    return 0;
  }

  if (has_flag(argc, argv, "--version") || has_flag(argc, argv, "-v")) {
    std::cout << "axon-system version " << AXON_SYSTEM_VERSION << "\n";
    return 0;
  }

  try {
    auto config = axon::system::default_system_config();
    const auto config_path = get_arg(argc, argv, "--config");
    const std::filesystem::path default_config_path = "/etc/axon/system.yaml";
    std::string config_error;
    if (config_path.has_value()) {
      if (!axon::system::load_system_config(config_path.value(), &config, &config_error)) {
        throw std::runtime_error(config_error);
      }
    } else {
      std::error_code ec;
      if (std::filesystem::exists(default_config_path, ec) &&
          !axon::system::load_system_config(default_config_path, &config, &config_error)) {
        throw std::runtime_error(config_error);
      }
    }

    if (const auto host = get_arg(argc, argv, "--host")) {
      config.host = host.value();
    }
    if (const auto port = get_arg(argc, argv, "--port")) {
      config.port = parse_port(port.value());
    }
    if (const auto state_dir = get_arg(argc, argv, "--state-dir")) {
      config.state_dir = state_dir.value();
      config.alert_options.state_dir = config.state_dir;
      for (auto& disk_path : config.resource_options.disk_paths) {
        if (disk_path.id == "state_dir") {
          disk_path.path = config.state_dir;
        }
      }
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    axon::system::SystemServiceOptions service_options;
    service_options.state_dir = config.state_dir;
    service_options.resource_options = config.resource_options;
    service_options.process_options = config.process_options;
    service_options.alert_options = config.alert_options;

    axon::system::SystemService service(service_options);
    std::string error;
    if (!service.initialize(&error)) {
      std::cerr << "axon-system initialization warning: " << error << std::endl;
    }

    axon::system::HttpServer server(config.host, config.port, service);
    if (!server.start()) {
      return 1;
    }

    std::cout << "axon-system listening on http://" << config.host << ":" << config.port
              << std::endl;
    while (!g_should_exit && !service.shutdown_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    server.stop();
    service.mark_stopped();
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "axon-system error: " << ex.what() << std::endl;
    return 1;
  }
}
