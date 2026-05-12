// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "http_server.hpp"
#include "system_service.hpp"

#ifndef AXON_SYSTEM_VERSION
#define AXON_SYSTEM_VERSION "0.4.0-dev"
#endif

namespace {

volatile std::sig_atomic_t g_should_exit = 0;

void handle_signal(int) {
  g_should_exit = 1;
}

std::string get_arg(
  int argc, char** argv, const std::string& name, const std::string& default_value
) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
  }
  return default_value;
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
  std::cout << "Usage: axon-system [options]\n"
            << "\nOptions:\n"
            << "  --host <host>        HTTP bind host (default: 127.0.0.1)\n"
            << "  --port <port>        HTTP bind port (default: 8091)\n"
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
    const auto host = get_arg(argc, argv, "--host", "127.0.0.1");
    const auto port = parse_port(get_arg(argc, argv, "--port", "8091"));
    const auto state_dir = get_arg(argc, argv, "--state-dir", "/var/lib/axon/system");

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    axon::system::SystemService service(state_dir);
    std::string error;
    if (!service.initialize(&error)) {
      std::cerr << "axon-system initialization warning: " << error << std::endl;
    }

    axon::system::HttpServer server(host, port, service);
    if (!server.start()) {
      return 1;
    }

    std::cout << "axon-system listening on http://" << host << ":" << port << std::endl;
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
