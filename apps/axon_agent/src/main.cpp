// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "agent_service.hpp"
#include "http_server.hpp"

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

void print_usage() {
  std::cout
    << "Usage: axon-agent [options]\n"
    << "\nOptions:\n"
    << "  --host <host>                 HTTP bind host (default: 0.0.0.0)\n"
    << "  --port <port>                 HTTP bind port (default: 8090)\n"
    << "  --robot-profile-path <path>   Robot profile root (default: /opt/axon/robots)\n"
    << "  --state-dir <path>            Runtime state dir (default: /var/lib/axon/agent)\n"
    << "  --action-manifest-dir <path>  Action manifest dir (default: /etc/axon/actions.d)\n"
    << "  --action-command-dir <path>   Approved action command dir (default: /opt/axon/actions)\n"
    << "  --help                        Show this help\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (has_flag(argc, argv, "--help")) {
    print_usage();
    return 0;
  }

  const auto host = get_arg(argc, argv, "--host", "0.0.0.0");
  const auto port = static_cast<std::uint16_t>(std::stoi(get_arg(argc, argv, "--port", "8090")));
  const auto profile_path = get_arg(argc, argv, "--robot-profile-path", "/opt/axon/robots");
  const auto state_dir = get_arg(argc, argv, "--state-dir", "/var/lib/axon/agent");
  const auto action_manifest_dir =
    get_arg(argc, argv, "--action-manifest-dir", "/etc/axon/actions.d");
  const auto action_command_dir = get_arg(argc, argv, "--action-command-dir", "/opt/axon/actions");

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  axon::agent::AgentService service(
    profile_path, state_dir, action_manifest_dir, action_command_dir
  );
  std::string error;
  if (!service.initialize(&error)) {
    std::cerr << "axon-agent initialization warning: " << error << std::endl;
  }

  axon::agent::HttpServer server(host, port, service);
  if (!server.start()) {
    return 1;
  }

  std::cout << "axon-agent listening on http://" << host << ":" << port << std::endl;
  while (!g_should_exit) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  server.stop();
  return 0;
}
