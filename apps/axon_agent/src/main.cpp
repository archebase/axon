// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "agent_service.hpp"
#include "http_server.hpp"
#include "keystone_action_sync.hpp"

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
    << "  --keystone-url <url>          Enable Keystone action sync/polling against this URL\n"
    << "  --keystone-token <token>      Optional Keystone bearer token\n"
    << "  --keystone-robot-id <id>      Robot ID for Keystone sync (default: active profile ID)\n"
    << "  --action-poll-interval-sec <n>       Keystone pending poll interval (default: 5)\n"
    << "  --action-catalog-sync-interval-sec <n>  Catalog sync interval (default: 60)\n"
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
  const auto keystone_url = get_arg(argc, argv, "--keystone-url", "");
  const auto keystone_token = get_arg(argc, argv, "--keystone-token", "");
  const auto keystone_robot_id = get_arg(argc, argv, "--keystone-robot-id", "");
  const auto action_poll_interval_sec =
    std::stoi(get_arg(argc, argv, "--action-poll-interval-sec", "5"));
  const auto action_catalog_sync_interval_sec =
    std::stoi(get_arg(argc, argv, "--action-catalog-sync-interval-sec", "60"));

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

  std::unique_ptr<axon::agent::KeystoneActionSync> action_sync;
  if (!keystone_url.empty()) {
    axon::agent::KeystoneActionSyncConfig sync_config;
    sync_config.enabled = true;
    sync_config.base_url = keystone_url;
    sync_config.auth_token = keystone_token;
    sync_config.robot_id = keystone_robot_id;
    sync_config.poll_interval = std::chrono::seconds(action_poll_interval_sec);
    sync_config.catalog_sync_interval = std::chrono::seconds(action_catalog_sync_interval_sec);
    action_sync = std::make_unique<axon::agent::KeystoneActionSync>(service, sync_config);
    action_sync->start();
  }

  std::cout << "axon-agent listening on http://" << host << ":" << port << std::endl;
  while (!g_should_exit) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  if (action_sync) {
    action_sync->stop();
  }
  server.stop();
  return 0;
}
