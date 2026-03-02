// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <iostream>
#include <string>
#include <vector>

#include "transfer_config.hpp"
#include "transfer_daemon.hpp"

namespace {

std::vector<std::string> config_path_hints() {
  const std::vector<std::string> candidates = {
    "./config/transfer_config.yaml",
    "./apps/axon_transfer/config/transfer_config.yaml",
    "../apps/axon_transfer/config/transfer_config.yaml",
    "/etc/axon/transfer_config.yaml"
  };

  return candidates;
}

void print_usage(const char* program) {
  std::cout << "Usage: " << program << " [options]\n";
  std::cout << "Options:\n";
  std::cout << "  -c, --config <path>   Path to config file\n";
  std::cout << "  -h, --help            Show this help message\n";
}

void print_config_hint() {
  std::cerr << "Error: --config is required.\n";
  std::cerr << "Hint: pass a config file explicitly, for example:\n";
  std::cerr << "  ./build/axon_transfer/axon_transfer --config "
               "./apps/axon_transfer/config/transfer_config.yaml\n";
  std::cerr << "Common config locations:\n";
  for (const auto& path : config_path_hints()) {
    std::cerr << "  - " << path << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;

  if (argc == 1) {
    print_usage(argv[0]);
    return 0;
  }

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-c" || arg == "--config") {
      if (i + 1 < argc) {
        config_path = argv[++i];
      } else {
        std::cerr << "Error: " << arg << " requires a path argument\n";
        return 1;
      }
    } else if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::cerr << "Error: Unknown argument: " << arg << "\n";
      print_usage(argv[0]);
      return 1;
    }
  }

  if (config_path.empty()) {
    print_usage(argv[0]);
    print_config_hint();
    return 1;
  }

  try {
    auto config = axon::transfer::load_config(config_path);

    if (config.device_id.empty()) {
      std::cerr << "Error: device_id is required in config\n";
      return 1;
    }

    if (config.factory_id.empty()) {
      std::cerr << "Error: factory_id is required in config\n";
      return 1;
    }

    axon::transfer::TransferDaemon daemon(config);
    daemon.run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
