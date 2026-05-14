/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

#include <cstring>
#include <iostream>
#include <string>

#include "embedded_assets.hpp"
#include "httplib.h"
#include "panel_api.hpp"

void print_usage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [OPTIONS]\n"
            << "\nOptions:\n"
            << "  --port PORT       HTTP server port (default: 8082)\n"
            << "  --rpc PORT        Recorder RPC port (default: panel port - 2)\n"
            << "  --config-dir DIR  Robot config directory (default: AXON_PANEL_CONFIG_DIR or "
               "~/.axon/config)\n"
            << "  --recording-dir DIR\n"
            << "                    Recording task directory (default: AXON_PANEL_RECORDING_DIR or "
               "/tmp/axon/recording)\n"
            << "  --version         Show version information\n"
            << "  --help            Show this help message\n"
            << std::endl;
}

int main(int argc, char* argv[]) {
  int port = 8082;    // Default: recorder HTTP RPC port (8080) + 2
  int rpc_port = -1;  // -1 means derive from panel port (port - 2)
  axon::panel::PanelApiOptions panel_options = axon::panel::make_default_panel_api_options();

  // Simple argument parsing
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      try {
        port = std::stoi(argv[++i]);
      } catch (const std::exception&) {
        std::cerr << "Invalid port value: " << argv[i] << std::endl;
        return 1;
      }
      if (port <= 0 || port > 65535) {
        std::cerr << "Port must be between 1 and 65535" << std::endl;
        return 1;
      }
    } else if (arg == "--rpc" && i + 1 < argc) {
      try {
        rpc_port = std::stoi(argv[++i]);
      } catch (const std::exception&) {
        std::cerr << "Invalid RPC port value: " << argv[i] << std::endl;
        return 1;
      }
      if (rpc_port <= 0 || rpc_port > 65535) {
        std::cerr << "RPC port must be between 1 and 65535" << std::endl;
        return 1;
      }
    } else if (arg == "--config-dir" && i + 1 < argc) {
      panel_options.config_dir = argv[++i];
      if (panel_options.config_dir.empty()) {
        std::cerr << "Config directory must not be empty" << std::endl;
        return 1;
      }
    } else if (arg == "--recording-dir" && i + 1 < argc) {
      panel_options.recording_dir = argv[++i];
      if (panel_options.recording_dir.empty()) {
        std::cerr << "Recording directory must not be empty" << std::endl;
        return 1;
      }
    } else if (arg == "--version") {
      std::cout << "AxonPanel v0.4.0" << std::endl;
      return 0;
    } else if (arg == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown option: " << arg << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }

  // Derive RPC port if not explicitly set
  int effective_rpc_port = (rpc_port > 0) ? rpc_port : (port - 2);

  // Validate derived RPC port
  if (effective_rpc_port <= 0 || effective_rpc_port > 65535) {
    std::cerr << "Derived RPC port " << effective_rpc_port
              << " is invalid. Use --rpc to specify a valid port." << std::endl;
    return 1;
  }

  httplib::Server svr;

  axon::panel::register_panel_api(svr, panel_options);

  // Serve runtime configuration as JavaScript
  svr.Get("/config.js", [effective_rpc_port](const httplib::Request& req, httplib::Response& res) {
    (void)req;
    std::string config_js = "window.__AXON_CONFIG__=" + std::to_string(effective_rpc_port) + ";";
    res.set_content(config_js, "application/javascript");
  });

  // Serve embedded assets
  svr.Get("/.*", [](const httplib::Request& req, httplib::Response& res) {
    std::string path = req.path;
    if (path == "/") {
      path = "/index.html";
    }

    auto asset = EmbeddedAssets::get(path);

    if (asset) {
      res.set_content(asset->data, asset->size, asset->mime_type.c_str());
    } else {
      // SPA fallback to index.html for client-side routing
      auto index = EmbeddedAssets::get("/index.html");
      if (index) {
        res.set_content(index->data, index->size, "text/html");
      } else {
        res.status = 404;
        res.set_content("Not Found", "text/plain");
      }
    }
  });

  // Initialize embedded assets before starting server (thread-safe)
  EmbeddedAssets::init();

  std::cout << "Starting AxonPanel on http://0.0.0.0:" << port << std::endl;
  std::cout << "Recorder RPC port: " << effective_rpc_port << std::endl;
  std::cout << "Config directory: " << panel_options.config_dir << std::endl;
  std::cout << "Recording directory: " << panel_options.recording_dir << std::endl;

  if (!svr.listen("0.0.0.0", port)) {
    std::cerr << "Failed to start server on port " << port << std::endl;
    return 1;
  }

  return 0;
}
