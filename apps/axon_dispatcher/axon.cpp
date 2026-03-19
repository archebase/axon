// SPDX-FileCopyrightText: 2026 ArcheBase
// SPDX-License-Identifier: MulanPSL-2.0

/*
 * Axon Unified CLI Dispatcher
 *
 * Git-style command dispatcher that forwards commands to subcommands:
 *   axon recorder [args] -> axon-recorder
 *   axon config [args]   -> axon-config
 *   axon transfer [args] -> axon-transfer
 *   axon panel [args]    -> axon-panel
 */

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#ifndef AXON_VERSION
#define AXON_VERSION "0.3.0"
#endif

// Command mappings
struct Command {
  const char* name;
  const char* binary;
  const char* description;
};

const Command COMMANDS[] = {
  {"recorder", "/opt/axon/bin/axon-recorder", "Record ROS messages to MCAP"},
  {"config", "/opt/axon/bin/axon-config", "Manage robot configuration"},
  {"transfer", "/opt/axon/bin/axon-transfer", "S3 upload daemon"},
  {"panel", "/opt/axon/bin/axon-panel", "Web control panel"},
  {"version", nullptr, nullptr},    // Special: handled internally
  {"--version", nullptr, nullptr},  // Special: handled internally
  {"-v", nullptr, nullptr},         // Special: handled internally
  {"help", nullptr, nullptr},       // Special: handled internally
  {"--help", nullptr, nullptr},     // Special: handled internally
  {"-h", nullptr, nullptr},         // Special: handled internally
  {nullptr, nullptr, nullptr}
};

void show_help(const char* program_name) {
  std::cout << "Axon - High-performance ROS Recorder\n\n";
  std::cout << "Usage: " << program_name << " <command> [args]\n\n";
  std::cout << "Available commands:\n";
  std::cout << "  recorder    Record ROS messages to MCAP\n";
  std::cout << "  config      Manage robot configuration\n";
  std::cout << "  transfer    S3 upload daemon\n";
  std::cout << "  panel       Web control panel\n";
  std::cout << "  version     Show version information\n";
  std::cout << "  help        Show this help message\n\n";
  std::cout << "See '" << program_name << " <command> --help' for more information on a command.\n";
}

void show_version() {
  std::cout << "Axon version " << AXON_VERSION << "\n\n";

  // Try to get versions from installed tools
  const char* tools[] = {"axon-recorder", "axon-config", "axon-transfer", "axon-panel"};

  std::cout << "Tool versions:\n";

  for (const char* tool : tools) {
    std::string cmd = std::string(tool) + " --version 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe) {
      char buffer[128];
      if (fgets(buffer, sizeof(buffer), pipe)) {
        // Remove trailing newline
        size_t len = strlen(buffer);
        if (len > 0 && buffer[len - 1] == '\n') buffer[len - 1] = '\0';
        std::cout << "  " << tool << "    " << buffer << "\n";
      }
      pclose(pipe);
    } else {
      std::cout << "  " << tool << "    (not installed)\n";
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    show_help(argv[0]);
    return 1;
  }

  const char* cmd = argv[1];

  // Handle built-in commands
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "--help") == 0 || strcmp(cmd, "-h") == 0) {
    show_help(argv[0]);
    return 0;
  }

  if (strcmp(cmd, "version") == 0 || strcmp(cmd, "--version") == 0 || strcmp(cmd, "-v") == 0) {
    show_version();
    return 0;
  }

  // Find and execute subcommand
  for (int i = 0; COMMANDS[i].name; ++i) {
    if (strcmp(cmd, COMMANDS[i].name) == 0) {
      if (COMMANDS[i].binary) {
        // Rebuild argv array for execvp
        std::vector<char*> exec_argv;
        exec_argv.push_back(const_cast<char*>(COMMANDS[i].binary));

        // Pass through remaining arguments
        for (int j = 2; j < argc; ++j) {
          exec_argv.push_back(argv[j]);
        }
        exec_argv.push_back(nullptr);

        // Execute the binary
        execvp(COMMANDS[i].binary, exec_argv.data());

        // If execvp returns, an error occurred
        std::cerr << "axon: error executing " << COMMANDS[i].binary << ": " << strerror(errno)
                  << "\n";
        return 1;
      }
    }
  }

  // Unknown command
  std::cerr << "axon: unknown command '" << cmd << "'\n";
  std::cerr << "Run '" << argv[0] << " help' for usage.\n";
  return 1;
}
