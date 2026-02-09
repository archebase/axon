// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// AxonConfig - Robot Configuration Management Tool
// Manages configuration files for embedding into MCAP recordings

#include <exception>
#include <iostream>

#include "commands.hpp"

/**
 * Main entry point for axon_config
 */
int main(int argc, char* argv[]) {
  axon::config::Commands commands;

  try {
    return commands.execute(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Error: Unknown exception occurred" << std::endl;
    return 1;
  }
}
