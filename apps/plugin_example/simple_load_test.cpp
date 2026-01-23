// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * @file simple_load_test.cpp
 * @brief Minimal test to verify plugin can be loaded and descriptor accessed
 *
 * This is a simplified test that only checks:
 * 1. The plugin library can be opened with dlopen
 * 2. The axon_get_plugin_descriptor symbol exists
 * 3. The descriptor is valid
 *
 * This does NOT require ROS2 to be running.
 */

#include <iostream>

#include "../axon_recorder/plugin_loader.hpp"

using namespace axon;

int main(int argc, char* argv[]) {
  std::cout << "=== Simple Plugin Load Test ===" << std::endl;

  // Parse arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <plugin_path>" << std::endl;
    std::cerr << "Example: " << argv[0] << " /path/to/libaxon_ros2_plugin.so" << std::endl;
    return 1;
  }

  std::string plugin_path = argv[1];
  std::cout << "Plugin path: " << plugin_path << std::endl << std::endl;

  // Create loader
  PluginLoader loader;

  // Test 1: Load plugin
  std::cout << "[TEST 1] Loading plugin..." << std::endl;
  auto plugin_name_opt = loader.load(plugin_path);
  if (!plugin_name_opt) {
    std::cerr << "[FAIL] Could not load plugin" << std::endl;
    std::cerr << "Error: " << loader.get_last_error() << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin loaded: " << *plugin_name_opt << std::endl << std::endl;

  // Test 2: Get descriptor
  std::cout << "[TEST 2] Getting descriptor..." << std::endl;
  const auto* descriptor = loader.get_descriptor(*plugin_name_opt);
  if (!descriptor) {
    std::cerr << "[FAIL] Could not get descriptor" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Descriptor found" << std::endl;

  // Print plugin info
  std::cout << "\nPlugin Information:" << std::endl;
  std::cout << "  ABI Version: " << descriptor->abi_version_major << "."
            << descriptor->abi_version_minor << std::endl;
  std::cout << "  Middleware: "
            << (descriptor->middleware_name ? descriptor->middleware_name : "N/A") << std::endl;
  std::cout << "  Middleware Version: "
            << (descriptor->middleware_version ? descriptor->middleware_version : "N/A")
            << std::endl;
  std::cout << "  Plugin Version: "
            << (descriptor->plugin_version ? descriptor->plugin_version : "N/A") << std::endl;

  // Test 3: Check vtable
  std::cout << "\n[TEST 3] Checking vtable..." << std::endl;
  if (!descriptor->vtable) {
    std::cerr << "[FAIL] Vtable is null" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Vtable exists" << std::endl;

  std::cout << "\nVtable Functions:" << std::endl;
  std::cout << "  init: " << (descriptor->vtable->init ? "✓" : "✗") << std::endl;
  std::cout << "  start: " << (descriptor->vtable->start ? "✓" : "✗") << std::endl;
  std::cout << "  stop: " << (descriptor->vtable->stop ? "✓" : "✗") << std::endl;
  std::cout << "  subscribe: " << (descriptor->vtable->subscribe ? "✓" : "✗") << std::endl;
  std::cout << "  publish: " << (descriptor->vtable->publish ? "✓" : "✗") << std::endl;

  // Test 4: Unload
  std::cout << "\n[TEST 4] Unloading plugin..." << std::endl;
  if (!loader.unload(*plugin_name_opt)) {
    std::cerr << "[FAIL] Could not unload plugin" << std::endl;
    return 1;
  }
  std::cout << "[PASS] Plugin unloaded" << std::endl;

  std::cout << "\n=== All Tests PASSED ===" << std::endl;
  return 0;
}
