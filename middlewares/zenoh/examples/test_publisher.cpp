// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <zenoh.hxx>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
  // Parse command-line arguments
  std::string router_endpoint = "tcp/127.0.0.1:7447";
  std::string keyexpr = "demo/test/data";
  int num_messages = 10;
  int interval_ms = 1000;

  if (argc > 1) {
    router_endpoint = argv[1];
  }
  if (argc > 2) {
    keyexpr = argv[2];
  }
  if (argc > 3) {
    num_messages = std::atoi(argv[3]);
  }
  if (argc > 4) {
    interval_ms = std::atoi(argv[4]);
  }

  std::cout << "Zenoh Test Publisher" << std::endl;
  std::cout << "Router: " << router_endpoint << std::endl;
  std::cout << "Key expression: " << keyexpr << std::endl;
  std::cout << "Publishing " << num_messages << " messages every " << interval_ms << "ms" << std::endl;

  try {
    // Create Zenoh config (use default which will scout for router)
    zenoh::Config config = zenoh::Config::create_default();

    // Open session
    std::cout << "Opening Zenoh session..." << std::endl;
    auto session = zenoh::Session::open(std::move(config));
    std::cout << "Session opened successfully" << std::endl;

    // Wait for subscribers to be ready
    std::cout << "Waiting for subscribers..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Publish messages
    for (int i = 0; i < num_messages; ++i) {
      std::string payload = "Hello from test publisher #" + std::to_string(i);
      std::cout << "Publishing message " << i << ": " << payload << std::endl;

      session.put(
        keyexpr,
        payload
      );

      if (i < num_messages - 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
      }
    }

    std::cout << "Published " << num_messages << " messages successfully" << std::endl;
    std::cout << "✓ Test publisher completed" << std::endl;
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
