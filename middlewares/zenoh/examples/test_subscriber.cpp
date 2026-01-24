// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <zenoh.hxx>

int main(int argc, char** argv) {
  // Parse command-line arguments
  std::string router_endpoint = "tcp/127.0.0.1:7447";
  std::string keyexpr = "demo/test/**";
  int expected_messages = 10;
  int timeout_sec = 30;

  if (argc > 1) {
    router_endpoint = argv[1];
  }
  if (argc > 2) {
    keyexpr = argv[2];
  }
  if (argc > 3) {
    expected_messages = std::atoi(argv[3]);
  }
  if (argc > 4) {
    timeout_sec = std::atoi(argv[4]);
  }

  std::cout << "Zenoh Test Subscriber" << std::endl;
  std::cout << "Router: " << router_endpoint << std::endl;
  std::cout << "Key expression: " << keyexpr << std::endl;
  std::cout << "Expecting " << expected_messages << " messages (timeout: " << timeout_sec << "s)"
            << std::endl;

  try {
    // Create Zenoh config (use default which will scout for router)
    zenoh::Config config = zenoh::Config::create_default();

    // Open session
    std::cout << "Opening Zenoh session..." << std::endl;
    auto session = zenoh::Session::open(std::move(config));
    std::cout << "Session opened successfully" << std::endl;

    // Message counter
    std::atomic<int> message_count{0};

    // Declare subscriber
    std::cout << "Subscribing to: " << keyexpr << std::endl;
    auto subscriber = session.declare_subscriber(
      keyexpr,
      [&message_count](const zenoh::Sample& sample) {
        auto payload = sample.get_payload().as_vector();
        std::string message(payload.begin(), payload.end());
        int count = ++message_count;
        std::cout << "Received message " << count << ": " << message << std::endl;
      },
      zenoh::closures::none  // on_drop callback
    );

    std::cout << "Subscriber created, waiting for messages..." << std::endl;

    // Wait for messages with timeout
    auto start_time = std::chrono::steady_clock::now();
    while (message_count < expected_messages) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       std::chrono::steady_clock::now() - start_time
      )
                       .count();

      if (elapsed >= timeout_sec) {
        std::cerr << "❌ Timeout: Only received " << message_count << "/" << expected_messages
                  << " messages after " << timeout_sec << "s" << std::endl;
        return 1;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "✓ Test subscriber completed successfully" << std::endl;
    std::cout << "  Received all " << expected_messages << " messages" << std::endl;
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
