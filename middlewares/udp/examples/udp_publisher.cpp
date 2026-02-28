// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/// @file udp_publisher.cpp
/// @brief UDP JSON Publisher for Testing
///
/// Sends JSON messages via UDP for testing the Axon UDP plugin.
///
/// Usage:
///   ./udp_publisher --port 4242 --rate 10 --count 100
///
/// Examples:
///   # Send GPS data at 10 Hz
///   ./udp_publisher --port 4242 --type gps --rate 10
///
///   # Send CAN data at 100 Hz
///   ./udp_publisher --port 4243 --type can --rate 100

#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using json = nlohmann::json;

void print_usage(const char* program_name) {
  std::cout << "UDP JSON Publisher for Axon Testing\n\n"
            << "Usage: " << program_name << " [options]\n\n"
            << "Options:\n"
            << "  --host <host>       Target host (default: 127.0.0.1)\n"
            << "  --port <port>       Target UDP port (required)\n"
            << "  --type <type>       Message type: gps, can, imu, simple (default: simple)\n"
            << "  --rate <hz>         Messages per second (default: 10)\n"
            << "  --count <n>         Number of messages to send (default: 0 = infinite)\n"
            << "  --json <json>       Custom JSON message (timestamp will be added)\n"
            << "  --verbose           Print each message\n"
            << "  --help              Show this help\n"
            << std::endl;
}

/// Get current timestamp in nanoseconds since epoch
uint64_t get_timestamp_ns() {
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

/// Generate GPS message
json generate_gps_message(uint64_t counter) {
  return {
    {"timestamp", get_timestamp_ns()},
    {"latitude", 37.7749 + (counter * 0.0001)},
    {"longitude", -122.4194 + (counter * 0.0001)},
    {"altitude", 10.0},
    {"hdop", 1.2},
    {"satellites", 12}
  };
}

/// Generate CAN message
json generate_can_message(uint64_t counter) {
  return {
    {"timestamp", get_timestamp_ns()},
    {"can_id", 0x123 + (counter % 10)},
    {"dlc", 8},
    {"data",
     {counter % 256,
      counter % 256,
      counter % 256,
      counter % 256,
      counter % 256,
      counter % 256,
      counter % 256,
      counter % 256}}
  };
}

/// Generate IMU message
json generate_imu_message(uint64_t counter) {
  (void)counter;  // Unused
  return {
    {"timestamp", get_timestamp_ns()},
    {"acceleration", {{"x", 0.1}, {"y", 0.2}, {"z", 9.8}}},
    {"gyroscope", {{"x", 0.01}, {"y", 0.02}, {"z", 0.03}}}
  };
}

/// Generate simple message
json generate_simple_message(uint64_t counter) {
  return {
    {"timestamp", get_timestamp_ns()},
    {"counter", counter},
    {"message", "test_" + std::to_string(counter)}
  };
}

int main(int argc, char* argv[]) {
  // Default values
  std::string host = "127.0.0.1";
  uint16_t port = 0;
  std::string type = "simple";
  double rate = 10.0;
  uint64_t count = 0;
  std::string custom_json;
  bool verbose = false;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else if (arg == "--host" && i + 1 < argc) {
      host = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      port = static_cast<uint16_t>(std::stoi(argv[++i]));
    } else if (arg == "--type" && i + 1 < argc) {
      type = argv[++i];
    } else if (arg == "--rate" && i + 1 < argc) {
      rate = std::stod(argv[++i]);
    } else if (arg == "--count" && i + 1 < argc) {
      count = std::stoull(argv[++i]);
    } else if (arg == "--json" && i + 1 < argc) {
      custom_json = argv[++i];
    } else if (arg == "--verbose") {
      verbose = true;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }

  // Validate required arguments
  if (port == 0) {
    std::cerr << "Error: --port is required" << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  std::cout << "UDP JSON Publisher" << std::endl;
  std::cout << "  Target: " << host << ":" << port << std::endl;
  std::cout << "  Type:   " << type << std::endl;
  std::cout << "  Rate:   " << rate << " Hz" << std::endl;
  std::cout << "  Count:  " << (count == 0 ? "infinite" : std::to_string(count)) << std::endl;
  std::cout << std::endl;

  try {
    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket(
      io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)
    );

    boost::asio::ip::udp::endpoint receiver(boost::asio::ip::make_address(host), port);

    uint64_t sent_count = 0;
    uint64_t total_bytes = 0;
    uint64_t counter = 0;

    auto interval = std::chrono::duration<double>(1.0 / rate);
    auto next_send = std::chrono::steady_clock::now();

    while (count == 0 || sent_count < count) {
      // Generate message
      json msg;
      if (!custom_json.empty()) {
        msg = json::parse(custom_json);
        msg["timestamp"] = get_timestamp_ns();
      } else if (type == "gps") {
        msg = generate_gps_message(counter);
      } else if (type == "can") {
        msg = generate_can_message(counter);
      } else if (type == "imu") {
        msg = generate_imu_message(counter);
      } else {
        msg = generate_simple_message(counter);
      }

      // Serialize
      std::string msg_str = msg.dump();
      total_bytes += msg_str.size();

      // Send
      socket.send_to(boost::asio::buffer(msg_str), receiver);
      sent_count++;
      counter++;

      if (verbose) {
        std::cout << "[" << sent_count << "] Sent " << msg_str.size() << " bytes to " << host << ":"
                  << port << std::endl;
        if (sent_count % 10 == 0) {
          std::cout << "    " << msg_str << std::endl;
        }
      }

      // Rate limiting
      if (rate > 0) {
        next_send += interval;
        std::this_thread::sleep_until(next_send);
      }
    }

    std::cout << std::endl;
    std::cout << "Summary:" << std::endl;
    std::cout << "  Messages sent: " << sent_count << std::endl;
    std::cout << "  Total bytes:   " << total_bytes << std::endl;
    std::cout << "  Average size:  " << std::fixed << std::setprecision(1)
              << (static_cast<double>(total_bytes) / sent_count) << " bytes" << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
