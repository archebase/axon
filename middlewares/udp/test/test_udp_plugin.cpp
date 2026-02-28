// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <boost/asio.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <thread>

#include "axon_udp/udp_plugin.hpp"
#include "axon_udp/udp_server.hpp"

namespace {

// =============================================================================
// Test Fixtures
// =============================================================================

class UdpServerTest : public ::testing::Test {
protected:
  void SetUp() override {
    io_context_ = std::make_unique<boost::asio::io_context>();
    server_ = std::make_unique<axon::udp::UdpServer>(*io_context_);
  }

  void TearDown() override {
    server_->stop();
    io_context_->stop();
  }

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<axon::udp::UdpServer> server_;
};

class UdpPluginTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a minimal configuration
    test_config_ = R"({
            "udp": {
                "enabled": true,
                "bind_address": "0.0.0.0",
                "buffer_size": 65536,
                "timestamp_extraction": {
                    "source": "field",
                    "field": "timestamp"
                },
                "streams": [
                    {
                        "name": "test_stream",
                        "port": 4299,
                        "topic": "/udp/test",
                        "schema": "raw_json",
                        "enabled": true
                    }
                ]
            }
        })";
  }

  void TearDown() override {
    // Nothing to clean up
  }

  std::string test_config_;
};

// =============================================================================
// UdpServer Tests
// =============================================================================

TEST_F(UdpServerTest, CreateAndDestroy) {
  EXPECT_FALSE(server_->is_running());
}

TEST_F(UdpServerTest, StartAndStop) {
  std::vector<axon::udp::UdpStreamConfig> streams;
  axon::udp::UdpStreamConfig config;
  config.name = "test";
  config.port = 4298;
  config.topic = "/udp/test";
  config.schema_name = "raw_json";
  config.enabled = true;
  streams.push_back(config);

  EXPECT_TRUE(server_->start("0.0.0.0", streams));
  EXPECT_TRUE(server_->is_running());

  server_->stop();
  EXPECT_FALSE(server_->is_running());
}

TEST_F(UdpServerTest, StartMultipleStreams) {
  std::vector<axon::udp::UdpStreamConfig> streams;

  axon::udp::UdpStreamConfig config1;
  config1.name = "test1";
  config1.port = 4296;
  config1.topic = "/udp/test1";
  config1.schema_name = "raw_json";
  config1.enabled = true;
  streams.push_back(config1);

  axon::udp::UdpStreamConfig config2;
  config2.name = "test2";
  config2.port = 4297;
  config2.topic = "/udp/test2";
  config2.schema_name = "raw_json";
  config2.enabled = true;
  streams.push_back(config2);

  EXPECT_TRUE(server_->start("0.0.0.0", streams));
  EXPECT_TRUE(server_->is_running());

  auto stats = server_->get_all_stats();
  EXPECT_EQ(stats.size(), 2);

  server_->stop();
}

TEST_F(UdpServerTest, DisabledStreamNotStarted) {
  std::vector<axon::udp::UdpStreamConfig> streams;

  axon::udp::UdpStreamConfig config;
  config.name = "disabled";
  config.port = 4295;
  config.topic = "/udp/disabled";
  config.schema_name = "raw_json";
  config.enabled = false;  // Disabled
  streams.push_back(config);

  EXPECT_TRUE(server_->start("0.0.0.0", streams));
  EXPECT_TRUE(server_->is_running());

  // Should have no active streams
  auto stats = server_->get_all_stats();
  EXPECT_EQ(stats.size(), 0);

  server_->stop();
}

// =============================================================================
// UdpPlugin Tests
// =============================================================================

TEST_F(UdpPluginTest, CreateAndDestroy) {
  axon::udp::UdpPlugin plugin;
  EXPECT_FALSE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
}

TEST_F(UdpPluginTest, InitWithValidConfig) {
  axon::udp::UdpPlugin plugin;
  EXPECT_TRUE(plugin.init(test_config_));
  EXPECT_TRUE(plugin.is_initialized());
}

TEST_F(UdpPluginTest, InitWithEmptyConfig) {
  axon::udp::UdpPlugin plugin;
  EXPECT_TRUE(plugin.init("{}"));  // Empty config is valid (UDP disabled by default)
  EXPECT_TRUE(plugin.is_initialized());
}

TEST_F(UdpPluginTest, InitWithInvalidJson) {
  axon::udp::UdpPlugin plugin;
  EXPECT_FALSE(plugin.init("not valid json"));
  EXPECT_FALSE(plugin.is_initialized());
}

TEST_F(UdpPluginTest, StartAndStop) {
  axon::udp::UdpPlugin plugin;
  ASSERT_TRUE(plugin.init(test_config_));

  EXPECT_TRUE(plugin.start());
  EXPECT_TRUE(plugin.is_running());

  EXPECT_TRUE(plugin.stop());
  EXPECT_FALSE(plugin.is_running());
}

TEST_F(UdpPluginTest, StartWithoutInitFails) {
  axon::udp::UdpPlugin plugin;
  EXPECT_FALSE(plugin.start());
}

TEST_F(UdpPluginTest, DoubleStartSucceeds) {
  axon::udp::UdpPlugin plugin;
  ASSERT_TRUE(plugin.init(test_config_));

  EXPECT_TRUE(plugin.start());
  EXPECT_TRUE(plugin.start());  // Second start should succeed (no-op)

  plugin.stop();
}

TEST_F(UdpPluginTest, StopWithoutStartSucceeds) {
  axon::udp::UdpPlugin plugin;
  ASSERT_TRUE(plugin.init(test_config_));

  EXPECT_TRUE(plugin.stop());  // Stop without start should succeed
}

TEST_F(UdpPluginTest, DisabledPluginDoesNotStartServer) {
  std::string disabled_config = R"({
        "udp": {
            "enabled": false
        }
    })";

  axon::udp::UdpPlugin plugin;
  ASSERT_TRUE(plugin.init(disabled_config));

  // Should succeed but not actually start receiving
  EXPECT_TRUE(plugin.start());
  EXPECT_TRUE(plugin.is_running());

  plugin.stop();
}

TEST_F(UdpPluginTest, SetMessageCallback) {
  axon::udp::UdpPlugin plugin;
  ASSERT_TRUE(plugin.init(test_config_));

  bool callback_called = false;
  plugin.set_message_callback([&callback_called](
                                const std::string& topic,
                                const std::vector<uint8_t>& data,
                                const std::string& message_type,
                                uint64_t timestamp
                              ) {
    callback_called = true;
  });

  // Callback is set, but we can't test it without sending actual UDP packets
  // This is tested in integration tests
}

// =============================================================================
// Timestamp Extraction Tests
// =============================================================================

TEST(TimestampExtractionTest, FieldTimestamp) {
  // Test JSON with timestamp field
  nlohmann::json json = {{"timestamp", 1708948800000000000ULL}, {"data", "test"}};

  EXPECT_TRUE(json.contains("timestamp"));
  EXPECT_EQ(json["timestamp"].get<uint64_t>(), 1708948800000000000ULL);
}

TEST(TimestampExtractionTest, NestedTimestamp) {
  // Test nested JSON with timestamp in header.stamp
  nlohmann::json json = {{"header", {{"stamp", 1708948800000000000ULL}}}, {"data", "test"}};

  EXPECT_TRUE(json.contains("header"));
  EXPECT_TRUE(json["header"].contains("stamp"));
  EXPECT_EQ(json["header"]["stamp"].get<uint64_t>(), 1708948800000000000ULL);
}

// =============================================================================
// Message Receive Test (Integration)
// =============================================================================

class UdpMessageReceiveTest : public ::testing::Test {
protected:
  void SetUp() override {
    io_context_ = std::make_unique<boost::asio::io_context>();
    server_ = std::make_unique<axon::udp::UdpServer>(*io_context_);

    // Create work guard to keep io_context running
    work_guard_ = std::make_unique<work_guard_type>(io_context_->get_executor());

    // Run io_context in background
    io_thread_ = std::make_unique<std::thread>([this]() {
      io_context_->run();
    });
  }

  void TearDown() override {
    server_->stop();
    work_guard_.reset();  // Allow io_context to stop
    io_context_->stop();
    if (io_thread_ && io_thread_->joinable()) {
      io_thread_->join();
    }
  }

  void SendUdpPacket(uint16_t port, const std::string& message) {
    boost::asio::io_context send_io;
    boost::asio::ip::udp::socket socket(
      send_io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)
    );

    boost::asio::ip::udp::endpoint receiver(boost::asio::ip::make_address("127.0.0.1"), port);

    socket.send_to(boost::asio::buffer(message), receiver);
  }

  using work_guard_type = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<axon::udp::UdpServer> server_;
  std::unique_ptr<std::thread> io_thread_;
  std::unique_ptr<work_guard_type> work_guard_;
};

TEST_F(UdpMessageReceiveTest, ReceiveMessage) {
  const uint16_t test_port = 4293;
  std::atomic<bool> callback_called{false};
  std::string received_topic;
  std::vector<uint8_t> received_data;

  // Configure stream
  std::vector<axon::udp::UdpStreamConfig> streams;
  axon::udp::UdpStreamConfig config;
  config.name = "test";
  config.port = test_port;
  config.topic = "/udp/test";
  config.schema_name = "raw_json";
  config.enabled = true;
  streams.push_back(config);

  // Set callback
  server_->set_message_callback(
    [&](const std::string& topic, const uint8_t* data, size_t size, uint64_t timestamp) {
      callback_called = true;
      received_topic = topic;
      received_data.assign(data, data + size);
    }
  );

  // Start server
  ASSERT_TRUE(server_->start("0.0.0.0", streams));

  // Give server time to start and init async receive
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Send test message multiple times to ensure delivery
  std::string test_message = R"({"timestamp": 1234567890, "data": "hello"})";
  for (int i = 0; i < 5 && !callback_called; ++i) {
    SendUdpPacket(test_port, test_message);
    // Wait for callback between sends
    for (int j = 0; j < 10 && !callback_called; ++j) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  EXPECT_TRUE(callback_called);
  EXPECT_EQ(received_topic, "/udp/test");
  EXPECT_EQ(std::string(received_data.begin(), received_data.end()), test_message);
}

TEST_F(UdpMessageReceiveTest, StatisticsUpdated) {
  const uint16_t test_port = 4292;

  // Configure stream
  std::vector<axon::udp::UdpStreamConfig> streams;
  axon::udp::UdpStreamConfig config;
  config.name = "test";
  config.port = test_port;
  config.topic = "/udp/test";
  config.schema_name = "raw_json";
  config.enabled = true;
  streams.push_back(config);

  // Start server
  ASSERT_TRUE(server_->start("0.0.0.0", streams));

  // Give server time to start and init async receive
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Send multiple messages
  std::string test_message = R"({"test": "data"})";
  for (int i = 0; i < 10; ++i) {
    SendUdpPacket(test_port, test_message);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Check statistics
  auto stats = server_->get_stats(test_port);
  EXPECT_GE(stats.packets_received, 1);  // At least one packet received
  EXPECT_GT(stats.bytes_received, 0);
}

}  // anonymous namespace
