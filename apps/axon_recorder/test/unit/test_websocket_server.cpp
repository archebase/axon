// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for WebSocketServer and WebSocketSession
 */

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "../src/http/websocket_server.hpp"
#include "../src/http/websocket_session.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;

using tcp = boost::asio::ip::tcp;

using namespace axon::recorder;

// ============================================================================
// WebSocketServer Tests
// ============================================================================

class WebSocketServerTest : public ::testing::Test {
protected:
  void SetUp() override {
    ioc_ = std::make_unique<net::io_context>(1);
  }

  void TearDown() override {
    if (server_) {
      server_->stop();
    }
    if (ioc_) {
      ioc_->stop();
    }
    if (ioc_thread_.joinable()) {
      ioc_thread_.join();
    }
  }

  void StartServer(uint16_t port = 0) {
    WebSocketServer::Config config;
    config.host = "127.0.0.1";
    config.port = port;
    config.max_connections = 10;
    config.ping_interval_ms = 5000;
    config.ping_timeout_ms = 2000;

    server_ = std::make_shared<WebSocketServer>(*ioc_, config);
    ASSERT_TRUE(server_->start());

    // Run io_context in background
    ioc_thread_ = std::thread([this]() {
      ioc_->run();
    });
  }

  std::unique_ptr<net::io_context> ioc_;
  std::shared_ptr<WebSocketServer> server_;
  std::thread ioc_thread_;
};

// Test basic server creation and destruction
TEST_F(WebSocketServerTest, CreateAndDestroy) {
  WebSocketServer::Config config;
  config.port = 18080;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_FALSE(server->is_running());
}

// Test server start and stop
TEST_F(WebSocketServerTest, StartAndStop) {
  WebSocketServer::Config config;
  config.port = 18081;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());
  EXPECT_TRUE(server->is_running());

  server->stop();
  EXPECT_FALSE(server->is_running());
}

// Test double start is idempotent
TEST_F(WebSocketServerTest, DoubleStart) {
  WebSocketServer::Config config;
  config.port = 18082;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());
  EXPECT_TRUE(server->start());  // Should return true (already running)

  server->stop();
}

// Test double stop is safe
TEST_F(WebSocketServerTest, DoubleStop) {
  WebSocketServer::Config config;
  config.port = 18083;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());

  server->stop();
  server->stop();  // Should not crash
}

// Test connection count
TEST_F(WebSocketServerTest, InitialConnectionCount) {
  WebSocketServer::Config config;
  config.port = 18084;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_EQ(server->connection_count(), 0);
}

// Test broadcast with no connections
TEST_F(WebSocketServerTest, BroadcastNoConnections) {
  WebSocketServer::Config config;
  config.port = 18085;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());

  // Should not crash
  server->broadcast("test", {{"key", "value"}});

  server->stop();
}

// Test get_url
TEST_F(WebSocketServerTest, GetUrl) {
  WebSocketServer::Config config;
  config.host = "127.0.0.1";
  config.port = 18086;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_EQ(server->get_url(), "ws://127.0.0.1:18086/ws");
}

// Test message handler registration
TEST_F(WebSocketServerTest, SetMessageHandler) {
  WebSocketServer::Config config;
  config.port = 18087;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());

  bool handler_called = false;
  server->set_message_handler([&](const std::string& client_id, const std::string& message) {
    handler_called = true;
  });

  server->stop();
}

// Test rate limiting config
TEST_F(WebSocketServerTest, RateLimitingConfig) {
  WebSocketServer::Config config;
  config.port = 18088;
  config.max_messages_per_second = 5;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());
  EXPECT_EQ(config.max_messages_per_second, 5);

  server->stop();
}

// Test max connections config
TEST_F(WebSocketServerTest, MaxConnectionsConfig) {
  WebSocketServer::Config config;
  config.port = 18089;
  config.max_connections = 2;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());
  EXPECT_EQ(config.max_connections, 2);

  server->stop();
}

// Test ping configuration
TEST_F(WebSocketServerTest, PingConfig) {
  WebSocketServer::Config config;
  config.port = 18090;
  config.ping_interval_ms = 10000;
  config.ping_timeout_ms = 5000;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());
  EXPECT_EQ(config.ping_interval_ms, 10000);
  EXPECT_EQ(config.ping_timeout_ms, 5000);

  server->stop();
}

// Test broadcast_to_subscribers with no connections
TEST_F(WebSocketServerTest, BroadcastToSubscribersNoConnections) {
  WebSocketServer::Config config;
  config.port = 18091;

  auto server = std::make_shared<WebSocketServer>(*ioc_, config);
  EXPECT_TRUE(server->start());

  // Should not crash
  server->broadcast_to_subscribers("state", "state", {{"current", "recording"}});

  server->stop();
}

// Test broadcast_to_subscribers delivers to subscribed client
TEST_F(WebSocketServerTest, BroadcastToSubscribersDeliversMessage) {
  StartServer(18092);

  net::io_context client_ioc;
  tcp::resolver resolver(client_ioc);
  auto const results = resolver.resolve("127.0.0.1", "18092");

  websocket::stream<tcp::socket> ws(client_ioc);
  net::connect(ws.next_layer(), results.begin(), results.end());
  ws.handshake("127.0.0.1", "/ws");

  beast::flat_buffer buffer;
  ws.read(buffer);
  auto connected = beast::buffers_to_string(buffer.data());
  EXPECT_NE(connected.find("\"type\":\"connected\""), std::string::npos);

  const std::string subscribe = R"({"action":"subscribe","events":["state"]})";
  ws.write(net::buffer(subscribe));

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  server_->broadcast_to_subscribers("state", "state", {{"current", "recording"}});

  buffer.consume(buffer.size());
  ws.read(buffer);
  auto message = beast::buffers_to_string(buffer.data());

  EXPECT_NE(message.find("\"type\":\"state\""), std::string::npos);
  EXPECT_NE(message.find("\"current\":\"recording\""), std::string::npos);
}

// ============================================================================
// WebSocketSession Tests
// ============================================================================

class WebSocketSessionTest : public ::testing::Test {
protected:
  void SetUp() override {
    ioc_ = std::make_unique<net::io_context>(1);
  }

  void TearDown() override {
    if (ioc_) {
      ioc_->stop();
    }
  }

  std::unique_ptr<net::io_context> ioc_;
};

// Test WebSocket session subscription
TEST_F(WebSocketSessionTest, Subscription) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  // Initially not subscribed to anything
  EXPECT_FALSE(session->is_subscribed("state"));
  EXPECT_FALSE(session->is_subscribed("stats"));

  // Subscribe to state
  session->subscribe("state");
  EXPECT_TRUE(session->is_subscribed("state"));
  EXPECT_FALSE(session->is_subscribed("stats"));

  // Subscribe to stats
  session->subscribe("stats");
  EXPECT_TRUE(session->is_subscribed("state"));
  EXPECT_TRUE(session->is_subscribed("stats"));

  // Unsubscribe from state
  session->unsubscribe("state");
  EXPECT_FALSE(session->is_subscribed("state"));
  EXPECT_TRUE(session->is_subscribed("stats"));
}

// Test WebSocket session client ID
TEST_F(WebSocketSessionTest, ClientId) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "conn_123");
  EXPECT_EQ(session->client_id(), "conn_123");
}

// Test WebSocket session initial state
TEST_F(WebSocketSessionTest, InitialState) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  auto events = session->subscribed_events();
  EXPECT_TRUE(events.empty());
}

// Test WebSocket session multiple subscriptions
TEST_F(WebSocketSessionTest, MultipleSubscriptions) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  session->subscribe("state");
  session->subscribe("stats");
  session->subscribe("config");
  session->subscribe("log");

  auto events = session->subscribed_events();
  EXPECT_EQ(events.size(), 4);
  EXPECT_TRUE(events.count("state"));
  EXPECT_TRUE(events.count("stats"));
  EXPECT_TRUE(events.count("config"));
  EXPECT_TRUE(events.count("log"));
}

// Test WebSocket session duplicate subscription
TEST_F(WebSocketSessionTest, DuplicateSubscription) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  session->subscribe("state");
  session->subscribe("state");  // Duplicate

  auto events = session->subscribed_events();
  EXPECT_EQ(events.size(), 1);  // Should still be 1
}

// Test WebSocket session unsubscribe non-existent
TEST_F(WebSocketSessionTest, UnsubscribeNonExistent) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  // Should not crash
  session->unsubscribe("non_existent");

  auto events = session->subscribed_events();
  EXPECT_TRUE(events.empty());
}

// Test WebSocket session message handler
TEST_F(WebSocketSessionTest, MessageHandler) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  bool handler_called = false;
  std::string received_message;

  session->set_message_handler([&](const std::string& message) {
    handler_called = true;
    received_message = message;
  });

  // Handler is set (we can't easily test it without a real connection)
  EXPECT_FALSE(handler_called);
}

// Test WebSocket session ping configuration
TEST_F(WebSocketSessionTest, PingConfiguration) {
  tcp::socket socket(*ioc_);

  // Create session with custom ping settings
  auto session = std::make_shared<WebSocketSession>(
    std::move(socket),
    "test_client",
    5000,  // ping_interval_ms
    2000   // ping_timeout_ms
  );

  EXPECT_EQ(session->client_id(), "test_client");
}

// Test WebSocket session close
TEST_F(WebSocketSessionTest, Close) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  // Should not crash
  session->close(1000, "Normal close");
}

// Test WebSocket session double close
TEST_F(WebSocketSessionTest, DoubleClose) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  session->close(1000, "Normal close");
  session->close(1000, "Normal close");  // Should not crash
}

// Test WebSocket session send when closed
TEST_F(WebSocketSessionTest, SendWhenClosed) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  session->close(1000, "Normal close");

  // Should not crash
  session->send("test message");
}

// Test WebSocket session send_message when closed
TEST_F(WebSocketSessionTest, SendMessageWhenClosed) {
  tcp::socket socket(*ioc_);
  auto session = std::make_shared<WebSocketSession>(std::move(socket), "test_client");

  session->close(1000, "Normal close");

  // Should not crash
  session->send_message("state", {{"current", "recording"}});
}

// ============================================================================
// WebSocketServer Config Tests
// ============================================================================

TEST(WebSocketConfigTest, DefaultConfig) {
  WebSocketServer::Config config;

  EXPECT_EQ(config.host, "0.0.0.0");
  EXPECT_EQ(config.port, 8080);
  EXPECT_EQ(config.max_connections, 100);
  EXPECT_EQ(config.ping_interval_ms, 30000);
  EXPECT_EQ(config.ping_timeout_ms, 10000);
  EXPECT_EQ(config.stats_interval_ms, 1000);
  EXPECT_EQ(config.max_messages_per_second, 100);
}

TEST(WebSocketConfigTest, CustomConfig) {
  WebSocketServer::Config config;
  config.host = "192.168.1.1";
  config.port = 9000;
  config.max_connections = 50;
  config.ping_interval_ms = 15000;
  config.ping_timeout_ms = 5000;
  config.stats_interval_ms = 500;
  config.max_messages_per_second = 200;

  EXPECT_EQ(config.host, "192.168.1.1");
  EXPECT_EQ(config.port, 9000);
  EXPECT_EQ(config.max_connections, 50);
  EXPECT_EQ(config.ping_interval_ms, 15000);
  EXPECT_EQ(config.ping_timeout_ms, 5000);
  EXPECT_EQ(config.stats_interval_ms, 500);
  EXPECT_EQ(config.max_messages_per_second, 200);
}
