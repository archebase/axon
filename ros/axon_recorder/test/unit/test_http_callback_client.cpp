/**
 * Unit tests for HttpCallbackClient
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "http_callback_client.hpp"
#include "task_config.hpp"

using namespace axon::recorder;

// ============================================================================
// StartCallbackPayload Tests
// ============================================================================

class StartCallbackPayloadTest : public ::testing::Test {
protected:
  StartCallbackPayload create_sample_payload() {
    StartCallbackPayload payload;
    payload.task_id = "task_001";
    payload.device_id = "robot_01";
    payload.status = "recording";
    payload.started_at = "2025-12-20T10:30:00Z";
    payload.topics = {"/camera/image", "/lidar/scan"};
    return payload;
  }
};

TEST_F(StartCallbackPayloadTest, ToJsonBasic) {
  auto payload = create_sample_payload();
  std::string json = payload.to_json();

  // Verify JSON structure
  EXPECT_TRUE(json.find("\"task_id\": \"task_001\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"device_id\": \"robot_01\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"status\": \"recording\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"started_at\": \"2025-12-20T10:30:00Z\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"topics\": [\"/camera/image\", \"/lidar/scan\"]") != std::string::npos);
}

TEST_F(StartCallbackPayloadTest, ToJsonEmptyTopics) {
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {};

  std::string json = payload.to_json();
  EXPECT_TRUE(json.find("\"topics\": []") != std::string::npos);
}

TEST_F(StartCallbackPayloadTest, ToJsonSpecialCharacters) {
  StartCallbackPayload payload;
  payload.task_id = "task with \"quotes\" and \\backslash";
  payload.device_id = "robot_01";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {};

  std::string json = payload.to_json();

  // Special characters should be escaped
  EXPECT_TRUE(json.find("\\\"quotes\\\"") != std::string::npos);
  EXPECT_TRUE(json.find("\\\\backslash") != std::string::npos);
}

// ============================================================================
// FinishCallbackPayload Tests
// ============================================================================

class FinishCallbackPayloadTest : public ::testing::Test {
protected:
  FinishCallbackPayload create_sample_payload() {
    FinishCallbackPayload payload;
    payload.task_id = "task_001";
    payload.device_id = "robot_01";
    payload.status = "finished";
    payload.started_at = "2025-12-20T10:30:00Z";
    payload.finished_at = "2025-12-20T10:35:00Z";
    payload.duration_sec = 300.0;
    payload.message_count = 15000;
    payload.file_size_bytes = 52428800;  // 50MB
    payload.output_path = "/data/recordings/task_001.mcap";
    payload.topics = {"/camera/image", "/lidar/scan"};
    payload.error = "";
    return payload;
  }
};

TEST_F(FinishCallbackPayloadTest, ToJsonBasic) {
  auto payload = create_sample_payload();
  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("\"task_id\": \"task_001\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"status\": \"finished\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"duration_sec\": 300.0") != std::string::npos);
  EXPECT_TRUE(json.find("\"message_count\": 15000") != std::string::npos);
  EXPECT_TRUE(json.find("\"file_size_bytes\": 52428800") != std::string::npos);
  EXPECT_TRUE(json.find("\"error\": null") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, ToJsonWithError) {
  auto payload = create_sample_payload();
  payload.status = "cancelled";
  payload.error = "Recording cancelled by user";

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("\"status\": \"cancelled\"") != std::string::npos);
  EXPECT_TRUE(
    json.find("\"error\": \"Recording cancelled by user\"") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, ToJsonLargeNumbers) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "finished";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.finished_at = "2025-12-20T11:30:00Z";
  payload.duration_sec = 3600.5;
  payload.message_count = 1000000000;        // 1 billion
  payload.file_size_bytes = 5368709120LL;    // 5GB
  payload.output_path = "/data/recordings/task_001.mcap";
  payload.topics = {};
  payload.error = "";

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("\"message_count\": 1000000000") != std::string::npos);
  EXPECT_TRUE(json.find("\"file_size_bytes\": 5368709120") != std::string::npos);
}

// ============================================================================
// HttpCallbackClient Tests
// ============================================================================

class HttpCallbackClientTest : public ::testing::Test {
protected:
  // Use shared_ptr since HttpCallbackClient uses enable_shared_from_this
  std::shared_ptr<HttpCallbackClient> client_ = std::make_shared<HttpCallbackClient>();

  TaskConfig create_sample_config() {
    TaskConfig config;
    config.task_id = "task_001";
    config.device_id = "robot_01";
    config.start_callback_url = "http://localhost:8080/api/recording/start";
    config.finish_callback_url = "http://localhost:8080/api/recording/finish";
    config.user_token = "jwt_token_123";
    return config;
  }
};

TEST_F(HttpCallbackClientTest, GetIso8601Timestamp) {
  std::string timestamp = HttpCallbackClient::get_iso8601_timestamp();

  // Verify ISO8601 format: YYYY-MM-DDTHH:MM:SSZ
  std::regex iso8601_regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)");
  EXPECT_TRUE(std::regex_match(timestamp, iso8601_regex))
    << "Timestamp: " << timestamp;
}

TEST_F(HttpCallbackClientTest, GetIso8601TimestampMultipleCalls) {
  std::string ts1 = HttpCallbackClient::get_iso8601_timestamp();

  // Brief delay
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  std::string ts2 = HttpCallbackClient::get_iso8601_timestamp();

  // Both should be valid timestamps (may or may not be different)
  std::regex iso8601_regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z)");
  EXPECT_TRUE(std::regex_match(ts1, iso8601_regex));
  EXPECT_TRUE(std::regex_match(ts2, iso8601_regex));
}

TEST_F(HttpCallbackClientTest, PostStartCallbackNoUrl) {
  TaskConfig config;
  config.task_id = "task_001";
  // No callback URL configured

  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "recording";

  auto result = client_->post_start_callback(config, payload);

  // Should succeed (no-op) when no URL is configured
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_TRUE(result.error_message.find("No start callback URL") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, PostFinishCallbackNoUrl) {
  TaskConfig config;
  config.task_id = "task_001";
  // No callback URL configured

  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";

  auto result = client_->post_finish_callback(config, payload);

  // Should succeed (no-op) when no URL is configured
  EXPECT_TRUE(result.success);
}

TEST_F(HttpCallbackClientTest, PostStartCallbackInvalidUrl) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "invalid-url-format";

  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "recording";

  auto result = client_->post_start_callback(config, payload);

  // Should fail with invalid URL
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("ERR_CALLBACK_FAILED") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, PostStartCallbackUnreachableServer) {
  // This test actually attempts a connection to a non-existent server
  // It should fail gracefully

  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://127.0.0.1:59999/nonexistent";

  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "recording";
  payload.started_at = HttpCallbackClient::get_iso8601_timestamp();
  payload.topics = {"/test/topic"};

  auto result = client_->post_start_callback(config, payload);

  // Should fail (can't connect)
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("ERR_CALLBACK_FAILED") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, ConfigWithTimeout) {
  HttpCallbackClient::Config config;
  config.request_timeout = std::chrono::seconds(10);

  auto client_with_config = std::make_shared<HttpCallbackClient>(config);

  // Just verify construction succeeds
  TaskConfig task_config;
  StartCallbackPayload payload;

  auto result = client_with_config->post_start_callback(task_config, payload);
  EXPECT_TRUE(result.success);  // No URL configured, so it's a no-op
}

// ============================================================================
// Callback Handler Tests
// ============================================================================

TEST_F(HttpCallbackClientTest, CallbackHandlerInvoked) {
  TaskConfig config;
  config.task_id = "task_001";
  // No URL - will succeed as no-op

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  bool handler_called = false;
  HttpCallbackResult received_result;

  auto handler = [&](const HttpCallbackResult& result) {
    handler_called = true;
    received_result = result;
  };

  auto result = client_->post_start_callback(config, payload, handler);

  EXPECT_TRUE(handler_called);
  EXPECT_TRUE(received_result.success);
  EXPECT_EQ(result.success, received_result.success);
}

TEST_F(HttpCallbackClientTest, CallbackHandlerNullptr) {
  TaskConfig config;

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  // Should not crash with nullptr handler
  auto result = client_->post_start_callback(config, payload, nullptr);
  EXPECT_TRUE(result.success);
}

// ============================================================================
// URL Parsing Tests (via error messages)
// ============================================================================

TEST_F(HttpCallbackClientTest, UrlParsing_HttpScheme) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/api/start";

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  // Will fail to connect, but shouldn't fail URL parsing
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, UrlParsing_HttpsScheme) {
  TaskConfig config;
  config.start_callback_url = "https://localhost:8443/api/start";

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  // Will fail (HTTPS not fully supported in this implementation)
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, UrlParsing_NoPort) {
  TaskConfig config;
  config.start_callback_url = "http://localhost/api/start";  // Default port 80

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  // Will fail to connect, but parsing should work
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, UrlParsing_NoPath) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080";  // No path

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  // Should use "/" as default path
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

// ============================================================================
// Timeout Configuration Tests
// ============================================================================

TEST_F(HttpCallbackClientTest, TimeoutConfiguration) {
  HttpCallbackClient::Config config;
  config.request_timeout = std::chrono::seconds(5);

  auto client = std::make_shared<HttpCallbackClient>(config);

  // Use localhost with a port that's unlikely to be listening
  // This fails fast (connection refused) rather than hanging
  TaskConfig task_config;
  task_config.start_callback_url = "http://127.0.0.1:59999/timeout";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client->post_start_callback(task_config, payload);

  // Should fail (connection refused)
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, CustomTimeoutValue) {
  HttpCallbackClient::Config config;
  config.request_timeout = std::chrono::seconds(10);

  auto client = std::make_shared<HttpCallbackClient>(config);

  // Just verify construction with custom timeout works
  TaskConfig task_config;
  // No URL - should be a no-op

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client->post_start_callback(task_config, payload);
  
  // No URL configured - should succeed as no-op
  EXPECT_TRUE(result.success);
}

// ============================================================================
// HTTPS Tests
// ============================================================================

TEST_F(HttpCallbackClientTest, HttpsUrlParsing) {
  TaskConfig config;
  config.start_callback_url = "https://api.example.com:443/recording/start";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Should fail to connect (no actual server), but URL parsing should work
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, HttpsDefaultPort) {
  TaskConfig config;
  config.start_callback_url = "https://api.example.com/recording/start";  // Default 443

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
  // Error should be about connection, not URL parsing
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(HttpCallbackClientTest, MalformedUrlScheme) {
  TaskConfig config;
  config.start_callback_url = "ftp://localhost:8080/api/start";  // Unsupported scheme

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("ERR_CALLBACK_FAILED") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, EmptyHost) {
  TaskConfig config;
  config.start_callback_url = "http:///path";  // Empty host

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, InvalidPort) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:invalid/api";  // Non-numeric port

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, PortOutOfRange) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:99999/api";  // Port > 65535

  StartCallbackPayload payload;
  auto result = client_->post_start_callback(config, payload);

  // Should either fail URL parsing or connection
  EXPECT_FALSE(result.success);
}

// ============================================================================
// Payload Edge Cases
// ============================================================================

TEST_F(HttpCallbackClientTest, EmptyPayload) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:9999/test";

  StartCallbackPayload payload;  // All defaults/empty

  auto result = client_->post_start_callback(config, payload);

  // Should fail due to connection, not payload validation
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, LongTaskId) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:9999/test";

  StartCallbackPayload payload;
  payload.task_id = std::string(10000, 'a');  // Very long task ID

  std::string json = payload.to_json();
  EXPECT_TRUE(json.find("\"task_id\": \"") != std::string::npos);
  EXPECT_GT(json.size(), 10000);
}

TEST_F(HttpCallbackClientTest, UnicodeInPayload) {
  StartCallbackPayload payload;
  payload.task_id = "task_日本語_123";
  payload.device_id = "robot_中文_01";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {"/camera/日本語"};

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("日本語") != std::string::npos);
  EXPECT_TRUE(json.find("中文") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, ManyTopics) {
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";

  // Add many topics
  for (int i = 0; i < 100; ++i) {
    payload.topics.push_back("/topic_" + std::to_string(i));
  }

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("/topic_0") != std::string::npos);
  EXPECT_TRUE(json.find("/topic_99") != std::string::npos);
}

// ============================================================================
// FinishCallbackPayload Extended Tests
// ============================================================================

TEST_F(FinishCallbackPayloadTest, ZeroDuration) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "cancelled";
  payload.duration_sec = 0.0;
  payload.message_count = 0;
  payload.file_size_bytes = 0;

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("\"duration_sec\": 0.0") != std::string::npos || 
              json.find("\"duration_sec\": 0") != std::string::npos);
  EXPECT_TRUE(json.find("\"message_count\": 0") != std::string::npos);
  EXPECT_TRUE(json.find("\"file_size_bytes\": 0") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, FloatingPointDuration) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  payload.duration_sec = 123.456789;
  payload.message_count = 1000;
  payload.file_size_bytes = 1000000;

  std::string json = payload.to_json();

  // Should contain decimal part
  EXPECT_TRUE(json.find("123.") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, ErrorWithSpecialCharacters) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "failed";
  payload.error = "Error: Disk full. Path \"/data/recordings\" is read-only";

  std::string json = payload.to_json();

  // Special characters should be escaped
  EXPECT_TRUE(json.find("\\\"") != std::string::npos);  // Escaped quotes
}

// ============================================================================
// Concurrent Request Tests
// ============================================================================

TEST_F(HttpCallbackClientTest, ConcurrentCallbacks) {
  TaskConfig config;
  config.start_callback_url = "http://127.0.0.1:59997/concurrent";

  const int NUM_THREADS = 4;
  const int REQUESTS_PER_THREAD = 5;

  std::atomic<int> completed{0};
  std::vector<std::thread> threads;

  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < REQUESTS_PER_THREAD; ++i) {
        StartCallbackPayload payload;
        payload.task_id = "task_" + std::to_string(t) + "_" + std::to_string(i);
        payload.status = "recording";

        auto result = client_->post_start_callback(config, payload);
        // All should fail (no server) but shouldn't crash
        ++completed;
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(completed.load(), NUM_THREADS * REQUESTS_PER_THREAD);
}

// ============================================================================
// Token Authentication Tests
// ============================================================================

TEST_F(HttpCallbackClientTest, WithUserToken) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:9999/api/start";
  config.user_token = "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.test";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Will fail to connect, but token should be included in request
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, EmptyUserToken) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:9999/api/start";
  config.user_token = "";  // Empty token

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Should still attempt request
  EXPECT_FALSE(result.success);
}

// ============================================================================
// URL with Query Parameters
// ============================================================================

TEST_F(HttpCallbackClientTest, UrlWithQueryParams) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/api/start?format=json&version=2";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // URL parsing should handle query params
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, UrlWithFragment) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/api/start#section";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

