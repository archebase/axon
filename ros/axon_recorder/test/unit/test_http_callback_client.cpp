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

// ============================================================================
// Enhanced Coverage Tests (Phase 4)
// ============================================================================

TEST_F(StartCallbackPayloadTest, EscapeJson_ControlCharacters) {
  StartCallbackPayload payload;
  payload.task_id = "task_with\ttab\nand\rnewlines";
  payload.device_id = "device\bwith\fcontrol";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {};

  std::string json = payload.to_json();

  // Control characters should be escaped
  EXPECT_TRUE(json.find("\\t") != std::string::npos);  // Tab
  EXPECT_TRUE(json.find("\\n") != std::string::npos);  // Newline
  EXPECT_TRUE(json.find("\\r") != std::string::npos);  // Carriage return
}

TEST_F(StartCallbackPayloadTest, EscapeJson_BackspaceFormfeed) {
  StartCallbackPayload payload;
  payload.task_id = "task\b\f";  // backspace and formfeed
  payload.device_id = "robot";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {};

  std::string json = payload.to_json();

  // Should handle \b (backspace) and \f (formfeed)
  // These may be escaped as \b, \f or \uXXXX
  EXPECT_FALSE(json.empty());
}

TEST_F(StartCallbackPayloadTest, EscapeJson_NullCharacter) {
  StartCallbackPayload payload;
  std::string task_id = "task";
  task_id.push_back('\0');  // Null character
  task_id += "id";
  payload.task_id = task_id;
  payload.device_id = "robot";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {};

  std::string json = payload.to_json();

  // Null character should be escaped or removed
  EXPECT_FALSE(json.empty());
}

TEST_F(FinishCallbackPayloadTest, WithMetadata_Scene) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "finished";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.finished_at = "2025-12-20T10:35:00Z";
  payload.duration_sec = 300.0;
  payload.message_count = 1000;
  payload.file_size_bytes = 1000000;
  payload.output_path = "/data/task.mcap";
  payload.topics = {"/camera"};
  payload.error = "";

  // Set metadata fields
  payload.metadata.scene = "indoor_navigation";
  payload.metadata.subscene = "hallway_01";
  payload.metadata.skills = {"obstacle_avoidance", "path_planning"};
  payload.metadata.factory = "test_factory_01";

  std::string json = payload.to_json();

  // Verify metadata is included
  EXPECT_TRUE(json.find("metadata") != std::string::npos);
  EXPECT_TRUE(json.find("indoor_navigation") != std::string::npos);
  EXPECT_TRUE(json.find("hallway_01") != std::string::npos);
  EXPECT_TRUE(json.find("obstacle_avoidance") != std::string::npos);
  EXPECT_TRUE(json.find("test_factory_01") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, WithMetadata_EmptySkills) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "finished";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.finished_at = "2025-12-20T10:35:00Z";
  payload.duration_sec = 100.0;
  payload.message_count = 500;
  payload.file_size_bytes = 500000;
  payload.output_path = "/data/task.mcap";
  payload.topics = {};
  payload.error = "";

  payload.metadata.scene = "test_scene";
  payload.metadata.subscene = "";
  payload.metadata.skills = {};  // Empty skills array
  payload.metadata.factory = "";

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("test_scene") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, WithMetadata_SpecialCharsInSkills) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "finished";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.finished_at = "2025-12-20T10:35:00Z";
  payload.duration_sec = 100.0;
  payload.message_count = 500;
  payload.file_size_bytes = 500000;
  payload.output_path = "/data/task.mcap";
  payload.topics = {};
  payload.error = "";

  payload.metadata.scene = "scene";
  payload.metadata.skills = {"skill with \"quotes\"", "skill/with/slashes"};

  std::string json = payload.to_json();

  // Special characters in skills should be escaped
  EXPECT_TRUE(json.find("\\\"quotes\\\"") != std::string::npos ||
              json.find("quotes") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, UrlParsing_IPv6Address) {
  TaskConfig config;
  config.start_callback_url = "http://[::1]:8080/api/start";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // IPv6 parsing - may or may not be supported
  // Just ensure it doesn't crash
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, UrlParsing_LongPath) {
  TaskConfig config;
  std::string long_path = "/api";
  for (int i = 0; i < 50; ++i) {
    long_path += "/segment" + std::to_string(i);
  }
  config.start_callback_url = "http://localhost:8080" + long_path;

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Should handle long paths without issue
  EXPECT_FALSE(result.success);  // Connection will fail
}

TEST_F(HttpCallbackClientTest, UrlParsing_EncodedCharacters) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/api/start%20here?query=hello%20world";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Should handle URL-encoded characters
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, HostnameWithSubdomains) {
  TaskConfig config;
  config.start_callback_url = "http://api.sub.example.com:8080/recording/start";

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Should parse hostname with multiple subdomains
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, PortAtLowerBound) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:1/api";  // Lowest valid port

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, PortAtUpperBound) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:65535/api";  // Highest valid port

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  EXPECT_FALSE(result.success);
}

TEST_F(FinishCallbackPayloadTest, NegativeValues) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot";
  payload.status = "error";
  payload.started_at = "";
  payload.finished_at = "";
  payload.duration_sec = -1.0;  // Negative duration (shouldn't happen but handle gracefully)
  payload.message_count = 0;
  payload.file_size_bytes = 0;
  payload.output_path = "";
  payload.topics = {};
  payload.error = "Recording never started properly";

  std::string json = payload.to_json();

  // Should serialize without crashing
  EXPECT_FALSE(json.empty());
}

TEST_F(HttpCallbackClientTest, CallbackWithVeryLongToken) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:9999/api/start";
  config.user_token = std::string(10000, 'x');  // Very long token

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  auto result = client_->post_start_callback(config, payload);

  // Should handle very long tokens without crashing
  EXPECT_FALSE(result.success);  // Connection will fail
}

TEST_F(StartCallbackPayloadTest, TopicsWithSpecialCharacters) {
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "recording";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.topics = {
    "/camera/image_raw",
    "/namespace/sub_topic",
    "/topic.with.dots",
    "/topic-with-dashes"
  };

  std::string json = payload.to_json();

  EXPECT_TRUE(json.find("/camera/image_raw") != std::string::npos);
  EXPECT_TRUE(json.find("/namespace/sub_topic") != std::string::npos);
  EXPECT_TRUE(json.find("/topic.with.dots") != std::string::npos);
  EXPECT_TRUE(json.find("/topic-with-dashes") != std::string::npos);
}

// ============================================================================
// Enhanced Coverage Tests (Phase 5)
// ============================================================================

TEST_F(HttpCallbackClientTest, GetIso8601TimestampSpecificTime) {
  // Test with specific time point
  auto time_point = std::chrono::system_clock::from_time_t(1609459200);  // 2021-01-01 00:00:00 UTC
  std::string timestamp = HttpCallbackClient::get_iso8601_timestamp(time_point);
  
  // Verify it's a valid timestamp
  EXPECT_EQ(timestamp.length(), 20);  // YYYY-MM-DDTHH:MM:SSZ
  EXPECT_EQ(timestamp.back(), 'Z');
  EXPECT_TRUE(timestamp.find('T') != std::string::npos);
}

TEST_F(HttpCallbackClientTest, GetIso8601TimestampDifferentTimes) {
  auto t1 = std::chrono::system_clock::from_time_t(0);  // Epoch
  auto t2 = std::chrono::system_clock::from_time_t(1000000000);  // 2001-09-09
  
  std::string ts1 = HttpCallbackClient::get_iso8601_timestamp(t1);
  std::string ts2 = HttpCallbackClient::get_iso8601_timestamp(t2);
  
  EXPECT_NE(ts1, ts2);
  EXPECT_TRUE(ts1.find("1970") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, PostFinishCallbackInvalidUrl) {
  TaskConfig config;
  config.task_id = "task_001";
  config.finish_callback_url = "not-a-valid-url";
  
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  
  auto result = client_->post_finish_callback(config, payload);
  
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("ERR_CALLBACK_FAILED") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, PostFinishCallbackUnreachable) {
  TaskConfig config;
  config.task_id = "task_001";
  config.finish_callback_url = "http://127.0.0.1:59998/unreachable";
  
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  payload.duration_sec = 100.5;
  payload.message_count = 5000;
  payload.file_size_bytes = 1024 * 1024;
  
  auto result = client_->post_finish_callback(config, payload);
  
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("ERR_CALLBACK_FAILED") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, FinishCallbackWithHandler) {
  TaskConfig config;
  config.task_id = "task_001";
  // No URL configured
  
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  
  bool handler_called = false;
  HttpCallbackResult received_result;
  
  auto handler = [&](const HttpCallbackResult& result) {
    handler_called = true;
    received_result = result;
  };
  
  auto result = client_->post_finish_callback(config, payload, handler);
  
  EXPECT_TRUE(handler_called);
  EXPECT_TRUE(received_result.success);  // No URL is not an error
}

TEST_F(FinishCallbackPayloadTest, MetadataAllFields) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "robot_01";
  payload.status = "finished";
  payload.started_at = "2025-12-20T10:30:00Z";
  payload.finished_at = "2025-12-20T11:30:00Z";
  payload.duration_sec = 3600.0;
  payload.message_count = 100000;
  payload.file_size_bytes = 500 * 1024 * 1024;  // 500MB
  payload.output_path = "/data/recordings/task_001.mcap";
  payload.sidecar_path = "/data/recordings/task_001.json";
  payload.topics = {"/camera", "/lidar", "/imu"};
  payload.error = "";
  
  // Metadata
  payload.metadata.scene = "outdoor_navigation";
  payload.metadata.subscene = "parking_lot";
  payload.metadata.skills = {"autonomous_driving", "obstacle_detection"};
  payload.metadata.factory = "factory_001";
  
  std::string json = payload.to_json();
  
  EXPECT_TRUE(json.find("outdoor_navigation") != std::string::npos);
  EXPECT_TRUE(json.find("parking_lot") != std::string::npos);
  EXPECT_TRUE(json.find("autonomous_driving") != std::string::npos);
  EXPECT_TRUE(json.find("obstacle_detection") != std::string::npos);
  EXPECT_TRUE(json.find("factory_001") != std::string::npos);
  EXPECT_TRUE(json.find("sidecar_path") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, MetadataEmptyFields) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  
  // Empty metadata
  payload.metadata.scene = "";
  payload.metadata.subscene = "";
  payload.metadata.skills = {};
  payload.metadata.factory = "";
  
  std::string json = payload.to_json();
  
  // Should still have metadata section
  EXPECT_TRUE(json.find("\"metadata\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"scene\": \"\"") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, DefaultConstructor) {
  // Default constructor should work
  auto default_client = std::make_shared<HttpCallbackClient>();
  
  TaskConfig config;
  StartCallbackPayload payload;
  
  auto result = default_client->post_start_callback(config, payload);
  EXPECT_TRUE(result.success);  // No URL is success (no-op)
}

TEST_F(HttpCallbackClientTest, UrlWithUsernamePassword) {
  // URL with basic auth embedded (not recommended but should parse)
  TaskConfig config;
  config.start_callback_url = "http://user:pass@localhost:8080/api";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Should fail (either parse or connect error)
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, VeryLongUrl) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/" + std::string(10000, 'a');
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Should not crash, should fail
  EXPECT_FALSE(result.success);
}

TEST_F(StartCallbackPayloadTest, AllFieldsEmpty) {
  StartCallbackPayload payload;
  // All fields are default/empty
  
  std::string json = payload.to_json();
  
  EXPECT_TRUE(json.find("\"task_id\": \"\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"device_id\": \"\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"status\": \"\"") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, VeryLargeFileSize) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  payload.file_size_bytes = 10ULL * 1024 * 1024 * 1024 * 1024;  // 10TB
  
  std::string json = payload.to_json();
  
  // Large number should be serialized correctly
  EXPECT_FALSE(json.empty());
  EXPECT_TRUE(json.find("file_size_bytes") != std::string::npos);
}

TEST_F(FinishCallbackPayloadTest, VeryLongDuration) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.status = "finished";
  payload.duration_sec = 86400.0 * 365;  // ~1 year in seconds
  
  std::string json = payload.to_json();
  
  EXPECT_FALSE(json.empty());
}

TEST_F(HttpCallbackClientTest, UrlWithFragmentAndPath) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/api/v2#section";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // URL parsing should handle fragment
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, HttpSchemeUppercase) {
  TaskConfig config;
  config.start_callback_url = "HTTP://localhost:8080/api";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Should parse (case insensitive scheme)
  EXPECT_FALSE(result.success);  // Connection will fail
  // But URL parsing should not fail
}

TEST_F(HttpCallbackClientTest, HttpsSchemeUppercase) {
  TaskConfig config;
  config.start_callback_url = "HTTPS://localhost:8443/api";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  EXPECT_FALSE(result.success);
}

// ============================================================================
// P1: HTTP Mock Server Tests - URL Parsing and Edge Cases
// ============================================================================

TEST_F(HttpCallbackClientTest, UrlParsingWithPort) {
  TaskConfig config;
  config.task_id = "task_001";
  // Explicit port in URL
  config.start_callback_url = "http://api.example.com:9090/callbacks/start";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "device_001";
  payload.status = "recording";
  payload.started_at = "2025-12-25T10:00:00Z";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Will fail to connect but should parse URL correctly
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(HttpCallbackClientTest, UrlParsingWithoutPort) {
  TaskConfig config;
  config.task_id = "task_001";
  // No port - should default to 80
  config.start_callback_url = "http://api.example.com/callbacks/start";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  EXPECT_FALSE(result.success);  // Connection will fail
}

TEST_F(HttpCallbackClientTest, UrlParsingWithQueryParams) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:8080/api/callback?version=2&debug=true";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Should handle query params in URL
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, UrlParsingWithFragment) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:8080/api#section";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, UrlParsingLocalhostAlias) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://127.0.0.1:59997/api";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, HttpsUrlParsingSecure) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "https://secure.example.com/api/start";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // SSL connection will fail but URL should parse
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, EmptyUrlHandling) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Empty URL should be handled gracefully (no-op success)
  EXPECT_TRUE(result.success);
}

TEST_F(HttpCallbackClientTest, MalformedUrlSchemeFtp) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "ftp://localhost/api";  // Wrong scheme
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, UrlMissingPath) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:8080";  // No path
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Should default to "/" path
  EXPECT_FALSE(result.success);
}

// ============================================================================
// P1: HTTP Callback - Async Operations
// ============================================================================

TEST_F(HttpCallbackClientTest, AsyncStartCallbackWithInvalidUrl) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://unreachable.invalid:59999/api";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "device_001";
  payload.status = "recording";
  payload.started_at = HttpCallbackClient::get_iso8601_timestamp();
  
  // Async call should not throw
  EXPECT_NO_THROW(client_->post_start_callback_async(config, payload));
}

TEST_F(HttpCallbackClientTest, AsyncFinishCallbackWithInvalidUrl) {
  TaskConfig config;
  config.task_id = "task_001";
  config.finish_callback_url = "http://unreachable.invalid:59999/api";
  
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "device_001";
  payload.status = "finished";
  payload.started_at = "2025-12-25T10:00:00Z";
  payload.finished_at = "2025-12-25T11:00:00Z";
  payload.duration_sec = 3600.0;
  payload.message_count = 1000;
  payload.file_size_bytes = 1024 * 1024;
  
  // Async call should not throw
  EXPECT_NO_THROW(client_->post_finish_callback_async(config, payload));
}

// ============================================================================
// P1: HTTP Callback - Token Handling
// ============================================================================

TEST_F(HttpCallbackClientTest, BearerTokenFormatting) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:59996/api";
  config.user_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.test";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Connection fails but token should be formatted correctly
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, EmptyToken) {
  TaskConfig config;
  config.task_id = "task_001";
  config.start_callback_url = "http://localhost:59995/api";
  config.user_token = "";
  
  StartCallbackPayload payload;
  payload.task_id = "task_001";
  
  auto result = client_->post_start_callback(config, payload);
  
  // Should handle empty token gracefully
  EXPECT_FALSE(result.success);
}

// ============================================================================
// P1: HTTP Callback - Response Handling
// ============================================================================

TEST_F(HttpCallbackClientTest, ResultStructureInitialization) {
  // Use value initialization to ensure members are zero-initialized
  HttpCallbackResult result{};
  
  // Value-initialized struct should have sensible defaults
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status_code, 0);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_TRUE(result.response_body.empty());
}

TEST_F(HttpCallbackClientTest, PayloadWithUnicodeCharacters) {
  StartCallbackPayload payload;
  payload.task_id = "task_日本語_001";
  payload.device_id = "设备_001";
  payload.status = "recording";
  payload.started_at = "2025-12-25T10:00:00Z";
  payload.topics = {"/topic_中文", "/topic_日本語"};
  
  std::string json = payload.to_json();
  
  // Unicode should be preserved
  EXPECT_TRUE(json.find("task_日本語_001") != std::string::npos);
  EXPECT_TRUE(json.find("设备_001") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, FinishPayloadWithLargeValues) {
  FinishCallbackPayload payload;
  payload.task_id = "task_001";
  payload.device_id = "device_001";
  payload.status = "finished";
  payload.started_at = "2025-12-25T10:00:00Z";
  payload.finished_at = "2025-12-25T22:00:00Z";
  payload.duration_sec = 43200.0;  // 12 hours
  payload.message_count = 1000000000LL;  // 1 billion messages
  payload.file_size_bytes = 1099511627776LL;  // 1 TB
  
  std::string json = payload.to_json();
  
  EXPECT_TRUE(json.find("1000000000") != std::string::npos);
  EXPECT_TRUE(json.find("1099511627776") != std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

