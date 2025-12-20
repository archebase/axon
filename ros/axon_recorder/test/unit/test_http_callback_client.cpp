/**
 * Unit tests for HttpCallbackClient
 */

#include <gtest/gtest.h>

#include <regex>
#include <string>

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
  HttpCallbackClient client_;

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

  auto result = client_.post_start_callback(config, payload);

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

  auto result = client_.post_finish_callback(config, payload);

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

  auto result = client_.post_start_callback(config, payload);

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

  auto result = client_.post_start_callback(config, payload);

  // Should fail (can't connect)
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("ERR_CALLBACK_FAILED") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, ConfigWithTimeout) {
  HttpCallbackClient::Config config;
  config.connect_timeout = std::chrono::seconds(5);
  config.request_timeout = std::chrono::seconds(10);
  config.max_retries = 2;

  HttpCallbackClient client_with_config(config);

  // Just verify construction succeeds
  TaskConfig task_config;
  StartCallbackPayload payload;

  auto result = client_with_config.post_start_callback(task_config, payload);
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

  auto result = client_.post_start_callback(config, payload, handler);

  EXPECT_TRUE(handler_called);
  EXPECT_TRUE(received_result.success);
  EXPECT_EQ(result.success, received_result.success);
}

TEST_F(HttpCallbackClientTest, CallbackHandlerNullptr) {
  TaskConfig config;

  StartCallbackPayload payload;
  payload.task_id = "task_001";

  // Should not crash with nullptr handler
  auto result = client_.post_start_callback(config, payload, nullptr);
  EXPECT_TRUE(result.success);
}

// ============================================================================
// URL Parsing Tests (via error messages)
// ============================================================================

TEST_F(HttpCallbackClientTest, UrlParsing_HttpScheme) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080/api/start";

  StartCallbackPayload payload;
  auto result = client_.post_start_callback(config, payload);

  // Will fail to connect, but shouldn't fail URL parsing
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, UrlParsing_HttpsScheme) {
  TaskConfig config;
  config.start_callback_url = "https://localhost:8443/api/start";

  StartCallbackPayload payload;
  auto result = client_.post_start_callback(config, payload);

  // Will fail (HTTPS not fully supported in this implementation)
  EXPECT_FALSE(result.success);
}

TEST_F(HttpCallbackClientTest, UrlParsing_NoPort) {
  TaskConfig config;
  config.start_callback_url = "http://localhost/api/start";  // Default port 80

  StartCallbackPayload payload;
  auto result = client_.post_start_callback(config, payload);

  // Will fail to connect, but parsing should work
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

TEST_F(HttpCallbackClientTest, UrlParsing_NoPath) {
  TaskConfig config;
  config.start_callback_url = "http://localhost:8080";  // No path

  StartCallbackPayload payload;
  auto result = client_.post_start_callback(config, payload);

  // Should use "/" as default path
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.find("Invalid URL format") != std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

