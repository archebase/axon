#ifndef AXON_RECORDER_HTTP_CALLBACK_CLIENT_HPP
#define AXON_RECORDER_HTTP_CALLBACK_CLIENT_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "task_config.hpp"

namespace axon {
namespace recorder {

/**
 * Payload for the start recording callback.
 */
struct StartCallbackPayload {
  std::string task_id;
  std::string device_id;
  std::string status;  // "recording"
  std::string started_at;  // ISO8601 timestamp
  std::vector<std::string> topics;

  std::string to_json() const;
};

/**
 * Metadata summary for finish callback (matches design spec).
 */
struct CallbackMetadata {
  std::string scene;
  std::string subscene;
  std::vector<std::string> skills;
  std::string factory;
};

/**
 * Payload for the finish recording callback.
 */
struct FinishCallbackPayload {
  std::string task_id;
  std::string device_id;
  std::string status;  // "finished" or "cancelled"
  std::string started_at;  // ISO8601 timestamp
  std::string finished_at;  // ISO8601 timestamp
  double duration_sec;
  int64_t message_count;
  int64_t file_size_bytes;
  std::string output_path;
  std::string sidecar_path;  // Path to the JSON sidecar file
  std::vector<std::string> topics;
  CallbackMetadata metadata;  // Task metadata summary
  std::string error;  // Empty if no error

  std::string to_json() const;
};

/**
 * Result of an HTTP callback request.
 */
struct HttpCallbackResult {
  bool success;
  int status_code;
  std::string error_message;
  std::string response_body;
};

/**
 * Callback function type for async HTTP operations.
 */
using HttpCallbackHandler = std::function<void(const HttpCallbackResult&)>;

/**
 * HttpCallbackClient sends HTTP POST callbacks to the server when recording
 * starts or finishes. Uses Boost.Beast for HTTP communication.
 *
 * Features:
 * - Async HTTP POST with non-blocking API
 * - JWT Bearer token authentication
 * - SSL/TLS support for HTTPS
 * - Timeout handling
 * - Error reporting
 *
 * Note: This class uses std::enable_shared_from_this to ensure safe async
 * operations. Always create instances using std::make_shared<HttpCallbackClient>().
 */
class HttpCallbackClient : public std::enable_shared_from_this<HttpCallbackClient> {
public:
  /**
   * Configuration for the HTTP client.
   */
  struct Config {
    std::chrono::seconds request_timeout{30};
  };

  HttpCallbackClient();
  explicit HttpCallbackClient(const Config& config);
  ~HttpCallbackClient();

  // Non-copyable
  HttpCallbackClient(const HttpCallbackClient&) = delete;
  HttpCallbackClient& operator=(const HttpCallbackClient&) = delete;

  /**
   * Post start callback notification to server.
   * This is called when recording transitions from READY to RECORDING.
   *
   * @param task_config The task configuration with callback URL and token
   * @param payload The callback payload
   * @param handler Optional async result handler (if null, operation is synchronous)
   * @return Result of the operation (for sync calls)
   */
  HttpCallbackResult post_start_callback(
    const TaskConfig& task_config, const StartCallbackPayload& payload,
    HttpCallbackHandler handler = nullptr
  );

  /**
   * Post finish callback notification to server.
   * This is called when recording transitions to IDLE (finish or cancel).
   *
   * @param task_config The task configuration with callback URL and token
   * @param payload The callback payload
   * @param handler Optional async result handler (if null, operation is synchronous)
   * @return Result of the operation (for sync calls)
   */
  HttpCallbackResult post_finish_callback(
    const TaskConfig& task_config, const FinishCallbackPayload& payload,
    HttpCallbackHandler handler = nullptr
  );

  /**
   * Post start callback asynchronously (fire-and-forget with logging).
   */
  void post_start_callback_async(const TaskConfig& task_config, const StartCallbackPayload& payload);

  /**
   * Post finish callback asynchronously (fire-and-forget with logging).
   */
  void post_finish_callback_async(
    const TaskConfig& task_config, const FinishCallbackPayload& payload
  );

  /**
   * Generate ISO8601 timestamp string for the current time.
   */
  static std::string get_iso8601_timestamp();

  /**
   * Convert a time_point to ISO8601 timestamp string.
   */
  static std::string get_iso8601_timestamp(std::chrono::system_clock::time_point tp);

private:
  /**
   * Internal HTTP POST implementation.
   */
  HttpCallbackResult http_post(
    const std::string& url, const std::string& body, const std::string& auth_token
  );

  /**
   * Parse URL into host, port, and path.
   */
  bool parse_url(
    const std::string& url, std::string& host, std::string& port, std::string& path, bool& use_ssl
  );

  Config config_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_HTTP_CALLBACK_CLIENT_HPP

