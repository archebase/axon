#ifndef AXON_UPLOADER_TEST_ERROR_INJECTION_HPP
#define AXON_UPLOADER_TEST_ERROR_INJECTION_HPP

#include <atomic>
#include <functional>
#include <string>

namespace axon {
namespace uploader {
namespace test {

/**
 * Simple error injection framework for testing error paths
 *
 * This provides basic error injection capabilities for testing.
 * For more complex scenarios (AWS SDK mocking, file system mocking),
 * consider using dependency injection or test doubles.
 */

/**
 * File system error injection
 *
 * Note: Full file system mocking requires more sophisticated approaches.
 * This provides basic helpers for common scenarios.
 */
class FileSystemErrorInjector {
public:
  // Simulate file not found by using non-existent path
  static std::string getNonExistentPath() {
    return "/nonexistent/path/that/does/not/exist/file.mcap";
  }

  // Simulate permission denied by using root-only path (on Unix)
  static std::string getPermissionDeniedPath() {
    return "/root/denied/file.mcap";
  }
};

/**
 * Database error injection
 *
 * Note: SQLite error injection typically requires mocking the database layer.
 * For unit tests, we can test with invalid database paths or corrupted databases.
 */
class DatabaseErrorInjector {
public:
  // Get a path that will cause database creation to fail
  static std::string getInvalidDbPath() {
    return "/invalid/path/that/cannot/be/created/state.db";
  }

  // Get a path to a read-only location (simulates permission issues)
  static std::string getReadOnlyDbPath() {
    return "/root/readonly/state.db";
  }
};

/**
 * Network error simulation
 *
 * Note: Full network error injection requires mocking HTTP clients.
 * For integration tests, we can use invalid endpoints.
 */
class NetworkErrorInjector {
public:
  // Get an invalid endpoint that will cause connection failures
  static std::string getInvalidEndpoint() {
    return "http://localhost:19999";  // Unlikely to be running
  }

  // Get an endpoint that will timeout quickly
  static std::string getTimeoutEndpoint() {
    return "http://192.0.2.1:9000";  // Test network (RFC 3330) - should timeout
  }
};

/**
 * Error code injection for S3 errors
 *
 * This helps test error code handling without requiring actual S3 errors.
 */
class S3ErrorCodeInjector {
public:
  // Common retryable error codes
  static constexpr const char* RETRYABLE_ERRORS[] = {
    "RequestTimeout", "ServiceUnavailable", "InternalError", "SlowDown", "Throttling"};

  // Common non-retryable error codes
  static constexpr const char* NON_RETRYABLE_ERRORS[] = {
    "AccessDenied", "NoSuchBucket", "InvalidArgument", "NoSuchKey"};

  static bool isRetryable(const std::string& error_code) {
    for (const char* retryable : RETRYABLE_ERRORS) {
      if (error_code == retryable) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace test
}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOADER_TEST_ERROR_INJECTION_HPP
