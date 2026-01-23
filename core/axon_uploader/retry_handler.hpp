// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RETRY_HANDLER_HPP
#define AXON_RETRY_HANDLER_HPP

#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <set>
#include <string>

namespace axon {
namespace uploader {

/**
 * Configuration for retry behavior
 */
struct RetryConfig {
  int max_retries = 5;                            // Maximum retry attempts
  std::chrono::milliseconds initial_delay{1000};  // Initial delay (1 second)
  std::chrono::milliseconds max_delay{300000};    // Maximum delay (5 minutes)
  double exponential_base = 2.0;                  // Exponential backoff base
  bool jitter = true;                             // Add random jitter
  double jitter_factor = 0.5;                     // Jitter range: [1-factor, 1+factor]
};

/**
 * Retry handler with exponential backoff and jitter
 *
 * Implements a retry strategy suitable for network operations:
 * - Exponential backoff to avoid thundering herd
 * - Random jitter to prevent synchronized retries
 * - Configurable maximum delay cap
 *
 * Thread-safe: multiple threads can call getDelay() concurrently.
 */
class RetryHandler {
public:
  /**
   * Create retry handler with configuration
   */
  explicit RetryHandler(const RetryConfig& config = {})
      : config_(config)
      , rng_(std::random_device{}()) {}

  /**
   * Calculate delay before next retry attempt
   *
   * Delay formula: initial_delay * (base ^ retry_count) * jitter
   * Where jitter is a random factor in [1 - jitter_factor, 1 + jitter_factor]
   *
   * @param retry_count Current retry attempt (0-indexed)
   * @return Delay duration before next retry
   */
  std::chrono::milliseconds getDelay(int retry_count) const {
    // Calculate base delay with exponential backoff
    double delay_ms = static_cast<double>(config_.initial_delay.count()) *
                      std::pow(config_.exponential_base, static_cast<double>(retry_count));

    // Cap at maximum delay
    delay_ms = std::min(delay_ms, static_cast<double>(config_.max_delay.count()));

    // Apply jitter if enabled (thread-safe via mutex)
    if (config_.jitter) {
      std::lock_guard<std::mutex> lock(rng_mutex_);
      std::uniform_real_distribution<> dist(
        1.0 - config_.jitter_factor, 1.0 + config_.jitter_factor
      );
      delay_ms *= dist(rng_);
    }

    // Ensure minimum delay of 1ms
    delay_ms = std::max(delay_ms, 1.0);

    return std::chrono::milliseconds(static_cast<int64_t>(delay_ms));
  }

  /**
   * Check if another retry should be attempted
   *
   * @param retry_count Current retry attempt (0-indexed)
   * @return true if retry_count < max_retries
   */
  bool shouldRetry(int retry_count) const {
    return retry_count < config_.max_retries;
  }

  /**
   * Get the maximum number of retries
   */
  int maxRetries() const {
    return config_.max_retries;
  }

  /**
   * Check if an error code is retryable
   *
   * Transient errors (network issues, rate limiting) should be retried.
   * Permanent errors (bad request, not found) should not.
   *
   * @param error_code S3/HTTP error code
   * @return true if the error is transient and should be retried
   */
  static bool isRetryableError(const std::string& error_code) {
    static const std::set<std::string> retryable = {// S3/HTTP errors
                                                    "RequestTimeout",
                                                    "ServiceUnavailable",
                                                    "InternalError",
                                                    "SlowDown",
                                                    "RequestTimeTooSkewed",
                                                    "OperationAborted",

                                                    // Network errors
                                                    "ConnectionReset",
                                                    "ConnectionTimeout",
                                                    "ConnectionRefused",
                                                    "NetworkingError",
                                                    "UnknownEndpoint",

                                                    // MinIO-specific
                                                    "XMinioServerNotInitialized",
                                                    "XAmzContentSHA256Mismatch",

                                                    // Generic
                                                    "Throttling",
                                                    "ThrottlingException",
                                                    "ProvisionedThroughputExceededException",
                                                    "TransientError"};
    return retryable.count(error_code) > 0;
  }

  /**
   * Calculate next retry time from now
   *
   * @param retry_count Current retry attempt (0-indexed)
   * @return Time point for next retry attempt
   */
  std::chrono::steady_clock::time_point nextRetryTime(int retry_count) const {
    return std::chrono::steady_clock::now() + getDelay(retry_count);
  }

  /**
   * Get configuration
   */
  const RetryConfig& config() const {
    return config_;
  }

private:
  RetryConfig config_;
  mutable std::mt19937 rng_;      // mutable for const getDelay()
  mutable std::mutex rng_mutex_;  // protects rng_ for thread-safe jitter
};

}  // namespace uploader
}  // namespace axon

#endif  // AXON_RETRY_HANDLER_HPP
