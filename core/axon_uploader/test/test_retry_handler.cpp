// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for RetryHandler
 */

#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "retry_handler.hpp"

using namespace axon::uploader;

class RetryHandlerTest : public ::testing::Test {
protected:
  void SetUp() override {
    RetryConfig config;
    config.max_retries = 5;
    config.initial_delay = std::chrono::milliseconds(1000);
    config.max_delay = std::chrono::milliseconds(60000);
    config.exponential_base = 2.0;
    config.jitter = false;  // Disable jitter for deterministic tests
    handler_ = std::make_unique<RetryHandler>(config);
  }

  std::unique_ptr<RetryHandler> handler_;
};

TEST_F(RetryHandlerTest, ExponentialBackoff) {
  // Without jitter, delays should be deterministic
  auto delay0 = handler_->getDelay(0);
  auto delay1 = handler_->getDelay(1);
  auto delay2 = handler_->getDelay(2);
  auto delay3 = handler_->getDelay(3);

  // Expected: 1000, 2000, 4000, 8000 ms
  EXPECT_EQ(delay0.count(), 1000);
  EXPECT_EQ(delay1.count(), 2000);
  EXPECT_EQ(delay2.count(), 4000);
  EXPECT_EQ(delay3.count(), 8000);
}

TEST_F(RetryHandlerTest, MaxDelayCapy) {
  // Very high retry count should be capped at max_delay
  auto delay = handler_->getDelay(10);  // 2^10 * 1000 = 1,024,000 ms, should be capped at 60,000

  EXPECT_EQ(delay.count(), 60000);
}

TEST_F(RetryHandlerTest, ShouldRetry) {
  EXPECT_TRUE(handler_->shouldRetry(0));
  EXPECT_TRUE(handler_->shouldRetry(1));
  EXPECT_TRUE(handler_->shouldRetry(4));
  EXPECT_FALSE(handler_->shouldRetry(5));  // max_retries = 5
  EXPECT_FALSE(handler_->shouldRetry(10));
}

TEST_F(RetryHandlerTest, MaxRetries) {
  EXPECT_EQ(handler_->maxRetries(), 5);
}

TEST_F(RetryHandlerTest, JitterEnabled) {
  RetryConfig config;
  config.initial_delay = std::chrono::milliseconds(1000);
  config.jitter = true;
  config.jitter_factor = 0.5;
  RetryHandler jitter_handler(config);

  // With jitter, delays should vary
  std::set<int64_t> delays;
  for (int i = 0; i < 100; ++i) {
    delays.insert(jitter_handler.getDelay(0).count());
  }

  // Should have multiple different values (jitter creates variation)
  EXPECT_GT(delays.size(), 1);

  // All delays should be within jitter range [500, 1500]
  for (auto d : delays) {
    EXPECT_GE(d, 500);
    EXPECT_LE(d, 1500);
  }
}

TEST_F(RetryHandlerTest, NextRetryTime) {
  auto now = std::chrono::steady_clock::now();
  auto next = handler_->nextRetryTime(0);

  // Should be approximately 1 second in the future (no jitter)
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(next - now);
  EXPECT_GE(diff.count(), 990);
  EXPECT_LE(diff.count(), 1100);
}

TEST_F(RetryHandlerTest, IsRetryableError) {
  // Retryable errors
  EXPECT_TRUE(RetryHandler::isRetryableError("RequestTimeout"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ServiceUnavailable"));
  EXPECT_TRUE(RetryHandler::isRetryableError("InternalError"));
  EXPECT_TRUE(RetryHandler::isRetryableError("SlowDown"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ConnectionTimeout"));
  EXPECT_TRUE(RetryHandler::isRetryableError("NetworkingError"));

  // Non-retryable errors
  EXPECT_FALSE(RetryHandler::isRetryableError("AccessDenied"));
  EXPECT_FALSE(RetryHandler::isRetryableError("NoSuchBucket"));
  EXPECT_FALSE(RetryHandler::isRetryableError("InvalidArgument"));
  EXPECT_FALSE(RetryHandler::isRetryableError("NoSuchKey"));
  EXPECT_FALSE(RetryHandler::isRetryableError("UnknownError"));
}

TEST_F(RetryHandlerTest, ConfigAccess) {
  const auto& config = handler_->config();
  EXPECT_EQ(config.max_retries, 5);
  EXPECT_EQ(config.initial_delay.count(), 1000);
  EXPECT_EQ(config.max_delay.count(), 60000);
  EXPECT_DOUBLE_EQ(config.exponential_base, 2.0);
  EXPECT_FALSE(config.jitter);
}

TEST_F(RetryHandlerTest, ZeroRetryCount) {
  RetryConfig config;
  config.max_retries = 0;
  RetryHandler no_retry_handler(config);

  EXPECT_FALSE(no_retry_handler.shouldRetry(0));
}

TEST_F(RetryHandlerTest, LargeInitialDelay) {
  RetryConfig config;
  config.initial_delay = std::chrono::milliseconds(30000);  // 30 seconds
  config.max_delay = std::chrono::milliseconds(60000);
  config.jitter = false;
  RetryHandler handler(config);

  auto delay0 = handler.getDelay(0);
  auto delay1 = handler.getDelay(1);

  EXPECT_EQ(delay0.count(), 30000);
  EXPECT_EQ(delay1.count(), 60000);  // 60000 > 30000 * 2, but capped at max
}

TEST_F(RetryHandlerTest, IsRetryableErrorAllCases) {
  // Test all retryable error codes from the implementation
  EXPECT_TRUE(RetryHandler::isRetryableError("RequestTimeout"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ServiceUnavailable"));
  EXPECT_TRUE(RetryHandler::isRetryableError("InternalError"));
  EXPECT_TRUE(RetryHandler::isRetryableError("SlowDown"));
  EXPECT_TRUE(RetryHandler::isRetryableError("RequestTimeTooSkewed"));
  EXPECT_TRUE(RetryHandler::isRetryableError("OperationAborted"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ConnectionReset"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ConnectionTimeout"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ConnectionRefused"));
  EXPECT_TRUE(RetryHandler::isRetryableError("NetworkingError"));
  EXPECT_TRUE(RetryHandler::isRetryableError("UnknownEndpoint"));
  EXPECT_TRUE(RetryHandler::isRetryableError("XMinioServerNotInitialized"));
  EXPECT_TRUE(RetryHandler::isRetryableError("XAmzContentSHA256Mismatch"));
  EXPECT_TRUE(RetryHandler::isRetryableError("Throttling"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ThrottlingException"));
  EXPECT_TRUE(RetryHandler::isRetryableError("ProvisionedThroughputExceededException"));
  EXPECT_TRUE(RetryHandler::isRetryableError("TransientError"));

  // Test non-retryable errors
  EXPECT_FALSE(RetryHandler::isRetryableError("AccessDenied"));
  EXPECT_FALSE(RetryHandler::isRetryableError("NoSuchBucket"));
  EXPECT_FALSE(RetryHandler::isRetryableError("InvalidArgument"));
  EXPECT_FALSE(RetryHandler::isRetryableError("NoSuchKey"));
  EXPECT_FALSE(RetryHandler::isRetryableError(""));
  EXPECT_FALSE(RetryHandler::isRetryableError("SomeRandomError"));
}

TEST_F(RetryHandlerTest, GetDelayWithJitterDisabled) {
  // Test getDelay() with jitter disabled (already tested, but ensure coverage)
  RetryConfig config;
  config.initial_delay = std::chrono::milliseconds(1000);
  config.max_delay = std::chrono::milliseconds(10000);
  config.jitter = false;
  RetryHandler handler(config);

  auto delay = handler.getDelay(0);
  EXPECT_EQ(delay.count(), 1000);

  delay = handler.getDelay(1);
  EXPECT_EQ(delay.count(), 2000);
}

TEST_F(RetryHandlerTest, GetDelayMinimumOneMs) {
  // Test that delay is always at least 1ms
  RetryConfig config;
  config.initial_delay = std::chrono::milliseconds(0);
  config.max_delay = std::chrono::milliseconds(1000);
  config.jitter = false;
  RetryHandler handler(config);

  auto delay = handler.getDelay(0);
  EXPECT_GE(delay.count(), 1);  // Should be at least 1ms
}

TEST_F(RetryHandlerTest, NextRetryTimeMultipleCalls) {
  // Test nextRetryTime() with multiple retry counts
  auto time0 = handler_->nextRetryTime(0);
  auto time1 = handler_->nextRetryTime(1);
  auto time2 = handler_->nextRetryTime(2);

  // Each should be further in the future
  EXPECT_GT(time1, time0);
  EXPECT_GT(time2, time1);
}

TEST_F(RetryHandlerTest, ShouldRetryBoundary) {
  // Test shouldRetry() at boundary conditions
  RetryConfig config;
  config.max_retries = 3;
  RetryHandler handler(config);

  EXPECT_TRUE(handler.shouldRetry(0));
  EXPECT_TRUE(handler.shouldRetry(1));
  EXPECT_TRUE(handler.shouldRetry(2));
  EXPECT_FALSE(handler.shouldRetry(3));  // At max_retries
  EXPECT_FALSE(handler.shouldRetry(4));  // Beyond max_retries
}

TEST_F(RetryHandlerTest, ExponentialBaseVariations) {
  // Test with different exponential bases
  RetryConfig config;
  config.initial_delay = std::chrono::milliseconds(1000);
  config.max_delay = std::chrono::milliseconds(100000);
  config.exponential_base = 1.5;  // Slower growth
  config.jitter = false;
  RetryHandler handler(config);

  auto delay0 = handler.getDelay(0);
  auto delay1 = handler.getDelay(1);
  auto delay2 = handler.getDelay(2);

  EXPECT_EQ(delay0.count(), 1000);
  EXPECT_EQ(delay1.count(), 1500);  // 1000 * 1.5
  EXPECT_EQ(delay2.count(), 2250);  // 1000 * 1.5^2
}

TEST_F(RetryHandlerTest, JitterFactorVariations) {
  // Test with different jitter factors
  RetryConfig config;
  config.initial_delay = std::chrono::milliseconds(1000);
  config.jitter = true;
  config.jitter_factor = 0.1;  // Small jitter
  RetryHandler handler(config);

  // With small jitter, delays should be in [900, 1100] range
  for (int i = 0; i < 10; ++i) {
    auto delay = handler.getDelay(0);
    EXPECT_GE(delay.count(), 900);
    EXPECT_LE(delay.count(), 1100);
  }
}
