# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for retry logic in Axon HTTP client."""

from unittest import mock
from unittest.mock import patch
from time import time

import pytest

from axon_client.retry import RetryConfig, retry_on_exception, calculate_delay


class TestRetryConfig:
    """Tests for RetryConfig dataclass."""

    def test_default_config(self):
        """Test default retry configuration."""
        config = RetryConfig()
        assert config.max_attempts == 3
        assert config.base_delay == 1.0
        assert config.max_delay == 60.0
        assert config.exponential_base == 2.0
        assert config.jitter == True
        assert config.jitter_range == 0.1

    def test_custom_config(self):
        """Test custom retry configuration."""
        config = RetryConfig(
            max_attempts=5,
            base_delay=0.5,
            max_delay=30.0,
            exponential_base=3.0,
            jitter=False
        )
        assert config.max_attempts == 5
        assert config.base_delay == 0.5
        assert config.max_delay == 30.0
        assert config.exponential_base == 3.0
        assert config.jitter is False


class TestCalculateDelay:
    """Tests for calculate_delay function."""

    def test_no_jitter(self):
        """Test delay calculation without jitter."""
        delay = calculate_delay(
            attempt=1,
            base_delay=1.0,
            exponential_base=2.0,
            max_delay=60.0,
            jitter=False
        )
        assert delay == 2.0  # 1.0 * 2^1

    def test_exponential_backoff(self):
        """Test exponential backoff progression."""
        assert calculate_delay(0, 1.0, 2.0, 100.0, False) == 1.0
        assert calculate_delay(1, 1.0, 2.0, 100.0, False) == 2.0
        assert calculate_delay(2, 1.0, 2.0, 100.0, False) == 4.0
        assert calculate_delay(3, 1.0, 2.0, 100.0, False) == 8.0

    def test_max_delay_cap(self):
        """Test that delay is capped at max_delay."""
        delay = calculate_delay(
            attempt=10,
            base_delay=1.0,
            exponential_base=2.0,
            max_delay=10.0,
            jitter=False
        )
        assert delay == 10.0

    def test_jitter_range(self):
        """Test that jitter keeps delay within expected range."""
        base_delay = 2.0
        delays = []
        for _ in range(100):
            delay = calculate_delay(
                attempt=1,
                base_delay=base_delay,
                exponential_base=2.0,
                max_delay=60.0,
                jitter=True,
                jitter_range=0.1
            )
            delays.append(delay)

        # With 10% jitter, delay should be in [1.8, 2.2]
        assert min(delays) >= 1.7  # Allow some margin
        assert max(delays) <= 2.3

    def test_different_exponential_base(self):
        """Test exponential backoff with different base."""
        delay = calculate_delay(
            attempt=2,
            base_delay=1.0,
            exponential_base=3.0,
            max_delay=100.0,
            jitter=False
        )
        assert delay == 9.0  # 1.0 * 3^2


class TestRetryOnException:
    """Tests for retry_on_exception decorator."""

    def test_success_on_first_attempt(self):
        """Test function succeeds on first attempt."""
        @retry_on_exception(RetryConfig(max_attempts=3))
        def failing_function():
            return "success"

        result = failing_function()
        assert result == "success"

    def test_retry_until_success(self):
        """Test function succeeds after retries."""
        attempts = [0]

        @retry_on_exception(RetryConfig(max_attempts=5, base_delay=0.01))
        def failing_function():
            attempts[0] += 1
            if attempts[0] < 3:
                raise ConnectionError("Failed")
            return "success"

        result = failing_function()
        assert result == "success"
        assert attempts[0] == 3

    def test_max_attempts_exceeded(self):
        """Test function fails after max attempts."""
        @retry_on_exception(RetryConfig(max_attempts=3, base_delay=0.01))
        def failing_function():
            raise ConnectionError("Failed")

        with pytest.raises(ConnectionError, match="Failed"):
            failing_function()

    def test_only_retry_on_specified_exceptions(self):
        """Test that only specified exceptions trigger retry."""
        @retry_on_exception(
            RetryConfig(max_attempts=3, base_delay=0.01),
            retryable_exceptions=(ConnectionError,)
        )
        def mixed_function():
            nonlocal attempt_count
            attempt_count += 1
            if attempt_count == 1:
                raise ConnectionError("Retry this")
            else:
                raise ValueError("Don't retry this")

        attempt_count = 0

        with pytest.raises(ValueError, match="Don't retry this"):
            mixed_function()

        # Should only retry once for ConnectionError
        assert attempt_count == 2

    def test_no_retry_for_unlisted_exception(self):
        """Test that unlisted exceptions are not retried."""
        @retry_on_exception(
            RetryConfig(max_attempts=3, base_delay=0.01),
            retryable_exceptions=(ConnectionError,)
        )
        def failing_function():
            raise ValueError("Don't retry")

        with pytest.raises(ValueError, match="Don't retry"):
            failing_function()

    def test_retry_timing(self):
        """Test that retries delay approximately the expected time."""
        @retry_on_exception(RetryConfig(max_attempts=3, base_delay=0.1, jitter=False))
        def failing_function():
            raise ConnectionError("Failed")

        start = time()
        try:
            failing_function()
        except ConnectionError:
            pass
        elapsed = time() - start

        # Should have 2 delays: 0.1s and 0.2s = 0.3s total (plus some overhead)
        assert elapsed >= 0.25  # Allow for timing variations
        assert elapsed < 0.5

    def test_zero_max_attempts(self):
        """Test that zero max_attempts means no retry."""
        @retry_on_exception(RetryConfig(max_attempts=1, base_delay=0.01))
        def failing_function():
            raise ConnectionError("Failed")

        with pytest.raises(ConnectionError):
            failing_function()

    def test_custom_retry_condition(self):
        """Test retry with custom condition function."""
        def should_retry(exc):
            return isinstance(exc, ConnectionError) and "retry" in str(exc)

        @retry_on_exception(
            RetryConfig(max_attempts=3, base_delay=0.01),
            retry_condition=should_retry
        )
        def conditional_function():
            nonlocal attempt_count
            attempt_count += 1
            if attempt_count == 1:
                raise ConnectionError("retry me")
            elif attempt_count == 2:
                raise ConnectionError("don't retry")
            return "success"

        attempt_count = 0

        result = conditional_function()
        assert result == "success"
        assert attempt_count == 3

    def test_on_retry_callback(self):
        """Test that on_retry callback is invoked."""
        callback_calls = []

        def retry_callback(attempt, delay, exception):
            callback_calls.append((attempt, delay, str(exception)))

        @retry_on_exception(
            RetryConfig(max_attempts=3, base_delay=0.01, jitter=False),
            on_retry=retry_callback
        )
        def failing_function():
            nonlocal attempt_count
            attempt_count += 1
            if attempt_count < 3:
                raise ConnectionError("Failed")
            return "success"

        attempt_count = 0
        failing_function()

        assert len(callback_calls) == 2
        assert callback_calls[0][0] == 1  # First retry
        assert callback_calls[1][0] == 2  # Second retry

    def test_preserve_function_metadata(self):
        """Test that decorator preserves function metadata."""
        @retry_on_exception(RetryConfig())
        def documented_function():
            """This is a documented function."""
            return "value"

        assert documented_function.__name__ == "documented_function"
        assert documented_function.__doc__ == "This is a documented function."


class TestRetryIntegration:
    """Integration tests for retry behavior."""

    def test_exponential_backoff_sequence(self):
        """Test that delays increase exponentially."""
        delays = []

        @retry_on_exception(
            RetryConfig(max_attempts=5, base_delay=0.05, exponential_base=2.0, jitter=False),
            on_retry=lambda a, d, e: delays.append(d)
        )
        def always_fail():
            raise ConnectionError("Fail")

        try:
            always_fail()
        except ConnectionError:
            pass

        # Should have 4 retries with delays: 0.05, 0.1, 0.2, 0.4
        assert len(delays) == 4
        assert delays[0] == 0.05
        assert delays[1] == 0.1
        assert delays[2] == 0.2
        assert delays[3] == 0.4

    def test_jitter_variance(self):
        """Test that jitter introduces variance in delays."""
        delays = []

        @retry_on_exception(
            RetryConfig(max_attempts=10, base_delay=0.01, jitter=True, jitter_range=0.5),
            on_retry=lambda a, d, e: delays.append(d)
        )
        def always_fail():
            raise ConnectionError("Fail")

        try:
            always_fail()
        except ConnectionError:
            pass

        # With 50% jitter, delays should vary
        assert len(set(delays)) > len(delays) // 2  # At least half should be different

    def test_network_like_failure_pattern(self):
        """Test retry behavior simulating network issues."""
        attempt_results = [
            ConnectionError("Connection refused"),
            ConnectionError("Connection refused"),
            ConnectionError("Connection refused"),
            "success"
        ]
        index = [0]

        @retry_on_exception(RetryConfig(max_attempts=5, base_delay=0.01))
        def flaky_network():
            result = attempt_results[index[0]]
            index[0] += 1
            if isinstance(result, Exception):
                raise result
            return result

        result = flaky_network()
        assert result == "success"
        assert index[0] == 4
