# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Retry logic with exponential backoff for Axon recorder client.
"""

import random
import time
from dataclasses import dataclass
from functools import wraps
from typing import Any, Callable, Tuple, Type

from .exceptions import AxonClientError, ConnectionError, TimeoutError


@dataclass
class RetryConfig:
    """
    Configuration for retry behavior

    Attributes:
        max_retries: Maximum number of retry attempts
        base_delay: Base delay in seconds before first retry
        max_delay: Maximum delay in seconds
        exponential_base: Multiplier for exponential backoff
        jitter: Whether to add random jitter to delay
    """

    max_retries: int = 3
    base_delay: float = 1.0
    max_delay: float = 30.0
    exponential_base: float = 2.0
    jitter: bool = True


def retry_on_exception(
    config: RetryConfig,
    retryable_exceptions: Tuple[Type[Exception], ...] = (ConnectionError, TimeoutError),
) -> Callable:
    """
    Decorator for retrying function calls on exception

    Args:
        config: Retry configuration
        retryable_exceptions: Tuple of exception types to retry on

    Example:
        @retry_on_exception(RetryConfig(max_retries=3))
        def my_function():
            # Do something that might fail
            pass
    """

    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            last_exception = None

            for attempt in range(config.max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except retryable_exceptions as e:
                    last_exception = e

                    if attempt == config.max_retries:
                        # Last attempt failed, raise
                        break

                    # Calculate delay with exponential backoff
                    delay = min(
                        config.base_delay * (config.exponential_base**attempt), config.max_delay
                    )

                    # Add jitter to prevent thundering herd
                    if config.jitter:
                        delay *= random.uniform(0.5, 1.5)

                    time.sleep(delay)
                except AxonClientError:
                    # Don't retry on other Axon errors
                    raise

            # All retries exhausted
            if last_exception:
                raise last_exception

        return wrapper

    return decorator
