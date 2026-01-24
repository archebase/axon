# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Axon HTTP RPC Client for Python

A Python client for controlling Axon recorder via HTTP RPC API.
Supports both synchronous and asynchronous operations.
"""

__version__ = "0.1.0"

# Core classes
from .client import AxonRecorderClient
from .async_client import AsyncAxonRecorderClient

# Data models
from .models import RecorderState, RpcResponse, Statistics, TaskConfig

# Exceptions
from .exceptions import (
    AxonClientError,
    ConnectionError as AxonConnectionError,
    RpcError,
    StateTransitionError,
    TaskIdMismatchError,
    TimeoutError as AxonTimeoutError,
)

# Optional Zenoh integration
try:
    from .zenoh_publisher import ZenohConfig, ZenohStatusPublisher

    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False

__all__ = [
    # Version
    "__version__",
    # Core classes
    "AxonRecorderClient",
    "AsyncAxonRecorderClient",
    # Data models
    "RecorderState",
    "RpcResponse",
    "Statistics",
    "TaskConfig",
    # Exceptions
    "AxonClientError",
    "AxonConnectionError",
    "RpcError",
    "StateTransitionError",
    "TaskIdMismatchError",
    "AxonTimeoutError",
    # Zenoh (conditional)
    "ZenohConfig",
    "ZenohStatusPublisher",
    "ZENOH_AVAILABLE",
]
