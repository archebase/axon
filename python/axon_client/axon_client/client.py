# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Synchronous HTTP client for Axon recorder RPC API.

Provides methods for all RPC endpoints defined in docs/designs/rpc-api-design.md
"""

import time
from typing import Any, Dict, Optional, Tuple

import requests

from .exceptions import (
    ConnectionError as AxonConnectionError,
    InvalidResponseError,
    RpcError,
    StateTransitionError,
    TaskIdMismatchError,
)
from .models import RecorderState, RpcResponse, Statistics, TaskConfig
from .retry import RetryConfig, retry_on_exception
from .utils import build_base_url


class AxonRecorderClient:
    """
    Synchronous HTTP client for Axon recorder RPC API.

    Example:
        >>> client = AxonRecorderClient("localhost", 8080)
        >>> config = TaskConfig(task_id="task_123", device_id="robot_01", topics=["/camera"])
        >>> client.config(config)
        >>> client.begin("task_123")
        >>> time.sleep(10)
        >>> client.finish("task_123")
    """

    # Default timeout for requests in seconds
    DEFAULT_TIMEOUT = 10.0

    def __init__(
        self,
        host: str = "localhost",
        port: int = 8080,
        base_url: str = None,
        timeout: float = DEFAULT_TIMEOUT,
        retry_config: RetryConfig = None,
    ):
        """
        Initialize the Axon recorder client.

        Args:
            host: Recorder host address (default: "localhost")
            port: Recorder port (default: 8080)
            base_url: Full base URL (overrides host/port if provided)
            timeout: Request timeout in seconds (default: 10.0)
            retry_config: Retry configuration for network operations
        """
        if base_url:
            self.base_url = base_url.rstrip("/")
        else:
            self.base_url = build_base_url(host, port)

        self.timeout = timeout
        self.retry_config = retry_config or RetryConfig()
        self.session = requests.Session()
        self._current_task_id: Optional[str] = None
        self._cached_state: Optional[RecorderState] = None

    # ==========================================================================
    # Configuration endpoints
    # ==========================================================================

    def config(self, task_config: TaskConfig) -> RpcResponse:
        """
        Cache task configuration (IDLE → READY transition)

        Args:
            task_config: Task configuration

        Returns:
            RpcResponse with success status

        Raises:
            RpcError: If config fails
            ConnectionError: If connection fails
        """
        if not task_config.validate():
            raise ValueError("Invalid task_config: task_id and device_id are required")

        payload = {"task_config": task_config.to_dict()}
        response = self._make_request("POST", "/rpc/config", payload)

        # Cache task_id for subsequent calls
        self._current_task_id = task_config.task_id
        self._cached_state = RecorderState.READY

        return response

    def clear(self) -> RpcResponse:
        """
        Clear cached configuration (READY → IDLE transition)

        Returns:
            RpcResponse with success status

        Raises:
            RpcError: If clear fails
        """
        response = self._make_request("POST", "/rpc/clear", {})
        self._current_task_id = None
        self._cached_state = RecorderState.IDLE
        return response

    # ==========================================================================
    # Recording control endpoints
    # ==========================================================================

    def _validate_task_id(self, task_id: str = None) -> str:
        """
        Validate and normalize task ID parameter.

        Args:
            task_id: Task ID (uses cached if not provided)

        Returns:
            Validated task ID

        Raises:
            ValueError: If task_id not provided and not cached
            TaskIdMismatchError: If task_id doesn't match cached config
        """
        task_id = task_id or self._current_task_id
        if not task_id:
            raise ValueError("task_id required (not cached)")

        if self._current_task_id and task_id != self._current_task_id:
            raise TaskIdMismatchError(self._current_task_id, task_id)

        return task_id

    def begin(self, task_id: str = None) -> RpcResponse:
        """
        Start recording (READY → RECORDING transition)

        Args:
            task_id: Task ID (uses cached if not provided)

        Returns:
            RpcResponse with success status

        Raises:
            TaskIdMismatchError: If task_id doesn't match cached config
            StateTransitionError: If not in READY state
            RpcError: If begin fails
        """
        task_id = self._validate_task_id(task_id)
        response = self._make_request("POST", "/rpc/begin", {"task_id": task_id})
        self._cached_state = RecorderState.RECORDING
        return response

    def finish(self, task_id: str = None) -> RpcResponse:
        """
        Stop recording and finalize MCAP file (RECORDING/PAUSED → IDLE)

        Args:
            task_id: Task ID (uses cached if not provided)

        Returns:
            RpcResponse with success status

        Raises:
            TaskIdMismatchError: If task_id doesn't match cached config
            RpcError: If finish fails
        """
        task_id = self._validate_task_id(task_id)
        response = self._make_request("POST", "/rpc/finish", {"task_id": task_id})
        self._current_task_id = None
        self._cached_state = RecorderState.IDLE
        return response

    def cancel(self, task_id: str = None) -> RpcResponse:
        """
        Cancel recording (RECORDING/PAUSED → IDLE, discards data)

        Args:
            task_id: Task ID (uses cached if not provided)

        Returns:
            RpcResponse with success status

        Raises:
            TaskIdMismatchError: If task_id doesn't match cached config
            RpcError: If cancel fails
        """
        task_id = self._validate_task_id(task_id)
        response = self._make_request("POST", "/rpc/cancel", {"task_id": task_id})
        self._current_task_id = None
        self._cached_state = RecorderState.IDLE
        return response

    def pause(self) -> RpcResponse:
        """
        Pause recording (RECORDING → PAUSED)

        Returns:
            RpcResponse with success status

        Raises:
            StateTransitionError: If not in RECORDING state
            RpcError: If pause fails
        """
        response = self._make_request("POST", "/rpc/pause", {})
        self._cached_state = RecorderState.PAUSED
        return response

    def resume(self) -> RpcResponse:
        """
        Resume recording (PAUSED → RECORDING)

        Returns:
            RpcResponse with success status

        Raises:
            StateTransitionError: If not in PAUSED state
            RpcError: If resume fails
        """
        response = self._make_request("POST", "/rpc/resume", {})
        self._cached_state = RecorderState.RECORDING
        return response

    def quit(self) -> RpcResponse:
        """
        Request graceful shutdown of the recorder

        Returns:
            RpcResponse with success status
        """
        response = self._make_request("POST", "/rpc/quit", {})
        return response

    # ==========================================================================
    # Query endpoints
    # ==========================================================================

    def get_state(self) -> Tuple[RecorderState, Optional[TaskConfig]]:
        """
        Get current recorder state and cached task config

        Returns:
            Tuple of (RecorderState, TaskConfig or None)

        Raises:
            ConnectionError: If connection fails
        """
        response = self._make_request("GET", "/rpc/state")

        # Parse state from response data
        state_str = response.data.get("state", "idle")
        state = RecorderState.from_string(state_str)
        self._cached_state = state

        # Parse task config if available
        task_config_data = response.data.get("task_config")
        task_config = None
        if task_config_data:
            task_config = TaskConfig.from_dict(task_config_data)

        return state, task_config

    def get_stats(self) -> Statistics:
        """
        Get recording statistics

        Returns:
            Statistics object with metrics

        Raises:
            ConnectionError: If connection fails
        """
        response = self._make_request("GET", "/rpc/stats")
        return Statistics.from_dict(response.data)

    def health(self) -> bool:
        """
        Check if recorder is healthy

        Returns:
            True if recorder is running
        """
        try:
            response = self._make_request("GET", "/", timeout=2.0)
            return response.data.get("status") == "running"
        except AxonConnectionError:
            # Connection errors mean server is not healthy
            return False
        except (InvalidResponseError, RpcError):
            # Server responded but not as expected - not healthy
            return False

    # ==========================================================================
    # Polling helpers
    # ==========================================================================

    def wait_for_state(
        self, target_state: RecorderState, timeout: float = 30.0, poll_interval: float = 0.5
    ) -> bool:
        """
        Wait until recorder reaches target state

        Args:
            target_state: State to wait for
            timeout: Maximum wait time in seconds
            poll_interval: Time between polls in seconds

        Returns:
            True if state reached, False if timeout

        Example:
            >>> client.begin("task_123")
            >>> client.wait_for_state(RecorderState.RECORDING, timeout=5.0)
        """
        start = time.time()
        while time.time() - start < timeout:
            state, _ = self.get_state()
            if state == target_state:
                return True
            time.sleep(poll_interval)
        return False

    def wait_for_recording_complete(self, timeout: float = 300.0, poll_interval: float = 1.0) -> bool:
        """
        Wait until recording finishes (returns to IDLE state)

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Time between polls in seconds

        Returns:
            True if recording finished, False if timeout
        """
        return self.wait_for_state(RecorderState.IDLE, timeout, poll_interval)

    # ==========================================================================
    # Context manager
    # ==========================================================================

    def __enter__(self) -> "AxonRecorderClient":
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - close session"""
        self.close()
        return False

    def close(self):
        """Close the HTTP session"""
        self.session.close()

    # ==========================================================================
    # Internal methods
    # ==========================================================================

    @retry_on_exception(RetryConfig(), retryable_exceptions=(ConnectionError,))
    def _make_request(
        self, method: str, endpoint: str, data: Dict[str, Any] = None, timeout: float = None
    ) -> RpcResponse:
        """
        Make HTTP request to RPC endpoint

        Args:
            method: HTTP method (GET or POST)
            endpoint: RPC endpoint path
            data: Request payload (for POST)
            timeout: Request timeout (uses default if not specified)

        Returns:
            RpcResponse with parsed response

        Raises:
            ConnectionError: If connection fails
            InvalidResponseError: If response is invalid
            RpcError: If RPC returns success=False
        """
        url = f"{self.base_url}{endpoint}"
        timeout = timeout or self.timeout

        try:
            if method == "GET":
                http_response = self.session.get(url, timeout=timeout)
            elif method == "POST":
                http_response = self.session.post(url, json=data, timeout=timeout)
            else:
                raise ValueError(f"Unsupported method: {method}")

        except requests.exceptions.ConnectException as e:
            raise AxonConnectionError(f"Failed to connect to {url}: {e}") from e
        except requests.exceptions.Timeout as e:
            raise AxonConnectionError(f"Request to {url} timed out") from e
        except requests.exceptions.RequestException as e:
            raise AxonConnectionError(f"Request to {url} failed: {e}") from e

        # Check for non-JSON responses (health check, etc.)
        content_type = http_response.headers.get("Content-Type", "")
        if "application/json" not in content_type:
            # Non-JSON response - return basic response
            return RpcResponse(success=http_response.status_code == 200, message="OK", data={})

        # Parse JSON response
        try:
            response_data = http_response.json()
        except ValueError as e:
            raise InvalidResponseError(f"Invalid JSON response: {e}") from e

        response = RpcResponse.from_dict(response_data)
        response.raise_for_error()

        return response
