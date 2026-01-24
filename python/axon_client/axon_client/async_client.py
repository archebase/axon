# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Asynchronous HTTP client for Axon recorder RPC API.

Provides async methods for all RPC endpoints using aiohttp.
"""

import asyncio
from typing import Any, Dict, Optional, Tuple

import aiohttp

from .exceptions import (
    ConnectionError as AxonConnectionError,
    InvalidResponseError,
    RpcError,
    TaskIdMismatchError,
)
from .models import RecorderState, RpcResponse, Statistics, TaskConfig
from .utils import build_base_url


class AsyncAxonRecorderClient:
    """
    Asynchronous HTTP client for Axon recorder RPC API.

    Example:
        >>> async with AsyncAxonRecorderClient("localhost", 8080) as client:
        ...     config = TaskConfig(task_id="task_123", device_id="robot_01", topics=["/camera"])
        ...     await client.config(config)
        ...     await client.begin("task_123")
        ...     await asyncio.sleep(10)
        ...     await client.finish("task_123")
    """

    DEFAULT_TIMEOUT = 10.0

    def __init__(
        self,
        host: str = "localhost",
        port: int = 8080,
        base_url: str = None,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        """
        Initialize the async Axon recorder client.

        Args:
            host: Recorder host address (default: "localhost")
            port: Recorder port (default: 8080)
            base_url: Full base URL (overrides host/port if provided)
            timeout: Request timeout in seconds (default: 10.0)
        """
        if base_url:
            self.base_url = base_url.rstrip("/")
        else:
            self.base_url = build_base_url(host, port)

        self.timeout = aiohttp.ClientTimeout(total=timeout)
        self._session: Optional[aiohttp.ClientSession] = None
        self._current_task_id: Optional[str] = None
        self._cached_state: Optional[RecorderState] = None

    async def __aenter__(self) -> "AsyncAxonRecorderClient":
        """Async context manager entry"""
        self._session = aiohttp.ClientSession(timeout=self.timeout)
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        await self.close()
        return False

    async def close(self):
        """Close the HTTP session"""
        if self._session:
            await self._session.close()
            self._session = None

    def _ensure_session(self) -> aiohttp.ClientSession:
        """Ensure session is created"""
        if self._session is None:
            self._session = aiohttp.ClientSession(timeout=self.timeout)
        return self._session

    # ==========================================================================
    # Configuration endpoints
    # ==========================================================================

    async def config(self, task_config: TaskConfig) -> RpcResponse:
        """Cache task configuration (IDLE → READY transition)"""
        if not task_config.validate():
            raise ValueError("Invalid task_config: task_id and device_id are required")

        payload = {"task_config": task_config.to_dict()}
        response = await self._make_request("POST", "/rpc/config", payload)

        self._current_task_id = task_config.task_id
        self._cached_state = RecorderState.READY
        return response

    async def clear(self) -> RpcResponse:
        """Clear cached configuration (READY → IDLE transition)"""
        response = await self._make_request("POST", "/rpc/clear", {})
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

    async def begin(self, task_id: str = None) -> RpcResponse:
        """Start recording (READY → RECORDING transition)"""
        task_id = self._validate_task_id(task_id)
        response = await self._make_request("POST", "/rpc/begin", {"task_id": task_id})
        self._cached_state = RecorderState.RECORDING
        return response

    async def finish(self, task_id: str = None) -> RpcResponse:
        """Stop recording and finalize MCAP file (RECORDING/PAUSED → IDLE)"""
        task_id = self._validate_task_id(task_id)
        response = await self._make_request("POST", "/rpc/finish", {"task_id": task_id})
        self._current_task_id = None
        self._cached_state = RecorderState.IDLE
        return response

    async def cancel(self, task_id: str = None) -> RpcResponse:
        """Cancel recording (RECORDING/PAUSED → IDLE, discards data)"""
        task_id = self._validate_task_id(task_id)
        response = await self._make_request("POST", "/rpc/cancel", {"task_id": task_id})
        self._current_task_id = None
        self._cached_state = RecorderState.IDLE
        return response

    async def pause(self) -> RpcResponse:
        """Pause recording (RECORDING → PAUSED)"""
        response = await self._make_request("POST", "/rpc/pause", {})
        self._cached_state = RecorderState.PAUSED
        return response

    async def resume(self) -> RpcResponse:
        """Resume recording (PAUSED → RECORDING)"""
        response = await self._make_request("POST", "/rpc/resume", {})
        self._cached_state = RecorderState.RECORDING
        return response

    async def quit(self) -> RpcResponse:
        """Request graceful shutdown of the recorder"""
        response = await self._make_request("POST", "/rpc/quit", {})
        return response

    # ==========================================================================
    # Query endpoints
    # ==========================================================================

    async def get_state(self) -> Tuple[RecorderState, Optional[TaskConfig]]:
        """Get current recorder state and cached task config"""
        response = await self._make_request("GET", "/rpc/state")

        state_str = response.data.get("state", "idle")
        state = RecorderState.from_string(state_str)
        self._cached_state = state

        task_config_data = response.data.get("task_config")
        task_config = None
        if task_config_data:
            task_config = TaskConfig.from_dict(task_config_data)

        return state, task_config

    async def get_stats(self) -> Statistics:
        """Get recording statistics"""
        response = await self._make_request("GET", "/rpc/stats")
        return Statistics.from_dict(response.data)

    async def health(self) -> bool:
        """Check if recorder is healthy"""
        try:
            response = await self._make_request("GET", "/", timeout=2.0)
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

    async def wait_for_state(
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
            >>> await client.begin("task_123")
            >>> await client.wait_for_state(RecorderState.RECORDING, timeout=5.0)
        """
        start = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start < timeout:
            state, _ = await self.get_state()
            if state == target_state:
                return True
            await asyncio.sleep(poll_interval)
        return False

    async def wait_for_recording_complete(self, timeout: float = 300.0, poll_interval: float = 1.0) -> bool:
        """
        Wait until recording finishes (returns to IDLE state)

        Args:
            timeout: Maximum wait time in seconds
            poll_interval: Time between polls in seconds

        Returns:
            True if recording finished, False if timeout
        """
        return await self.wait_for_state(RecorderState.IDLE, timeout, poll_interval)

    # ==========================================================================
    # Internal methods
    # ==========================================================================

    async def _make_request(
        self, method: str, endpoint: str, data: Dict[str, Any] = None, timeout: float = None
    ) -> RpcResponse:
        """Make async HTTP request to RPC endpoint"""
        url = f"{self.base_url}{endpoint}"
        session = self._ensure_session()

        req_timeout = aiohttp.ClientTimeout(total=timeout) if timeout else self.timeout

        try:
            if method == "GET":
                async with session.get(url, timeout=req_timeout) as http_response:
                    return await self._parse_response(http_response)
            elif method == "POST":
                async with session.post(url, json=data, timeout=req_timeout) as http_response:
                    return await self._parse_response(http_response)
            else:
                raise ValueError(f"Unsupported method: {method}")

        except aiohttp.ClientConnectorError as e:
            raise AxonConnectionError(f"Failed to connect to {url}: {e}") from e
        except aiohttp.ClientError as e:
            raise AxonConnectionError(f"Request to {url} failed: {e}") from e

    async def _parse_response(self, http_response: aiohttp.ClientResponse) -> RpcResponse:
        """Parse HTTP response into RpcResponse"""
        content_type = http_response.headers.get("Content-Type", "")

        if "application/json" not in content_type:
            return RpcResponse(success=http_response.status == 200, message="OK", data={})

        try:
            response_data = await http_response.json()
        except ValueError as e:
            raise InvalidResponseError(f"Invalid JSON response: {e}") from e

        response = RpcResponse.from_dict(response_data)
        response.raise_for_error()

        return response
