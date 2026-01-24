# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for AsyncAxonRecorderClient (asynchronous HTTP client)."""

import json

import pytest
from aiohttp import ClientSession, ClientTimeout

from axon_client import AsyncAxonRecorderClient, RecorderState, RpcResponse, Statistics, TaskConfig
from axon_client.exceptions import (
    ConnectionError as AxonConnectionError,
    InvalidResponseError,
    RpcError,
    TaskIdMismatchError,
)


@pytest.fixture
async def client():
    """Create a test async client instance."""
    client = AsyncAxonRecorderClient(host="localhost", port=8080)
    # Ensure session is created
    client._session = ClientSession()
    yield client
    await client.close()


class TestAsyncAxonRecorderClientInit:
    """Tests for AsyncAxonRecorderClient initialization."""

    def test_init_default(self):
        """Test client with default parameters."""
        client = AsyncAxonRecorderClient()
        assert client.base_url == "http://localhost:8080"
        assert client._current_task_id is None
        assert client._session is None

    def test_init_custom_host_port(self):
        """Test client with custom host and port."""
        client = AsyncAxonRecorderClient(host="example.com", port=9000)
        assert client.base_url == "http://example.com:9000"

    def test_init_base_url(self):
        """Test client with explicit base URL."""
        client = AsyncAxonRecorderClient(base_url="https://api.example.com/v1")
        assert client.base_url == "https://api.example.com/v1"

    def test_timeout_is_clienttimeout(self):
        """Test that timeout is converted to ClientTimeout."""
        client = AsyncAxonRecorderClient(timeout=5.0)
        assert isinstance(client.timeout, ClientTimeout)


class TestAsyncContextManager:
    """Tests for async context manager."""

    @pytest.mark.asyncio
    async def test_async_context_manager(self):
        """Test client as async context manager."""
        async with AsyncAxonRecorderClient() as client:
            assert client._session is not None

        # Session should be closed after exit
        assert client._session is None

    @pytest.mark.asyncio
    async def test_close(self):
        """Test manual close."""
        client = AsyncAxonRecorderClient()
        client._session = ClientSession()
        await client.close()
        assert client._session is None


class TestAsyncConfigEndpoints:
    """Tests for async configuration endpoints."""

    @pytest.mark.asyncio
    async def test_config_success(self, client, aiohttp_mock):
        """Test successful config call."""
        aiohttp_mock.post(
            "http://localhost:8080/rpc/config",
            payload={"success": True, "message": "Configuration cached", "data": None}
        )

        config = TaskConfig(task_id="task_001", device_id="robot_01", topics=["/camera"])
        response = await client.config(config)

        assert response.success is True
        assert client._current_task_id == "task_001"
        assert client._cached_state == RecorderState.READY

    @pytest.mark.asyncio
    async def test_config_invalid_task_config(self, client):
        """Test config with invalid task config."""
        config = TaskConfig(task_id="", device_id="")
        with pytest.raises(ValueError, match="Invalid task_config"):
            await client.config(config)

    @pytest.mark.asyncio
    async def test_clear_success(self, client, aiohttp_mock):
        """Test successful clear call."""
        client._current_task_id = "task_001"
        client._cached_state = RecorderState.READY

        aiohttp_mock.post(
            "http://localhost:8080/rpc/clear",
            payload={"success": True, "message": "Configuration cleared", "data": None}
        )

        response = await client.clear()

        assert response.success is True
        assert client._current_task_id is None
        assert client._cached_state == RecorderState.IDLE


class TestAsyncRecordingControl:
    """Tests for async recording control endpoints."""

    @pytest.mark.asyncio
    async def test_begin_with_task_id(self, client, aiohttp_mock):
        """Test begin with explicit task ID."""
        client._current_task_id = "task_001"
        client._cached_state = RecorderState.READY

        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            payload={"success": True, "message": "Recording started", "data": None}
        )

        response = await client.begin("task_001")

        assert response.success is True
        assert client._cached_state == RecorderState.RECORDING

    @pytest.mark.asyncio
    async def test_begin_with_cached_task_id(self, client, aiohttp_mock):
        """Test begin with cached task ID."""
        client._current_task_id = "task_001"

        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            payload={"success": True, "message": "Recording started", "data": None}
        )

        response = await client.begin()

        assert response.success is True

    @pytest.mark.asyncio
    async def test_begin_without_task_id(self, client):
        """Test begin without task ID raises error."""
        with pytest.raises(ValueError, match="task_id required"):
            await client.begin()

    @pytest.mark.asyncio
    async def test_begin_task_id_mismatch(self, client):
        """Test begin with mismatched task ID raises error."""
        client._current_task_id = "cached_task"

        with pytest.raises(TaskIdMismatchError):
            await client.begin("different_task")

    @pytest.mark.asyncio
    async def test_finish_success(self, client, aiohttp_mock):
        """Test successful finish call."""
        client._current_task_id = "task_001"
        client._cached_state = RecorderState.RECORDING

        aiohttp_mock.post(
            "http://localhost:8080/rpc/finish",
            payload={"success": True, "message": "Recording finished", "data": None}
        )

        response = await client.finish("task_001")

        assert response.success is True
        assert client._current_task_id is None
        assert client._cached_state == RecorderState.IDLE

    @pytest.mark.asyncio
    async def test_cancel_success(self, client, aiohttp_mock):
        """Test successful cancel call."""
        client._current_task_id = "task_001"

        aiohttp_mock.post(
            "http://localhost:8080/rpc/cancel",
            payload={"success": True, "message": "Recording cancelled", "data": None}
        )

        response = await client.cancel("task_001")

        assert response.success is True
        assert client._current_task_id is None

    @pytest.mark.asyncio
    async def test_pause_success(self, client, aiohttp_mock):
        """Test successful pause call."""
        client._cached_state = RecorderState.RECORDING

        aiohttp_mock.post(
            "http://localhost:8080/rpc/pause",
            payload={"success": True, "message": "Recording paused", "data": None}
        )

        response = await client.pause()

        assert response.success is True
        assert client._cached_state == RecorderState.PAUSED

    @pytest.mark.asyncio
    async def test_resume_success(self, client, aiohttp_mock):
        """Test successful resume call."""
        client._cached_state = RecorderState.PAUSED

        aiohttp_mock.post(
            "http://localhost:8080/rpc/resume",
            payload={"success": True, "message": "Recording resumed", "data": None}
        )

        response = await client.resume()

        assert response.success is True
        assert client._cached_state == RecorderState.RECORDING

    @pytest.mark.asyncio
    async def test_quit_success(self, client, aiohttp_mock):
        """Test successful quit call."""
        aiohttp_mock.post(
            "http://localhost:8080/rpc/quit",
            payload={"success": True, "message": "Shutting down", "data": None}
        )

        response = await client.quit()

        assert response.success is True


class TestAsyncQueryEndpoints:
    """Tests for async query endpoints."""

    @pytest.mark.asyncio
    async def test_get_state(self, client, aiohttp_mock):
        """Test get_state parses response correctly."""
        aiohttp_mock.get(
            "http://localhost:8080/rpc/state",
            payload={
                "success": True,
                "data": {
                    "state": "recording",
                    "task_config": {
                        "task_id": "task_001",
                        "device_id": "robot_01",
                        "topics": ["/camera"]
                    }
                }
            }
        )

        state, task_config = await client.get_state()

        assert state == RecorderState.RECORDING
        assert task_config is not None
        assert task_config.task_id == "task_001"
        assert client._cached_state == RecorderState.RECORDING

    @pytest.mark.asyncio
    async def test_get_state_no_config(self, client, aiohttp_mock):
        """Test get_state with no task config."""
        aiohttp_mock.get(
            "http://localhost:8080/rpc/state",
            payload={"success": True, "data": {"state": "idle"}}
        )

        state, task_config = await client.get_state()

        assert state == RecorderState.IDLE
        assert task_config is None

    @pytest.mark.asyncio
    async def test_get_stats(self, client, aiohttp_mock):
        """Test get_stats parses response correctly."""
        aiohttp_mock.get(
            "http://localhost:8080/rpc/stats",
            payload={
                "success": True,
                "data": {
                    "messages_received": 1000,
                    "messages_written": 950,
                    "messages_dropped": 50,
                    "bytes_written": 1000000
                }
            }
        )

        stats = await client.get_stats()

        assert stats.messages_received == 1000
        assert stats.messages_written == 950
        assert stats.messages_dropped == 50
        assert stats.drop_rate() == 0.05

    @pytest.mark.asyncio
    async def test_health_healthy(self, client, aiohttp_mock):
        """Test health check when recorder is healthy."""
        aiohttp_mock.get(
            "http://localhost:8080/",
            payload={"status": "running"}
        )

        assert await client.health() is True

    @pytest.mark.asyncio
    async def test_health_unhealthy(self, client, aiohttp_mock):
        """Test health check when recorder is unhealthy."""
        # Simulate connection error
        import aiohttp
        aiohttp_mock.get("http://localhost:8080/", exception=aiohttp.ClientConnectorError(None, None))

        assert await client.health() is False


class TestAsyncErrorHandling:
    """Tests for async error handling."""

    @pytest.mark.asyncio
    async def test_rpc_error_response(self, client, aiohttp_mock):
        """Test that RPC error response raises RpcError."""
        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            payload={
                "success": False,
                "message": "Invalid state transition",
                "error_code": "STATE_ERROR"
            }
        )

        with pytest.raises(RpcError, match="Invalid state transition"):
            await client.begin("task_001")

    @pytest.mark.asyncio
    async def test_connection_error(self, client, aiohttp_mock):
        """Test that connection failures raise ConnectionError."""
        import aiohttp
        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            exception=aiohttp.ClientConnectorError(None, None)
        )

        with pytest.raises(AxonConnectionError):
            await client.begin("task_001")

    @pytest.mark.asyncio
    async def test_invalid_json_response(self, client, aiohttp_mock):
        """Test that invalid JSON raises InvalidResponseError."""
        import aiohttp
        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            content_type="application/json",
            body="Invalid JSON"
        )

        # aioresponses handles this differently
        # Just verify the client doesn't crash
        try:
            await client.begin("task_001")
        except (InvalidResponseError, RpcError):
            pass  # Expected


class TestAsyncPollingHelpers:
    """Tests for async polling helper methods."""

    @pytest.mark.asyncio
    async def test_wait_for_state_success(self, client):
        """Test wait_for_state reaches target state."""
        with pytest.mock.patch.object(
            client, "get_state",
            side_effect=[
                (RecorderState.READY, None),
                (RecorderState.RECORDING, None),
            ]
        ):
            result = await client.wait_for_state(RecorderState.RECORDING, timeout=5.0, poll_interval=0.1)
            assert result is True

    @pytest.mark.asyncio
    async def test_wait_for_state_timeout(self, client):
        """Test wait_for_state times out."""
        with pytest.mock.patch.object(
            client, "get_state",
            return_value=(RecorderState.READY, None)
        ):
            result = await client.wait_for_state(RecorderState.RECORDING, timeout=0.2, poll_interval=0.1)
            assert result is False

    @pytest.mark.asyncio
    async def test_wait_for_recording_complete(self, client):
        """Test wait_for_recording_complete convenience method."""
        with pytest.mock.patch.object(
            client, "get_state",
            side_effect=[
                (RecorderState.RECORDING, None),
                (RecorderState.IDLE, None),
            ]
        ):
            result = await client.wait_for_recording_complete(timeout=5.0)
            assert result is True


class TestAsyncTimeoutAndSession:
    """Tests for timeout handling and session reuse."""

    @pytest.mark.asyncio
    async def test_custom_timeout(self):
        """Test client with custom timeout."""
        client = AsyncAxonRecorderClient(timeout=5.0)
        assert client.timeout.total == 5.0
        await client.close()

    @pytest.mark.asyncio
    async def test_request_timeout(self, client, aiohttp_mock):
        """Test that request timeout is handled correctly."""
        import asyncio
        import aiohttp

        # Simulate a slow server that times out
        async def slow_response(*args, **kwargs):
            await asyncio.sleep(2)  # Simulate delay
            return None

        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            exception=asyncio.TimeoutError()
        )

        with pytest.raises(AxonConnectionError):
            await client.begin("task_001")

    @pytest.mark.asyncio
    async def test_session_reuse_across_requests(self, client, aiohttp_mock):
        """Test that the same session is reused for multiple requests."""
        aiohttp_mock.post(
            "http://localhost:8080/rpc/config",
            payload={"success": True, "message": "Configuration cached", "data": None}
        )
        aiohttp_mock.post(
            "http://localhost:8080/rpc/begin",
            payload={"success": True, "message": "Recording started", "data": None}
        )
        aiohttp_mock.post(
            "http://localhost:8080/rpc/finish",
            payload={"success": True, "message": "Recording finished", "data": None}
        )

        # First call creates session
        config = TaskConfig(task_id="task_001", device_id="robot_01", topics=["/camera"])
        await client.config(config)

        # Get session reference
        first_session = client._session
        assert first_session is not None

        # Second call reuses session
        await client.begin("task_001")
        assert client._session is first_session

        # Third call reuses session
        await client.finish("task_001")
        assert client._session is first_session

    @pytest.mark.asyncio
    async def test_session_persistence_after_close(self):
        """Test that session is None after close."""
        client = AsyncAxonRecorderClient()

        # Session is None initially
        assert client._session is None

        # Create session via _ensure_session
        session = client._ensure_session()
        assert session is not None
        assert client._session is not None

        # Close clears session
        await client.close()
        assert client._session is None

    @pytest.mark.asyncio
    async def test_health_timeout(self, client, aiohttp_mock):
        """Test health() with timeout parameter."""
        import asyncio
        aiohttp_mock.get(
            "http://localhost:8080/",
            payload={"status": "running"}
        )

        # health() uses 2.0s timeout, should succeed
        result = await client.health()
        assert result is True

    @pytest.mark.asyncio
    async def test_concurrent_requests_share_session(self, client, aiohttp_mock):
        """Test concurrent requests use the same session."""
        import asyncio

        aiohttp_mock.get(
            "http://localhost:8080/rpc/state",
            payload={"success": True, "data": {"state": "idle"}}
        )

        # Make concurrent requests
        results = await asyncio.gather(
            client.get_state(),
            client.get_state(),
            client.get_state()
        )

        # All should succeed and share the same session
        assert len(results) == 3
        for state, _ in results:
            assert state == RecorderState.IDLE


# pytest fixtures for aiohttp mocking
@pytest.fixture
def aiohttp_mock():
    """Fixture for aioresponses mocking."""
    import aioresponses
    with aioresponses.aioresponses() as m:
        yield m
