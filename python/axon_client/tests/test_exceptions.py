# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for Axon client exceptions."""

import pytest

from axon_client.exceptions import (
    AxonClientError,
    ConnectionError as AxonConnectionError,
    InvalidResponseError,
    RpcError,
    StateTransitionError,
    TaskIdMismatchError,
    ConfigError,
    TimeoutError,
)


class TestExceptionHierarchy:
    """Tests for exception class hierarchy."""

    def test_base_exception(self):
        """Test base AxonClientError."""
        exc = AxonClientError("Base error")
        assert str(exc) == "Base error"
        assert isinstance(exc, Exception)

    def test_connection_error_inherits_from_base(self):
        """Test ConnectionError inherits from AxonClientError."""
        exc = AxonConnectionError("Connection failed")
        assert isinstance(exc, AxonClientError)
        assert isinstance(exc, Exception)

    def test_invalid_response_error_inherits_from_base(self):
        """Test InvalidResponseError inherits from AxonClientError."""
        exc = InvalidResponseError("Invalid JSON")
        assert isinstance(exc, AxonClientError)

    def test_rpc_error_inherits_from_base(self):
        """Test RpcError inherits from AxonClientError."""
        exc = RpcError("RPC failed")
        assert isinstance(exc, AxonClientError)

    def test_state_transition_error_inherits_from_rpc(self):
        """Test StateTransitionError inherits from RpcError."""
        exc = StateTransitionError("Invalid transition")
        assert isinstance(exc, RpcError)
        assert isinstance(exc, AxonClientError)


class TestConnectionError:
    """Tests for ConnectionError."""

    def test_default_message(self):
        """Test ConnectionError with default message."""
        exc = AxonConnectionError()
        assert "connection" in str(exc).lower()

    def test_custom_message(self):
        """Test ConnectionError with custom message."""
        exc = AxonConnectionError("Failed to connect to localhost:8080")
        assert str(exc) == "Failed to connect to localhost:8080"

    def test_with_cause(self):
        """Test ConnectionError with underlying cause."""
        cause = OSError("Connection refused")
        exc = AxonConnectionError("Connection failed", cause)
        assert str(exc) == "Connection failed"
        assert exc.__cause__ is cause

    def test_from_underlying_exception(self):
        """Test creating ConnectionError from underlying exception."""
        try:
            raise OSError("Connection refused")
        except OSError as e:
            exc = AxonConnectionError("Failed to connect") from e
            assert exc.__cause__ is e
            assert isinstance(exc, AxonConnectionError)


class TestInvalidResponseError:
    """Tests for InvalidResponseError."""

    def test_default_message(self):
        """Test InvalidResponseError with default message."""
        exc = InvalidResponseError()
        assert "response" in str(exc).lower()

    def test_custom_message(self):
        """Test InvalidResponseError with custom message."""
        exc = InvalidResponseError("Invalid JSON response")
        assert str(exc) == "Invalid JSON response"

    def test_with_response_body(self):
        """Test InvalidResponseError includes response body."""
        exc = InvalidResponseError("Invalid response", body="Not JSON")
        assert "Invalid response" in str(exc)

    def test_with_status_code(self):
        """Test InvalidResponseError with HTTP status code."""
        exc = InvalidResponseError("Unexpected status", status_code=500)
        assert "Unexpected status" in str(exc)


class TestRpcError:
    """Tests for RpcError."""

    def test_default_message(self):
        """Test RpcError with default message."""
        exc = RpcError("RPC call failed")
        assert str(exc) == "RPC call failed"

    def test_with_error_code(self):
        """Test RpcError with error code."""
        exc = RpcError("Invalid state transition", error_code="STATE_ERROR")
        assert str(exc) == "Invalid state transition"

    def test_error_code_attribute(self):
        """Test that error_code is stored as attribute."""
        exc = RpcError("Error", error_code="CUSTOM_ERROR")
        assert exc.error_code == "CUSTOM_ERROR"

    def test_with_details(self):
        """Test RpcError with additional details."""
        details = {"current_state": "idle", "requested_action": "begin"}
        exc = RpcError("Cannot execute", details=details)
        assert exc.details == details

    def test_raise_for_error_with_success_true(self):
        """Test raise_for_error does nothing when success is True."""
        from axon_client.models import RpcResponse
        response = RpcResponse(success=True, message="OK", data={})
        # Should not raise
        response.raise_for_error()

    def test_raise_for_error_with_success_false(self):
        """Test raise_for_error raises RpcError when success is False."""
        from axon_client.models import RpcResponse
        response = RpcResponse(success=False, message="RPC failed", data={})
        with pytest.raises(RpcError, match="RPC failed"):
            response.raise_for_error()

    def test_raise_for_error_with_error_code(self):
        """Test raise_for_error includes error code."""
        from axon_client.models import RpcResponse
        response = RpcResponse(
            success=False,
            message="State error",
            data={},
            error_code="STATE_ERROR"
        )
        with pytest.raises(RpcError) as exc_info:
            response.raise_for_error()
        assert exc_info.value.error_code == "STATE_ERROR"


class TestStateTransitionError:
    """Tests for StateTransitionError."""

    def test_default_message(self):
        """Test StateTransitionError with default message."""
        exc = StateTransitionError("Invalid transition")
        assert str(exc) == "Invalid transition"

    def test_with_state_info(self):
        """Test StateTransitionError with current and target states."""
        exc = StateTransitionError(
            "Cannot transition",
            current_state="IDLE",
            target_state="RECORDING"
        )
        assert str(exc) == "Cannot transition"
        assert exc.current_state == "IDLE"
        assert exc.target_state == "RECORDING"

    def test_inherits_from_rpc_error(self):
        """Test StateTransitionError is an RpcError."""
        exc = StateTransitionError("Bad transition")
        assert isinstance(exc, RpcError)


class TestTaskIdMismatchError:
    """Tests for TaskIdMismatchError."""

    def test_default_message(self):
        """Test TaskIdMismatchError with default message."""
        exc = TaskIdMismatchError("cached_task", "new_task")
        assert "cached_task" in str(exc)
        assert "new_task" in str(exc)

    def test_message_format(self):
        """Test TaskIdMismatchError message format."""
        exc = TaskIdMismatchError("task_abc", "task_xyz")
        expected_msg = "Task ID mismatch: cached='task_abc', provided='task_xyz'"
        assert str(exc) == expected_msg

    def test_cached_task_attribute(self):
        """Test that cached_task is stored as attribute."""
        exc = TaskIdMismatchError("cached", "provided")
        assert exc.cached_task == "cached"
        assert exc.provided_task == "provided"

    def test_both_empty(self):
        """Test TaskIdMismatchError with empty strings."""
        exc = TaskIdMismatchError("", "")
        assert "Task ID mismatch" in str(exc)


class TestConfigError:
    """Tests for ConfigError."""

    def test_default_message(self):
        """Test ConfigError with default message."""
        exc = ConfigError("Invalid configuration")
        assert str(exc) == "Invalid configuration"

    def test_with_field_name(self):
        """Test ConfigError with field name."""
        exc = ConfigError("Missing required field", field="task_id")
        assert str(exc) == "Missing required field"
        assert exc.field == "task_id"

    def test_with_validation_details(self):
        """Test ConfigError with validation details."""
        details = {"missing_fields": ["task_id", "device_id"]}
        exc = ConfigError("Validation failed", details=details)
        assert exc.details == details


class TestTimeoutError:
    """Tests for TimeoutError."""

    def test_default_message(self):
        """Test TimeoutError with default message."""
        exc = TimeoutError("Request timed out")
        assert str(exc) == "Request timed out"

    def test_with_timeout_value(self):
        """Test TimeoutError with timeout duration."""
        exc = TimeoutError("Operation timed out", timeout=30.0)
        assert str(exc) == "Operation timed out"
        assert exc.timeout == 30.0

    def test_inherits_from_connection_error(self):
        """Test TimeoutError inherits from ConnectionError."""
        exc = TimeoutError("Timeout")
        assert isinstance(exc, AxonConnectionError)


class TestExceptionCatching:
    """Tests for exception catching patterns."""

    def test_catch_base_exception(self):
        """Test catching base AxonClientError catches all."""
        caught = False

        try:
            raise RpcError("RPC failed")
        except AxonClientError:
            caught = True

        assert caught

    def test_catch_specific_before_general(self):
        """Test catching specific exception before general."""
        caught_specific = False
        caught_general = False

        try:
            raise StateTransitionError("Bad transition")
        except StateTransitionError:
            caught_specific = True
        except RpcError:
            caught_general = True

        assert caught_specific and not caught_general

    def test_reraise_with_context(self):
        """Test reraising with additional context."""
        try:
            try:
                raise AxonConnectionError("Connection failed")
            except AxonConnectionError as e:
                raise RpcError("RPC failed after connection error") from e
        except RpcError as exc:
            assert isinstance(exc.__cause__, AxonConnectionError)


class TestExceptionInAsyncContext:
    """Tests for exceptions in async contexts."""

    @pytest.mark.asyncio
    async def test_exception_in_async_function(self):
        """Test that exceptions propagate through async functions."""
        async def failing_function():
            raise RpcError("Async RPC failed")

        with pytest.raises(RpcError, match="Async RPC failed"):
            await failing_function()

    @pytest.mark.asyncio
    async def test_connection_error_in_async_client(self):
        """Test ConnectionError in async client scenario."""
        async def async_connect():
            raise AxonConnectionError("Async connection failed")

        with pytest.raises(AxonConnectionError):
            await async_connect()
