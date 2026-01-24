# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Exception hierarchy for Axon recorder client.
"""


class AxonClientError(Exception):
    """Base exception for all Axon client errors"""

    pass


class RpcError(AxonClientError):
    """
    RPC call returned success=False

    Attributes:
        message: Error message from the server
        state: Current recorder state (if available)
        data: Additional response data
    """

    def __init__(self, message: str, state: str = None, data: dict = None):
        super().__init__(message)
        self.state = state
        self.data = data or {}

    def __str__(self) -> str:
        if self.state:
            return f"RpcError(state={self.state}): {super().__str__()}"
        return f"RpcError: {super().__str__()}"


class StateTransitionError(AxonClientError):
    """Invalid state transition attempted"""

    def __init__(self, from_state: str, to_state: str, message: str = None):
        self.from_state = from_state
        self.to_state = to_state
        msg = message or f"Cannot transition from {from_state} to {to_state}"
        super().__init__(msg)


class TaskIdMismatchError(AxonClientError):
    """Task ID in request doesn't match cached config"""

    def __init__(self, expected: str, provided: str):
        self.expected = expected
        self.provided = provided
        super().__init__(f"Task ID mismatch: expected '{expected}', got '{provided}'")


class ConnectionError(AxonClientError):
    """Failed to connect to recorder"""

    pass


class TimeoutError(AxonClientError):
    """Request timed out"""

    pass


class InvalidResponseError(AxonClientError):
    """Invalid response from server"""

    pass
