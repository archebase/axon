# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Data models for Axon recorder client.

These classes match the C++ structures in apps/axon_recorder/task_config.hpp
and apps/axon_recorder/state_machine.hpp
"""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional


class RecorderState(Enum):
    """Recorder state matching RecorderState enum in state_machine.hpp"""

    IDLE = "idle"
    READY = "ready"
    RECORDING = "recording"
    PAUSED = "paused"

    @classmethod
    def from_string(cls, value: str) -> "RecorderState":
        """Parse string to RecorderState"""
        for state in cls:
            if state.value == value.lower():
                return state
        return cls.IDLE


@dataclass
class TaskConfig:
    """
    Task configuration matching TaskConfig struct in task_config.hpp

    Fields:
        task_id: Unique task identifier
        device_id: Device identifier
        data_collector_id: Data collector identifier
        order_id: Order identifier
        operator_name: Operator name
        scene: Scene identifier
        subscene: Sub-scene identifier
        skills: List of skills
        factory: Factory identifier
        topics: List of ROS topics to record
        start_callback_url: URL to call when recording starts
        finish_callback_url: URL to call when recording finishes
        user_token: Authentication token
    """

    task_id: str
    device_id: str
    data_collector_id: str = ""
    order_id: str = ""
    operator_name: str = ""
    scene: str = ""
    subscene: str = ""
    skills: List[str] = field(default_factory=list)
    factory: str = ""
    topics: List[str] = field(default_factory=list)
    start_callback_url: str = ""
    finish_callback_url: str = ""
    user_token: str = ""

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            "task_id": self.task_id,
            "device_id": self.device_id,
            "data_collector_id": self.data_collector_id,
            "order_id": self.order_id,
            "operator_name": self.operator_name,
            "scene": self.scene,
            "subscene": self.subscene,
            "skills": self.skills,
            "factory": self.factory,
            "topics": self.topics,
            "start_callback_url": self.start_callback_url,
            "finish_callback_url": self.finish_callback_url,
            "user_token": self.user_token,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TaskConfig":
        """Create from dictionary (JSON deserialization)"""
        return cls(
            task_id=data.get("task_id", ""),
            device_id=data.get("device_id", ""),
            data_collector_id=data.get("data_collector_id", ""),
            order_id=data.get("order_id", ""),
            operator_name=data.get("operator_name", ""),
            scene=data.get("scene", ""),
            subscene=data.get("subscene", ""),
            skills=data.get("skills", []),
            factory=data.get("factory", ""),
            topics=data.get("topics", []),
            start_callback_url=data.get("start_callback_url", ""),
            finish_callback_url=data.get("finish_callback_url", ""),
            user_token=data.get("user_token", ""),
        )

    def validate(self) -> bool:
        """Validate required fields"""
        return bool(self.task_id and self.device_id)


@dataclass
class Statistics:
    """
    Recording statistics matching get_statistics() in recorder.hpp

    Fields:
        messages_received: Total messages received from middleware
        messages_written: Total messages written to MCAP
        messages_dropped: Total messages dropped (queue full)
        bytes_written: Total bytes written to MCAP
    """

    messages_received: int = 0
    messages_written: int = 0
    messages_dropped: int = 0
    bytes_written: int = 0

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Statistics":
        """Create from dictionary (JSON deserialization)"""
        return cls(
            messages_received=data.get("messages_received", 0),
            messages_written=data.get("messages_written", 0),
            messages_dropped=data.get("messages_dropped", 0),
            bytes_written=data.get("bytes_written", 0),
        )

    def drop_rate(self) -> float:
        """Calculate drop rate as ratio of dropped to received"""
        if self.messages_received == 0:
            return 0.0
        return self.messages_dropped / self.messages_received

    def __str__(self) -> str:
        """String representation for logging"""
        return (
            f"Statistics(received={self.messages_received}, "
            f"written={self.messages_written}, "
            f"dropped={self.messages_dropped}, "
            f"bytes={self.bytes_written})"
        )


@dataclass
class RpcResponse:
    """
    Response from an RPC endpoint

    Fields:
        success: True if the request succeeded
        message: Human-readable message
        data: Additional data (varies by endpoint)
        state: Current recorder state (if available)
    """

    success: bool
    message: str
    data: Dict[str, Any] = field(default_factory=dict)
    state: Optional[RecorderState] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "RpcResponse":
        """Create from dictionary (JSON deserialization)"""
        state_str = data.get("data", {}).get("state")
        state = RecorderState.from_string(state_str) if state_str else None

        return cls(
            success=data.get("success", False),
            message=data.get("message", ""),
            data=data.get("data", {}),
            state=state,
        )

    def raise_for_error(self) -> None:
        """Raise an exception if success is False"""
        if not self.success:
            raise RpcError(self.message, self.state.value if self.state else None, self.data)
