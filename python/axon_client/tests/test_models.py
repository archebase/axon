# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for axon_client data models."""

import pytest

from axon_client import RecorderState, Statistics, TaskConfig


class TestTaskConfig:
    """Tests for TaskConfig dataclass"""

    def test_create_minimal(self):
        """Test creating minimal valid config"""
        config = TaskConfig(task_id="task_001", device_id="robot_01")
        assert config.task_id == "task_001"
        assert config.device_id == "robot_01"
        assert config.validate() is True

    def test_create_full(self):
        """Test creating full config with all fields"""
        config = TaskConfig(
            task_id="task_001",
            device_id="robot_01",
            data_collector_id="collector_01",
            order_id="order_001",
            operator_name="operator_01",
            scene="warehouse",
            subscene="area_a",
            skills=["picking", "placing"],
            factory="factory_01",
            topics=["/camera", "/lidar"],
            start_callback_url="http://server/start",
            finish_callback_url="http://server/finish",
            user_token="token123",
        )
        assert len(config.topics) == 2
        assert len(config.skills) == 2
        assert config.validate() is True

    def test_to_dict(self):
        """Test serialization to dictionary"""
        config = TaskConfig(
            task_id="task_001", device_id="robot_01", topics=["/camera"]
        )
        data = config.to_dict()
        assert data["task_id"] == "task_001"
        assert data["device_id"] == "robot_01"
        assert data["topics"] == ["/camera"]

    def test_from_dict(self):
        """Test deserialization from dictionary"""
        data = {
            "task_id": "task_001",
            "device_id": "robot_01",
            "topics": ["/camera", "/lidar"],
        }
        config = TaskConfig.from_dict(data)
        assert config.task_id == "task_001"
        assert config.device_id == "robot_01"
        assert config.topics == ["/camera", "/lidar"]

    def test_validate_missing_required(self):
        """Test validation fails with missing required fields"""
        config = TaskConfig(task_id="", device_id="")
        assert config.validate() is False


class TestRecorderState:
    """Tests for RecorderState enum"""

    def test_from_string(self):
        """Test parsing state from string"""
        assert RecorderState.from_string("idle") == RecorderState.IDLE
        assert RecorderState.from_string("IDLE") == RecorderState.IDLE
        assert RecorderState.from_string("ready") == RecorderState.READY
        assert RecorderState.from_string("recording") == RecorderState.RECORDING
        assert RecorderState.from_string("paused") == RecorderState.PAUSED

    def test_from_string_invalid(self):
        """Test parsing invalid string returns IDLE"""
        assert RecorderState.from_string("invalid") == RecorderState.IDLE


class TestStatistics:
    """Tests for Statistics dataclass"""

    def test_create_default(self):
        """Test creating with default values"""
        stats = Statistics()
        assert stats.messages_received == 0
        assert stats.messages_written == 0
        assert stats.messages_dropped == 0
        assert stats.bytes_written == 0

    def test_create_values(self):
        """Test creating with specific values"""
        stats = Statistics(
            messages_received=1000, messages_written=950, messages_dropped=50
        )
        assert stats.messages_received == 1000
        assert stats.messages_written == 950
        assert stats.messages_dropped == 50

    def test_drop_rate(self):
        """Test drop rate calculation"""
        stats = Statistics(messages_received=1000, messages_dropped=50)
        assert stats.drop_rate() == 0.05

    def test_drop_rate_zero_received(self):
        """Test drop rate with zero received"""
        stats = Statistics()
        assert stats.drop_rate() == 0.0

    def test_from_dict(self):
        """Test deserialization from dictionary"""
        data = {
            "messages_received": 1000,
            "messages_written": 950,
            "messages_dropped": 50,
            "bytes_written": 1000000,
        }
        stats = Statistics.from_dict(data)
        assert stats.messages_received == 1000
        assert stats.messages_written == 950
