# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for ZenohStatusPublisher with mocked zenoh module."""

from unittest import mock
from unittest.mock import MagicMock, patch

import pytest

from axon_client import RecorderState, Statistics, TaskConfig
from axon_client.zenoh_publisher import ZenohConfig, ZenohStatusPublisher


class TestZenohConfig:
    """Tests for ZenohConfig dataclass."""

    def test_default_config(self):
        """Test default Zenoh configuration."""
        config = ZenohConfig()
        assert config.locator == ""
        assert config.mode == "client"
        assert config.key_expression == "axon/recorder/{device_id}/status"

    def test_custom_config(self):
        """Test custom Zenoh configuration."""
        config = ZenohConfig(
            locator="tcp/10.0.0.1:7447",
            mode="peer",
            key_expression="fleet/{device_id}/recorder/status"
        )
        assert config.locator == "tcp/10.0.0.1:7447"
        assert config.mode == "peer"
        assert config.key_expression == "fleet/{device_id}/recorder/status"


class TestZenohStatusPublisherInit:
    """Tests for ZenohStatusPublisher initialization."""

    def test_init_default(self):
        """Test publisher with default config."""
        pub = ZenohStatusPublisher()
        assert pub.config.locator == ""
        assert pub.config.mode == "client"
        assert pub._session is None
        assert pub._running is False

    def test_init_custom_config(self):
        """Test publisher with custom config."""
        config = ZenohConfig(locator="tcp/localhost:7447", mode="peer")
        pub = ZenohStatusPublisher(config)
        assert pub.config == config

    def test_init_creates_lock(self):
        """Test that publisher creates a lock for thread safety."""
        pub = ZenohStatusPublisher()
        assert pub._lock is not None


class TestZenohStatusPublisherStart:
    """Tests for starting Zenoh publisher."""

    def test_start_without_zenoh(self):
        """Test start returns False when zenoh is not installed."""
        pub = ZenohStatusPublisher()
        with patch.dict("sys.modules", {"zenoh": None}):
            result = pub.start()
            assert result is False
            assert pub._running is False

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_start_success(self, mock_zenoh):
        """Test successful start with mocked zenoh."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        result = pub.start()

        assert result is True
        assert pub._running is True
        assert pub._session is not None
        mock_zenoh.open.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_start_with_locator(self, mock_zenoh):
        """Test start with locator configuration."""
        mock_config = MagicMock()
        mock_zenoh.Config.return_value = mock_config
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session

        config = ZenohConfig(locator="tcp/10.0.0.1:7447")
        pub = ZenohStatusPublisher(config)
        pub.start()

        mock_config.connect.assert_called_once_with("tcp/10.0.0.1:7447")

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_start_peer_mode(self, mock_zenoh):
        """Test start with peer mode."""
        mock_config = MagicMock()
        mock_zenoh.Config.return_value = mock_config
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.Mode.Peer = "peer"

        config = ZenohConfig(mode="peer")
        pub = ZenohStatusPublisher(config)
        pub.start()

        mock_config.set_mode.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_start_failure(self, mock_zenoh):
        """Test start raises on connection failure."""
        mock_zenoh.Config.side_effect = RuntimeError("Connection failed")

        pub = ZenohStatusPublisher()
        with pytest.raises(RuntimeError, match="Failed to connect to Zenoh"):
            pub.start()


class TestZenohStatusPublisherStop:
    """Tests for stopping Zenoh publisher."""

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_stop_closes_session(self, mock_zenoh):
        """Test stop closes the zenoh session."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()
        assert pub._running is True

        pub.stop()
        assert pub._running is False
        mock_session.close.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_stop_without_start(self, mock_zenoh):
        """Test stop when not started."""
        pub = ZenohStatusPublisher()
        pub.stop()  # Should not raise
        assert pub._running is False


class TestZenohStatusPublisherPublishState:
    """Tests for publishing state updates."""

    def test_publish_state_not_running(self):
        """Test publish_state returns False when not running."""
        pub = ZenohStatusPublisher()
        result = pub.publish_state("robot_01", RecorderState.RECORDING)
        assert result is False

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_state_success(self, mock_zenoh):
        """Test successful state publish."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        result = pub.publish_state("robot_01", RecorderState.RECORDING)
        assert result is True
        mock_session.put.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_state_with_task_config(self, mock_zenoh):
        """Test publish_state includes task config."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        task_config = TaskConfig(
            task_id="task_001",
            device_id="robot_01",
            topics=["/camera"]
        )
        result = pub.publish_state("robot_01", RecorderState.RECORDING, task_config)

        assert result is True
        call_args = mock_session.put.call_args
        import json
        payload = json.loads(call_args[0][1])
        assert "task_config" in payload

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_state_payload_format(self, mock_zenoh):
        """Test publish_state sends correct payload format."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        pub.publish_state("robot_01", RecorderState.PAUSED)

        call_args = mock_session.put.call_args
        key = call_args[0][0]
        payload_json = call_args[0][1]

        assert key == "axon/recorder/robot_01/status"

        import json
        payload = json.loads(payload_json)
        assert payload["device_id"] == "robot_01"
        assert payload["state"] == "paused"
        assert "timestamp" in payload

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_state_error_handling(self, mock_zenoh):
        """Test publish_state handles errors gracefully."""
        mock_session = MagicMock()
        mock_session.put.side_effect = Exception("Publish failed")
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        result = pub.publish_state("robot_01", RecorderState.RECORDING)
        assert result is False


class TestZenohStatusPublisherPublishStats:
    """Tests for publishing statistics."""

    def test_publish_stats_not_running(self):
        """Test publish_stats returns False when not running."""
        pub = ZenohStatusPublisher()
        stats = Statistics(messages_received=1000, messages_written=950)
        result = pub.publish_stats("robot_01", stats)
        assert result is False

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_stats_success(self, mock_zenoh):
        """Test successful stats publish."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        stats = Statistics(
            messages_received=1000,
            messages_written=950,
            messages_dropped=50,
            bytes_written=1000000
        )
        result = pub.publish_stats("robot_01", stats)

        assert result is True
        mock_session.put.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_stats_payload_format(self, mock_zenoh):
        """Test publish_stats sends correct payload format."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        stats = Statistics(
            messages_received=1000,
            messages_written=950,
            messages_dropped=50,
            bytes_written=1000000
        )
        pub.publish_stats("robot_01", stats)

        call_args = mock_session.put.call_args
        key = call_args[0][0]
        payload_json = call_args[0][1]

        assert key == "axon/recorder/robot_01/status/stats"

        import json
        payload = json.loads(payload_json)
        assert payload["device_id"] == "robot_01"
        assert payload["messages_received"] == 1000
        assert payload["messages_written"] == 950
        assert payload["messages_dropped"] == 50
        assert payload["bytes_written"] == 1000000
        assert abs(payload["drop_rate"] - 0.05) < 0.001
        assert "timestamp" in payload


class TestZenohStatusPublisherPublishError:
    """Tests for publishing error messages."""

    def test_publish_error_not_running(self):
        """Test publish_error returns False when not running."""
        pub = ZenohStatusPublisher()
        result = pub.publish_error("robot_01", "Disk full")
        assert result is False

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_error_success(self, mock_zenoh):
        """Test successful error publish."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        result = pub.publish_error("robot_01", "Recording failed")
        assert result is True
        mock_session.put.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_publish_error_payload_format(self, mock_zenoh):
        """Test publish_error sends correct payload format."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        pub.publish_error("robot_01", "Out of disk space")

        call_args = mock_session.put.call_args
        key = call_args[0][0]
        payload_json = call_args[0][1]

        assert key == "axon/recorder/robot_01/status/error"

        import json
        payload = json.loads(payload_json)
        assert payload["device_id"] == "robot_01"
        assert payload["error"] == "Out of disk space"
        assert "timestamp" in payload


class TestZenohStatusPublisherContextManager:
    """Tests for context manager behavior."""

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_context_manager(self, mock_zenoh):
        """Test publisher as context manager."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        with ZenohStatusPublisher() as pub:
            assert pub._running is True
            assert pub._session is not None

        # Should be stopped after exit
        assert pub._running is False
        mock_session.close.assert_called_once()

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_context_manager_custom_config(self, mock_zenoh):
        """Test context manager with custom config."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        config = ZenohConfig(locator="tcp/localhost:7447")
        with ZenohStatusPublisher(config) as pub:
            assert pub.config.locator == "tcp/localhost:7447"


class TestZenohStatusPublisherThreadSafety:
    """Tests for thread safety."""

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_concurrent_publish(self, mock_zenoh):
        """Test concurrent publish calls are thread-safe."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        pub = ZenohStatusPublisher()
        pub.start()

        import threading
        results = []

        def publish_worker():
            result = pub.publish_state("robot_01", RecorderState.RECORDING)
            results.append(result)

        threads = [threading.Thread(target=publish_worker) for _ in range(10)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # All should succeed without raising exceptions
        assert len(results) == 10
        assert all(r is True for r in results)


class TestZenohStatusPublisherCustomKeyExpression:
    """Tests for custom key expressions."""

    @mock.patch("axon_client.zenoh_publisher.zenoh", create=True)
    def test_custom_key_expression(self, mock_zenoh):
        """Test publishing with custom key expression."""
        mock_session = MagicMock()
        mock_zenoh.open.return_value = mock_session
        mock_zenoh.Config.return_value = MagicMock()

        config = ZenohConfig(key_expression="custom/{device_id}/path")
        pub = ZenohStatusPublisher(config)
        pub.start()

        pub.publish_state("robot_01", RecorderState.RECORDING)

        call_args = mock_session.put.call_args
        key = call_args[0][0]
        assert key == "custom/robot_01/path"
