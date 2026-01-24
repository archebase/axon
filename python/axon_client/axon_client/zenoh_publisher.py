# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Optional Zenoh publisher for broadcasting recorder status.

This module is optional and requires the zenoh Python package.
If zenoh is not installed, ZENOH_AVAILABLE will be False.
"""

import json
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional

from .models import RecorderState, Statistics, TaskConfig


@dataclass
class ZenohConfig:
    """
    Configuration for Zenoh status publisher

    Attributes:
        locator: Zenoh router address (empty for default discovery)
        mode: Zenoh mode ("client" or "peer")
        key_expression: Key expression template for status updates
              Use {device_id} placeholder for device-specific keys
    """

    locator: str = ""
    mode: str = "client"
    key_expression: str = "axon/recorder/{device_id}/status"


class ZenohStatusPublisher:
    """
    Publishes recorder status updates to Zenoh.

    This is an optional component for fleet monitoring scenarios
    where multiple recorders need to broadcast their state.

    Example:
        >>> publisher = ZenohStatusPublisher(ZenohConfig())
        >>> publisher.start()
        >>> publisher.publish_state("robot_01", RecorderState.RECORDING)
        >>> publisher.stop()
    """

    def __init__(self, config: ZenohConfig = None):
        """
        Initialize the Zenoh status publisher.

        Args:
            config: Zenoh configuration

        Note:
            This will not fail if zenoh is not installed.
            Call start() to actually connect.
        """
        self.config = config or ZenohConfig()
        self._session = None
        self._publisher = None
        self._lock = threading.Lock()
        self._running = False

    def start(self) -> bool:
        """
        Connect to Zenoh and start publishing.

        Returns:
            True if connected successfully, False if zenoh not available

        Raises:
            Exception: If Zenoh connection fails
        """
        try:
            import zenoh

            zenoh_config = zenoh.Config()
            if self.config.locator:
                zenoh_config.connect(self.config.locator)
            if self.config.mode == "peer":
                zenoh_config.set_mode(zenoh.Config.Mode.Peer)

            self._session = zenoh.open(zenoh_config)
            self._running = True
            return True

        except ImportError:
            # zenoh not installed - silently disable
            return False
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Zenoh: {e}") from e

    def stop(self):
        """Close Zenoh session and stop publishing."""
        with self._lock:
            if self._session:
                self._session.close()
                self._session = None
            self._running = False

    def _publish(self, key: str, payload: Dict[str, Any]) -> bool:
        """
        Internal helper to publish payload to Zenoh.

        Args:
            key: Zenoh key expression
            payload: Data to publish as JSON

        Returns:
            True if published, False if not running or error occurred
        """
        # Check running state with lock to avoid race condition with stop()
        with self._lock:
            if not self._running or not self._session:
                return False
            # Capture session reference to use outside lock
            session = self._session

        try:
            session.put(key, json.dumps(payload))
            return True
        except Exception as e:
            # Log error for debugging but don't raise
            import logging
            logging.getLogger(__name__).warning(f"Zenoh publish failed for {key}: {e}")
            return False

    def publish_state(
        self, device_id: str, state: RecorderState, task_config: TaskConfig = None
    ) -> bool:
        """
        Publish recorder state to Zenoh.

        Args:
            device_id: Device identifier
            state: Current recorder state
            task_config: Optional task configuration

        Returns:
            True if published, False if not running or zenoh unavailable
        """
        key = self.config.key_expression.format(device_id=device_id)

        payload = {
            "device_id": device_id,
            "state": state.value,
            "timestamp": self._get_timestamp(),
        }

        if task_config:
            payload["task_config"] = task_config.to_dict()

        return self._publish(key, payload)

    def publish_stats(self, device_id: str, stats: Statistics) -> bool:
        """
        Publish recording statistics to Zenoh.

        Args:
            device_id: Device identifier
            stats: Current statistics

        Returns:
            True if published, False if not running or zenoh unavailable
        """
        key = self.config.key_expression.format(device_id=device_id) + "/stats"

        payload = {
            "device_id": device_id,
            "messages_received": stats.messages_received,
            "messages_written": stats.messages_written,
            "messages_dropped": stats.messages_dropped,
            "bytes_written": stats.bytes_written,
            "drop_rate": stats.drop_rate(),
            "timestamp": self._get_timestamp(),
        }

        return self._publish(key, payload)

    def publish_error(self, device_id: str, error: str) -> bool:
        """
        Publish error message to Zenoh.

        Args:
            device_id: Device identifier
            error: Error message

        Returns:
            True if published, False if not running or zenoh unavailable
        """
        key = self.config.key_expression.format(device_id=device_id) + "/error"

        payload = {
            "device_id": device_id,
            "error": error,
            "timestamp": self._get_timestamp(),
        }

        return self._publish(key, payload)

    def _get_timestamp(self) -> int:
        """Get current Unix timestamp in milliseconds"""
        import time

        return int(time.time() * 1000)

    def __enter__(self) -> "ZenohStatusPublisher":
        """Context manager entry"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()
        return False
