#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Zenoh integration example.

This example demonstrates:
1. Using the HTTP client to control the recorder
2. Publishing status updates to Zenoh
3. Subscribing to Zenoh status updates (monitoring)
"""

import sys
import time
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from axon_client import (
    AxonRecorderClient,
    RecorderState,
    TaskConfig,
    ZenohConfig,
    ZenohStatusPublisher,
    ZENOH_AVAILABLE,
)


def main():
    if not ZENOH_AVAILABLE:
        print("ERROR: Zenoh is not installed. Install with:")
        print("  pip install eclipse-zenoh")
        return 1

    print("Zenoh Integration Example")
    print("=" * 50)

    device_id = "robot_01"
    task_id = "zenoh_example_task"

    # Start Zenoh publisher for status updates
    zenoh_config = ZenohConfig(
        locator="",  # Use default discovery
        mode="client",
        key_expression=f"axon/recorder/{device_id}/status",
    )

    print("\nStarting Zenoh publisher...")
    publisher = ZenohStatusPublisher(zenoh_config)

    if not publisher.start():
        print("WARNING: Failed to start Zenoh publisher (continuing without it)")

    try:
        # Connect to recorder
        print("Connecting to Axon recorder...")
        client = AxonRecorderClient(host="localhost", port=8080)

        if not client.health():
            print("ERROR: Recorder is not running")
            return 1

        print("✓ Recorder is healthy")

        # Configure task
        config = TaskConfig(
            task_id=task_id,
            device_id=device_id,
            scene="warehouse",
            topics=["/camera/image_raw", "/lidar/scan"],
        )

        print(f"\nConfiguring task: {task_id}")
        response = client.config(config)

        # Publish state to Zenoh
        publisher.publish_state(device_id, RecorderState.READY, config)
        print("✓ Published READY state to Zenoh")

        # Start recording
        print(f"\nStarting recording...")
        response = client.begin(task_id)

        # Publish recording state
        publisher.publish_state(device_id, RecorderState.RECORDING, config)
        print("✓ Published RECORDING state to Zenoh")

        # Record with periodic status updates
        duration = 10
        print(f"\nRecording for {duration} seconds with status updates...")

        for i in range(duration):
            time.sleep(1)

            # Get and publish statistics
            stats = client.get_stats()
            publisher.publish_stats(device_id, stats)

            print(
                f"  [{i+1}/{duration}] "
                f"Received: {stats.messages_received}, "
                f"Written: {stats.messages_written}, "
                f"Dropped: {stats.messages_dropped}"
            )

        # Stop recording
        print(f"\nStopping recording...")
        response = client.finish(task_id)

        # Publish final state
        publisher.publish_state(device_id, RecorderState.IDLE)
        print("✓ Published IDLE state to Zenoh")

        # Final statistics
        stats = client.get_stats()
        print(f"\nFinal Statistics:")
        print(f"  Messages received: {stats.messages_received}")
        print(f"  Messages written: {stats.messages_written}")
        print(f"  Messages dropped: {stats.messages_dropped}")

        print("\n✓ Example completed successfully")

        # Demonstrate subscribing to status updates
        print("\n" + "=" * 50)
        print("To monitor status updates, run in another terminal:")
        print(f"  zenoh sub {zenoh_config.key_expression.format(device_id=device_id)}")
        print("=" * 50)

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        publisher.publish_state(device_id, RecorderState.IDLE)
        return 130

    except Exception as e:
        print(f"\nERROR: {e}")
        publisher.publish_error(device_id, str(e))
        return 1

    finally:
        publisher.stop()
        client.close()


if __name__ == "__main__":
    sys.exit(main())
