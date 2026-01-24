#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Basic recording example using Axon HTTP RPC client.

This example demonstrates:
1. Connecting to the recorder
2. Configuring a recording task
3. Starting recording
4. Waiting while recording
5. Stopping recording
6. Getting statistics
"""

import sys
import time
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from axon_client import AxonRecorderClient, RecorderState, Statistics, TaskConfig


def main():
    # Connect to recorder (default: localhost:8080)
    print("Connecting to Axon recorder...")
    client = AxonRecorderClient(host="localhost", port=8080)

    try:
        # Check health
        if not client.health():
            print("ERROR: Recorder is not running")
            return 1

        print("✓ Recorder is healthy")

        # Create task configuration
        config = TaskConfig(
            task_id="example_task_001",
            device_id="robot_01",
            scene="warehouse",
            operator_name="example_operator",
            topics=["/camera/image_raw", "/lidar/scan", "/imu/data"],
        )

        print(f"\nConfiguring recording for task: {config.task_id}")
        print(f"  Device: {config.device_id}")
        print(f"  Scene: {config.scene}")
        print(f"  Topics: {', '.join(config.topics)}")

        # Configure (IDLE → READY)
        response = client.config(config)
        if not response.success:
            print(f"ERROR: Config failed: {response.message}")
            return 1

        print("✓ Configuration cached")

        # Start recording (READY → RECORDING)
        print(f"\nStarting recording...")
        response = client.begin(config.task_id)
        if not response.success:
            print(f"ERROR: Begin failed: {response.message}")
            return 1

        print("✓ Recording started")

        # Wait for state transition
        if client.wait_for_state(RecorderState.RECORDING, timeout=5.0):
            print("✓ State: RECORDING")
        else:
            print("WARNING: State did not transition to RECORDING")

        # Record for 10 seconds
        duration = 10
        print(f"\nRecording for {duration} seconds...")

        for i in range(duration):
            time.sleep(1)
            # Get statistics every second
            stats = client.get_stats()
            print(
                f"  [{i+1}/{duration}] "
                f"Received: {stats.messages_received}, "
                f"Written: {stats.messages_written}, "
                f"Dropped: {stats.messages_dropped}"
            )

        # Stop recording (RECORDING → IDLE)
        print(f"\nStopping recording...")
        response = client.finish(config.task_id)
        if not response.success:
            print(f"ERROR: Finish failed: {response.message}")
            return 1

        print("✓ Recording stopped")

        # Get final statistics
        stats = client.get_stats()
        print(f"\nFinal Statistics:")
        print(f"  Messages received: {stats.messages_received}")
        print(f"  Messages written: {stats.messages_written}")
        print(f"  Messages dropped: {stats.messages_dropped}")
        print(f"  Bytes written: {stats.bytes_written:,}")
        print(f"  Drop rate: {stats.drop_rate():.2%}")

        if stats.messages_dropped > 0:
            print("WARNING: Some messages were dropped!")
            return 1

        print("\n✓ Recording completed successfully")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        try:
            client.cancel()
        except Exception:
            pass
        return 130

    except Exception as e:
        print(f"\nERROR: {e}")
        return 1

    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
