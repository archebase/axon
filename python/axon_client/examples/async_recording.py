#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Async recording example using Axon HTTP RPC client.

This example demonstrates async operations and concurrent control.
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from axon_client import AsyncAxonRecorderClient, RecorderState, TaskConfig


async def record_task(client: AsyncAxonRecorderClient, task_id: str, duration: int):
    """Record a single task"""
    config = TaskConfig(
        task_id=task_id,
        device_id="robot_01",
        scene="warehouse",
        topics=["/camera/image_raw", "/lidar/scan"],
    )

    print(f"[{task_id}] Configuring...")
    await client.config(config)

    print(f"[{task_id}] Starting recording...")
    await client.begin(task_id)

    # Wait for recording state
    await client.wait_for_state(RecorderState.RECORDING, timeout=5.0)
    print(f"[{task_id}] Recording for {duration} seconds...")

    for i in range(duration):
        await asyncio.sleep(1)
        stats = await client.get_stats()
        print(
            f"[{task_id}] [{i+1}/{duration}] "
            f"Received: {stats.messages_received}, Written: {stats.messages_written}"
        )

    print(f"[{task_id}] Stopping...")
    await client.finish(task_id)

    stats = await client.get_stats()
    print(f"[{task_id}] Final: {stats.messages_written} messages written")


async def monitor_state(client: AsyncAxonRecorderClient, interval: float = 2.0):
    """Monitor recorder state periodically"""
    while True:
        state, task_config = await client.get_state()
        stats = await client.get_stats()
        print(
            f"[Monitor] State: {state.value}, "
            f"Written: {stats.messages_written}, "
            f"Dropped: {stats.messages_dropped}"
        )
        await asyncio.sleep(interval)


async def main():
    """Main async function"""
    print("Connecting to Axon recorder (async)...")

    async with AsyncAxonRecorderClient(host="localhost", port=8080) as client:
        # Check health
        if not await client.health():
            print("ERROR: Recorder is not running")
            return 1

        print("✓ Recorder is healthy")

        # Start monitoring in background
        monitor_task = asyncio.create_task(monitor_state(client))

        # Record a task
        await record_task(client, "async_task_001", duration=10)

        # Cancel monitoring
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            pass

        print("\n✓ Async recording completed")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(130)
