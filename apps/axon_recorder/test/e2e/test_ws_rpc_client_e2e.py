#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
E2E test for WebSocket RPC client mode.

This test:
1. Starts a mock keystone server
2. Connects to the mock server as a WebSocket client
3. Sends RPC commands and verifies responses
4. Verifies state transitions
"""

import asyncio
import json
import os
import subprocess
import sys
import time
from typing import Dict, Optional

try:
    import websockets
except ImportError:
    print("Error: websockets library not found")
    print("Install with: pip install websockets")
    sys.exit(1)


class E2ETestResult:
    """Result of an E2E test."""
    def __init__(self):
        self.success = False
        self.error = None
        self.state_transitions = []
        self.messages_received = 0


class WsRpcClientE2ETest:
    """E2E test for WebSocket RPC client mode."""

    def __init__(self):
        self.server_process = None
        self.recorder_process = None
        self.server_port = 8092
        self.ws_url = f"ws://localhost:{self.server_port}/rpc"
        self.test_result = E2ETestResult()
        self.shutdown_event = asyncio.Event()

    async def start_mock_server(self):
        """Start the mock keystone server."""
        test_dir = os.path.dirname(os.path.dirname(__file__))
        self.server_process = subprocess.Popen(
            [sys.executable, os.path.join(test_dir, "mock_keystone_server.py"), "--port", str(self.server_port)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=test_dir
        )
        # Wait for server to start
        await asyncio.sleep(2)

        # Check if server started successfully
        if self.server_process.poll() is not None:
            stderr_output = self.server_process.stderr.read().decode() if self.server_process.stderr else "Unknown error"
            raise RuntimeError(f"Failed to start mock server: {stderr_output}")

    async def send_rpc_command(self, websocket, action: str, params: Dict = None) -> Dict:
        """Send an RPC command to the server."""
        request_id = f"req_{int(time.time() * 1000)}"
        request = {
            "action": action,
            "request_id": request_id,
        }
        if params:
            request["params"] = params

        await websocket.send(json.dumps(request))

        # Wait for response
        response_str = await asyncio.wait_for(websocket.recv(), timeout=10.0)
        return json.loads(response_str)

    async def wait_for_state_update(self, websocket, timeout: float = 5.0) -> Optional[Dict]:
        """Wait for a state update message."""
        try:
            message = await asyncio.wait_for(websocket.recv(), timeout=timeout)
            data = json.loads(message)
            if data.get("type") == "state_update":
                return data
            return None
        except asyncio.TimeoutError:
            return None

    async def run_test(self) -> E2ETestResult:
        """Run the full E2E test."""
        result = E2ETestResult()

        try:
            # 1. Start mock server
            print("Starting mock keystone server...")
            await self.start_mock_server()
            print(f"Mock server started on port {self.server_port}")

            # 2. Connect to the server
            print(f"Connecting to {self.ws_url}...")
            async with websockets.connect(self.ws_url) as websocket:
                print("Connected to mock server")

                # 3. Send get_state command
                print("Testing get_state...")
                response = await self.send_rpc_command(websocket, "get_state")
                if not response:
                    result.error = "No response for get_state"
                    return result

                if not response.get("success"):
                    result.error = f"get_state failed: {response.get('message')}"
                    return result

                result.state_transitions.append(f"get_state: {response.get('data', {}).get('state')}")
                print(f"  State: {response.get('data', {}).get('state')}")

                # 4. Send config command
                print("Testing config...")
                task_config = {
                    "task_id": "e2e_test_task",
                    "device_id": "test_device",
                    "topics": ["/test/topic1", "/test/topic2"],
                    "start_callback_url": "",
                    "finish_callback_url": ""
                }
                response = await self.send_rpc_command(websocket, "config", {"task_config": task_config})
                if not response.get("success"):
                    result.error = f"config failed: {response.get('message')}"
                    return result

                result.state_transitions.append(f"config: {response.get('data', {}).get('state')}")
                print(f"  State: {response.get('data', {}).get('state')}")

                # 5. Send begin command
                print("Testing begin...")
                response = await self.send_rpc_command(websocket, "begin", {"task_id": "e2e_test_task"})
                if not response.get("success"):
                    result.error = f"begin failed: {response.get('message')}"
                    return result

                result.state_transitions.append(f"begin: {response.get('data', {}).get('state')}")
                print(f"  State: {response.get('data', {}).get('state')}")

                # 6. Send finish command
                print("Testing finish...")
                response = await self.send_rpc_command(websocket, "finish", {"task_id": "e2e_test_task"})
                if not response.get("success"):
                    result.error = f"finish failed: {response.get('message')}"
                    return result

                result.state_transitions.append(f"finish: {response.get('data', {}).get('state')}")
                print(f"  State: {response.get('data', {}).get('state')}")

                # 7. Verify final state
                print("Verifying final state...")
                response = await self.send_rpc_command(websocket, "get_state")
                final_state = response.get("data", {}).get("state")
                print(f"  Final state: {final_state}")

                if final_state == "idle":
                    result.success = True
                else:
                    result.error = f"Expected final state 'idle', got '{final_state}'"

        except Exception as e:
            result.error = str(e)
            import traceback
            traceback.print_exc()
        finally:
            await self.cleanup()

        return result

    async def cleanup(self):
        """Clean up test resources."""
        if self.server_process:
            self.server_process.terminate()
            try:
                self.server_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.server_process.kill()
                self.server_process.wait()

    def test_full_workflow(self):
        """Run the full workflow test."""
        result = asyncio.run(self.run_test())

        # Print results
        print("\n" + "=" * 60)
        print("E2E Test Results")
        print("=" * 60)
        print(f"Success: {result.success}")
        if result.error:
            print(f"Error: {result.error}")
        print(f"State transitions: {result.state_transitions}")
        print("=" * 60)

        if not result.success:
            raise AssertionError(f"E2E test failed: {result.error}")


def main():
    """Main entry point."""
    test = WsRpcClientE2ETest()

    try:
        test.test_full_workflow()
        print("\n✅ E2E test PASSED!")
        return 0
    except AssertionError as e:
        print(f"\n❌ E2E test FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ E2E test ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
