#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
E2E test for WebSocket RPC client mode.

This test:
1. Starts a test WebSocket server (simulates keystone server)
2. Starts axon_recorder in WS RPC client mode (connects to test server)
3. Sends RPC commands to the connected recorder
4. Verifies the recorder's responses
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


class TestWebSocketServer:
    """Test WebSocket server that sends RPC commands to connected recorder."""

    def __init__(self, port: int = 8093):
        self.port = port
        self.recorder_ws = None
        self.client_connected = asyncio.Event()
        self.response_queue = asyncio.Queue()
        self.request_counter = 0
        self._server = None

    async def handle_client(self, websocket):
        """Handle a WebSocket client (the recorder) connection."""
        print(f"[Server] Recorder connected from {websocket.remote_address}")
        self.recorder_ws = websocket
        self.client_connected.set()

        try:
            async for message in websocket:
                data = json.loads(message)
                print(f"[Server] Received: {json.dumps(data)[:200]}")
                await self.response_queue.put(data)
        except websockets.exceptions.ConnectionClosed:
            print("[Server] Recorder disconnected")
        finally:
            self.recorder_ws = None
            self.client_connected.clear()

    async def send_rpc_command(self, action: str, params: Dict = None, timeout: float = 10.0) -> Optional[Dict]:
        """Send an RPC command to the connected recorder and wait for response."""
        if not self.recorder_ws:
            print("[Server] ERROR: No recorder connected")
            return None

        self.request_counter += 1
        request_id = f"req_{int(time.time() * 1000)}_{self.request_counter}"
        request = {
            "action": action,
            "request_id": request_id,
        }
        if params:
            request["params"] = params

        print(f"[Server] Sending RPC command: {action} (request_id: {request_id})")
        await self.recorder_ws.send(json.dumps(request))

        # Wait for response with matching request_id
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                remaining = deadline - time.time()
                response = await asyncio.wait_for(
                    self.response_queue.get(), timeout=min(remaining, 1.0)
                )
                if response.get("request_id") == request_id:
                    return response
                # Not matching, continue waiting
            except asyncio.TimeoutError:
                continue
        print(f"[Server] Timeout waiting for response to {request_id}")
        return None

    async def start(self):
        """Start the test WebSocket server."""
        self._server = await websockets.serve(self.handle_client, "localhost", self.port)
        print(f"[Server] Test WebSocket server started on port {self.port}")

    async def stop(self):
        """Stop the test WebSocket server."""
        if self._server:
            self._server.close()
            await self._server.wait_closed()


class WsRpcClientE2ETest:
    """E2E test for WebSocket RPC client mode."""

    def __init__(self):
        self.test_server = None
        self.recorder_process = None
        # Use a different port to avoid conflicts
        self.server_port = 8093
        self.ws_url = f"ws://localhost:{self.server_port}/rpc"
        self.project_root = self._find_project_root()

    def _find_project_root(self) -> str:
        """Find the project root directory."""
        current = os.path.dirname(os.path.abspath(__file__))
        while current != "/":
            if os.path.exists(os.path.join(current, "Makefile")):
                return current
            current = os.path.dirname(current)
        return os.getcwd()

    async def start_test_server(self):
        """Start the test WebSocket server."""
        self.test_server = TestWebSocketServer(port=self.server_port)
        await self.test_server.start()
        # Give server time to fully bind
        await asyncio.sleep(0.5)

    async def start_recorder(self):
        """Start the axon_recorder in WS RPC client mode."""
        recorder_bin = os.path.join(self.project_root, "build/axon_recorder/axon_recorder")
        config_file = os.path.join(self.project_root, "apps/axon_recorder/config/default_config_ros2.yaml")
        plugin_path = os.path.join(self.project_root, "build/middlewares/udp_plugin/libaxon_udp.so")

        if not os.path.exists(recorder_bin):
            raise RuntimeError(f"Recorder binary not found: {recorder_bin}")

        # Create temp config directory
        temp_config_dir = os.path.join(self.project_root, "apps/axon_recorder/test/e2e")
        os.makedirs(temp_config_dir, exist_ok=True)
        temp_config = os.path.join(temp_config_dir, "temp_test_config.yaml")

        # Read and modify config
        with open(config_file, 'r') as f:
            config_content = f.read()

        # Replace the ws_client URL with our test server URL
        config_content = config_content.replace(
            "url: ws://localhost:8090/rpc",
            f"url: {self.ws_url}"
        )

        with open(temp_config, 'w') as f:
            f.write(config_content)

        cmd = [
            recorder_bin,
            "--ws-client",
            "--ws-url", self.ws_url,
            "--config", temp_config,
        ]

        if os.path.exists(plugin_path):
            cmd.extend(["--plugin", plugin_path])

        print(f"[Test] Starting recorder: {' '.join(cmd)}")

        self.recorder_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=self.project_root
        )

        # Wait for recorder to connect
        print("[Test] Waiting for recorder to connect...")
        try:
            await asyncio.wait_for(self.test_server.client_connected.wait(), timeout=15.0)
            print("[Test] Recorder connected!")
        except asyncio.TimeoutError:
            # Check if process died
            if self.recorder_process.poll() is not None:
                stdout, stderr = self.recorder_process.communicate()
                raise RuntimeError(f"Recorder failed to start:\nstdout: {stdout.decode()}\nstderr: {stderr.decode()}")
            raise RuntimeError("Timeout waiting for recorder to connect")

    async def cleanup(self):
        """Clean up test resources."""
        if self.recorder_process:
            self.recorder_process.terminate()
            try:
                self.recorder_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.recorder_process.kill()
                self.recorder_process.wait()

        if self.test_server:
            await self.test_server.stop()

        # Clean up temp config
        temp_config = os.path.join(self.project_root, "apps/axon_recorder/test/e2e/temp_test_config.yaml")
        if os.path.exists(temp_config):
            os.remove(temp_config)

    async def run_test(self) -> E2ETestResult:
        """Run the full E2E test."""
        result = E2ETestResult()

        try:
            # 1. Start test WebSocket server
            print("=" * 60)
            print("Step 1: Starting test WebSocket server...")
            await self.start_test_server()
            print(f"  Server listening on port {self.server_port}")

            # 2. Start recorder in WS client mode
            print("\nStep 2: Starting recorder in WS client mode...")
            await self.start_recorder()
            print("  Recorder started and connected")

            # 3. Send test command (connectivity check)
            print("\nStep 3: Sending test command...")
            response = await self.test_server.send_rpc_command("test", {"echo": "hello"})
            if not response:
                result.error = "No response for test command"
                return result

            if not response.get("success"):
                result.error = f"test command failed: {response.get('message')}"
                return result

            echo_value = response.get("data", {}).get("echo")
            if echo_value != "hello":
                result.error = f"Test command echo mismatch: expected 'hello', got '{echo_value}'"
                return result

            result.state_transitions.append(f"test: success (echo={echo_value})")
            print(f"  Response: success, echo={echo_value}")

            # 4. Send get_state command
            print("\nStep 4: Sending get_state command...")
            response = await self.test_server.send_rpc_command("get_state")
            if not response:
                result.error = "No response for get_state"
                return result

            if not response.get("success"):
                result.error = f"get_state failed: {response.get('message')}"
                return result

            state = response.get("data", {}).get("state")
            result.state_transitions.append(f"get_state: {state}")
            print(f"  State: {state}")

            # 5. Send config command
            print("\nStep 5: Sending config command...")
            task_config = {
                "task_id": "e2e_test_task",
                "device_id": "test_device",
                "topics": ["/test/topic1", "/test/topic2"],
                "start_callback_url": "",
                "finish_callback_url": ""
            }
            response = await self.test_server.send_rpc_command("config", {"task_config": task_config})
            if not response:
                result.error = "No response for config"
                return result

            if not response.get("success"):
                result.error = f"config failed: {response.get('message')}"
                return result

            state = response.get("data", {}).get("state")
            result.state_transitions.append(f"config: {state}")
            print(f"  State: {state}")

            # 6. Send begin command
            print("\nStep 6: Sending begin command...")
            response = await self.test_server.send_rpc_command("begin", {"task_id": "e2e_test_task"})
            if not response:
                result.error = "No response for begin"
                return result

            if not response.get("success"):
                result.error = f"begin failed: {response.get('message')}"
                return result

            state = response.get("data", {}).get("state")
            result.state_transitions.append(f"begin: {state}")
            print(f"  State: {state}")

            # 7. Send finish command
            print("\nStep 7: Sending finish command...")
            response = await self.test_server.send_rpc_command("finish", {"task_id": "e2e_test_task"})
            if not response:
                result.error = "No response for finish"
                return result

            if not response.get("success"):
                result.error = f"finish failed: {response.get('message')}"
                return result

            state = response.get("data", {}).get("state")
            result.state_transitions.append(f"finish: {state}")
            print(f"  State: {state}")

            # 8. Verify final state
            print("\nStep 8: Verifying final state...")
            response = await self.test_server.send_rpc_command("get_state")
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
