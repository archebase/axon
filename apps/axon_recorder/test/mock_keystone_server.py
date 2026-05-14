#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Mock Keystone Server for testing WebSocket RPC client mode.

This server simulates a keystone server that:
1. Accepts WebSocket connections from axon_recorder
2. Sends RPC commands (config, begin, finish, etc.)
3. Receives state updates from the recorder

Usage:
    python mock_keystone_server.py [--port PORT]

Example:
    # Terminal 1: Start mock server
    python mock_keystone_server.py --port 8090

    # Terminal 2: Start recorder in WS client mode
    ./build/axon_recorder/axon-recorder \\
        --ws-client \\
        --ws-url ws://localhost:8090/rpc \\
        --config config/default_config_ros2.yaml
"""

import asyncio
import argparse
import json
import sys
import time
from datetime import datetime
from typing import Dict, Any, Optional

try:
    import websockets
    from websockets.server import serve
except ImportError:
    print("Error: websockets library not found")
    print("Install with: pip install websockets")
    sys.exit(1)


class MockKeystoneServer:
    """Mock keystone server for testing WebSocket RPC client mode."""

    def __init__(self, port: int = 8090):
        self.port = port
        self.clients: Dict[str, Any] = {}  # client_id -> websocket
        self.client_states: Dict[str, str] = {}  # client_id -> state
        self.client_configs: Dict[str, Dict] = {}  # websocket -> task_config
        self.request_counter = 0
        self.running = True

    def get_timestamp(self) -> str:
        return datetime.utcnow().isoformat() + "Z"

    def get_timestamp_ms(self) -> int:
        return int(time.time() * 1000)

    async def handle_rpc_request(self, websocket, request: Dict) -> Dict:
        """Handle an RPC request from the recorder."""
        action = request.get("action", "")
        request_id = request.get("request_id", "")
        params = request.get("params", {})

        print(
            f"[{self.get_timestamp()}] Received RPC request: {action} (request_id: {request_id})"
        )

        response = {
            "type": "rpc_response",
            "request_id": request_id,
            "success": False,
            "message": "",
            "data": {},
        }

        if action == "get_state":
            response["success"] = True
            response["message"] = "State retrieved"
            # Get state from cached state
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id:
                response["data"]["state"] = self.client_states.get(client_id, "idle")
            else:
                response["data"]["state"] = "idle"

        elif action == "get_stats":
            response["success"] = True
            response["message"] = "Stats retrieved"
            response["data"]["messages_received"] = 0
            response["data"]["messages_written"] = 0
            response["data"]["messages_dropped"] = 0
            response["data"]["bytes_written"] = 0

        elif action == "config":
            # Store task config
            task_config = params.get("task_config", {})
            task_id = task_config.get("task_id", "unknown")
            self.client_configs[websocket] = task_config
            # Get client_id
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id:
                self.client_states[client_id] = "ready"
            response["success"] = True
            response["message"] = "Configuration set successfully"
            response["data"]["state"] = "ready"
            response["data"]["task_id"] = task_id
            print(f"[{self.get_timestamp()}] Configured recorder: task_id={task_id}")

        elif action == "begin":
            task_id = params.get("task_id", "")
            # Verify task_id
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id and websocket in self.client_configs:
                expected_task_id = self.client_configs[websocket].get("task_id", "")
                if task_id == expected_task_id:
                    self.client_states[client_id] = "recording"
                    response["success"] = True
                    response["message"] = "Recording started"
                    response["data"]["state"] = "recording"
                    print(
                        f"[{self.get_timestamp()}] Started recording: task_id={task_id}"
                    )
                else:
                    response["success"] = False
                    response["message"] = (
                        f"Task ID mismatch: expected {expected_task_id}, got {task_id}"
                    )
            else:
                # Allow begin without config for testing
                if client_id:
                    self.client_states[client_id] = "recording"
                response["success"] = True
                response["message"] = "Recording started"
                response["data"]["state"] = "recording"

        elif action == "pause":
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id:
                self.client_states[client_id] = "paused"
            response["success"] = True
            response["message"] = "Recording paused"
            response["data"]["state"] = "paused"
            print(f"[{self.get_timestamp()}] Paused recording")

        elif action == "resume":
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id:
                self.client_states[client_id] = "recording"
            response["success"] = True
            response["message"] = "Recording resumed"
            response["data"]["state"] = "recording"
            print(f"[{self.get_timestamp()}] Resumed recording")

        elif action == "finish":
            task_id = params.get("task_id", "")
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id and websocket in self.client_configs:
                expected_task_id = self.client_configs[websocket].get("task_id", "")
                if task_id == expected_task_id:
                    self.client_states[client_id] = "idle"
                    del self.client_configs[websocket]
                    response["success"] = True
                    response["message"] = "Recording finished successfully"
                    response["data"]["state"] = "idle"
                    print(
                        f"[{self.get_timestamp()}] Finished recording: task_id={task_id}"
                    )
                else:
                    response["success"] = False
                    response["message"] = (
                        f"Task ID mismatch: expected {expected_task_id}, got {task_id}"
                    )
            else:
                # Allow finish without config for testing
                if client_id:
                    self.client_states[client_id] = "idle"
                if websocket in self.client_configs:
                    del self.client_configs[websocket]
                response["success"] = True
                response["message"] = "Recording finished successfully"
                response["data"]["state"] = "idle"

        elif action == "test":
            # Simple connectivity test command
            response["success"] = True
            response["message"] = "Test command received"
            response["data"]["echo"] = params.get("echo", "pong")
            print(f"[{self.get_timestamp()}] Test command received")

        elif action == "cancel":
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id:
                self.client_states[client_id] = "idle"
                if websocket in self.client_configs:
                    del self.client_configs[websocket]
            response["success"] = True
            response["message"] = "Recording cancelled"
            response["data"]["state"] = "idle"
            print(f"[{self.get_timestamp()}] Cancelled recording")

        elif action == "clear":
            client_id = next(
                (k for k, v in self.clients.items() if v == websocket), None
            )
            if client_id:
                self.client_states[client_id] = "idle"
                if websocket in self.client_configs:
                    del self.client_configs[websocket]
            response["success"] = True
            response["message"] = "Configuration cleared"
            response["data"]["state"] = "idle"
            print(f"[{self.get_timestamp()}] Cleared configuration")

        elif action == "quit":
            response["success"] = True
            response["message"] = "Quit acknowledged"
            print(f"[{self.get_timestamp()}] Client requested quit")

        else:
            response["success"] = False
            response["message"] = f"Unknown action: {action}"

        return response

    async def handle_message(self, websocket, message: str, client_id: str):
        """Handle incoming WebSocket message."""
        try:
            data = json.loads(message)
            msg_type = data.get("type", "")

            if msg_type == "state_update":
                # Recorder sent a state update
                state_data = data.get("data", {})
                prev_state = state_data.get("previous", "unknown")
                curr_state = state_data.get("current", "unknown")
                task_id = state_data.get("task_id", "")
                self.client_states[client_id] = curr_state
                print(
                    f"[{self.get_timestamp()}] State update: {prev_state} -> {curr_state} (task: {task_id})"
                )

            elif msg_type == "rpc_response":
                # Handle responses to server-initiated requests (e.g. initial get_state)
                if data.get("success") and "state" in data.get("data", {}):
                    state = data["data"]["state"]
                    self.client_states[client_id] = state
                    print(
                        f"[{self.get_timestamp()}] Client {client_id} initial status: state={state}"
                    )
                else:
                    print(
                        f"[{self.get_timestamp()}] WARNING: Received unexpected rpc_response from client"
                    )

            else:
                # Treat as RPC request
                response = await self.handle_rpc_request(websocket, data)
                await websocket.send(json.dumps(response))

        except json.JSONDecodeError as e:
            print(f"[{self.get_timestamp()}] ERROR: Invalid JSON: {e}")
        except Exception as e:
            print(f"[{self.get_timestamp()}] ERROR: {e}")

    async def handle_client(self, websocket):
        """Handle a new WebSocket client connection."""
        client_id = f"client_{id(websocket)}"
        self.clients[client_id] = websocket
        self.client_states[client_id] = "idle"

        remote = websocket.remote_address
        print(
            f"[{self.get_timestamp()}] Client connected: {client_id} from {remote[0]}:{remote[1]}"
        )

        # Query initial status from the newly connected recorder
        self.request_counter += 1
        request_id = f"req_{self.request_counter}"
        await websocket.send(
            json.dumps(
                {
                    "action": "get_state",
                    "request_id": request_id,
                    "timestamp_ms": self.get_timestamp_ms(),
                }
            )
        )

        try:
            async for message in websocket:
                await self.handle_message(websocket, message, client_id)
        except websockets.exceptions.ConnectionClosed:
            print(f"[{self.get_timestamp()}] Client disconnected: {client_id}")
        finally:
            if client_id in self.clients:
                del self.clients[client_id]
            if client_id in self.client_states:
                del self.client_states[client_id]
            if websocket in self.client_configs:
                del self.client_configs[websocket]

    async def send_rpc_command(self, client_id: str, action: str, params: Dict = None):
        """Send an RPC command to a specific client."""
        if client_id not in self.clients:
            print(f"ERROR: Client {client_id} not found")
            return

        self.request_counter += 1
        request = {
            "action": action,
            "request_id": f"req_{self.request_counter}",
            "timestamp_ms": self.get_timestamp_ms(),
        }
        if params:
            request["params"] = params

        await self.clients[client_id].send(json.dumps(request))

    async def start(self):
        """Start the mock server."""
        print(
            f"[{self.get_timestamp()}] Starting Mock Keystone Server on port {self.port}"
        )
        print(
            f"[{self.get_timestamp()}] WebSocket endpoint: ws://localhost:{self.port}/rpc"
        )

        async with serve(self.handle_client, "localhost", self.port):
            # Keep running
            while self.running:
                await asyncio.sleep(1)


def main():
    parser = argparse.ArgumentParser(
        description="Mock Keystone Server for testing WebSocket RPC client mode"
    )
    parser.add_argument(
        "--port", type=int, default=8090, help="Port to listen on (default: 8090)"
    )
    args = parser.parse_args()

    server = MockKeystoneServer(port=args.port)

    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")


if __name__ == "__main__":
    main()
