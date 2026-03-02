#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

import asyncio
import os
import time
from collections import defaultdict, deque
from typing import Any

import httpx
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field


RECORDER_RPC_BASE = os.getenv("AXON_RECORDER_RPC_BASE", "http://127.0.0.1:8080")
MAX_EVENTS_PER_DEVICE = int(os.getenv("AXON_FLEET_MAX_EVENTS", "500"))

app = FastAPI(title="Axon Fleet Manager Example", version="0.1.0")


class UploadRequestBody(BaseModel):
    task_id: str
    priority: int = 0


class CancelBody(BaseModel):
    task_id: str


class RecorderConfigBody(BaseModel):
    task_config: dict[str, Any] = Field(default_factory=dict)


class RecorderTaskBody(BaseModel):
    task_id: str


class TransferHub:
    def __init__(self) -> None:
        self._connections: dict[str, WebSocket] = {}
        self._events: dict[str, deque[dict[str, Any]]] = defaultdict(
            lambda: deque(maxlen=MAX_EVENTS_PER_DEVICE)
        )
        self._lock = asyncio.Lock()

    async def connect(self, device_id: str, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            old = self._connections.get(device_id)
            self._connections[device_id] = websocket
            if old and old != websocket:
                await old.close(code=1000, reason="replaced by newer connection")
        self._record(device_id, "system", {"event": "connected"})

    async def disconnect(self, device_id: str, websocket: WebSocket) -> None:
        async with self._lock:
            current = self._connections.get(device_id)
            if current == websocket:
                del self._connections[device_id]
        self._record(device_id, "system", {"event": "disconnected"})

    async def send(self, device_id: str, payload: dict[str, Any]) -> None:
        async with self._lock:
            websocket = self._connections.get(device_id)
        if websocket is None:
            raise HTTPException(
                status_code=404, detail=f"device not connected: {device_id}"
            )

        await websocket.send_json(payload)
        self._record(device_id, "outbound", payload)

    def list_devices(self) -> list[str]:
        return sorted(self._connections.keys())

    def events(self, device_id: str, limit: int) -> list[dict[str, Any]]:
        if limit <= 0:
            return []
        return list(self._events[device_id])[-limit:]

    def _record(self, device_id: str, direction: str, payload: dict[str, Any]) -> None:
        self._events[device_id].append(
            {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "direction": direction,
                "payload": payload,
            }
        )


hub = TransferHub()


async def forward_rpc(
    path: str, payload: dict[str, Any] | None = None
) -> dict[str, Any]:
    url = f"{RECORDER_RPC_BASE}{path}"
    async with httpx.AsyncClient(timeout=5.0) as client:
        if payload is None:
            response = await client.get(url)
        else:
            response = await client.post(url, json=payload)

    try:
        body = response.json()
    except Exception:
        body = {"text": response.text}

    if response.status_code >= 400:
        raise HTTPException(status_code=response.status_code, detail=body)

    return body


@app.get("/health")
async def health() -> dict[str, Any]:
    return {
        "ok": True,
        "recorder_rpc_base": RECORDER_RPC_BASE,
        "connected_transfer_devices": hub.list_devices(),
    }


@app.websocket("/transfer/{device_id}")
async def transfer_socket(websocket: WebSocket, device_id: str) -> None:
    await hub.connect(device_id, websocket)
    try:
        while True:
            payload = await websocket.receive_json()
            hub._record(device_id, "inbound", payload)
    except WebSocketDisconnect:
        await hub.disconnect(device_id, websocket)


@app.get("/api/v1/transfer/devices")
async def transfer_devices() -> dict[str, Any]:
    return {"devices": hub.list_devices()}


@app.get("/api/v1/transfer/{device_id}/events")
async def transfer_events(device_id: str, limit: int = 100) -> dict[str, Any]:
    if limit > MAX_EVENTS_PER_DEVICE:
        limit = MAX_EVENTS_PER_DEVICE
    return {"device_id": device_id, "events": hub.events(device_id, limit)}


@app.post("/api/v1/transfer/{device_id}/upload_request")
async def transfer_upload_request(
    device_id: str, body: UploadRequestBody
) -> dict[str, Any]:
    msg = {"type": "upload_request", "task_id": body.task_id, "priority": body.priority}
    await hub.send(device_id, msg)
    return {"ok": True, "sent": msg}


@app.post("/api/v1/transfer/{device_id}/upload_all")
async def transfer_upload_all(device_id: str) -> dict[str, Any]:
    msg = {"type": "upload_all"}
    await hub.send(device_id, msg)
    return {"ok": True, "sent": msg}


@app.post("/api/v1/transfer/{device_id}/status_query")
async def transfer_status_query(device_id: str) -> dict[str, Any]:
    msg = {"type": "status_query"}
    await hub.send(device_id, msg)
    return {"ok": True, "sent": msg}


@app.post("/api/v1/transfer/{device_id}/cancel")
async def transfer_cancel(device_id: str, body: CancelBody) -> dict[str, Any]:
    msg = {"type": "cancel", "task_id": body.task_id}
    await hub.send(device_id, msg)
    return {"ok": True, "sent": msg}


@app.get("/api/v1/recorder/status")
async def recorder_status() -> dict[str, Any]:
    return await forward_rpc("/rpc/status")


@app.post("/api/v1/recorder/config")
async def recorder_config(body: RecorderConfigBody) -> dict[str, Any]:
    return await forward_rpc("/rpc/config", body.model_dump())


@app.post("/api/v1/recorder/begin")
async def recorder_begin(body: RecorderTaskBody) -> dict[str, Any]:
    return await forward_rpc("/rpc/begin", body.model_dump())


@app.post("/api/v1/recorder/finish")
async def recorder_finish(body: RecorderTaskBody) -> dict[str, Any]:
    return await forward_rpc("/rpc/finish", body.model_dump())


@app.post("/api/v1/recorder/pause")
async def recorder_pause() -> dict[str, Any]:
    return await forward_rpc("/rpc/pause", {})


@app.post("/api/v1/recorder/resume")
async def recorder_resume() -> dict[str, Any]:
    return await forward_rpc("/rpc/resume", {})
