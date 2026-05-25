#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Minimal NiceGUI panel for validating axon-agent HTTP RPC.

Run:
  pip install nicegui httpx
  python3 apps/axon_agent/scripts/agent_nicegui_panel.py --agent-url http://127.0.0.1:8090
"""

from __future__ import annotations

import argparse
from typing import Any

import httpx
from nicegui import ui


class AgentClient:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")
        self.client = httpx.Client(timeout=5.0)

    @staticmethod
    def _decode_response(response: httpx.Response) -> dict[str, Any]:
        try:
            payload = response.json()
        except ValueError:
            response.raise_for_status()
            return {"success": response.is_success, "message": response.text}

        if response.is_error and isinstance(payload, dict):
            return payload
        response.raise_for_status()
        return payload

    def rpc_get(self, path: str) -> dict[str, Any]:
        response = self.client.get(f"{self.base_url}/agent/rpc/{path}")
        return self._decode_response(response)

    def rpc_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        response = self.client.post(f"{self.base_url}/agent/rpc/{path}", json=payload)
        return self._decode_response(response)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal axon-agent NiceGUI panel")
    parser.add_argument("--agent-url", default="http://127.0.0.1:8090", help="axon-agent base URL")
    parser.add_argument("--host", default="0.0.0.0", help="NiceGUI bind host")
    parser.add_argument("--port", type=int, default=8095, help="NiceGUI bind port")
    return parser.parse_args()


args = parse_args()
client = AgentClient(args.agent_url)

PROCESS_ROWS = [
    ("robot_startup", "Robot"),
    ("recorder", "Recorder"),
    ("transfer", "Transfer"),
]

profile_select: ui.select
log_process_select: ui.select
log_stream_select: ui.select
log_tail_input: ui.number
log_view: ui.textarea
state_view: ui.json_editor
profiles_view: ui.json_editor
status_label: ui.label
process_status_labels: dict[str, ui.label] = {}
process_detail_labels: dict[str, ui.label] = {}


def show_status(message: str, *, error: bool = False) -> None:
    status_label.text = message
    status_label.classes(replace="text-red-600" if error else "text-green-700")


def call_agent(action: str, callback) -> None:  # type: ignore[no-untyped-def]
    try:
        result = callback()
        show_status(f"{action}: {result.get('message', 'ok')}", error=not result.get("success", False))
        refresh(announce=False)
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"{action} failed: {exc}", error=True)


def process_status(process: dict[str, Any]) -> tuple[str, str]:
    if process.get("running", False):
        return "Running", "bg-green-100 text-green-800 border border-green-300"

    state = str(process.get("state", "")).lower()
    if state == "stopped" or not process:
        return "Stopped", "bg-gray-100 text-gray-700 border border-gray-300"

    return state or "Unknown", "bg-amber-100 text-amber-800 border border-amber-300"


def update_process_rows(process_items: dict[str, Any]) -> None:
    for process_id, _label in PROCESS_ROWS:
        process = process_items.get(process_id, {})
        status_text, status_classes = process_status(process)

        if status_label_widget := process_status_labels.get(process_id):
            status_label_widget.text = status_text
            status_label_widget.classes(
                replace=f"min-w-20 rounded px-3 py-1 text-center text-sm font-semibold {status_classes}"
            )

        details = []
        pid = int(process.get("pid", -1) or -1)
        if pid > 0:
            details.append(f"PID {pid}")

        health = process.get("health", {})
        health_message = health.get("message")
        if health_message:
            details.append(str(health_message))

        last_error = process.get("last_error")
        if last_error:
            details.append(f"Error: {last_error}")

        if detail_label := process_detail_labels.get(process_id):
            detail_label.text = " | ".join(details) if details else "Not started"


def refresh(*, announce: bool = True) -> None:
    try:
        profiles = client.rpc_get("profiles")
        state = client.rpc_get("state")
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"refresh failed: {exc}", error=True)
        return

    profile_items = profiles.get("data", {}).get("profiles", [])
    options = {
        item["profile_id"]: f"{item['profile_id']} ({item.get('robot_model', 'unknown')})"
        for item in profile_items
    }
    profile_select.options = options
    if not profile_select.value and options:
        profile_select.value = next(iter(options))
    profile_select.update()

    profiles_view.content = {"json": profiles}
    state_view.content = {"json": state}

    process_items = state.get("data", {}).get("processes", {})
    update_process_rows(process_items)

    process_options = {process_id: process_id for process_id in process_items.keys()}
    if not process_options:
        process_options = {
            "robot_startup": "robot_startup",
            "recorder": "recorder",
            "transfer": "transfer",
        }
    log_process_select.options = process_options
    if log_process_select.value not in process_options:
        log_process_select.value = next(iter(process_options))
    log_process_select.update()

    if announce:
        show_status("refreshed")


def select_profile() -> None:
    if not profile_select.value:
        show_status("select profile failed: no profile selected", error=True)
        return
    call_agent(
        "select profile",
        lambda: client.rpc_post("profile/select", {"profile_id": profile_select.value}),
    )


def start_process(process_id: str) -> None:
    call_agent("start " + process_id, lambda: client.rpc_post("process/start", {"process_id": process_id}))


def stop_process(process_id: str, *, force: bool = False) -> None:
    endpoint = "process/force_stop" if force else "process/stop"
    action = ("force stop " if force else "stop ") + process_id
    call_agent(action, lambda: client.rpc_post(endpoint, {"process_id": process_id}))


def read_log() -> None:
    if not log_process_select.value:
        show_status("read log failed: no process selected", error=True)
        return

    try:
        tail_bytes = int(log_tail_input.value or 0)
        result = client.rpc_post(
            "process/log",
            {
                "process_id": log_process_select.value,
                "stream": log_stream_select.value or "stdout",
                "tail_bytes": tail_bytes,
            },
        )
        log_view.value = result.get("data", {}).get("content", "")
        log_view.update()
        show_status(f"read log: {result.get('message', 'ok')}", error=not result.get("success", False))
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"read log failed: {exc}", error=True)


@ui.page("/")
def main_page() -> None:
    global log_process_select, log_stream_select, log_tail_input, log_view
    global profile_select, profiles_view, state_view, status_label

    ui.page_title("Axon Agent Validation Panel")

    with ui.column().classes("w-full max-w-6xl mx-auto p-4 gap-4"):
        ui.label("Axon Agent Validation Panel").classes("text-2xl font-bold")
        ui.label(f"Agent URL: {args.agent_url}").classes("text-gray-600")
        status_label = ui.label("ready")

        with ui.row().classes("items-end gap-2"):
            profile_select = ui.select(label="Robot Profile", options={}).classes("w-96")
            ui.button("Refresh", on_click=refresh)
            ui.button("Select Profile", on_click=select_profile)

        with ui.card().classes("w-full"):
            ui.label("Process Controls").classes("text-lg font-semibold")
            for pid, label in PROCESS_ROWS:
                with ui.row().classes("w-full items-center gap-3 py-2 border-b border-gray-200 last:border-b-0"):
                    ui.label(label).classes("w-28 text-lg font-semibold")
                    process_status_labels[pid] = ui.label("Unknown").classes(
                        "min-w-20 rounded px-3 py-1 text-center text-sm font-semibold bg-gray-100 text-gray-700"
                    )
                    process_detail_labels[pid] = ui.label("Not started").classes(
                        "min-w-0 flex-1 truncate text-sm text-gray-600"
                    )
                    ui.button("Start", on_click=lambda process_id=pid: start_process(process_id))
                    ui.button("Stop", on_click=lambda process_id=pid: stop_process(process_id))
                    ui.button(
                        "Force Stop",
                        color="negative",
                        on_click=lambda process_id=pid: stop_process(process_id, force=True),
                    )

        with ui.tabs().classes("w-full") as tabs:
            state_tab = ui.tab("State")
            profiles_tab = ui.tab("Profiles")
            logs_tab = ui.tab("Logs")

        with ui.tab_panels(tabs, value=state_tab).classes("w-full"):
            with ui.tab_panel(state_tab):
                state_view = ui.json_editor({"content": {"json": {}}}).classes("w-full")
            with ui.tab_panel(profiles_tab):
                profiles_view = ui.json_editor({"content": {"json": {}}}).classes("w-full")
            with ui.tab_panel(logs_tab):
                with ui.row().classes("items-end gap-2"):
                    log_process_select = ui.select(label="Process", options={}).classes("w-64")
                    log_stream_select = ui.select(
                        label="Stream",
                        options={"stdout": "stdout", "stderr": "stderr"},
                        value="stdout",
                    ).classes("w-40")
                    log_tail_input = ui.number(label="Tail Bytes", value=65536, min=0, max=4194304).classes("w-40")
                    ui.button("Read Log", on_click=read_log)
                log_view = ui.textarea(label="Log").props("readonly").classes("w-full h-96 font-mono")

    refresh()
    ui.timer(2.0, lambda: refresh(announce=False))


ui.run(host=args.host, port=args.port, title="Axon Agent Validation Panel", reload=False)
