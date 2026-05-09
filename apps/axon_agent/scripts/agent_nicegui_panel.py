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

    def rpc_get(self, path: str) -> dict[str, Any]:
        response = self.client.get(f"{self.base_url}/agent/rpc/{path}")
        response.raise_for_status()
        return response.json()

    def rpc_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        response = self.client.post(f"{self.base_url}/agent/rpc/{path}", json=payload)
        response.raise_for_status()
        return response.json()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal axon-agent NiceGUI panel")
    parser.add_argument("--agent-url", default="http://127.0.0.1:8090", help="axon-agent base URL")
    parser.add_argument("--host", default="0.0.0.0", help="NiceGUI bind host")
    parser.add_argument("--port", type=int, default=8095, help="NiceGUI bind port")
    return parser.parse_args()


args = parse_args()
client = AgentClient(args.agent_url)

profile_select: ui.select
log_process_select: ui.select
log_stream_select: ui.select
log_tail_input: ui.number
log_view: ui.textarea
state_view: ui.json_editor
profiles_view: ui.json_editor
status_label: ui.label


def show_status(message: str, *, error: bool = False) -> None:
    status_label.text = message
    status_label.classes(replace="text-red-600" if error else "text-green-700")


def call_agent(action: str, callback) -> None:  # type: ignore[no-untyped-def]
    try:
        result = callback()
        show_status(f"{action}: {result.get('message', 'ok')}", error=not result.get("success", False))
        refresh()
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"{action} failed: {exc}", error=True)


def refresh() -> None:
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
            with ui.row().classes("gap-2"):
                for pid, label in [
                    ("robot_startup", "Robot"),
                    ("recorder", "Recorder"),
                    ("transfer", "Transfer"),
                ]:
                    ui.button(f"Start {label}", on_click=lambda process_id=pid: start_process(process_id))
                    ui.button(f"Stop {label}", on_click=lambda process_id=pid: stop_process(process_id))
                    ui.button(
                        f"Force Stop {label}",
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


ui.run(host=args.host, port=args.port, title="Axon Agent Validation Panel", reload=False)
