#!/usr/bin/env python3
#
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""NiceGUI panel for validating axon-agent HTTP RPC and axon-system telemetry.

Run:
  pip install nicegui httpx
  python3 apps/axon_agent/scripts/agent_nicegui_panel.py \
    --agent-url http://127.0.0.1:8090 \
    --system-url http://127.0.0.1:8091
"""

from __future__ import annotations

import argparse
import json
import time
from collections import deque
from typing import Any, Callable

from fastapi import Request
import httpx
from nicegui import ui


def decode_response(response: httpx.Response) -> dict[str, Any]:
    try:
        payload = response.json()
    except ValueError:
        response.raise_for_status()
        return {"success": response.is_success, "message": response.text}

    if response.is_error and isinstance(payload, dict):
        return payload
    response.raise_for_status()
    return payload


class AgentClient:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")
        self.client = httpx.Client(timeout=5.0)

    def rpc_get(self, path: str) -> dict[str, Any]:
        response = self.client.get(f"{self.base_url}/agent/rpc/{path}")
        return decode_response(response)

    def rpc_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        response = self.client.post(f"{self.base_url}/agent/rpc/{path}", json=payload)
        return decode_response(response)


class SystemClient:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")
        self.client = httpx.Client(timeout=2.0)

    def get_health(self) -> dict[str, Any]:
        response = self.client.get(f"{self.base_url}/health")
        return decode_response(response)

    def get_metrics(self) -> dict[str, Any]:
        response = self.client.get(f"{self.base_url}/rpc/metrics")
        return decode_response(response)

    def get_alerts(self) -> dict[str, Any]:
        response = self.client.get(f"{self.base_url}/rpc/alerts")
        return decode_response(response)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Axon agent and system NiceGUI panel")
    parser.add_argument("--agent-url", default="http://127.0.0.1:8090", help="axon-agent base URL")
    parser.add_argument("--system-url", default="http://127.0.0.1:8091", help="axon-system base URL")
    parser.add_argument(
        "--history-seconds",
        type=int,
        default=300,
        help="seconds of in-memory system metric history to show",
    )
    parser.add_argument("--language", choices=("zh", "en"), default="zh", help="default UI language")
    parser.add_argument("--host", default="0.0.0.0", help="NiceGUI bind host")
    parser.add_argument("--port", type=int, default=8095, help="NiceGUI bind port")
    return parser.parse_args()


args = parse_args()
client = AgentClient(args.agent_url)
system_client = SystemClient(args.system_url)

PROCESS_ROWS = [
    ("robot_startup", "process_robot"),
    ("recorder", "process_recorder"),
    ("transfer", "process_transfer"),
]

SYSTEM_HISTORY: deque[dict[str, Any]] = deque()
current_language = "zh"

TEXT = {
    "zh": {
        "action_failed": "失败",
        "agent_error": "Agent 错误",
        "agent_pid": "Agent PID",
        "action_args_json": "参数 JSON",
        "action_execute": "执行 Action",
        "action_execute_failed_bad_args": "执行失败：参数必须是 JSON object",
        "action_execute_failed_no_action": "执行失败：未选择 Action",
        "action_execute_failed_no_request": "执行失败：request_id 不能为空",
        "action_id": "Action",
        "action_request_id": "Request ID",
        "action_sync": "Keystone Action Sync",
        "actions_summary": "已加载 Action {loaded} 个 | 诊断 {diagnostics} 条",
        "agent_url": "Agent 地址",
        "alerts_counts": "告警 firing {firing} | pending {pending}",
        "alerts_zero": "告警 firing 0 | pending 0",
        "active_profile": "当前配置",
        "adapter": "Adapter",
        "available": "可用",
        "chart_cpu_memory": "CPU 和内存",
        "chart_network_rate": "网络速率",
        "completed": "完成",
        "count": "数量",
        "cpu_na": "CPU 暂无数据",
        "detail_empty": "暂无详细数据。",
        "details": "详细",
        "disk": "磁盘",
        "disk_na": "磁盘 暂无数据",
        "disk_unavailable": "磁盘 {disk_id} 不可用：{error}",
        "en": "EN",
        "executions": "执行记录",
        "failed": "失败",
        "force_stop": "强制停止",
        "force_stop_action": "强制停止",
        "header_title": "Axon Agent 验证面板",
        "health": "健康",
        "incident_bundle": "Incident bundle {state}",
        "invalid": "无效",
        "lang_label": "语言",
        "load": "负载",
        "loaded": "已加载",
        "log_process": "进程",
        "log_stream": "日志流",
        "logs": "日志",
        "memory": "内存",
        "memory_na": "内存 暂无数据",
        "network_na": "网络 暂无数据",
        "network_rates": "网络 RX {rx} | TX {tx}",
        "no_log_loaded": "尚未读取日志。",
        "not_started": "尚未启动",
        "no_data": "暂无数据",
        "page_title": "Axon Agent 验证面板",
        "pending": "等待",
        "performance": "性能",
        "processes": "进程",
        "process_controls": "进程控制",
        "process_recorder": "录制",
        "process_robot": "机器人",
        "process_transfer": "传输",
        "profiles": "配置",
        "read_log": "读取日志",
        "read_log_action": "读取日志",
        "read_log_failed_no_process": "读取日志失败：未选择进程",
        "ready": "就绪",
        "recorder": "Recorder",
        "recorder_summary": "Recorder {state} | Sidecar {sidecar} | {incident} | Time gap {time_gap}",
        "recorder_unavailable": "Recorder 诊断不可用：{message}",
        "refresh": "刷新",
        "refresh_failed": "刷新失败",
        "refreshed": "已刷新",
        "rejected": "拒绝",
        "robot_profile": "机器人配置",
        "rpc_error": "RPC 错误",
        "rpc_reachable": "RPC 可达",
        "running": "运行中",
        "select_profile": "选择配置",
        "select_profile_action": "选择配置",
        "select_profile_failed_no_profile": "选择配置失败：未选择配置",
        "start": "启动",
        "start_action": "启动",
        "stop": "停止",
        "stop_action": "停止",
        "stopped": "已停止",
        "system": "系统",
        "system_prefix": "系统",
        "system_refresh_failed": "系统刷新失败",
        "system_url": "系统地址",
        "tab_action_executions": "Action 记录",
        "tab_action_sync": "Keystone Sync",
        "tab_actions": "Actions",
        "tab_agent_report": "Agent Report",
        "tab_logs": "日志",
        "tab_profiles": "配置",
        "tab_recorder_status": "Recorder 诊断",
        "tab_state": "状态",
        "tab_system_alerts": "系统告警",
        "tab_system_metrics": "系统指标",
        "tail_bytes": "Tail 字节数",
        "unknown": "未知",
        "unknown_error": "未知错误",
        "used": "已用",
        "view_detail": "详细",
        "view_raw": "Raw",
        "view_simple": "简约",
        "zh": "中文",
    },
    "en": {
        "action_failed": "failed",
        "agent_error": "Agent error",
        "agent_pid": "Agent PID",
        "action_args_json": "Args JSON",
        "action_execute": "Execute Action",
        "action_execute_failed_bad_args": "execute failed: args must be a JSON object",
        "action_execute_failed_no_action": "execute failed: no action selected",
        "action_execute_failed_no_request": "execute failed: request_id is required",
        "action_id": "Action",
        "action_request_id": "Request ID",
        "action_sync": "Keystone Action Sync",
        "actions_summary": "Loaded actions {loaded} | diagnostics {diagnostics}",
        "agent_url": "Agent URL",
        "alerts_counts": "Alerts firing {firing} | pending {pending}",
        "alerts_zero": "Alerts firing 0 | pending 0",
        "active_profile": "Active Profile",
        "adapter": "Adapter",
        "available": "available",
        "chart_cpu_memory": "CPU and memory",
        "chart_network_rate": "Network rate",
        "completed": "Completed",
        "count": "Count",
        "cpu_na": "CPU n/a",
        "detail_empty": "No detail data.",
        "details": "Details",
        "disk": "Disk",
        "disk_na": "Disk n/a",
        "disk_unavailable": "Disk {disk_id} unavailable: {error}",
        "en": "EN",
        "executions": "Executions",
        "failed": "Failed",
        "force_stop": "Force Stop",
        "force_stop_action": "force stop",
        "header_title": "Axon Agent Validation Panel",
        "health": "Health",
        "incident_bundle": "Incident bundle {state}",
        "invalid": "Invalid",
        "lang_label": "Language",
        "load": "load",
        "loaded": "Loaded",
        "log_process": "Process",
        "log_stream": "Stream",
        "logs": "Logs",
        "memory": "Memory",
        "memory_na": "Memory n/a",
        "network_na": "Network n/a",
        "network_rates": "Network RX {rx} | TX {tx}",
        "no_log_loaded": "No log loaded.",
        "not_started": "Not started",
        "no_data": "No data",
        "page_title": "Axon Agent Validation Panel",
        "pending": "Pending",
        "performance": "Performance",
        "processes": "Processes",
        "process_controls": "Process Controls",
        "process_recorder": "Recorder",
        "process_robot": "Robot",
        "process_transfer": "Transfer",
        "profiles": "Profiles",
        "read_log": "Read Log",
        "read_log_action": "read log",
        "read_log_failed_no_process": "read log failed: no process selected",
        "ready": "ready",
        "recorder": "Recorder",
        "recorder_summary": "Recorder {state} | Sidecar {sidecar} | {incident} | Time gap {time_gap}",
        "recorder_unavailable": "Recorder diagnostics unavailable: {message}",
        "refresh": "Refresh",
        "refresh_failed": "refresh failed",
        "refreshed": "refreshed",
        "rejected": "Rejected",
        "robot_profile": "Robot Profile",
        "rpc_error": "RPC error",
        "rpc_reachable": "RPC reachable",
        "running": "Running",
        "select_profile": "Select Profile",
        "select_profile_action": "select profile",
        "select_profile_failed_no_profile": "select profile failed: no profile selected",
        "start": "Start",
        "start_action": "start",
        "stop": "Stop",
        "stop_action": "stop",
        "stopped": "Stopped",
        "system": "System",
        "system_prefix": "System",
        "system_refresh_failed": "System refresh failed",
        "system_url": "System URL",
        "tab_action_executions": "Action Executions",
        "tab_action_sync": "Keystone Sync",
        "tab_actions": "Actions",
        "tab_agent_report": "Agent Report",
        "tab_logs": "Logs",
        "tab_profiles": "Profiles",
        "tab_recorder_status": "Recorder Diagnostics",
        "tab_state": "State",
        "tab_system_alerts": "System Alerts",
        "tab_system_metrics": "System Metrics",
        "tail_bytes": "Tail Bytes",
        "unknown": "Unknown",
        "unknown_error": "unknown error",
        "used": "used",
        "view_detail": "Detail",
        "view_raw": "Raw",
        "view_simple": "Simple",
        "zh": "中文",
    },
}

profile_select: ui.select
action_select: ui.select
action_request_input: ui.input
action_args_textarea: ui.textarea
action_expires_input: ui.input
log_process_select: ui.select
log_stream_select: ui.select
log_tail_input: ui.number
log_view: ui.code
action_result_view: ui.code
status_label: ui.label
system_status_label: ui.label
system_error_label: ui.label
system_overview_container: ui.column
system_resource_chart: ui.echart
system_network_chart: ui.echart
MODULE_VIEWS: dict[str, dict[str, Any]] = {}
process_status_labels: dict[str, ui.label] = {}
process_detail_labels: dict[str, ui.label] = {}


def normalize_language(language: str | None) -> str:
    return language if language in TEXT else "zh"


def t(key: str, **kwargs: Any) -> str:
    template = TEXT.get(current_language, TEXT["zh"]).get(key, TEXT["en"].get(key, key))
    return template.format(**kwargs) if kwargs else template


def set_current_language(language: str | None) -> None:
    global current_language
    current_language = normalize_language(language)


def render_language_switch(language: str) -> None:
    set_current_language(language)
    with ui.row().classes(
        "fixed top-4 right-4 z-50 items-center gap-1 rounded border border-gray-200 "
        + "bg-white p-1 shadow-lg"
    ):
        ui.label(t("lang_label")).classes("px-2 text-xs font-semibold text-gray-600")
        zh_button = ui.button(t("zh"), on_click=lambda: ui.navigate.to("/?lang=zh")).props("dense")
        en_button = ui.button(t("en"), on_click=lambda: ui.navigate.to("/?lang=en")).props("dense")
        if language == "zh":
            zh_button.props("unelevated color=primary")
            en_button.props("flat")
        else:
            zh_button.props("flat")
            en_button.props("unelevated color=primary")


def show_status(message: str, *, error: bool = False) -> None:
    status_label.text = message
    status_label.classes(replace="text-red-600" if error else "text-green-700")


def call_agent(action: str, callback) -> None:  # type: ignore[no-untyped-def]
    try:
        result = callback()
        show_status(f"{action}: {result.get('message', 'ok')}", error=not result.get("success", False))
        refresh(announce=False)
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"{action} {t('action_failed')}: {exc}", error=True)


def safe_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def safe_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def format_percent(value: Any) -> str:
    number = safe_float(value)
    return "n/a" if number is None else f"{number:.1f}%"


def format_bytes(value: Any) -> str:
    number = safe_float(value)
    if number is None:
        return "n/a"

    units = ["B", "KiB", "MiB", "GiB", "TiB", "PiB"]
    unit_index = 0
    while abs(number) >= 1024.0 and unit_index < len(units) - 1:
        number /= 1024.0
        unit_index += 1
    if unit_index == 0:
        return f"{number:.0f} {units[unit_index]}"
    return f"{number:.1f} {units[unit_index]}"


def format_rate(value: Any) -> str:
    return f"{format_bytes(value)}/s"


def response_data(response: dict[str, Any]) -> dict[str, Any]:
    data = response.get("data", {})
    return data if isinstance(data, dict) else {}


def safe_agent_get(path: str) -> dict[str, Any]:
    try:
        return client.rpc_get(path)
    except Exception as exc:  # noqa: BLE001 - diagnostics should show endpoint failures inline.
        return {"success": False, "message": str(exc), "data": {"path": path}}


def json_text(payload: Any) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)


VIEW_MODES = ("simple", "detail", "raw")


def as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def status_tone(value: Any) -> str:
    text = str(value).lower()
    if text in {"running", "healthy", "ok", "ready", "completed", "succeeded", "true"}:
        return "good"
    if text in {"degraded", "pending", "queued", "stopping", "warning"}:
        return "warn"
    if text in {"failed", "unhealthy", "error", "rejected", "expired", "false"}:
        return "bad"
    return "neutral"


def tone_classes(tone: str) -> str:
    return {
        "good": "border-green-200 bg-green-50 text-green-900",
        "warn": "border-amber-200 bg-amber-50 text-amber-900",
        "bad": "border-red-200 bg-red-50 text-red-900",
        "accent": "border-sky-200 bg-sky-50 text-sky-900",
        "neutral": "border-gray-200 bg-white text-gray-900",
    }.get(tone, "border-gray-200 bg-white text-gray-900")


def badge_classes(tone: str) -> str:
    return {
        "good": "bg-green-100 text-green-800 border border-green-300",
        "warn": "bg-amber-100 text-amber-800 border border-amber-300",
        "bad": "bg-red-100 text-red-800 border border-red-300",
        "accent": "bg-sky-100 text-sky-800 border border-sky-300",
        "neutral": "bg-gray-100 text-gray-700 border border-gray-300",
    }.get(tone, "bg-gray-100 text-gray-700 border border-gray-300")


def set_module_mode(module_key: str, mode: str) -> None:
    views = MODULE_VIEWS.get(module_key)
    if not views:
        return
    for name in VIEW_MODES:
        views[name].set_visibility(name == mode)


def create_module_views(module_key: str, *, default: str = "simple", raw_height: str = "h-96") -> None:
    with ui.row().classes("w-full justify-end"):
        ui.toggle(
            {
                "simple": t("view_simple"),
                "detail": t("view_detail"),
                "raw": t("view_raw"),
            },
            value=default,
            on_change=lambda event, key=module_key: set_module_mode(key, str(event.value)),
        ).props("dense unelevated")

    simple = ui.column().classes("w-full gap-3")
    detail = ui.column().classes("w-full gap-3")
    raw = ui.column().classes("w-full")
    with raw:
        raw_code = ui.code("{}", language="json").classes(f"w-full {raw_height}")

    MODULE_VIEWS[module_key] = {
        "simple": simple,
        "detail": detail,
        "raw": raw,
        "raw_code": raw_code,
    }
    set_module_mode(module_key, default)


def update_module_views(
    module_key: str,
    payload: Any,
    render_simple: Callable[[Any], None],
    render_detail: Callable[[Any], None],
) -> None:
    views = MODULE_VIEWS[module_key]
    views["raw_code"].set_content(json_text(payload))

    views["simple"].clear()
    with views["simple"]:
        render_simple(payload)

    views["detail"].clear()
    with views["detail"]:
        render_detail(payload)


def add_stat_card(title: str, value: Any, subtitle: str = "", *, tone: str = "neutral") -> None:
    with ui.element("div").classes(f"rounded border p-3 {tone_classes(tone)}"):
        ui.label(title).classes("text-xs font-semibold text-gray-600")
        ui.label(str(value)).classes("text-xl font-semibold")
        if subtitle:
            ui.label(subtitle).classes("text-xs text-gray-600")


def add_stat_grid(cards: list[tuple[str, Any, str, str]]) -> None:
    with ui.element("div").classes("grid w-full grid-cols-1 gap-3 md:grid-cols-3"):
        for title, value, subtitle, tone in cards:
            add_stat_card(title, value, subtitle, tone=tone)


def add_badge(text: Any, *, tone: str = "neutral") -> None:
    ui.label(str(text)).classes(f"rounded px-2 py-1 text-xs font-semibold {badge_classes(tone)}")


def add_kv_section(title: str, rows: list[tuple[str, Any]]) -> None:
    with ui.element("div").classes("w-full rounded border border-gray-200 bg-white"):
        ui.label(title).classes("border-b border-gray-200 px-3 py-2 text-sm font-semibold")
        with ui.element("div").classes("grid grid-cols-1 gap-0 md:grid-cols-2"):
            if not rows:
                ui.label(t("detail_empty")).classes("px-3 py-2 text-sm text-gray-500")
            for label, value in rows:
                with ui.element("div").classes("border-b border-gray-100 px-3 py-2"):
                    ui.label(label).classes("text-xs font-semibold text-gray-500")
                    ui.label(compact_value(value)).classes("break-words text-sm text-gray-900")


def add_object_cards(
    title: str,
    items: dict[str, Any],
    describe: Callable[[str, dict[str, Any]], list[str]],
) -> None:
    ui.label(title).classes("text-sm font-semibold text-gray-700")
    if not items:
        ui.label(t("no_data")).classes("text-sm text-gray-500")
        return
    with ui.element("div").classes("grid w-full grid-cols-1 gap-3 lg:grid-cols-2"):
        for key, raw_item in items.items():
            item = as_dict(raw_item)
            tone = status_tone(item.get("status", item.get("state", item.get("running", "unknown"))))
            with ui.element("div").classes(f"rounded border p-3 {tone_classes(tone)}"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label(str(key)).classes("text-sm font-semibold")
                    add_badge(item.get("status", item.get("state", t("unknown"))), tone=tone)
                for line in describe(key, item):
                    ui.label(line).classes("text-xs text-gray-700")


def compact_value(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float, str)):
        return str(value)
    return json.dumps(value, ensure_ascii=False, sort_keys=True)


def count_running(processes: dict[str, Any]) -> int:
    return sum(1 for process in processes.values() if as_dict(process).get("running", False))


def nested_response_data(response: dict[str, Any]) -> dict[str, Any]:
    outer = response_data(response)
    inner = outer.get("data")
    return as_dict(inner) if isinstance(inner, dict) else outer


def system_metrics_data(payload: dict[str, Any]) -> dict[str, Any]:
    metrics_response = as_dict(payload.get("metrics"))
    return response_data(metrics_response) if metrics_response else response_data(payload)


def system_health_data(payload: dict[str, Any]) -> dict[str, Any]:
    health_response = as_dict(payload.get("health"))
    return response_data(health_response) if health_response else {}


def render_state_simple(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    agent = as_dict(data.get("agent"))
    profile = as_dict(data.get("active_profile"))
    adapter = as_dict(data.get("adapter"))
    processes = as_dict(data.get("processes"))
    components = as_dict(data.get("components"))
    recorder_rpc = as_dict(as_dict(components.get("recorder")).get("rpc_state"))

    add_stat_grid(
        [
            (
                "Agent",
                agent.get("state", t("unknown")),
                f"v{agent.get('version', 'n/a')} | PID {agent.get('pid', 'n/a')}",
                status_tone(agent.get("state", "unknown")),
            ),
            (
                t("active_profile"),
                profile.get("profile_id", t("unknown")),
                profile.get("robot_model", ""),
                "accent" if profile else "warn",
            ),
            (
                t("processes"),
                f"{count_running(processes)}/{len(processes)}",
                t("running"),
                "good" if count_running(processes) else "neutral",
            ),
            (
                t("adapter"),
                "loaded" if adapter.get("loaded") else "not loaded",
                adapter.get("profile_id", ""),
                "good" if adapter.get("loaded") else "neutral",
            ),
            (
                t("recorder"),
                "reachable" if recorder_rpc.get("reachable") else "unreachable",
                recorder_rpc.get("url", recorder_rpc.get("last_error", "")),
                "good" if recorder_rpc.get("reachable") else "warn",
            ),
            (
                t("executions"),
                as_dict(data.get("action_executions")).get("count", 0),
                json.dumps(
                    as_dict(as_dict(data.get("action_executions")).get("counts_by_status")),
                    ensure_ascii=False,
                    sort_keys=True,
                ),
                "neutral",
            ),
        ]
    )
    if data.get("last_error"):
        add_stat_card(t("agent_error"), data.get("last_error"), tone="bad")


def describe_process(_key: str, item: dict[str, Any]) -> list[str]:
    lines = []
    pid = safe_int(item.get("pid"))
    if pid is not None and pid > 0:
        lines.append(f"PID {pid}")
    if item.get("executable"):
        lines.append(str(item.get("executable")))
    health = as_dict(item.get("health"))
    if health:
        lines.append(f"{t('health')}: {health.get('status', health.get('message', t('unknown')))}")
    if item.get("last_error"):
        lines.append(f"{t('agent_error')}: {item.get('last_error')}")
    return lines or [t("no_data")]


def render_state_detail(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    agent = as_dict(data.get("agent"))
    profile = as_dict(data.get("active_profile"))
    adapter = as_dict(data.get("adapter"))
    processes = as_dict(data.get("processes"))
    components = as_dict(data.get("components"))

    add_kv_section(
        "Agent",
        [
            ("state", agent.get("state")),
            ("version", agent.get("version")),
            ("pid", agent.get("pid")),
            ("started_at", agent.get("started_at")),
            ("uptime_sec", agent.get("uptime_sec")),
            ("profile_root", agent.get("profile_root")),
            ("state_dir", agent.get("state_dir")),
            ("last_error", data.get("last_error")),
        ],
    )
    add_kv_section(
        t("active_profile"),
        [
            ("profile_id", profile.get("profile_id")),
            ("adapter_id", profile.get("adapter_id")),
            ("robot_model", profile.get("robot_model")),
            ("auto_start", profile.get("auto_start")),
            ("root_dir", profile.get("root_dir")),
            ("recorder_yaml", profile.get("recorder_yaml")),
            ("transfer_yaml", profile.get("transfer_yaml")),
        ],
    )
    add_kv_section(
        t("adapter"),
        [
            ("loaded", adapter.get("loaded")),
            ("profile_id", adapter.get("profile_id")),
            ("library", adapter.get("library")),
            ("runtime", adapter.get("runtime")),
            ("descriptor", adapter.get("descriptor")),
        ],
    )
    add_object_cards(t("processes"), processes, describe_process)
    add_kv_section("Components", [(key, value) for key, value in components.items()])


def render_profiles_simple(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    profiles = as_list(data.get("profiles"))
    active = as_dict(data.get("active_profile"))
    auto_start_count = sum(1 for profile in profiles if as_dict(profile).get("auto_start", False))
    add_stat_grid(
        [
            (t("profiles"), len(profiles), "", "accent"),
            (t("active_profile"), active.get("profile_id", t("unknown")), active.get("robot_model", ""), "good" if active else "warn"),
            ("auto_start", auto_start_count, "", "neutral"),
        ]
    )


def render_profiles_detail(payload: dict[str, Any]) -> None:
    profiles = as_list(response_data(payload).get("profiles"))
    if not profiles:
        ui.label(t("no_data")).classes("text-sm text-gray-500")
        return
    with ui.element("div").classes("grid w-full grid-cols-1 gap-3 lg:grid-cols-2"):
        for raw_profile in profiles:
            profile = as_dict(raw_profile)
            with ui.element("div").classes("rounded border border-gray-200 bg-white p-3"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label(profile.get("profile_id", t("unknown"))).classes("text-sm font-semibold")
                    add_badge("auto" if profile.get("auto_start") else "manual", tone="accent" if profile.get("auto_start") else "neutral")
                ui.label(profile.get("robot_model", "")).classes("text-xs text-gray-600")
                add_kv_section(
                    t("details"),
                    [
                        ("adapter_id", profile.get("adapter_id")),
                        ("abi_version", profile.get("abi_version")),
                        ("capabilities", profile.get("capabilities")),
                        ("root_dir", profile.get("root_dir")),
                        ("recorder_yaml", profile.get("recorder_yaml")),
                        ("transfer_yaml", profile.get("transfer_yaml")),
                    ],
                )


def render_report_simple(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    profiles = as_list(data.get("profiles"))
    processes = as_dict(data.get("processes"))
    actions = as_dict(data.get("actions"))
    executions = as_dict(data.get("action_executions"))
    add_stat_grid(
        [
            (t("profiles"), len(profiles), as_dict(data.get("active_profile")).get("profile_id", ""), "accent"),
            (t("processes"), f"{count_running(processes)}/{len(processes)}", t("running"), "neutral"),
            (t("loaded"), actions.get("loaded_count", 0), f"{t('invalid')} {actions.get('invalid_count', 0)}", "good" if actions.get("invalid_count", 0) == 0 else "warn"),
            (t("executions"), executions.get("count", 0), compact_value(executions.get("counts_by_status", {})), "neutral"),
        ]
    )


def render_report_detail(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    render_profiles_detail({"data": {"profiles": data.get("profiles", [])}})
    add_object_cards(t("processes"), as_dict(data.get("processes")), describe_process)
    add_kv_section(t("adapter"), [(key, value) for key, value in as_dict(data.get("adapter")).items()])
    add_kv_section("Actions", [(key, value) for key, value in as_dict(data.get("actions")).items() if key != "actions"])


def render_actions_simple(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    diagnostics = as_list(data.get("diagnostics"))
    add_stat_grid(
        [
            (t("loaded"), data.get("loaded_count", len(as_list(data.get("actions")))), "", "good"),
            (t("invalid"), data.get("invalid_count", 0), "", "good" if data.get("invalid_count", 0) == 0 else "bad"),
            ("diagnostics", len(diagnostics), "", "warn" if diagnostics else "neutral"),
        ]
    )


def render_actions_detail(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    actions = [as_dict(item) for item in as_list(data.get("actions"))]
    diagnostics = [as_dict(item) for item in as_list(data.get("diagnostics"))]
    if actions:
        with ui.element("div").classes("grid w-full grid-cols-1 gap-3 lg:grid-cols-2"):
            for action in actions:
                with ui.element("div").classes("rounded border border-gray-200 bg-white p-3"):
                    ui.label(action.get("title", action.get("id", t("unknown")))).classes("text-sm font-semibold")
                    ui.label(action.get("id", "")).classes("text-xs text-gray-500")
                    ui.label(action.get("description", "")).classes("text-sm text-gray-700")
                    add_kv_section(
                        t("details"),
                        [
                            ("timeout_sec", action.get("timeout_sec")),
                            ("run_as", action.get("run_as")),
                            ("requires_approval", action.get("requires_approval")),
                            ("allowed_roles", action.get("allowed_roles")),
                            ("args_schema", action.get("args_schema")),
                        ],
                    )
    else:
        ui.label(t("no_data")).classes("text-sm text-gray-500")
    if diagnostics:
        add_kv_section("Diagnostics", [(item.get("severity", "diagnostic"), item) for item in diagnostics])


def render_executions_simple(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    counts = as_dict(data.get("counts_by_status"))
    add_stat_grid(
        [
            (t("executions"), data.get("count", 0), "", "accent"),
            (t("completed"), counts.get("completed", 0), "", "good"),
            (t("running"), counts.get("running", 0), "", "warn" if counts.get("running", 0) else "neutral"),
            (t("failed"), counts.get("failed", 0), "", "bad" if counts.get("failed", 0) else "neutral"),
            (t("rejected"), counts.get("rejected", 0), "", "bad" if counts.get("rejected", 0) else "neutral"),
            (t("pending"), counts.get("queued", 0), "", "warn" if counts.get("queued", 0) else "neutral"),
        ]
    )


def render_executions_detail(payload: dict[str, Any]) -> None:
    records = [as_dict(item) for item in as_list(response_data(payload).get("records"))]
    if not records:
        ui.label(t("no_data")).classes("text-sm text-gray-500")
        return
    with ui.element("div").classes("grid w-full grid-cols-1 gap-3"):
        for record in records:
            tone = status_tone(record.get("status"))
            with ui.element("div").classes(f"rounded border p-3 {tone_classes(tone)}"):
                with ui.row().classes("w-full items-center justify-between"):
                    ui.label(record.get("request_id", t("unknown"))).classes("text-sm font-semibold")
                    add_badge(record.get("status", t("unknown")), tone=tone)
                ui.label(f"{record.get('action_id', '')} | exit={record.get('exit_code', 'n/a')}").classes(
                    "text-xs text-gray-700"
                )
                if record.get("error_summary"):
                    ui.label(record.get("error_summary")).classes("text-xs text-red-700")
                add_kv_section(
                    t("details"),
                    [
                        ("queued_at", record.get("queued_at")),
                        ("started_at", record.get("started_at")),
                        ("finished_at", record.get("finished_at")),
                        ("expires_at", record.get("expires_at")),
                        ("duplicate_count", record.get("duplicate_count")),
                        ("stdout_truncated", record.get("stdout_truncated")),
                        ("stderr_truncated", record.get("stderr_truncated")),
                    ],
                )


def render_recorder_simple(payload: dict[str, Any]) -> None:
    proxy = response_data(payload)
    status = nested_response_data(payload)
    metadata = as_dict(status.get("metadata"))
    time_gap = as_dict(status.get("keystone_time_gap"))
    add_stat_grid(
        [
            ("Proxy", "reachable" if proxy.get("reachable") else "unreachable", proxy.get("url", ""), "good" if proxy.get("reachable") else "warn"),
            (t("recorder"), status.get("state", t("unknown")), payload.get("message", ""), status_tone(status.get("state", payload.get("success")))),
            ("Sidecar", f"{metadata.get('sidecar_generation_mode', 'n/a')}/{metadata.get('sidecar_generated', 'n/a')}", metadata.get("sidecar_path", ""), "neutral"),
            ("Incident", metadata.get("incident_bundle_created", "n/a"), metadata.get("incident_bundle_path", metadata.get("incident_bundle_error", "")), "good" if metadata.get("incident_bundle_created") else "neutral"),
            ("Time gap", time_gap.get("status", t("unknown")), f"rtt={time_gap.get('round_trip_ms', 'n/a')} ms", status_tone(time_gap.get("status"))),
        ]
    )


def render_recorder_detail(payload: dict[str, Any]) -> None:
    proxy = response_data(payload)
    status = nested_response_data(payload)
    add_kv_section(
        "Proxy",
        [
            ("configured", proxy.get("configured")),
            ("queryable", proxy.get("queryable")),
            ("reachable", proxy.get("reachable")),
            ("url", proxy.get("url")),
            ("rpc_path", proxy.get("rpc_path")),
            ("status_code", proxy.get("status_code")),
            ("last_error", proxy.get("last_error")),
        ],
    )
    add_kv_section("Recorder", [(key, value) for key, value in status.items() if key not in {"metadata", "keystone_time_gap"}])
    add_kv_section("Metadata", [(key, value) for key, value in as_dict(status.get("metadata")).items()])
    add_kv_section("Keystone Time Gap", [(key, value) for key, value in as_dict(status.get("keystone_time_gap")).items()])


def render_action_sync_simple(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    enabled = bool(data.get("enabled", False))
    running = bool(data.get("running", False))
    websocket_enabled = bool(data.get("websocket_enabled", False))
    websocket_connected = bool(data.get("websocket_connected", False))
    failures = safe_int(data.get("consecutive_failures")) or 0
    add_stat_grid(
        [
            (
                t("action_sync"),
                "running" if running else ("enabled" if enabled else "disabled"),
                data.get("robot_id", ""),
                "good" if running else ("warn" if enabled else "neutral"),
            ),
            (
                "Catalog",
                data.get("catalog_sync_count", 0),
                f"last {data.get('last_catalog_sync_at', 'n/a')}",
                "neutral",
            ),
            (
                "Poll",
                data.get("poll_count", 0),
                f"last {data.get('last_poll_at', 'n/a')}",
                "neutral",
            ),
            (
                "Notifications",
                data.get("notification_count", 0),
                f"last {data.get('last_notification_at', 'n/a')}",
                "accent" if data.get("notification_count", 0) else "neutral",
            ),
            (
                "WebSocket",
                "connected" if websocket_connected else ("enabled" if websocket_enabled else "disabled"),
                data.get("websocket_url", ""),
                "good" if websocket_connected else ("warn" if websocket_enabled else "neutral"),
            ),
            (
                "Failures",
                failures,
                data.get("last_error", ""),
                "bad" if failures else "good",
            ),
        ]
    )


def render_action_sync_detail(payload: dict[str, Any]) -> None:
    data = response_data(payload)
    add_kv_section(
        t("action_sync"),
        [
            ("enabled", data.get("enabled")),
            ("running", data.get("running")),
            ("base_url", data.get("base_url")),
            ("robot_id", data.get("robot_id")),
            ("pending_limit", data.get("pending_limit")),
            ("consecutive_failures", data.get("consecutive_failures")),
            ("current_backoff_ms", data.get("current_backoff_ms")),
            ("last_error", data.get("last_error")),
        ],
    )
    add_kv_section(
        "WebSocket",
        [
            ("websocket_enabled", data.get("websocket_enabled")),
            ("websocket_connected", data.get("websocket_connected")),
            ("websocket_url", data.get("websocket_url")),
            ("last_notification_at", data.get("last_notification_at")),
            ("notification_count", data.get("notification_count")),
        ],
    )
    add_kv_section(
        "Counters",
        [
            ("catalog_sync_count", data.get("catalog_sync_count")),
            ("poll_count", data.get("poll_count")),
            ("action_requests_executed", data.get("action_requests_executed")),
            ("status_updates_sent", data.get("status_updates_sent")),
            ("last_catalog_sync_at", data.get("last_catalog_sync_at")),
            ("last_poll_at", data.get("last_poll_at")),
        ],
    )


def render_system_metrics_simple(payload: dict[str, Any]) -> None:
    metrics = system_metrics_data(payload)
    health = system_health_data(payload)
    cpu = as_dict(metrics.get("cpu"))
    memory = as_dict(metrics.get("memory"))
    disk = select_primary_disk(metrics.get("disk", []))
    process_items = as_dict(metrics.get("processes"))
    cards = [
        (
            t("system"),
            health.get("state", t("unknown")),
            f"sampler={health.get('sampler_running', 'n/a')}",
            status_tone(health.get("state", "unknown")),
        )
    ]
    cards.extend(system_metric_cards(metrics, cpu, memory, disk, process_items))
    add_stat_grid(cards)


def render_system_metrics_detail(payload: dict[str, Any]) -> None:
    metrics = system_metrics_data(payload)
    health = system_health_data(payload)
    add_kv_section("Health", [(key, value) for key, value in health.items()])
    add_kv_section("CPU", [(key, value) for key, value in as_dict(metrics.get("cpu")).items()])
    add_kv_section(t("memory"), [(key, value) for key, value in as_dict(metrics.get("memory")).items()])
    disks = [as_dict(item) for item in as_list(metrics.get("disk"))]
    if disks:
        add_kv_section(t("disk"), [(disk.get("id", f"disk-{index}"), disk) for index, disk in enumerate(disks)])
    add_kv_section("Network", [(key, value) for key, value in as_dict(metrics.get("network")).items()])
    add_object_cards(t("processes"), as_dict(metrics.get("processes")), describe_system_process)
    add_kv_section("Sample Cadence", [(key, value) for key, value in as_dict(metrics.get("sample_cadence_ms")).items()])


def render_system_alerts_simple(payload: dict[str, Any]) -> None:
    alerts = response_data(payload)
    delivery = as_dict(alerts.get("delivery"))
    add_stat_grid(
        [
            ("firing", alerts.get("firing_count", 0), "", "bad" if alerts.get("firing_count", 0) else "good"),
            ("pending", alerts.get("pending_count", 0), "", "warn" if alerts.get("pending_count", 0) else "neutral"),
            ("resolved", alerts.get("resolved_count", 0), "", "neutral"),
            ("events", len(as_list(alerts.get("events"))), "", "accent"),
            ("delivery", delivery.get("queued_count", 0), delivery.get("last_error", ""), "warn" if delivery.get("queued_count", 0) else "good"),
        ]
    )


def render_system_alerts_detail(payload: dict[str, Any]) -> None:
    alerts = response_data(payload)
    add_kv_section("Delivery", [(key, value) for key, value in as_dict(alerts.get("delivery")).items()])
    add_object_cards("Rules", as_dict(alerts.get("rules")), describe_alert_rule)
    events = [as_dict(item) for item in as_list(alerts.get("events"))]
    if events:
        add_kv_section("Recent Events", [(event.get("event_id", f"event-{index}"), event) for index, event in enumerate(events[-10:])])


def describe_system_process(_key: str, process: dict[str, Any]) -> list[str]:
    lines = []
    if process.get("binary"):
        lines.append(str(process.get("binary")))
    pid = safe_int(process.get("pid"))
    if pid is not None and pid > 0:
        lines.append(f"PID {pid}")
    resources = as_dict(process.get("resources"))
    if resources:
        lines.append(f"CPU {format_percent(resources.get('cpu_percent'))} | RSS {format_bytes(resources.get('rss_bytes'))}")
    health = as_dict(process.get("health"))
    if health:
        lines.append(f"{t('health')}: {health.get('state', health.get('error', health.get('reachable', t('unknown'))))}")
    if process.get("message"):
        lines.append(str(process.get("message")))
    return lines or [t("no_data")]


def describe_alert_rule(_key: str, rule: dict[str, Any]) -> list[str]:
    lines = [f"severity={rule.get('severity', t('unknown'))}"]
    if rule.get("last_observed_at"):
        lines.append(f"last={rule.get('last_observed_at')}")
    if rule.get("values"):
        lines.append(f"values={compact_value(rule.get('values'))}")
    return lines


def system_metric_cards(
    metrics: dict[str, Any],
    cpu: dict[str, Any],
    memory: dict[str, Any],
    disk: dict[str, Any],
    process_items: dict[str, Any],
) -> list[tuple[str, Any, str, str]]:
    rx_rate = network_rate(metrics, "rx_bytes_per_sec")
    tx_rate = network_rate(metrics, "tx_bytes_per_sec")
    running_count = sum(1 for process in process_items.values() if as_dict(process).get("status") == "healthy")
    return [
        ("CPU", format_percent(cpu.get("usage_percent")), f"{t('load')} {safe_float(cpu.get('load1')) or 0.0:.2f}", "accent"),
        (
            t("memory"),
            format_percent(memory.get("used_percent")),
            f"{format_bytes(memory.get('used_bytes'))} / {format_bytes(memory.get('total_bytes'))}",
            "accent",
        ),
        (
            t("disk"),
            format_percent(disk.get("used_percent")) if disk else "n/a",
            f"{disk.get('id', 'disk')} | {format_bytes(disk.get('available_bytes'))} {t('available')}" if disk else "",
            "good" if disk.get("available", False) else "warn",
        ),
        ("Network", format_rate(rx_rate), f"TX {format_rate(tx_rate)}", "neutral"),
        (t("processes"), f"{running_count}/{len(process_items)}", "healthy", "good" if running_count == len(process_items) and process_items else "warn"),
    ]


def update_action_panels(actions: dict[str, Any], executions: dict[str, Any]) -> None:
    actions_data = response_data(actions)
    action_items = actions_data.get("actions", [])
    if not isinstance(action_items, list):
        action_items = []

    diagnostics = actions_data.get("diagnostics", [])
    if not isinstance(diagnostics, list):
        diagnostics = []

    options = {
        item.get("id", ""): f"{item.get('id', '')} - {item.get('title', '')}".strip(" -")
        for item in action_items
        if isinstance(item, dict) and item.get("id")
    }
    action_select.options = options
    if action_select.value not in options:
        action_select.value = next(iter(options), None)
    action_select.update()

    update_module_views("actions", actions, render_actions_simple, render_actions_detail)
    update_module_views(
        "action_executions",
        executions,
        render_executions_simple,
        render_executions_detail,
    )


def update_recorder_status_panel(recorder_status: dict[str, Any]) -> None:
    update_module_views(
        "recorder_status",
        recorder_status,
        render_recorder_simple,
        render_recorder_detail,
    )


def process_status(process: dict[str, Any]) -> tuple[str, str]:
    if process.get("running", False):
        return t("running"), "bg-green-100 text-green-800 border border-green-300"

    state = str(process.get("state", "")).lower()
    if state == "stopped" or not process:
        return t("stopped"), "bg-gray-100 text-gray-700 border border-gray-300"

    return state or t("unknown"), "bg-amber-100 text-amber-800 border border-amber-300"


def system_process_summary(process: dict[str, Any]) -> str:
    if not process:
        return ""

    parts = [f"{t('system_prefix')} {process.get('status', 'unknown')}"]
    pid = safe_int(process.get("pid"))
    if pid is not None and pid > 0:
        parts.append(f"PID {pid}")

    resources = process.get("resources", {})
    if isinstance(resources, dict):
        if "cpu_percent" in resources:
            parts.append(f"CPU {format_percent(resources.get('cpu_percent'))}")
        if "rss_bytes" in resources:
            parts.append(f"RSS {format_bytes(resources.get('rss_bytes'))}")
        io = resources.get("io", {})
        if isinstance(io, dict) and io.get("available", False):
            parts.append(
                "I/O "
                + f"r {format_bytes(io.get('read_bytes'))} "
                + f"w {format_bytes(io.get('write_bytes'))}"
            )

    health = process.get("health", {})
    if isinstance(health, dict):
        if health.get("reachable", False):
            parts.append(t("rpc_reachable"))
        elif health.get("error"):
            parts.append(f"{t('rpc_error')}: {health.get('error')}")

    message = process.get("message")
    if message:
        parts.append(str(message))

    return " | ".join(parts)


def update_process_rows(process_items: dict[str, Any], system_process_items: dict[str, Any]) -> None:
    for process_id, _label in PROCESS_ROWS:
        process = process_items.get(process_id, {})
        status_text, status_classes = process_status(process)

        if status_label_widget := process_status_labels.get(process_id):
            status_label_widget.text = status_text
            status_label_widget.classes(
                replace=f"min-w-20 rounded px-3 py-1 text-center text-sm font-semibold {status_classes}"
            )

        details = []
        pid = safe_int(process.get("pid"))
        if pid is not None and pid > 0:
            details.append(f"{t('agent_pid')} {pid}")

        health = process.get("health", {})
        if isinstance(health, dict):
            health_message = health.get("message")
            if health_message:
                details.append(str(health_message))

        last_error = process.get("last_error")
        if last_error:
            details.append(f"{t('agent_error')}: {last_error}")

        system_detail = system_process_summary(system_process_items.get(process_id, {}))
        if system_detail:
            details.append(system_detail)

        if detail_label := process_detail_labels.get(process_id):
            detail_label.text = " | ".join(details) if details else t("not_started")


def select_primary_disk(disks: Any) -> dict[str, Any]:
    if not isinstance(disks, list) or not disks:
        return {}
    for disk in disks:
        if isinstance(disk, dict) and disk.get("available", False):
            return disk
    first = disks[0]
    return first if isinstance(first, dict) else {}


def network_rate(metrics: dict[str, Any], key: str) -> float:
    network = metrics.get("network", {})
    if not isinstance(network, dict):
        return 0.0
    interfaces = network.get("interfaces", [])
    if not isinstance(interfaces, list):
        return 0.0
    total = 0.0
    for item in interfaces:
        if isinstance(item, dict):
            total += safe_float(item.get(key)) or 0.0
    return total


def chart_options(title: str, labels: list[str], y_suffix: str, series: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "animation": False,
        "title": {"text": title, "left": 0, "textStyle": {"fontSize": 13}},
        "tooltip": {"trigger": "axis"},
        "legend": {"top": 0, "right": 0},
        "grid": {"left": 48, "right": 16, "top": 40, "bottom": 32},
        "xAxis": {"type": "category", "boundaryGap": False, "data": labels},
        "yAxis": {"type": "value", "axisLabel": {"formatter": f"{{value}}{y_suffix}"}},
        "series": series,
    }


def update_system_charts() -> None:
    labels = [point["label"] for point in SYSTEM_HISTORY]
    cpu = [point["cpu"] for point in SYSTEM_HISTORY]
    memory = [point["memory"] for point in SYSTEM_HISTORY]
    rx = [point["rx"] for point in SYSTEM_HISTORY]
    tx = [point["tx"] for point in SYSTEM_HISTORY]

    system_resource_chart.options.clear()
    system_resource_chart.options.update(
        chart_options(
            t("chart_cpu_memory"),
            labels,
            "%",
            [
                {"name": "CPU", "type": "line", "showSymbol": False, "data": cpu},
                {"name": "Memory", "type": "line", "showSymbol": False, "data": memory},
            ],
        )
    )
    system_resource_chart.update()

    system_network_chart.options.clear()
    system_network_chart.options.update(
        chart_options(
            t("chart_network_rate"),
            labels,
            " B/s",
            [
                {"name": "RX", "type": "line", "showSymbol": False, "data": rx},
                {"name": "TX", "type": "line", "showSymbol": False, "data": tx},
            ],
        )
    )
    system_network_chart.update()


def remember_system_metrics(metrics: dict[str, Any]) -> None:
    now = time.time()
    cutoff = now - max(args.history_seconds, 1)
    while SYSTEM_HISTORY and SYSTEM_HISTORY[0]["timestamp"] < cutoff:
        SYSTEM_HISTORY.popleft()

    cpu = metrics.get("cpu", {})
    memory = metrics.get("memory", {})
    if not isinstance(cpu, dict) or not isinstance(memory, dict):
        return

    SYSTEM_HISTORY.append(
        {
            "timestamp": now,
            "label": time.strftime("%H:%M:%S", time.localtime(now)),
            "cpu": round(safe_float(cpu.get("usage_percent")) or 0.0, 2),
            "memory": round(safe_float(memory.get("used_percent")) or 0.0, 2),
            "rx": round(network_rate(metrics, "rx_bytes_per_sec"), 2),
            "tx": round(network_rate(metrics, "tx_bytes_per_sec"), 2),
        }
    )


def update_system_overview(
    health_response: dict[str, Any], metrics_response: dict[str, Any], alerts_response: dict[str, Any]
) -> None:
    health = response_data(health_response)
    metrics = response_data(metrics_response)
    alerts = response_data(alerts_response)

    state = str(health.get("state", "unknown"))
    success = bool(health_response.get("success", False))
    if success and state == "running":
        status_classes = "bg-green-100 text-green-800 border border-green-300"
    elif success and state == "degraded":
        status_classes = "bg-amber-100 text-amber-800 border border-amber-300"
    else:
        status_classes = "bg-red-100 text-red-800 border border-red-300"

    system_status_label.text = state
    system_status_label.classes(
        replace=f"min-w-24 rounded px-3 py-1 text-center text-sm font-semibold {status_classes}"
    )
    system_error_label.text = str(health.get("last_error") or metrics_response.get("message") or "")

    cpu = metrics.get("cpu", {})
    memory = metrics.get("memory", {})
    disk = select_primary_disk(metrics.get("disk", []))
    process_items = as_dict(metrics.get("processes"))

    system_overview_container.clear()
    with system_overview_container:
        cards = [
            (
                t("system"),
                state,
                f"sampler={health.get('sampler_running', 'n/a')}",
                status_tone(state if success else "failed"),
            )
        ]
        cards.extend(system_metric_cards(metrics, as_dict(cpu), as_dict(memory), disk, process_items))
        cards.append(
            (
                "Alerts",
                f"{alerts.get('firing_count', 0)} firing",
                f"{alerts.get('pending_count', 0)} pending | {alerts.get('resolved_count', 0)} resolved",
                "bad" if alerts.get("firing_count", 0) else "good",
            )
        )
        add_stat_grid(cards)
        add_object_cards(t("processes"), process_items, describe_system_process)

    remember_system_metrics(metrics)
    update_system_charts()


def refresh_system() -> dict[str, Any]:
    try:
        health = system_client.get_health()
        metrics = system_client.get_metrics()
        alerts = system_client.get_alerts()
    except Exception as exc:  # noqa: BLE001 - validation panel should keep agent controls alive.
        error_payload = {"success": False, "message": str(exc)}
        system_status_label.text = t("unknown")
        system_status_label.classes(
            replace="min-w-24 rounded px-3 py-1 text-center text-sm font-semibold "
            + "bg-red-100 text-red-800 border border-red-300"
        )
        system_error_label.text = f"{t('system_refresh_failed')}: {exc}"
        system_overview_container.clear()
        with system_overview_container:
            add_stat_card(t("system"), t("unknown"), str(exc), tone="bad")
        update_module_views("system_metrics", error_payload, render_system_metrics_simple, render_system_metrics_detail)
        update_module_views("system_alerts", error_payload, render_system_alerts_simple, render_system_alerts_detail)
        return {}

    system_payload = {
        "success": bool(health.get("success", False)) and bool(metrics.get("success", False)),
        "message": metrics.get("message", health.get("message", "")),
        "health": health,
        "metrics": metrics,
    }
    update_module_views(
        "system_metrics",
        system_payload,
        render_system_metrics_simple,
        render_system_metrics_detail,
    )
    update_module_views("system_alerts", alerts, render_system_alerts_simple, render_system_alerts_detail)
    update_system_overview(health, metrics, alerts)
    return metrics


def refresh(*, announce: bool = True, language: str | None = None) -> None:
    if language is not None:
        set_current_language(language)

    system_metrics = refresh_system()
    system_process_items = response_data(system_metrics).get("processes", {})
    if not isinstance(system_process_items, dict):
        system_process_items = {}

    try:
        profiles = client.rpc_get("profiles")
        state = client.rpc_get("state")
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        error_payload = {
            "success": False,
            "message": str(exc),
            "agent_url": args.agent_url,
        }
        update_module_views("profiles", error_payload, render_profiles_simple, render_profiles_detail)
        update_module_views("state", error_payload, render_state_simple, render_state_detail)
        show_status(f"{t('refresh_failed')}: {exc}", error=True)
        return

    report = safe_agent_get("report")
    actions = safe_agent_get("actions")
    action_executions = safe_agent_get("actions/executions")
    action_sync = safe_agent_get("action-sync/status")
    recorder_status = safe_agent_get("recorder/status")

    profile_items = profiles.get("data", {}).get("profiles", [])
    options = {
        item["profile_id"]: f"{item['profile_id']} ({item.get('robot_model', 'unknown')})"
        for item in profile_items
    }
    profile_select.options = options
    if not profile_select.value and options:
        profile_select.value = next(iter(options))
    profile_select.update()

    update_module_views("profiles", profiles, render_profiles_simple, render_profiles_detail)
    update_module_views("state", state, render_state_simple, render_state_detail)
    update_module_views("report", report, render_report_simple, render_report_detail)
    update_action_panels(actions, action_executions)
    update_module_views("action_sync", action_sync, render_action_sync_simple, render_action_sync_detail)
    update_recorder_status_panel(recorder_status)

    process_items = state.get("data", {}).get("processes", {})
    update_process_rows(process_items, system_process_items)

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
        show_status(t("refreshed"))


def select_profile(language: str | None = None) -> None:
    if language is not None:
        set_current_language(language)

    if not profile_select.value:
        show_status(t("select_profile_failed_no_profile"), error=True)
        return
    call_agent(
        t("select_profile_action"),
        lambda: client.rpc_post("profile/select", {"profile_id": profile_select.value}),
    )


def start_process(process_id: str, language: str | None = None) -> None:
    if language is not None:
        set_current_language(language)

    call_agent(
        f"{t('start_action')} {process_id}",
        lambda: client.rpc_post("process/start", {"process_id": process_id}),
    )


def stop_process(process_id: str, *, force: bool = False, language: str | None = None) -> None:
    if language is not None:
        set_current_language(language)

    endpoint = "process/force_stop" if force else "process/stop"
    action = f"{t('force_stop_action') if force else t('stop_action')} {process_id}"
    call_agent(action, lambda: client.rpc_post(endpoint, {"process_id": process_id}))


def read_log(language: str | None = None) -> None:
    if language is not None:
        set_current_language(language)

    if not log_process_select.value:
        show_status(t("read_log_failed_no_process"), error=True)
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
        log_view.set_content(result.get("data", {}).get("content", ""))
        show_status(f"{t('read_log_action')}: {result.get('message', 'ok')}", error=not result.get("success", False))
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"{t('read_log_action')} {t('action_failed')}: {exc}", error=True)


def execute_action(language: str | None = None) -> None:
    if language is not None:
        set_current_language(language)

    if not action_select.value:
        show_status(t("action_execute_failed_no_action"), error=True)
        return

    request_id = str(action_request_input.value or "").strip()
    if not request_id:
        show_status(t("action_execute_failed_no_request"), error=True)
        return

    try:
        args_payload = json.loads(action_args_textarea.value or "{}")
    except json.JSONDecodeError as exc:
        show_status(f"{t('action_execute_failed_bad_args')}: {exc}", error=True)
        return

    if not isinstance(args_payload, dict):
        show_status(t("action_execute_failed_bad_args"), error=True)
        return

    payload: dict[str, Any] = {
        "request_id": request_id,
        "action_id": action_select.value,
        "args": args_payload,
    }
    expires_at = str(action_expires_input.value or "").strip()
    if expires_at:
        payload["expires_at"] = expires_at

    try:
        result = client.rpc_post("actions/execute", payload)
        action_result_view.set_content(json_text(result))
        show_status(
            f"{t('action_execute')}: {result.get('message', 'ok')}",
            error=not result.get("success", False),
        )
        refresh(announce=False, language=language)
    except Exception as exc:  # noqa: BLE001 - validation panel should surface raw errors.
        show_status(f"{t('action_execute')} {t('action_failed')}: {exc}", error=True)


@ui.page("/")
def main_page(request: Request) -> None:
    global action_args_textarea, action_expires_input, action_request_input, action_select
    global action_result_view
    global log_process_select, log_stream_select, log_tail_input, log_view
    global profile_select, status_label
    global system_error_label, system_network_chart, system_overview_container
    global system_resource_chart, system_status_label

    language = normalize_language(request.query_params.get("lang", args.language))
    set_current_language(language)
    ui.page_title(t("page_title"))
    render_language_switch(language)

    with ui.column().classes("w-full max-w-6xl mx-auto p-4 gap-4"):
        ui.label(t("header_title")).classes("text-2xl font-bold")
        ui.label(f"{t('agent_url')}: {args.agent_url} | {t('system_url')}: {args.system_url}").classes(
            "text-gray-600"
        )
        status_label = ui.label(t("ready"))

        with ui.row().classes("items-end gap-2"):
            profile_select = ui.select(label=t("robot_profile"), options={}).classes("w-96")
            ui.button(t("refresh"), on_click=lambda lang=language: refresh(language=lang))
            ui.button(t("select_profile"), on_click=lambda lang=language: select_profile(lang))

        with ui.card().classes("w-full"):
            with ui.row().classes("w-full items-center gap-3"):
                ui.label(t("system")).classes("text-lg font-semibold")
                system_status_label = ui.label(t("unknown")).classes(
                    "min-w-24 rounded px-3 py-1 text-center text-sm font-semibold "
                    + "bg-gray-100 text-gray-700 border border-gray-300"
                )
                system_error_label = ui.label("").classes("min-w-0 flex-1 truncate text-sm text-red-600")
            system_overview_container = ui.column().classes("w-full gap-3")

        with ui.card().classes("w-full"):
            ui.label(t("process_controls")).classes("text-lg font-semibold")
            for pid, label_key in PROCESS_ROWS:
                with ui.row().classes("w-full items-center gap-3 py-2 border-b border-gray-200 last:border-b-0"):
                    ui.label(t(label_key)).classes("w-28 text-lg font-semibold")
                    process_status_labels[pid] = ui.label(t("unknown")).classes(
                        "min-w-20 rounded px-3 py-1 text-center text-sm font-semibold bg-gray-100 text-gray-700"
                    )
                    process_detail_labels[pid] = ui.label(t("not_started")).classes(
                        "min-w-0 flex-1 truncate text-sm text-gray-600"
                    )
                    ui.button(t("start"), on_click=lambda process_id=pid, lang=language: start_process(process_id, lang))
                    ui.button(t("stop"), on_click=lambda process_id=pid, lang=language: stop_process(process_id, language=lang))
                    ui.button(
                        t("force_stop"),
                        color="negative",
                        on_click=lambda process_id=pid, lang=language: stop_process(
                            process_id, force=True, language=lang
                        ),
                    )

        with ui.tabs().classes("w-full") as tabs:
            state_tab = ui.tab(t("tab_state"))
            profiles_tab = ui.tab(t("tab_profiles"))
            report_tab = ui.tab(t("tab_agent_report"))
            actions_tab = ui.tab(t("tab_actions"))
            action_executions_tab = ui.tab(t("tab_action_executions"))
            action_sync_tab = ui.tab(t("tab_action_sync"))
            recorder_status_tab = ui.tab(t("tab_recorder_status"))
            system_metrics_tab = ui.tab(t("tab_system_metrics"))
            system_alerts_tab = ui.tab(t("tab_system_alerts"))
            logs_tab = ui.tab(t("tab_logs"))

        with ui.tab_panels(tabs, value=state_tab).classes("w-full"):
            with ui.tab_panel(state_tab):
                create_module_views("state")
            with ui.tab_panel(profiles_tab):
                create_module_views("profiles")
            with ui.tab_panel(report_tab):
                create_module_views("report")
            with ui.tab_panel(actions_tab):
                with ui.row().classes("w-full items-end gap-3 flex-wrap"):
                    action_select = ui.select(label=t("action_id"), options={}).classes("w-96")
                    action_request_input = ui.input(label=t("action_request_id")).classes("w-72")
                    action_expires_input = ui.input(label="expires_at").classes("w-72")
                    ui.button(t("action_execute"), on_click=lambda lang=language: execute_action(lang)).classes("w-40")
                action_args_textarea = ui.textarea(label=t("action_args_json"), value="{}").classes("w-full")
                action_result_view = ui.code("{}", language="json").classes("w-full h-48")
                create_module_views("actions")
            with ui.tab_panel(action_executions_tab):
                create_module_views("action_executions")
            with ui.tab_panel(action_sync_tab):
                create_module_views("action_sync")
            with ui.tab_panel(recorder_status_tab):
                create_module_views("recorder_status")
            with ui.tab_panel(system_metrics_tab):
                create_module_views("system_metrics")
            with ui.tab_panel(system_alerts_tab):
                create_module_views("system_alerts")
            with ui.tab_panel(logs_tab):
                with ui.row().classes("w-full justify-center items-end gap-4 flex-wrap"):
                    log_process_select = ui.select(label=t("log_process"), options={}).classes("w-80")
                    log_stream_select = ui.select(
                        label=t("log_stream"),
                        options={"stdout": "stdout", "stderr": "stderr"},
                        value="stdout",
                    ).classes("w-52")
                    log_tail_input = ui.number(label=t("tail_bytes"), value=65536, min=0, max=4194304).classes("w-56")
                    ui.button(t("read_log"), on_click=lambda lang=language: read_log(lang)).classes("w-36")
                log_view = ui.code(t("no_log_loaded"), language="text").classes("w-full font-mono").style(
                    "height: 70vh; min-height: 560px; background: #f8fafc;"
                )

        with ui.card().classes("w-full"):
            ui.label(t("performance")).classes("text-lg font-semibold")
            with ui.grid(columns=2).classes("w-full gap-4"):
                system_resource_chart = ui.echart({}).classes("w-full h-64")
                system_network_chart = ui.echart({}).classes("w-full h-64")

    refresh(language=language)
    ui.timer(2.0, lambda lang=language: refresh(announce=False, language=lang))


ui.run(host=args.host, port=args.port, title="Axon Agent 验证面板", reload=False)
