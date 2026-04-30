#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0
#
# axon-log-filter.py — stdin filter for axon-recorder launched under tmux.
#
# Responsibilities:
#   * Pass every log line through to stdout (so tmux / tee keep seeing it).
#   * Detect the first occurrence of
#       "Error in XmlRpcDispatch::work: error in poll (-1)."
#     and dump a diagnostics bundle (process state, fd list, network info,
#     recent journal, tmux capture) under $AXON_LOG_DIR_FILTER.
#   * Suppress the noisy repeated copies of that line and print a periodic
#     summary instead, so the recorder window stays readable.
#
# Environment variables:
#   AXON_LOG_DIR_FILTER   directory where diagnostics dumps are written
#                         (required)
#   AXON_SESSION          tmux session name used for `tmux capture-pane`
#                         (default: "axon")
#
# Usage (typically invoked from axon-start.sh):
#   <recorder stdout/stderr> | AXON_LOG_DIR_FILTER=/var/log/axon \
#       python3 /opt/axon/bin/axon-log-filter.py | tee -a recorder.out

from __future__ import annotations

import datetime
import json
import os
import subprocess
import sys
import time
from pathlib import Path


PATTERN = "Error in XmlRpcDispatch::work: error in poll (-1)."
ALLOW_INITIAL = 3
SUMMARY_INTERVAL = 5.0


def now_local() -> str:
    return (
        datetime.datetime.now(datetime.timezone.utc)
        .astimezone()
        .isoformat(timespec="milliseconds")
    )


def run_cmd(argv, timeout=3.0):
    started = time.monotonic()
    try:
        proc = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            check=False,
        )
        return {
            "argv": argv,
            "returncode": proc.returncode,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "elapsed_sec": round(time.monotonic() - started, 3),
        }
    except FileNotFoundError as exc:
        return {
            "argv": argv,
            "returncode": None,
            "stdout": "",
            "stderr": "not found: {0!r}".format(exc),
            "elapsed_sec": round(time.monotonic() - started, 3),
        }
    except subprocess.TimeoutExpired as exc:
        stderr_tail = exc.stderr if exc.stderr else ""
        return {
            "argv": argv,
            "returncode": None,
            "stdout": exc.stdout or "",
            "stderr": "timeout after {0}s: {1}".format(timeout, stderr_tail),
            "elapsed_sec": round(time.monotonic() - started, 3),
        }


def write_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8", errors="replace")


def write_cmd(path: Path, result: dict) -> None:
    write_text(
        path,
        "\n".join(
            [
                "# argv: " + " ".join(result.get("argv", [])),
                "# returncode: " + str(result.get("returncode")),
                "# elapsed_sec: " + str(result.get("elapsed_sec")),
                "===== stdout =====",
                result.get("stdout", ""),
                "===== stderr =====",
                result.get("stderr", ""),
            ]
        ),
    )


def find_recorder_pids():
    result = run_cmd(["pgrep", "-f", "/opt/axon/bin/axon-recorder"], timeout=1.0)
    pids = []
    for line in result.get("stdout", "").splitlines():
        try:
            pid = int(line.strip())
        except ValueError:
            continue
        pids.append(pid)
    return sorted(set(pids))


def read_proc(pid, name):
    try:
        return (Path("/proc") / str(pid) / name).read_text(
            encoding="utf-8", errors="replace"
        )
    except OSError as exc:
        return "<failed to read /proc/{0}/{1}: {2!r}>\n".format(pid, name, exc)


def collect_proc(pids):
    processes = []
    for pid in pids:
        proc = {"pid": pid}
        fd_dir = Path("/proc") / str(pid) / "fd"
        try:
            proc["fd_count"] = len(list(fd_dir.iterdir()))
        except OSError as exc:
            proc["fd_error"] = repr(exc)
        for name in ("cmdline", "limits", "status"):
            text = read_proc(pid, name)
            if name == "cmdline":
                text = text.replace("\x00", " ").strip()
            proc[name] = text
        processes.append(proc)
    return {"pids": pids, "processes": processes}


def collect_fd_links(pids):
    data = {}
    for pid in pids:
        fd_dir = Path("/proc") / str(pid) / "fd"
        entries = []
        try:
            fds = sorted(
                fd_dir.iterdir(),
                key=lambda p: int(p.name) if p.name.isdigit() else -1,
            )
        except OSError as exc:
            data[str(pid)] = {"error": repr(exc)}
            continue
        for fd in fds:
            try:
                target = os.readlink(fd)
            except OSError as exc:
                target = "<readlink failed: {0!r}>".format(exc)
            entries.append(
                {
                    "fd": fd.name,
                    "target": target,
                    "fdinfo": read_proc(pid, "fdinfo/" + fd.name),
                }
            )
        data[str(pid)] = {"fd_count": len(entries), "fds": entries}
    return data


def dump_poll_failure(log_dir: Path) -> Path:
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = log_dir / ("recorder_xmlrpc_poll_failure_" + stamp)
    out_dir.mkdir(parents=True, exist_ok=True)

    pids = find_recorder_pids()
    summary = {
        "dumped_at": now_local(),
        "reason": "first XmlRpcDispatch poll(-1) line observed by axon-log-filter",
        "recorder_pids": pids,
        "environment": {
            key: os.environ.get(key)
            for key in (
                "ROS_MASTER_URI",
                "ROS_IP",
                "ROS_HOSTNAME",
                "ROS_NAMESPACE",
                "ROS_DISTRO",
            )
        },
    }

    write_text(
        out_dir / "summary.json",
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True),
    )
    write_text(
        out_dir / "process.json",
        json.dumps(collect_proc(pids), ensure_ascii=False, indent=2, sort_keys=True),
    )
    write_text(
        out_dir / "fd_links.json",
        json.dumps(
            collect_fd_links(pids), ensure_ascii=False, indent=2, sort_keys=True
        ),
    )

    commands = {
        "ss_tanp.log": ["ss", "-tanp"],
        "ss_tanpi.log": ["ss", "-tanpi"],
        "ss_s.log": ["ss", "-s"],
        "ip_addr.log": ["ip", "addr"],
        "ip_route.log": ["ip", "route"],
        "ip_neigh.log": ["ip", "neigh", "show"],
        "getent_jzrobot_a.log": ["getent", "ahostsv4", "jzrobot-a"],
        "hostname.log": ["hostname"],
        "hostname_i.log": ["hostname", "-I"],
        "journal_kernel_recent.log": [
            "journalctl",
            "-k",
            "--since",
            "-2 min",
            "--no-pager",
        ],
        "journal_recent.log": ["journalctl", "--since", "-2 min", "--no-pager"],
    }
    for filename, argv in commands.items():
        write_cmd(out_dir / filename, run_cmd(argv, timeout=3.0))

    tmux_target = os.environ.get("AXON_SESSION", "axon") + ":recorder.0"
    write_cmd(
        out_dir / "tmux_recorder_capture.log",
        run_cmd(
            ["tmux", "capture-pane", "-t", tmux_target, "-p", "-S", "-"],
            timeout=5.0,
        ),
    )
    return out_dir


def main() -> int:
    log_dir_env = os.environ.get("AXON_LOG_DIR_FILTER")
    if not log_dir_env:
        print(
            "[axon-log-filter] AXON_LOG_DIR_FILTER is not set; passing through without diagnostics",
            file=sys.stderr,
            flush=True,
        )
    log_dir = Path(log_dir_env) if log_dir_env else None

    seen = 0
    suppressed = 0
    last_summary = 0.0
    dumped = False

    for line in sys.stdin:
        if PATTERN not in line:
            if suppressed:
                print(
                    "[axon-log-filter] suppressed {0} repeated XmlRpcDispatch poll(-1) lines".format(
                        suppressed
                    ),
                    flush=True,
                )
                suppressed = 0
                last_summary = time.monotonic()
            print(line, end="", flush=True)
            continue

        seen += 1
        if not dumped and log_dir is not None:
            dumped = True
            try:
                out_dir = dump_poll_failure(log_dir)
                print(
                    "[axon-log-filter] captured XmlRpcDispatch poll(-1) diagnostics at {0}".format(
                        out_dir
                    ),
                    flush=True,
                )
            except Exception as exc:
                print(
                    "[axon-log-filter] failed to capture XmlRpcDispatch poll(-1) diagnostics: {0!r}".format(
                        exc
                    ),
                    flush=True,
                )

        now = time.monotonic()
        if seen <= ALLOW_INITIAL:
            print(line, end="", flush=True)
            continue

        suppressed += 1
        if now - last_summary >= SUMMARY_INTERVAL:
            print(
                "[axon-log-filter] suppressed {0} repeated XmlRpcDispatch poll(-1) lines in the last {1:.0f}s".format(
                    suppressed, SUMMARY_INTERVAL
                ),
                flush=True,
            )
            suppressed = 0
            last_summary = now

    if suppressed:
        print(
            "[axon-log-filter] suppressed {0} repeated XmlRpcDispatch poll(-1) lines before exit".format(
                suppressed
            ),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        # Downstream consumer (e.g. tee) closed; silently exit.
        sys.exit(0)
