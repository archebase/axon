#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Keep the latest Axon/ROS diagnostics in memory and dump them on Ctrl-C.

This is intended for interactive reproduction: keep the terminal visible, wait
for ROS/XMLRPC errors, then press Ctrl-C immediately. The script writes only the
last in-memory window to disk so the saved logs are focused around the failure.
"""

from __future__ import annotations

import argparse
import datetime as dt
import errno
import json
import os
import subprocess
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any


def now_local() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="milliseconds")


def timestamp_name() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def run_cmd(argv: list[str], timeout: float = 2.0) -> dict[str, Any]:
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
            "stderr": f"not found: {exc}",
            "elapsed_sec": round(time.monotonic() - started, 3),
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "argv": argv,
            "returncode": None,
            "stdout": exc.stdout or "",
            "stderr": f"timeout after {timeout}s: {exc.stderr or ''}",
            "elapsed_sec": round(time.monotonic() - started, 3),
        }


def find_pids(pattern: str) -> list[int]:
    result = run_cmd(["pgrep", "-f", pattern], timeout=1.0)
    pids: list[int] = []
    for line in result.get("stdout", "").splitlines():
        try:
            pid = int(line.strip())
        except ValueError:
            continue
        if pid != os.getpid():
            pids.append(pid)
    return sorted(set(pids))


def read_proc_file(pid: int, name: str) -> str | None:
    try:
        return (Path("/proc") / str(pid) / name).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None


def collect_proc(pattern: str) -> dict[str, Any]:
    pids = find_pids(pattern)
    processes: list[dict[str, Any]] = []
    for pid in pids:
        proc: dict[str, Any] = {"pid": pid}
        fd_dir = Path("/proc") / str(pid) / "fd"
        try:
            proc["fd_count"] = len(list(fd_dir.iterdir()))
        except OSError as exc:
            proc["fd_error"] = repr(exc)
        limits = read_proc_file(pid, "limits")
        status = read_proc_file(pid, "status")
        cmdline = read_proc_file(pid, "cmdline")
        if limits is not None:
            proc["limits"] = limits
        if status is not None:
            proc["status"] = status
        if cmdline is not None:
            proc["cmdline"] = cmdline.replace("\x00", " ").strip()
        processes.append(proc)
    return {"pattern": pattern, "pids": pids, "processes": processes}


def collect_fd_links(pids: list[int]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for pid in pids:
        fd_dir = Path("/proc") / str(pid) / "fd"
        entries: list[dict[str, Any]] = []
        try:
            fds = sorted(fd_dir.iterdir(), key=lambda p: int(p.name) if p.name.isdigit() else -1)
        except OSError as exc:
            result[str(pid)] = {"error": repr(exc)}
            continue
        for fd in fds:
            try:
                target = os.readlink(fd)
            except OSError as exc:
                target = f"<readlink failed: {exc!r}>"
            info = read_proc_file(pid, f"fdinfo/{fd.name}")
            entries.append({"fd": fd.name, "target": target, "fdinfo": info})
        result[str(pid)] = {"fd_count": len(entries), "fds": entries}
    return result


def errno_reference() -> str:
    names = [
        "EAGAIN",
        "EWOULDBLOCK",
        "EBADF",
        "EINTR",
        "EINVAL",
        "ENOMEM",
        "EMFILE",
        "ENFILE",
        "ECONNREFUSED",
        "ECONNRESET",
        "ETIMEDOUT",
        "EHOSTUNREACH",
        "ENETUNREACH",
        "EADDRNOTAVAIL",
    ]
    lines = ["# errno/perror reference for common ROS1 XMLRPC socket failures"]
    seen: set[int] = set()
    for name in names:
        number = getattr(errno, name, None)
        if not isinstance(number, int):
            continue
        alias = ""
        if number in seen:
            alias = " alias"
        seen.add(number)
        lines.append(f"{name}={number}{alias}: {os.strerror(number)}")
    return "\n".join(lines) + "\n"


def tail_file(path: Path, max_bytes: int) -> str:
    try:
        size = path.stat().st_size
        with path.open("rb") as f:
            f.seek(max(0, size - max_bytes))
            data = f.read()
        return data.decode("utf-8", errors="replace")
    except OSError as exc:
        return f"failed to read {path}: {exc!r}\n"


def write_command_result(path: Path, result: dict[str, Any]) -> None:
    text = [
        f"# argv: {' '.join(result.get('argv', []))}",
        f"# returncode: {result.get('returncode')}",
        f"# elapsed_sec: {result.get('elapsed_sec')}",
        "===== stdout =====",
        result.get("stdout", ""),
        "===== stderr =====",
        result.get("stderr", ""),
    ]
    write_text(path, "\n".join(text))


def attach_short_strace(pid: int, out_path: Path, seconds: float) -> dict[str, Any]:
    if seconds <= 0:
        return {"skipped": "--opportunistic-strace-seconds <= 0"}
    argv = [
        "timeout",
        "--signal=INT",
        str(seconds),
        "strace",
        "-f",
        "-tt",
        "-T",
        "-p",
        str(pid),
        "-e",
        "trace=poll,ppoll,select,pselect6,accept,accept4,close,socket,connect,fcntl",
        "-s",
        "200",
        "-o",
        str(out_path),
    ]
    return run_cmd(argv, timeout=seconds + 3.0)


def write_opportunistic_logs(args: argparse.Namespace, out_dir: Path, samples: list[dict[str, Any]]) -> None:
    opportunistic_dir = out_dir / "opportunistic_on_interrupt"
    opportunistic_dir.mkdir(parents=True, exist_ok=True)

    write_text(opportunistic_dir / "errno_reference.txt", errno_reference())

    proc_info = collect_proc(args.process_pattern)
    write_text(opportunistic_dir / "process_now.json", json.dumps(proc_info, ensure_ascii=False, indent=2, sort_keys=True))
    pids = proc_info.get("pids", []) if isinstance(proc_info, dict) else []
    if isinstance(pids, list):
        int_pids = [pid for pid in pids if isinstance(pid, int)]
    else:
        int_pids = []
    write_text(opportunistic_dir / "fd_links_now.json", json.dumps(collect_fd_links(int_pids), ensure_ascii=False, indent=2, sort_keys=True))

    commands = {
        "ss_tanp_now.log": ["ss", "-tanp"],
        "ss_tanpi_now.log": ["ss", "-tanpi"],
        "ss_s_now.log": ["ss", "-s"],
        "ip_neigh_now.log": ["ip", "neigh", "show"],
        "getent_hostname_now.log": ["getent", "ahostsv4", args.hostname] if args.hostname else [],
        "dmesg_tail_now.log": ["dmesg", "--ctime", "--color=never"],
        "journal_kernel_recent.log": ["journalctl", "-k", "--since", args.journal_since, "--no-pager"],
        "journal_recent.log": ["journalctl", "--since", args.journal_since, "--no-pager"],
    }
    for filename, argv in commands.items():
        if argv:
            write_command_result(opportunistic_dir / filename, run_cmd(argv, timeout=args.opportunistic_cmd_timeout))

    for path_text in args.extra_log:
        path = Path(path_text).expanduser()
        safe = str(path).strip("/").replace("/", "_") or "root"
        write_text(opportunistic_dir / f"extra_log_tail_{safe}", tail_file(path, args.extra_log_bytes))

    if int_pids:
        strace_summary: dict[str, Any] = {}
        for pid in int_pids:
            strace_path = opportunistic_dir / f"strace_pid_{pid}_short.log"
            strace_summary[str(pid)] = attach_short_strace(pid, strace_path, args.opportunistic_strace_seconds)
        write_text(opportunistic_dir / "strace_summary.json", json.dumps(strace_summary, ensure_ascii=False, indent=2, sort_keys=True))

    if samples:
        last = samples[-1]
        tmux_tail = last.get("tmux_tail")
        if isinstance(tmux_tail, dict):
            write_command_result(opportunistic_dir / "tmux_tail_last_sample.log", tmux_tail)


def capture_tmux(target: str, start: str) -> dict[str, Any]:
    return run_cmd(["tmux", "capture-pane", "-t", target, "-p", "-S", start], timeout=2.0)


def count_tcp_states(ss_result: dict[str, Any]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for line in ss_result.get("stdout", "").splitlines():
        parts = line.split()
        if not parts:
            continue
        state = parts[0]
        if state in {"State", "Netid"}:
            continue
        counts[state] = counts.get(state, 0) + 1
    return counts


def collect_sample(args: argparse.Namespace, started: float) -> dict[str, Any]:
    proc = collect_proc(args.process_pattern)
    ss_result = run_cmd(["ss", "-tanp"], timeout=2.0)
    sample: dict[str, Any] = {
        "timestamp": now_local(),
        "monotonic_sec": round(time.monotonic() - started, 3),
        "process": proc,
        "ss_tanp": ss_result,
        "tcp_state_counts": count_tcp_states(ss_result),
        "getent_jzrobot_a": run_cmd(["getent", "ahostsv4", args.hostname], timeout=1.0) if args.hostname else None,
    }

    if not args.no_tmux:
        sample["tmux_tail"] = capture_tmux(args.tmux_target, args.tmux_lines)

    return sample


def short_status(sample: dict[str, Any]) -> str:
    fd_count = "?"
    pid_text = "-"
    process = sample.get("process", {})
    if isinstance(process, dict):
        pids = process.get("pids")
        if isinstance(pids, list) and pids:
            pid_text = ",".join(str(pid) for pid in pids)
        procs = process.get("processes")
        if isinstance(procs, list) and procs:
            fd_count = procs[0].get("fd_count", "?")
    host_ok = "?"
    getent = sample.get("getent_jzrobot_a")
    if isinstance(getent, dict):
        host_ok = "ok" if getent.get("returncode") == 0 else "fail"
    tcp_counts = sample.get("tcp_state_counts", {})
    tcp_text = "-"
    if isinstance(tcp_counts, dict) and tcp_counts:
        interesting = ["ESTAB", "SYN-SENT", "CLOSE-WAIT", "FIN-WAIT-1", "FIN-WAIT-2", "TIME-WAIT", "LISTEN"]
        parts = [f"{key}={tcp_counts[key]}" for key in interesting if key in tcp_counts]
        tcp_text = ",".join(parts) if parts else str(tcp_counts)
    return (
        f"{sample['timestamp']} pid={pid_text} fd={fd_count} host={host_ok} tcp={tcp_text}"
    )


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False, sort_keys=True))
            f.write("\n")


def write_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8", errors="replace")


def dump_logs(args: argparse.Namespace, samples: list[dict[str, Any]], reason: str) -> Path:
    out_dir = Path(args.output_dir or f"axon_ring_capture_{timestamp_name()}").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    write_jsonl(out_dir / "samples_last_window.jsonl", samples)


    summary = {
        "dumped_at": now_local(),
        "reason": reason,
        "argv": sys.argv,
        "args": vars(args),
        "sample_count": len(samples),
        "environment": {
            key: os.environ.get(key)
            for key in ("ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME", "ROS_NAMESPACE")
        },
        "hostname": run_cmd(["hostname"], timeout=1.0),
        "hostname_i": run_cmd(["hostname", "-I"], timeout=1.0),
        "ip_addr": run_cmd(["ip", "addr"], timeout=2.0),
        "ip_route": run_cmd(["ip", "route"], timeout=2.0),
    }
    write_text(out_dir / "summary.json", json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))

    lines = ["# Axon ring capture status lines", f"# dumped_at: {now_local()}", ""]
    lines.extend(short_status(sample) for sample in samples)
    write_text(out_dir / "screen_summary.log", "\n".join(lines) + "\n")

    if samples:
        last = samples[-1]
        ss = last.get("ss_tanp")
        if isinstance(ss, dict):
            write_text(out_dir / "last_ss_tanp.log", ss.get("stdout", "") + ss.get("stderr", ""))
        proc = last.get("process")
        write_text(out_dir / "last_process.json", json.dumps(proc, ensure_ascii=False, indent=2, sort_keys=True))

    if not args.no_tmux:
        blocks: list[str] = []
        for sample in samples:
            tmux_tail = sample.get("tmux_tail")
            if not isinstance(tmux_tail, dict):
                continue
            blocks.append(f"===== {sample['timestamp']} tmux {args.tmux_target} =====\n")
            blocks.append(tmux_tail.get("stdout", ""))
            if tmux_tail.get("stderr"):
                blocks.append("\n----- stderr -----\n" + tmux_tail.get("stderr", ""))
            blocks.append("\n")
        write_text(out_dir / "tmux_tail_samples.log", "".join(blocks))

        full_tmux = capture_tmux(args.tmux_target, "-")
        write_text(
            out_dir / "tmux_full_on_interrupt.log",
            full_tmux.get("stdout", "") + ("\n----- stderr -----\n" + full_tmux.get("stderr", "") if full_tmux.get("stderr") else ""),
        )

    if not args.no_opportunistic:
        write_opportunistic_logs(args, out_dir, samples)

    return out_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keep latest Axon/ROS diagnostics in memory and dump them on Ctrl-C.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--interval", type=float, default=1.0, help="Sample interval in seconds")
    parser.add_argument("--window", type=float, default=60.0, help="Seconds of samples kept in memory")
    parser.add_argument("--output-dir", default="", help="Output directory; default creates axon_ring_capture_<timestamp>")
    parser.add_argument("--process-pattern", default="axon-recorder", help="pgrep -f pattern for recorder process")
    parser.add_argument("--hostname", default="jzrobot-a", help="Hostname to resolve each sample; empty disables getent")
    parser.add_argument("--tmux-target", default="axon:recorder.0", help="tmux target for recorder pane")
    parser.add_argument("--tmux-lines", default="-200", help="tmux capture-pane -S value for per-sample tail")
    parser.add_argument("--no-tmux", action="store_true", help="Disable tmux capture")
    parser.add_argument("--no-opportunistic", action="store_true", help="Disable Ctrl-C one-shot opportunistic diagnostics")
    parser.add_argument("--opportunistic-cmd-timeout", type=float, default=3.0, help="Timeout for Ctrl-C one-shot commands")
    parser.add_argument("--opportunistic-strace-seconds", type=float, default=1.5, help="Attach short strace to recorder on Ctrl-C; 0 disables")
    parser.add_argument("--journal-since", default="-2 min", help="journalctl --since value captured on Ctrl-C")
    parser.add_argument("--extra-log", action="append", default=[], help="Extra log file to tail on Ctrl-C; repeatable")
    parser.add_argument("--extra-log-bytes", type=int, default=512 * 1024, help="Bytes to keep from each --extra-log")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    max_samples = max(1, int(args.window / max(args.interval, 0.001)) + 1)
    samples: deque[dict[str, Any]] = deque(maxlen=max_samples)
    started = time.monotonic()
    reason = "completed"

    print("[axon-ring-capture] sampling in memory; press Ctrl-C right after the error")
    print(f"[axon-ring-capture] interval={args.interval}s window={args.window}s max_samples={max_samples}")

    try:
        while True:
            loop_started = time.monotonic()
            sample = collect_sample(args, started)
            samples.append(sample)
            print(short_status(sample), flush=True)
            elapsed = time.monotonic() - loop_started
            time.sleep(max(0.05, args.interval - elapsed))
    except KeyboardInterrupt:
        reason = "keyboard_interrupt"
        print("\n[axon-ring-capture] Ctrl-C received; dumping in-memory samples...", flush=True)
    finally:
        out_dir = dump_logs(args, list(samples), reason)
        print(f"[axon-ring-capture] wrote {len(samples)} samples to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
