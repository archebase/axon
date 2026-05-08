#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0
#
# axon-start.sh — one-shot tmux launcher for axon apps
#
# Each app runs in its own tmux window inside a single session named "axon":
#   window 0: recorder
#   window 1: transfer
#   window 2: panel        (optional, only if --with-panel)
#
# Assumes Debian package layout:
#   /opt/axon/bin/axon-recorder
#   /opt/axon/bin/axon-transfer
#   /opt/axon/bin/axon-panel
#   /opt/axon/plugins/libaxon_ros1_plugin.so (or ros2/udp)
#   /etc/axon/recorder.yaml
#   /etc/axon/transfer.yaml
#
# Usage:
#   axon-start                       # start recorder+transfer in tmux (ROS1)
#   axon-start ros2                  # same but ROS2 plugin
#   axon-start ros1 --with-panel     # also start axon-panel
#   axon-start attach                # attach to the session
#   axon-start status                # list windows and their state
#   axon-start logs recorder         # tail last 200 lines of one app
#   axon-start stop                  # kill the whole axon tmux session
#   axon-start restart [ros1|ros2]
#
# Env overrides:
#   AXON_SESSION=axon
#   AXON_RECORDER_CFG=/etc/axon/recorder.yaml
#   AXON_TRANSFER_CFG=/etc/axon/transfer.yaml
#   AXON_LOG_DIR=/var/log/axon
#   AXON_WS_TOKEN=<jwt>
#   ROS_MASTER_URI=http://localhost:11311

set -euo pipefail

# ------------------------------ config ------------------------------
SESSION=${AXON_SESSION:-axon}

RECORDER_BIN=${RECORDER_BIN:-/opt/axon/bin/axon-recorder}
TRANSFER_BIN=${TRANSFER_BIN:-/opt/axon/bin/axon-transfer}
PANEL_BIN=${PANEL_BIN:-/opt/axon/bin/axon-panel}
PLUGIN_DIR=${PLUGIN_DIR:-/opt/axon/plugins}

RECORDER_CFG=${AXON_RECORDER_CFG:-/etc/axon/recorder.yaml}
TRANSFER_CFG=${AXON_TRANSFER_CFG:-/etc/axon/transfer.yaml}

LOG_DIR=${AXON_LOG_DIR:-/var/log/axon}
RECORDER_LOG="$LOG_DIR/recorder.out"
TRANSFER_LOG="$LOG_DIR/transfer.out"
PANEL_LOG="$LOG_DIR/panel.out"

# Resolve the directory this script lives in so we can locate sibling helpers
# (e.g. axon-log-filter.py) regardless of how the user invoked us.
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
LOG_FILTER=${AXON_LOG_FILTER:-$SCRIPT_DIR/axon-log-filter.py}

# ------------------------------ helpers -----------------------------
c_green='\033[0;32m'; c_red='\033[0;31m'; c_yel='\033[0;33m'; c_rst='\033[0m'
info() { echo -e "${c_green}[axon]${c_rst} $*"; }
warn() { echo -e "${c_yel}[axon]${c_rst} $*"; }
err()  { echo -e "${c_red}[axon]${c_rst} $*" >&2; }

require() {
  command -v "$1" >/dev/null 2>&1 || { err "missing dependency: $1"; exit 127; }
}

resolve_plugin() {
  case "${1:-ros1}" in
    ros1)  echo "$PLUGIN_DIR/libaxon_ros1_plugin.so" ;;
    ros2)  echo "$PLUGIN_DIR/libaxon_ros2_plugin.so" ;;
    udp)   echo "$PLUGIN_DIR/libaxon_udp_plugin.so"  ;;
    /*)    echo "$1" ;;
    *)     err "unknown plugin '$1' (ros1|ros2|udp or abs path)"; exit 2 ;;
  esac
}

session_exists() { tmux has-session -t "$SESSION" 2>/dev/null; }

ensure_dirs() {
  mkdir -p "$LOG_DIR" 2>/dev/null || sudo mkdir -p "$LOG_DIR"
  # make sure current user can write (script may run as user or root)
  [ -w "$LOG_DIR" ] || sudo chown -R "$(id -u)":"$(id -g)" "$LOG_DIR" || true
  mkdir -p /data/axon /var/lib/axon/transfer 2>/dev/null || true
}

# Build the shell command a tmux window will run. We wrap the binary with
# `tee` into a logfile so users get live tty output AND a persistent log,
# and with a trailing shell so the window stays open after the process exits
# (handy for seeing the exit message / restarting manually).
wrap_cmd() {
  # $1 = human name, $2 = logfile, $3... = command
  local name=$1 log=$2; shift 2
  local cmd; cmd=$(printf '%q ' "$@")
  local wrapper="/tmp/.axon_${name}_wrapper.sh"
  cat > "$wrapper" <<INNER_EOF
#!/usr/bin/env bash
set -euo pipefail
printf '\033[1;36m==> %s starting\033[0m\n' '$name'
printf 'log: %s\n\n' '$log'
set +e
eval $cmd 2>&1 | tee -a '$log'
ec=\${PIPESTATUS[0]}
set -e
printf '\n\033[1;31m==> %s exited with %d\033[0m\n' '$name' "\$ec"
exec bash
INNER_EOF
  chmod +x "$wrapper"
  echo "$wrapper"
}

wrap_ros1_cmd() {
  # $1 = human name, $2 = logfile, $3... = command
  #
  # Same as wrap_cmd, but also:
  #   * sources /opt/ros/noetic/setup.bash if ROS env is not set yet
  #   * pipes stdout/stderr through axon-log-filter.py so that the noisy
  #     "Error in XmlRpcDispatch::work: error in poll (-1)." flood from roscpp
  #     is collapsed and a diagnostics bundle is dumped on first occurrence.
  local name=$1 log=$2; shift 2
  local cmd; cmd=$(printf '%q ' "$@")
  local wrapper="/tmp/.axon_${name}_wrapper.sh"

  if [ ! -f "$LOG_FILTER" ]; then
    warn "log filter not found at $LOG_FILTER; falling back to plain wrap_cmd"
    wrap_cmd "$name" "$log" "$@"
    return
  fi

  cat > "$wrapper" <<INNER_EOF
#!/usr/bin/env bash
set -euo pipefail
if [ -z "\${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi
export ROS_MASTER_URI="\${ROS_MASTER_URI:-http://localhost:11311}"
export AXON_SESSION='$SESSION'
printf '\033[1;36m==> %s starting\033[0m\n' '$name'
printf 'log: %s\n\n' '$log'
set +e
eval $cmd 2>&1 | AXON_LOG_DIR_FILTER='$LOG_DIR' python3 '$LOG_FILTER' | tee -a '$log'
ec=\${PIPESTATUS[0]}
set -e
printf '\n\033[1;31m==> %s exited with %d\033[0m\n' '$name' "\$ec"
exec bash
INNER_EOF
  chmod +x "$wrapper"
  echo "$wrapper"
}

# ------------------------------ actions -----------------------------
start_all() {
  require tmux
  ensure_dirs

  local plugin_name="${1:-ros1}"
  local plugin; plugin=$(resolve_plugin "$plugin_name")

  [ -x "$RECORDER_BIN" ] || { err "missing $RECORDER_BIN"; exit 3; }
  [ -x "$TRANSFER_BIN" ] || { err "missing $TRANSFER_BIN"; exit 3; }
  [ -f "$RECORDER_CFG" ] || { err "missing $RECORDER_CFG"; exit 3; }
  [ -f "$TRANSFER_CFG" ] || { err "missing $TRANSFER_CFG"; exit 3; }
  [ -f "$plugin" ]       || { err "missing plugin $plugin"; exit 3; }

  if session_exists; then
    warn "tmux session '$SESSION' already running. Use '$0 attach' or '$0 restart'."
    return 0
  fi

  info "creating tmux session '$SESSION'"

  # Force prefix to Ctrl+a to avoid conflicts with terminal/shell bindings
  local rec_wrapper; rec_wrapper=$(wrap_ros1_cmd recorder "$RECORDER_LOG" \
    "$RECORDER_BIN" --config "$RECORDER_CFG" --plugin "$plugin")
  tmux new-session -d -s "$SESSION" -n recorder "$rec_wrapper"

  tmux set-option -g prefix C-a
  tmux unbind-key C-b
  tmux bind-key C-a send-prefix

  # Window 1: transfer
  local tr_wrapper; tr_wrapper=$(wrap_cmd transfer "$TRANSFER_LOG" \
    "$TRANSFER_BIN" --config "$TRANSFER_CFG")
  tmux new-window -t "$SESSION:" -n transfer "$tr_wrapper"

  # Window 2 (optional): panel
  if [ "${WITH_PANEL:-0}" = "1" ] && [ -x "$PANEL_BIN" ]; then
    local pa_wrapper; pa_wrapper=$(wrap_cmd panel "$PANEL_LOG" "$PANEL_BIN")
    tmux new-window -t "$SESSION:" -n panel "$pa_wrapper"
  fi

  tmux select-window -t "$SESSION:0"
  info "started. Windows:"
  tmux list-windows -t "$SESSION" -F '  #I: #W (#{pane_current_command})'
  info "attach with: tmux attach -t $SESSION   (detach: Ctrl-b d)"
}

stop_all() {
  if ! session_exists; then
    warn "no tmux session '$SESSION'"; return 0
  fi
  info "killing tmux session '$SESSION'"
  tmux kill-session -t "$SESSION"
  info "stopped."
}

status_all() {
  if ! session_exists; then
    warn "session '$SESSION' is not running"; return 0
  fi
  info "session '$SESSION' windows:"
  tmux list-windows -t "$SESSION" \
    -F '  #I: #W   cmd=#{pane_current_command}  active=#{?window_active,yes,no}'
}

attach_all() {
  session_exists || { err "session '$SESSION' not running"; exit 1; }
  exec tmux attach -t "$SESSION"
}

logs_one() {
  local which="${1:-recorder}"
  local f
  case "$which" in
    recorder) f="$RECORDER_LOG" ;;
    transfer) f="$TRANSFER_LOG" ;;
    panel)    f="$PANEL_LOG" ;;
    *) err "logs target must be recorder|transfer|panel"; exit 2 ;;
  esac
  [ -f "$f" ] || { err "no log yet at $f"; exit 1; }
  tail -n 200 -F "$f"
}

# ------------------------------ arg parse ---------------------------
PLUGIN_ARG=""
WITH_PANEL=0

parse_flags() {
  # Extract --with-panel out of the arg list, leave the rest in $@
  local out=()
  for a in "$@"; do
    case "$a" in
      --with-panel) WITH_PANEL=1 ;;
      *) out+=("$a") ;;
    esac
  done
  ARGS=("${out[@]}")
}

CMD="${1:-start}"; shift || true
parse_flags "$@"
export WITH_PANEL

case "$CMD" in
  start)
    start_all "${ARGS[0]:-ros1}"
    ;;
  stop)
    stop_all
    ;;
  restart)
    stop_all
    sleep 0.3
    start_all "${ARGS[0]:-ros1}"
    ;;
  status)
    status_all
    ;;
  attach)
    attach_all
    ;;
  logs)
    logs_one "${ARGS[0]:-recorder}"
    ;;
  -h|--help|help)
    sed -n '2,40p' "$0"
    ;;
  *)
    err "unknown command '$CMD'"
    sed -n '2,40p' "$0"
    exit 64
    ;;
esac
