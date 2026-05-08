#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -euo pipefail

GITHUB_REPO="archebase/axon"
DEFAULT_AXON_VERSION="latest"
DEFAULT_PLUGIN="ros1"
INSTALL_ROOT="/opt/axon"
CONFIG_DIR="/etc/axon"
SERVICE_NAME="axon.service"

FACTORY=""
WS=""
ROBOT_MODEL=""
KEYSTONE=""
AXON_VERSION="$DEFAULT_AXON_VERSION"
PLUGIN="$DEFAULT_PLUGIN"
WITH_PANEL=0

c_green='\033[0;32m'
c_yel='\033[0;33m'
c_red='\033[0;31m'
c_rst='\033[0m'

info() { printf "%b[axon-install]%b %s\n" "$c_green" "$c_rst" "$*"; }
warn() { printf "%b[axon-install]%b %s\n" "$c_yel" "$c_rst" "$*"; }
err() { printf "%b[axon-install]%b %s\n" "$c_red" "$c_rst" "$*" >&2; }

usage() {
  cat <<'USAGE'
Usage:
  wget -qO- https://example.com/install.sh | bash -s -- \
    --factory FACTORY --ws WORKSTATION --robot_model MODEL --keystone KEYSTONE_URL

Required:
  --factory        Factory name used for registration and config path.
  --ws             Workstation name/number used for registration.
  --robot_model    Robot model used for registration and config path.
  --keystone       Keystone base URL, for example http://keystone.local:8080.

Optional:
  --version        GitHub release tag to install, or "latest". Default: latest.
  --plugin         Plugin argument passed to axon-start.sh. Default: ros1.
  --with-panel     Start panel from axon-start.sh when installed.
  -h, --help       Show this help.

What it does:
  1. POSTs {factory, ws, robot_model} to $KEYSTONE/register.
  2. Downloads recorder.yaml and transfer.yaml from
     $KEYSTONE/configs/$FACTORY/$ROBOT_MODEL/ and writes them to /etc/axon/.
  3. Installs axon-all from the selected GitHub release when recorder or
     transfer is missing from /opt/axon/bin/.
  4. Installs /opt/axon/axon-start.sh and enables axon.service.
USAGE
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    err "missing dependency: $1"
    exit 127
  }
}

run_as_root() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    need_cmd sudo
    sudo "$@"
  fi
}

parse_args() {
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --factory)
        FACTORY=${2:-}
        shift 2
        ;;
      --ws)
        WS=${2:-}
        shift 2
        ;;
      --robot_model|--robot-model)
        ROBOT_MODEL=${2:-}
        shift 2
        ;;
      --keystone)
        KEYSTONE=${2:-}
        shift 2
        ;;
      --version)
        AXON_VERSION=${2:-}
        shift 2
        ;;
      --plugin)
        PLUGIN=${2:-}
        shift 2
        ;;
      --with-panel)
        WITH_PANEL=1
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        err "unknown argument: $1"
        usage
        exit 64
        ;;
    esac
  done
}

validate_args() {
  local missing=0

  for name in FACTORY WS ROBOT_MODEL KEYSTONE; do
    if [ -z "${!name}" ]; then
      err "missing required argument: --$(printf '%s' "$name" | tr '[:upper:]_' '[:lower:]-')"
      missing=1
    fi
  done

  [ "$missing" -eq 0 ] || {
    usage
    exit 64
  }

  KEYSTONE=${KEYSTONE%/}
}

json_escape() {
  local s=${1//\\/\\\\}
  s=${s//\"/\\\"}
  s=${s//$'\n'/\\n}
  s=${s//$'\r'/\\r}
  s=${s//$'\t'/\\t}
  printf '%s' "$s"
}

url_encode() {
  local LC_ALL=C s=$1 out="" c i
  for ((i = 0; i < ${#s}; i++)); do
    c=${s:i:1}
    case "$c" in
      [a-zA-Z0-9.~_-]) out+="$c" ;;
      *) printf -v out '%s%%%02X' "$out" "'$c" ;;
    esac
  done
  printf '%s' "$out"
}

http_get() {
  local url=$1 output=$2
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL --retry 3 --retry-delay 1 -o "$output" "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget -q --tries=3 -O "$output" "$url"
  else
    err "missing dependency: curl or wget"
    exit 127
  fi
}

http_post_json() {
  local url=$1 body=$2 output=$3
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL --retry 3 --retry-delay 1 \
      -H 'Content-Type: application/json' \
      -d "$body" \
      -o "$output" \
      "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget -q --tries=3 \
      --header='Content-Type: application/json' \
      --post-data="$body" \
      -O "$output" \
      "$url"
  else
    err "missing dependency: curl or wget"
    exit 127
  fi
}

detect_deb_arch() {
  local arch
  arch=$(dpkg --print-architecture)
  case "$arch" in
    amd64|arm64) printf '%s' "$arch" ;;
    *)
      err "unsupported Debian architecture: $arch"
      exit 2
      ;;
  esac
}

detect_os_codename() {
  local codename=""
  if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    codename=${VERSION_CODENAME:-}
  fi

  if [ -z "$codename" ] && command -v lsb_release >/dev/null 2>&1; then
    codename=$(lsb_release -cs)
  fi

  case "$codename" in
    focal|jammy|noble) printf '%s' "$codename" ;;
    "")
      err "could not detect Ubuntu/Debian codename"
      exit 2
      ;;
    *)
      err "unsupported OS codename: $codename. Expected focal, jammy, or noble."
      exit 2
      ;;
  esac
}

github_api_get() {
  local url=$1 output=$2
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL \
      -H 'Accept: application/vnd.github+json' \
      -H 'X-GitHub-Api-Version: 2022-11-28' \
      -o "$output" \
      "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget -q \
      --header='Accept: application/vnd.github+json' \
      --header='X-GitHub-Api-Version: 2022-11-28' \
      -O "$output" \
      "$url"
  else
    err "missing dependency: curl or wget"
    exit 127
  fi
}

extract_asset_url() {
  local metadata=$1 asset_regex=$2

  if command -v python3 >/dev/null 2>&1; then
    python3 - "$metadata" "$asset_regex" <<'PY'
import json
import re
import sys

path, pattern = sys.argv[1], sys.argv[2]
with open(path, encoding="utf-8") as f:
    data = json.load(f)

regex = re.compile(pattern)
for asset in data.get("assets", []):
    name = asset.get("name", "")
    if regex.fullmatch(name):
        print(asset.get("browser_download_url", ""))
        sys.exit(0)
sys.exit(1)
PY
    return
  fi

  if command -v jq >/dev/null 2>&1; then
    jq -r --arg re "$asset_regex" \
      '.assets[] | select(.name | test($re)) | .browser_download_url' \
      "$metadata" | head -n 1
    return
  fi

  err "missing dependency: python3 or jq is required to parse GitHub release metadata"
  exit 127
}

release_metadata_url() {
  if [ "$AXON_VERSION" = "latest" ]; then
    printf 'https://api.github.com/repos/%s/releases/latest' "$GITHUB_REPO"
  else
    printf 'https://api.github.com/repos/%s/releases/tags/%s' "$GITHUB_REPO" "$AXON_VERSION"
  fi
}

install_dependencies() {
  need_cmd apt-get
  need_cmd dpkg
  need_cmd systemctl
  need_cmd install
}

ensure_runtime_tools() {
  if command -v tmux >/dev/null 2>&1; then
    return 0
  fi

  info "installing runtime dependency: tmux"
  run_as_root apt-get update
  run_as_root apt-get install -y tmux
}

register_node() {
  local body response_file
  response_file=$(mktemp)
  body=$(printf '{"factory":"%s","ws":"%s","robot_model":"%s"}' \
    "$(json_escape "$FACTORY")" \
    "$(json_escape "$WS")" \
    "$(json_escape "$ROBOT_MODEL")")

  info "registering node at $KEYSTONE/register"
  http_post_json "$KEYSTONE/register" "$body" "$response_file"
  rm -f "$response_file"
}

download_configs() {
  local tmpdir factory_path model_path name url tmp
  tmpdir=$(mktemp -d)
  factory_path=$(url_encode "$FACTORY")
  model_path=$(url_encode "$ROBOT_MODEL")

  run_as_root install -d -m 0755 "$CONFIG_DIR"

  for name in recorder.yaml transfer.yaml; do
    url="$KEYSTONE/configs/$factory_path/$model_path/$name"
    tmp="$tmpdir/$name"
    info "downloading $url"
    http_get "$url" "$tmp"
    run_as_root install -m 0644 "$tmp" "$CONFIG_DIR/$name"
  done

  rm -rf "$tmpdir"
}

is_axon_installed() {
  [ -x "$INSTALL_ROOT/bin/axon-recorder" ] && [ -x "$INSTALL_ROOT/bin/axon-transfer" ]
}

install_axon_package() {
  local codename arch metadata asset_regex asset_url tmpdir deb api_url

  if is_axon_installed; then
    info "axon recorder and transfer are already installed"
    return 0
  fi

  codename=$(detect_os_codename)
  arch=$(detect_deb_arch)
  tmpdir=$(mktemp -d)
  metadata="$tmpdir/release.json"
  deb="$tmpdir/axon-all.deb"
  api_url=$(release_metadata_url)

  info "fetching GitHub release metadata: $api_url"
  github_api_get "$api_url" "$metadata"

  asset_regex="axon-all-${codename}_[^/]+_${arch}\\.deb"
  asset_url=$(extract_asset_url "$metadata" "$asset_regex" || true)

  if [ -z "$asset_url" ]; then
    rm -rf "$tmpdir"
    err "could not find release asset matching: $asset_regex"
    err "release URL: https://github.com/$GITHUB_REPO/releases"
    exit 1
  fi

  info "downloading $asset_url"
  http_get "$asset_url" "$deb"

  info "installing $deb"
  if ! run_as_root apt-get install -y "$deb"; then
    warn "apt-get install failed; retrying with dpkg and dependency repair"
    run_as_root dpkg -i "$deb" || run_as_root apt-get -f install -y
  fi

  rm -rf "$tmpdir"

  if ! is_axon_installed; then
    err "installation completed, but axon-recorder or axon-transfer is still missing"
    exit 1
  fi
}

install_start_script() {
  run_as_root install -d -m 0755 "$INSTALL_ROOT"

  if [ -f "./packaging/deb/scripts/axon-start.sh" ]; then
    info "installing axon-start.sh from local repository"
    run_as_root install -m 0755 "./packaging/deb/scripts/axon-start.sh" "$INSTALL_ROOT/axon-start.sh"
    if [ -f "./packaging/deb/scripts/axon-log-filter.py" ]; then
      run_as_root install -m 0755 "./packaging/deb/scripts/axon-log-filter.py" "$INSTALL_ROOT/axon-log-filter.py"
    fi
    return 0
  fi

  if [ -f "$INSTALL_ROOT/bin/axon-start.sh" ]; then
    info "installing axon-start.sh from package"
    run_as_root install -m 0755 "$INSTALL_ROOT/bin/axon-start.sh" "$INSTALL_ROOT/axon-start.sh"
    return 0
  fi

  if [ -f "/usr/share/axon/axon-start.sh" ]; then
    info "installing axon-start.sh from /usr/share/axon"
    run_as_root install -m 0755 "/usr/share/axon/axon-start.sh" "$INSTALL_ROOT/axon-start.sh"
    return 0
  fi

  warn "could not find packaged axon-start.sh; writing fallback launcher"
  run_as_root tee "$INSTALL_ROOT/axon-start.sh" >/dev/null <<'FALLBACK_AXON_START'
#!/usr/bin/env bash
set -euo pipefail

SESSION=${AXON_SESSION:-axon}
RECORDER_BIN=${RECORDER_BIN:-/opt/axon/bin/axon-recorder}
TRANSFER_BIN=${TRANSFER_BIN:-/opt/axon/bin/axon-transfer}
PANEL_BIN=${PANEL_BIN:-/opt/axon/bin/axon-panel}
PLUGIN_DIR=${PLUGIN_DIR:-/opt/axon/plugins}
RECORDER_CFG=${AXON_RECORDER_CFG:-/etc/axon/recorder.yaml}
TRANSFER_CFG=${AXON_TRANSFER_CFG:-/etc/axon/transfer.yaml}
LOG_DIR=${AXON_LOG_DIR:-/var/log/axon}

info() { printf '[axon] %s\n' "$*"; }
err() { printf '[axon] %s\n' "$*" >&2; }

resolve_plugin() {
  case "${1:-ros1}" in
    ros1) echo "$PLUGIN_DIR/libaxon_ros1_plugin.so" ;;
    ros2) echo "$PLUGIN_DIR/libaxon_ros2_plugin.so" ;;
    udp) echo "$PLUGIN_DIR/libaxon_udp_plugin.so" ;;
    /*) echo "$1" ;;
    *) err "unknown plugin '$1' (ros1|ros2|udp or abs path)"; exit 2 ;;
  esac
}

session_exists() { tmux has-session -t "$SESSION" 2>/dev/null; }

ensure_dirs() {
  mkdir -p "$LOG_DIR" /data/axon /var/lib/axon/transfer
}

start_all() {
  local plugin_arg=${1:-ros1}
  local plugin
  plugin=$(resolve_plugin "$plugin_arg")

  command -v tmux >/dev/null 2>&1 || { err "missing dependency: tmux"; exit 127; }
  ensure_dirs

  [ -x "$RECORDER_BIN" ] || { err "missing $RECORDER_BIN"; exit 3; }
  [ -x "$TRANSFER_BIN" ] || { err "missing $TRANSFER_BIN"; exit 3; }
  [ -f "$RECORDER_CFG" ] || { err "missing $RECORDER_CFG"; exit 3; }
  [ -f "$TRANSFER_CFG" ] || { err "missing $TRANSFER_CFG"; exit 3; }
  [ -f "$plugin" ] || { err "missing plugin $plugin"; exit 3; }

  if session_exists; then
    info "tmux session '$SESSION' already running"
    return 0
  fi

  tmux new-session -d -s "$SESSION" -n recorder \
    "$RECORDER_BIN --config '$RECORDER_CFG' --plugin '$plugin' 2>&1 | tee -a '$LOG_DIR/recorder.out'"
  tmux new-window -t "$SESSION:" -n transfer \
    "$TRANSFER_BIN --config '$TRANSFER_CFG' 2>&1 | tee -a '$LOG_DIR/transfer.out'"

  if [ "${WITH_PANEL:-0}" = "1" ] && [ -x "$PANEL_BIN" ]; then
    tmux new-window -t "$SESSION:" -n panel \
      "$PANEL_BIN 2>&1 | tee -a '$LOG_DIR/panel.out'"
  fi

  info "started tmux session '$SESSION'"
}

stop_all() {
  if session_exists; then
    tmux kill-session -t "$SESSION"
    info "stopped tmux session '$SESSION'"
  fi
}

case "${1:-start}" in
  start)
    shift || true
    plugin=${1:-ros1}
    shift || true
    for arg in "$@"; do
      [ "$arg" = "--with-panel" ] && export WITH_PANEL=1
    done
    start_all "$plugin"
    ;;
  stop)
    stop_all
    ;;
  restart)
    shift || true
    stop_all
    start_all "${1:-ros1}"
    ;;
  status)
    session_exists && tmux list-windows -t "$SESSION" || info "session '$SESSION' is not running"
    ;;
  attach)
    exec tmux attach -t "$SESSION"
    ;;
  *)
    err "usage: $0 {start|stop|restart|status|attach} [ros1|ros2|udp] [--with-panel]"
    exit 64
    ;;
esac
FALLBACK_AXON_START
  run_as_root chmod 0755 "$INSTALL_ROOT/axon-start.sh"
}

write_systemd_service() {
  local service_file=/etc/systemd/system/$SERVICE_NAME
  local with_panel_arg=""

  if [ "$WITH_PANEL" -eq 1 ]; then
    with_panel_arg=" --with-panel"
  fi

  info "writing $service_file"
  run_as_root tee "$service_file" >/dev/null <<EOF
[Unit]
Description=Axon tmux launcher
Documentation=https://github.com/$GITHUB_REPO
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$INSTALL_ROOT/axon-start.sh start $PLUGIN$with_panel_arg
ExecStop=$INSTALL_ROOT/axon-start.sh stop
TimeoutStartSec=60
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF

  run_as_root systemctl daemon-reload
  run_as_root systemctl enable "$SERVICE_NAME"
}

main() {
  parse_args "$@"
  validate_args
  install_dependencies
  register_node
  download_configs
  install_axon_package
  ensure_runtime_tools
  install_start_script
  write_systemd_service

  info "installation completed"
  info "start now with: sudo systemctl start $SERVICE_NAME"
}

main "$@"
