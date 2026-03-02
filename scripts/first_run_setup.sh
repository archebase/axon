#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

set -euo pipefail

MODE="product"
BASE_DIR=""
OWNER="$(id -un)"
GROUP="$(id -gn)"
DIR_MODE="750"
FILE_MODE="640"
DRY_RUN="false"

usage() {
  cat <<EOF
Usage: $0 [options]

First-run setup for all Axon apps runtime directories.

Options:
  -m, --mode <product|dev>  Working directory mode (default: product)
  -b, --base-dir <path>     Explicit base dir (overrides --mode)
  -o, --owner <owner>       Owner for created paths (default: current user)
  -g, --group <group>       Group for created paths (default: current group)
  -d, --dir-mode <mode>     Directory mode in octal (default: 750)
  -f, --file-mode <mode>    File mode in octal (default: 640)
      --dry-run             Print actions only
  -h, --help                Show this help

Policy mapping:
  product => /axon
  dev     => /tmp/axon

Created paths:
  <base>/recording
  <base>/config
  <base>/transfer
  <base>/transfer/failed_uploads
  <base>/transfer/transfer_state.db
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -m|--mode)
      MODE="$2"
      shift 2
      ;;
    -b|--base-dir)
      BASE_DIR="$2"
      shift 2
      ;;
    -o|--owner)
      OWNER="$2"
      shift 2
      ;;
    -g|--group)
      GROUP="$2"
      shift 2
      ;;
    -d|--dir-mode)
      DIR_MODE="$2"
      shift 2
      ;;
    -f|--file-mode)
      FILE_MODE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${BASE_DIR}" ]]; then
  case "${MODE}" in
    product) BASE_DIR="/axon" ;;
    dev) BASE_DIR="/tmp/axon" ;;
    *)
      echo "Invalid mode: ${MODE}. Must be 'product' or 'dev'." >&2
      exit 1
      ;;
  esac
fi

if ! [[ "${DIR_MODE}" =~ ^[0-7]{3,4}$ ]]; then
  echo "Invalid dir mode: ${DIR_MODE}" >&2
  exit 1
fi

if ! [[ "${FILE_MODE}" =~ ^[0-7]{3,4}$ ]]; then
  echo "Invalid file mode: ${FILE_MODE}" >&2
  exit 1
fi

RECORDING_DIR="${BASE_DIR}/recording"
CONFIG_DIR="${BASE_DIR}/config"
TRANSFER_DIR="${BASE_DIR}/transfer"
FAILED_UPLOADS_DIR="${TRANSFER_DIR}/failed_uploads"
STATE_DB_PATH="${TRANSFER_DIR}/transfer_state.db"

DIRS=(
  "${RECORDING_DIR}"
  "${CONFIG_DIR}"
  "${TRANSFER_DIR}"
  "${FAILED_UPLOADS_DIR}"
)

echo "Mode: ${MODE}"
echo "Base dir: ${BASE_DIR}"
echo "Owner: ${OWNER}:${GROUP}"
echo "Directory mode: ${DIR_MODE}"
echo "File mode: ${FILE_MODE}"

for dir_path in "${DIRS[@]}"; do
  if [[ "${DRY_RUN}" == "true" ]]; then
    echo "[dry-run] mkdir -p ${dir_path}"
    echo "[dry-run] chown ${OWNER}:${GROUP} ${dir_path}"
    echo "[dry-run] chmod ${DIR_MODE} ${dir_path}"
  else
    mkdir -p "${dir_path}"
    chown "${OWNER}:${GROUP}" "${dir_path}"
    chmod "${DIR_MODE}" "${dir_path}"
  fi
done

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "[dry-run] touch ${STATE_DB_PATH}"
  echo "[dry-run] chown ${OWNER}:${GROUP} ${STATE_DB_PATH}"
  echo "[dry-run] chmod ${FILE_MODE} ${STATE_DB_PATH}"
else
  touch "${STATE_DB_PATH}"
  chown "${OWNER}:${GROUP}" "${STATE_DB_PATH}"
  chmod "${FILE_MODE}" "${STATE_DB_PATH}"
fi

echo "First-run setup complete"
