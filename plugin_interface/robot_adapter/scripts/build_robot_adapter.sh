#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  build_robot_adapter.sh --file <adapter.cpp> --output <libadapter.so> [options]

Options:
  --file <path>          Adapter implementation source file.
  --output <path>        Output shared object path.
  --cxx <compiler>       C++ compiler (default: c++).
  --extra-cflags <args>  Extra compiler flags.
  --extra-ldflags <args> Extra linker flags.
  --help                 Show this help.

Example:
  ./scripts/build_robot_adapter.sh \
    --file examples/demo_robot_adapter.cpp \
    --output /tmp/libaxon_demo_robot.so
USAGE
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
interface_root="$(cd "${script_dir}/.." && pwd)"
include_dir="${interface_root}/include"

source_file=""
output_file=""
cxx="${CXX:-c++}"
extra_cflags=""
extra_ldflags=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --file)
      source_file="${2:-}"
      shift 2
      ;;
    --output)
      output_file="${2:-}"
      shift 2
      ;;
    --cxx)
      cxx="${2:-}"
      shift 2
      ;;
    --extra-cflags)
      extra_cflags="${2:-}"
      shift 2
      ;;
    --extra-ldflags)
      extra_ldflags="${2:-}"
      shift 2
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${source_file}" || -z "${output_file}" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -f "${source_file}" ]]; then
  echo "source file not found: ${source_file}" >&2
  exit 1
fi

mkdir -p "$(dirname "${output_file}")"

# shellcheck disable=SC2086
"${cxx}" -std=c++17 -fPIC -shared -Wall -Wextra -Wpedantic \
  -I"${include_dir}" ${extra_cflags} \
  "${source_file}" -o "${output_file}" ${extra_ldflags}

for symbol in axon_agent_get_adapter_descriptor axon_agent_create_adapter axon_agent_destroy_adapter; do
  if ! nm -D "${output_file}" | grep -q " ${symbol}$"; then
    echo "required symbol is missing from ${output_file}: ${symbol}" >&2
    exit 1
  fi
done

echo "built ${output_file}"
