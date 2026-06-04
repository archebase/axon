#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -u

# =============================================================================
# Axon Parallel Docker Package Build Script
# =============================================================================
# Builds package groups for Ubuntu focal, jammy, and noble concurrently.
# Each distro writes to its own log file. The console shows launch status,
# periodic latest-log snapshots, and a final summary.
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$(dirname "${SCRIPT_DIR}")/../.." && pwd)"
PACKAGE_DIR="${PROJECT_ROOT}/packaging/deb"
DOCKER_DIR="${PROJECT_ROOT}/docker"
OUTPUT_DIR="${PACKAGE_DIR}/output"
LOG_ROOT="${PACKAGE_DIR}/logs"
RUN_ID="$(date +%Y%m%d-%H%M%S)"
LOG_DIR="${LOG_ROOT}/package-all-${RUN_ID}"
STATUS_INTERVAL="${PACKAGE_STATUS_INTERVAL:-30}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

first_non_empty_env() {
    local var_name=""
    local value=""

    for var_name in "$@"; do
        value="${!var_name:-}"
        if [ -n "$value" ]; then
            printf "%s" "$value"
            return 0
        fi
    done

    return 1
}

docker_info_proxy_field() {
    local field="$1"
    local value=""

    value="$(docker info --format "{{.${field}}}" 2>/dev/null || true)"
    if [ "$value" = "<no value>" ]; then
        value=""
    fi
    printf "%s" "$value"
}

use_docker_info_proxy_fallback() {
    local mode="${AXON_PACKAGE_USE_DOCKER_INFO_PROXY:-auto}"
    local host_os=""

    case "$mode" in
        1|true|TRUE|yes|YES|on|ON) return 0 ;;
        0|false|FALSE|no|NO|off|OFF) return 1 ;;
    esac

    host_os="$(uname -s 2>/dev/null || true)"
    [ "$host_os" != "Darwin" ]
}

proxy_for_container() {
    local proxy="$1"

    if [ -n "$proxy" ]; then
        case "$proxy" in
            *://*) ;;
            *) proxy="http://${proxy}" ;;
        esac
    fi
    proxy="${proxy//\/\/127.0.0.1/\/\/host.docker.internal}"
    proxy="${proxy//\/\/localhost/\/\/host.docker.internal}"
    proxy="${proxy//\/\/\[::1\]/\/\/host.docker.internal}"
    printf "%s" "$proxy"
}

detect_host_country_code() {
    local country="${AXON_PACKAGE_COUNTRY:-}"

    if [ -z "$country" ] && command -v curl &> /dev/null; then
        country="$(curl -fsSL --max-time 8 https://ipinfo.io/country 2>/dev/null || true)"
    fi
    if [ -z "$country" ] && command -v curl &> /dev/null; then
        country="$(curl -fsSL --max-time 8 https://ifconfig.co/country-iso 2>/dev/null || true)"
    fi

    country="$(printf "%s" "$country" | tr -d '[:space:]' | tr '[:lower:]' '[:upper:]')"
    printf "%.2s" "$country"
}

configure_package_apt_mirror_env() {
    local requested="${AXON_PACKAGE_APT_MIRROR:-auto}"
    local country=""

    case "$requested" in
        ""|auto)
            country="$(detect_host_country_code)"
            if [ "$country" = "CN" ]; then
                AXON_PACKAGE_APT_MIRROR="tsinghua"
            else
                AXON_PACKAGE_APT_MIRROR="default"
            fi
            ;;
        cn|CN|china|China|tsinghua|tuna)
            AXON_PACKAGE_APT_MIRROR="tsinghua"
            ;;
        default|none|off|http://*|https://*)
            AXON_PACKAGE_APT_MIRROR="$requested"
            ;;
        *)
            log_warn "Unsupported AXON_PACKAGE_APT_MIRROR='${requested}', using default Ubuntu apt mirror"
            AXON_PACKAGE_APT_MIRROR="default"
            ;;
    esac

    export AXON_PACKAGE_APT_MIRROR
}

configure_package_proxy_env() {
    local host_http_proxy=""
    local host_https_proxy=""
    local host_all_proxy=""
    local host_no_proxy=""

    host_http_proxy="$(first_non_empty_env HTTP_PROXY http_proxy HTTPPROXY || true)"
    host_https_proxy="$(first_non_empty_env HTTPS_PROXY https_proxy HTTPSPROXY || true)"
    host_all_proxy="$(first_non_empty_env ALL_PROXY all_proxy ALLPROXY || true)"
    host_no_proxy="$(first_non_empty_env NO_PROXY no_proxy NOPROXY || true)"

    if [ -z "$host_http_proxy" ] && [ -z "$host_https_proxy" ] && [ -z "$host_all_proxy" ] && use_docker_info_proxy_fallback; then
        host_http_proxy="$(docker_info_proxy_field HTTPProxy)"
        host_https_proxy="$(docker_info_proxy_field HTTPSProxy)"
        host_no_proxy="${host_no_proxy:-$(docker_info_proxy_field NoProxy)}"
    fi

    AXON_PACKAGE_HTTP_PROXY="$(proxy_for_container "${host_http_proxy:-${host_https_proxy:-$host_all_proxy}}")"
    AXON_PACKAGE_HTTPS_PROXY="$(proxy_for_container "${host_https_proxy:-${host_http_proxy:-$host_all_proxy}}")"
    AXON_PACKAGE_ALL_PROXY="$(proxy_for_container "$host_all_proxy")"
    AXON_PACKAGE_NO_PROXY="${host_no_proxy:-localhost,127.0.0.1,::1}"

    if [ -n "$AXON_PACKAGE_HTTP_PROXY" ] || [ -n "$AXON_PACKAGE_HTTPS_PROXY" ] || [ -n "$AXON_PACKAGE_ALL_PROXY" ]; then
        AXON_PACKAGE_PROXY_INHERITED=1
    else
        AXON_PACKAGE_PROXY_INHERITED=0
    fi

    export AXON_PACKAGE_HTTP_PROXY
    export AXON_PACKAGE_HTTPS_PROXY
    export AXON_PACKAGE_ALL_PROXY
    export AXON_PACKAGE_NO_PROXY
    export AXON_PACKAGE_PROXY_INHERITED
}

run_step() {
    local description="$1"
    shift

    log_section "${description}"
    "$@"
}

build_distro() {
    local ubuntu_distro="$1"

    log_section "Building packages for Ubuntu ${ubuntu_distro}"
    run_step "Standalone packages (${ubuntu_distro})" \
        "${SCRIPT_DIR}/build-in-docker.sh" "standalone-${ubuntu_distro}" || return 1

    case "$ubuntu_distro" in
        focal)
            if [ -f "${DOCKER_DIR}/Dockerfile.package-ros1" ]; then
                run_step "ROS1 Noetic plugin (focal)" \
                    "${SCRIPT_DIR}/build-in-docker.sh" ros1 || return 1
            else
                log_warn "Skipping ROS1 Noetic: Dockerfile.package-ros1 not found"
            fi
            ;;
        jammy)
            if [ -f "${DOCKER_DIR}/Dockerfile.package-ros2.humble" ]; then
                run_step "ROS2 Humble plugin (jammy)" \
                    "${SCRIPT_DIR}/build-in-docker.sh" --distro humble || return 1
            else
                log_warn "Skipping ROS2 Humble: Dockerfile.package-ros2.humble not found"
            fi
            ;;
        noble)
            if [ -f "${DOCKER_DIR}/Dockerfile.package-ros2.jazzy" ]; then
                run_step "ROS2 Jazzy plugin (noble)" \
                    "${SCRIPT_DIR}/build-in-docker.sh" --distro jazzy || return 1
            else
                log_warn "Skipping ROS2 Jazzy: Dockerfile.package-ros2.jazzy not found"
            fi
            ;;
        *)
            log_error "Unsupported Ubuntu distro: ${ubuntu_distro}"
            return 1
            ;;
    esac

    run_step "UDP plugin (${ubuntu_distro})" \
        "${SCRIPT_DIR}/build-in-docker.sh" "udp-${ubuntu_distro}" || return 1
    log_section "Ubuntu ${ubuntu_distro} build complete"
}

latest_log_line() {
    local log_file="$1"
    local latest=""

    if [ ! -s "$log_file" ]; then
        echo "(waiting for output)"
        return
    fi

    latest="$(awk 'NF { line=$0 } END { print line }' "$log_file" 2>/dev/null || true)"
    latest="$(printf "%s" "$latest" | sed -E 's/\x1B\[[0-9;]*[A-Za-z]//g')"

    if [ -z "$latest" ]; then
        echo "(waiting for output)"
    else
        printf "%.140s\n" "$latest"
    fi
}

print_progress_snapshot() {
    local distro=""
    local i=0
    local state=""
    local latest=""
    local status_value=""

    echo ""
    log_section "Package Build Progress ($(date +%H:%M:%S))"
    if [ "${AXON_PACKAGE_PROXY_INHERITED:-0}" -eq 1 ]; then
        log_info "Host proxy inherited for Docker package builds."
    fi
    if [ "${AXON_PACKAGE_APT_MIRROR:-default}" != "default" ] && [ "${AXON_PACKAGE_APT_MIRROR:-default}" != "none" ] && [ "${AXON_PACKAGE_APT_MIRROR:-default}" != "off" ]; then
        log_info "Using ${AXON_PACKAGE_APT_MIRROR} Ubuntu apt mirror for Docker package builds."
    fi
    printf "%-10s %-8s %s\n" "Distro" "Status" "Latest output"
    printf "%-10s %-8s %s\n" "------" "------" "-------------"

    for ((i = 0; i < ${#distros[@]}; i++)); do
        distro="${distros[$i]}"
        if [ -f "${status_files[$i]}" ]; then
            status_value="$(cat "${status_files[$i]}" 2>/dev/null || echo 1)"
            if [ "$status_value" -eq 0 ]; then
                state="PASS"
            else
                state="FAIL"
            fi
        else
            state="RUNNING"
        fi

        latest="$(latest_log_line "${logs[$i]}")"
        printf "%-10s %-8s %s\n" "$distro" "$state" "$latest"
    done
}

monitor_progress() {
    local completed=0
    local distro=""
    local i=0

    while true; do
        completed=0
        for ((i = 0; i < ${#distros[@]}; i++)); do
            distro="${distros[$i]}"
            if [ -f "${status_files[$i]}" ]; then
                completed=$((completed + 1))
            fi
        done

        if [ "$completed" -eq "${#distros[@]}" ]; then
            break
        fi

        sleep "$STATUS_INTERVAL"
        print_progress_snapshot
    done
}

if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker first."
    exit 1
fi

configure_package_proxy_env
configure_package_apt_mirror_env

mkdir -p "${OUTPUT_DIR}" "${LOG_DIR}"

distros=(focal jammy noble)
pids=()
logs=()
status_files=()
statuses=()

log_section "Building all packages in Docker"
log_info "Running Ubuntu distro builds in parallel: ${distros[*]}"
log_info "Logs: ${LOG_DIR}"
log_info "Progress snapshot interval: ${STATUS_INTERVAL}s"
if [ "${AXON_PACKAGE_PROXY_INHERITED}" -eq 1 ]; then
    log_info "Host proxy inherited for Docker package builds."
fi
if [ "${AXON_PACKAGE_APT_MIRROR}" != "default" ] && [ "${AXON_PACKAGE_APT_MIRROR}" != "none" ] && [ "${AXON_PACKAGE_APT_MIRROR}" != "off" ]; then
    log_info "Using ${AXON_PACKAGE_APT_MIRROR} Ubuntu apt mirror for Docker package builds."
fi

for ((i = 0; i < ${#distros[@]}; i++)); do
    distro="${distros[$i]}"
    log_file="${LOG_DIR}/${distro}.log"
    status_file="${LOG_DIR}/${distro}.status"
    logs[$i]="$log_file"
    status_files[$i]="$status_file"
    rm -f "$status_file"
    (
        echo "Log file: ${log_file}"
        echo "Started: $(date -R)"
        build_distro "$distro"
        status=$?
        echo "Finished: $(date -R)"
        echo "$status" > "$status_file"
        exit "$status"
    ) >"$log_file" 2>&1 &
    pids[$i]=$!
    log_info "Started ${distro} build (pid ${pids[$i]}, log ${log_file})"
done

print_progress_snapshot
monitor_progress

overall_status=0
for ((i = 0; i < ${#distros[@]}; i++)); do
    distro="${distros[$i]}"
    if wait "${pids[$i]}"; then
        statuses[$i]=0
    else
        statuses[$i]=$?
        overall_status=1
    fi
done

echo ""
log_section "Package Build Summary"
printf "%-10s %-8s %-10s %s\n" "Distro" "Status" "Packages" "Log"
printf "%-10s %-8s %-10s %s\n" "------" "------" "--------" "---"

for ((i = 0; i < ${#distros[@]}; i++)); do
    distro="${distros[$i]}"
    distro_output="${OUTPUT_DIR}/${distro}"
    package_count=0
    if [ -d "$distro_output" ]; then
        package_count="$(find "$distro_output" -maxdepth 1 -name '*.deb' -type f 2>/dev/null | wc -l | tr -d ' ')"
    fi

    if [ "${statuses[$i]}" -eq 0 ]; then
        status_label="PASS"
    else
        status_label="FAIL"
    fi

    printf "%-10s %-8s %-10s %s\n" "$distro" "$status_label" "$package_count" "${logs[$i]}"
done

echo ""
if [ "$overall_status" -eq 0 ]; then
    log_info "All package builds completed successfully."
else
    log_error "One or more package builds failed. Inspect the log file above for details."
fi

exit "$overall_status"
