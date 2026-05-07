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
    local state=""
    local latest=""
    local status_value=""

    echo ""
    log_section "Package Build Progress ($(date +%H:%M:%S))"
    printf "%-10s %-8s %s\n" "Distro" "Status" "Latest output"
    printf "%-10s %-8s %s\n" "------" "------" "-------------"

    for distro in "${distros[@]}"; do
        if [ -f "${status_files[$distro]}" ]; then
            status_value="$(cat "${status_files[$distro]}" 2>/dev/null || echo 1)"
            if [ "$status_value" -eq 0 ]; then
                state="PASS"
            else
                state="FAIL"
            fi
        else
            state="RUNNING"
        fi

        latest="$(latest_log_line "${logs[$distro]}")"
        printf "%-10s %-8s %s\n" "$distro" "$state" "$latest"
    done
}

monitor_progress() {
    local completed=0
    local distro=""

    while true; do
        completed=0
        for distro in "${distros[@]}"; do
            if [ -f "${status_files[$distro]}" ]; then
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

mkdir -p "${OUTPUT_DIR}" "${LOG_DIR}"

distros=(focal jammy noble)
declare -A pids
declare -A logs
declare -A status_files
declare -A statuses

log_section "Building all packages in Docker"
log_info "Running Ubuntu distro builds in parallel: ${distros[*]}"
log_info "Logs: ${LOG_DIR}"
log_info "Progress snapshot interval: ${STATUS_INTERVAL}s"

for distro in "${distros[@]}"; do
    log_file="${LOG_DIR}/${distro}.log"
    status_file="${LOG_DIR}/${distro}.status"
    logs["$distro"]="$log_file"
    status_files["$distro"]="$status_file"
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
    pids["$distro"]=$!
    log_info "Started ${distro} build (pid ${pids[$distro]}, log ${log_file})"
done

print_progress_snapshot
monitor_progress

overall_status=0
for distro in "${distros[@]}"; do
    if wait "${pids[$distro]}"; then
        statuses["$distro"]=0
    else
        statuses["$distro"]=$?
        overall_status=1
    fi
done

echo ""
log_section "Package Build Summary"
printf "%-10s %-8s %-10s %s\n" "Distro" "Status" "Packages" "Log"
printf "%-10s %-8s %-10s %s\n" "------" "------" "--------" "---"

for distro in "${distros[@]}"; do
    distro_output="${OUTPUT_DIR}/${distro}"
    package_count=0
    if [ -d "$distro_output" ]; then
        package_count="$(find "$distro_output" -maxdepth 1 -name '*.deb' -type f 2>/dev/null | wc -l | tr -d ' ')"
    fi

    if [ "${statuses[$distro]}" -eq 0 ]; then
        status_label="PASS"
    else
        status_label="FAIL"
    fi

    printf "%-10s %-8s %-10s %s\n" "$distro" "$status_label" "$package_count" "${logs[$distro]}"
done

echo ""
if [ "$overall_status" -eq 0 ]; then
    log_info "All package builds completed successfully."
else
    log_error "One or more package builds failed. Inspect the log file above for details."
fi

exit "$overall_status"
