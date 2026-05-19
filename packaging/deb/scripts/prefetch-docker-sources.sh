#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$(dirname "${SCRIPT_DIR}")/../.." && pwd)"
CACHE_DIR="${AXON_PACKAGE_SOURCE_CACHE_DIR:-${PROJECT_ROOT}/packaging/deb/source-cache}"
TMP_ROOT="${CACHE_DIR}/.tmp"
LOCK_DIR="${CACHE_DIR}/.prefetch.lock"

MCAP_COMMIT="${AXON_PACKAGE_MCAP_COMMIT:-a7eeff1383cc95a29dd9c5cac94104d277b5414e}"
HTTPLIB_VERSION="${AXON_PACKAGE_HTTPLIB_VERSION:-0.15.3}"
AWS_SDK_CPP_VERSION="${AXON_PACKAGE_AWS_SDK_CPP_VERSION:-1.11.479}"

RETRY_ATTEMPTS="${AXON_PACKAGE_SOURCE_RETRY_ATTEMPTS:-5}"
RETRY_DELAY="${AXON_PACKAGE_SOURCE_RETRY_DELAY:-10}"
LOCK_TIMEOUT="${AXON_PACKAGE_SOURCE_LOCK_TIMEOUT:-3600}"
GIT_JOBS="${AXON_PACKAGE_SOURCE_GIT_JOBS:-4}"

RUN_TMP_DIR=""
LOCK_ACQUIRED=0

log_info() {
    echo "[INFO] $1"
}

log_warn() {
    echo "[WARN] $1" >&2
}

log_error() {
    echo "[ERROR] $1" >&2
}

cleanup() {
    if [ -n "$RUN_TMP_DIR" ] && [ -d "$RUN_TMP_DIR" ]; then
        rm -rf "$RUN_TMP_DIR"
    fi
    if [ "$LOCK_ACQUIRED" -eq 1 ]; then
        rmdir "$LOCK_DIR" 2>/dev/null || true
    fi
}
trap cleanup EXIT

archive_valid() {
    local archive="$1"

    [ -s "$archive" ] && tar -tzf "$archive" >/dev/null 2>&1
}

retry_command() {
    local attempt=1
    local status=0

    while [ "$attempt" -le "$RETRY_ATTEMPTS" ]; do
        if "$@"; then
            return 0
        fi

        status="$?"
        if [ "$attempt" -eq "$RETRY_ATTEMPTS" ]; then
            return "$status"
        fi

        log_warn "Command failed with ${status}; retrying in ${RETRY_DELAY}s (${attempt}/${RETRY_ATTEMPTS}): $*"
        sleep "$RETRY_DELAY"
        attempt=$((attempt + 1))
    done

    return 1
}

acquire_lock() {
    local waited=0

    mkdir -p "$CACHE_DIR"
    while ! mkdir "$LOCK_DIR" 2>/dev/null; do
        if [ "$waited" -ge "$LOCK_TIMEOUT" ]; then
            log_error "Timed out waiting for package source prefetch lock: ${LOCK_DIR}"
            return 1
        fi
        log_info "Waiting for package source prefetch lock: ${LOCK_DIR}"
        sleep 5
        waited=$((waited + 5))
    done

    LOCK_ACQUIRED=1
    mkdir -p "$TMP_ROOT"
    RUN_TMP_DIR="$(mktemp -d "${TMP_ROOT}/prefetch.XXXXXX")"
}

check_required_commands() {
    local command_name=""

    for command_name in curl git mktemp tar; do
        if ! command -v "$command_name" >/dev/null 2>&1; then
            log_error "Required command not found: ${command_name}"
            return 1
        fi
    done
}

download_archive() {
    local name="$1"
    local url="$2"
    local output="$3"
    local tmp_file="${RUN_TMP_DIR}/$(basename "$output").download"

    if archive_valid "$output"; then
        log_info "Using cached ${name}: ${output}"
        return 0
    fi

    if [ -e "$output" ]; then
        log_warn "Removing invalid cached ${name}: ${output}"
        rm -f "$output"
    fi

    log_info "Downloading ${name}: ${url}"
    retry_command curl -fL --show-error --silent --connect-timeout 30 --max-time 600 -o "$tmp_file" "$url"
    tar -tzf "$tmp_file" >/dev/null
    mv "$tmp_file" "$output"
    log_info "Cached ${name}: ${output}"
}

git_with_timeouts() {
    git -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=60 "$@"
}

clone_aws_sdk_cpp() {
    local work_dir="$1"

    rm -rf "$work_dir"
    git_with_timeouts clone \
        --depth 1 \
        --branch "$AWS_SDK_CPP_VERSION" \
        --single-branch \
        https://github.com/aws/aws-sdk-cpp.git \
        "$work_dir"
}

prefetch_aws_sdk_cpp() {
    local output="${CACHE_DIR}/aws-sdk-cpp-${AWS_SDK_CPP_VERSION}.tar.gz"
    local work_dir="${RUN_TMP_DIR}/aws-sdk-cpp-${AWS_SDK_CPP_VERSION}"
    local tmp_archive="${RUN_TMP_DIR}/$(basename "$output")"

    if archive_valid "$output"; then
        log_info "Using cached AWS SDK for C++: ${output}"
        return 0
    fi

    if [ -e "$output" ]; then
        log_warn "Removing invalid cached AWS SDK for C++: ${output}"
        rm -f "$output"
    fi

    log_info "Cloning AWS SDK for C++ ${AWS_SDK_CPP_VERSION}"
    retry_command clone_aws_sdk_cpp "$work_dir"

    log_info "Fetching AWS SDK for C++ submodules"
    (
        cd "$work_dir"
        git_with_timeouts submodule sync --recursive
        retry_command git_with_timeouts submodule update --init --recursive --jobs "$GIT_JOBS"
    )

    log_info "Packing AWS SDK for C++ source cache"
    tar --exclude-vcs -czf "$tmp_archive" -C "$RUN_TMP_DIR" "aws-sdk-cpp-${AWS_SDK_CPP_VERSION}"
    tar -tzf "$tmp_archive" >/dev/null
    mv "$tmp_archive" "$output"
    log_info "Cached AWS SDK for C++: ${output}"
}

main() {
    check_required_commands
    acquire_lock

    download_archive \
        "MCAP ${MCAP_COMMIT}" \
        "https://github.com/foxglove/mcap/archive/${MCAP_COMMIT}.tar.gz" \
        "${CACHE_DIR}/mcap-${MCAP_COMMIT}.tar.gz"

    download_archive \
        "cpp-httplib ${HTTPLIB_VERSION}" \
        "https://github.com/yhirose/cpp-httplib/archive/refs/tags/v${HTTPLIB_VERSION}.tar.gz" \
        "${CACHE_DIR}/cpp-httplib-${HTTPLIB_VERSION}.tar.gz"

    prefetch_aws_sdk_cpp

    log_info "Docker package source cache is ready: ${CACHE_DIR}"
}

main "$@"
