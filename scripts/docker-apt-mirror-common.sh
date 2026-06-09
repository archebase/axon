#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

detect_axon_docker_country_code() {
    local country="${AXON_DOCKER_COUNTRY:-${AXON_PACKAGE_COUNTRY:-}}"

    if [ -z "$country" ] && command -v curl &>/dev/null; then
        country="$(curl -fsSL --max-time 8 https://ipinfo.io/country 2>/dev/null || true)"
    fi
    if [ -z "$country" ] && command -v curl &>/dev/null; then
        country="$(curl -fsSL --max-time 8 https://ifconfig.co/country-iso 2>/dev/null || true)"
    fi

    country="$(printf "%s" "$country" | tr -d '[:space:]' | tr '[:lower:]' '[:upper:]')"
    printf "%.2s" "$country"
}

resolve_axon_docker_apt_mirror() {
    local requested="${AXON_APT_MIRROR:-${AXON_PACKAGE_APT_MIRROR:-auto}}"
    local country=""

    case "$requested" in
        ""|auto)
            country="$(detect_axon_docker_country_code)"
            if [ "$country" = "CN" ]; then
                printf "%s" "tsinghua"
            else
                printf "%s" "default"
            fi
            ;;
        cn|CN|china|China|tsinghua|tuna)
            printf "%s" "tsinghua"
            ;;
        default|none|off|http://*|https://*)
            printf "%s" "$requested"
            ;;
        *)
            echo "Unsupported AXON_APT_MIRROR='${requested}', using default Ubuntu apt mirror" >&2
            printf "%s" "default"
            ;;
    esac
}

configure_axon_docker_apt_mirror() {
    AXON_DOCKER_APT_MIRROR="$(resolve_axon_docker_apt_mirror)"
    export AXON_DOCKER_APT_MIRROR
    DOCKER_APT_MIRROR_BUILD_ARGS=(--build-arg "AXON_APT_MIRROR=${AXON_DOCKER_APT_MIRROR}")
}
