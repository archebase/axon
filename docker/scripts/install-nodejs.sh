#!/bin/sh
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -u

node_major="${NODE_MAJOR:-18}"
curl_retry_args="${CURL_RETRY_ARGS:---retry 5 --retry-delay 10 --retry-connrefused --retry-all-errors --connect-timeout 30 --max-time 600}"
retry_attempts="${CURL_RETRY_ATTEMPTS:-5}"
retry_delay="${CURL_RETRY_DELAY:-10}"
nodesource_keyring="/usr/share/keyrings/nodesource.gpg"
nodesource_list="/etc/apt/sources.list.d/nodesource.list"

run_with_retry() {
    attempt=1
    while [ "$attempt" -le "$retry_attempts" ]; do
        "$@"
        status="$?"
        if [ "$status" -eq 0 ]; then
            return 0
        fi

        if [ "$attempt" -eq "$retry_attempts" ]; then
            return "$status"
        fi

        echo "Command failed with ${status}; retrying in ${retry_delay}s (${attempt}/${retry_attempts})." >&2
        sleep "$retry_delay"
        attempt=$((attempt + 1))
    done
}

curl_fetch() {
    # Intentionally allow word splitting so Docker build args can pass curl flags.
    # shellcheck disable=SC2086
    run_with_retry curl -fsSL ${curl_retry_args} "$@"
}

apt_get_retry() {
    run_with_retry apt-get \
        -o Acquire::Retries=5 \
        -o Acquire::http::Timeout=30 \
        -o Acquire::https::Timeout=30 \
        "$@"
}

node_major_version() {
    node -p 'process.versions.node.split(".")[0]' 2>/dev/null
}

verify_nodejs() {
    installed_major="$(node_major_version || true)"
    if [ "$installed_major" != "$node_major" ]; then
        echo "Node.js ${node_major}.x was not installed." >&2
        return 1
    fi

    npm --version >/dev/null 2>&1 || return 1
}

dpkg_arch() {
    arch="$(dpkg --print-architecture)"
    case "$arch" in
        amd64|arm64|armhf)
            printf "%s" "$arch"
            ;;
        *)
            echo "Unsupported NodeSource architecture: ${arch}" >&2
            return 1
            ;;
    esac
}

node_binary_arch() {
    arch="$(dpkg --print-architecture)"
    case "$arch" in
        amd64)
            printf "x64"
            ;;
        arm64)
            printf "arm64"
            ;;
        armhf)
            printf "armv7l"
            ;;
        *)
            echo "Unsupported Node.js binary architecture: ${arch}" >&2
            return 1
            ;;
    esac
}

write_nodesource_pin() {
    package="$1"
    file="/etc/apt/preferences.d/${package}"

    {
        echo "Package: ${package}"
        echo "Pin: origin deb.nodesource.com"
        echo "Pin-Priority: 600"
    } > "$file"
}

clear_nodesource_repo() {
    rm -f \
        "$nodesource_keyring" \
        "$nodesource_list" \
        /etc/apt/preferences.d/nodejs \
        /etc/apt/preferences.d/nsolid
}

configure_nodesource_repo() {
    arch="$(dpkg_arch)" || return 1
    key_file="/tmp/nodesource-repo.gpg.key"

    mkdir -p /usr/share/keyrings /etc/apt/sources.list.d /etc/apt/preferences.d || return 1
    clear_nodesource_repo

    curl_fetch -o "$key_file" https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key || return 1
    gpg --dearmor -o "$nodesource_keyring" "$key_file" || return 1
    chmod 644 "$nodesource_keyring" || return 1
    rm -f "$key_file"

    echo "deb [arch=${arch} signed-by=${nodesource_keyring}] https://deb.nodesource.com/node_${node_major}.x nodistro main" > "$nodesource_list" || return 1
    write_nodesource_pin nodejs || return 1
    write_nodesource_pin nsolid || return 1
}

install_from_nodesource() {
    echo "Installing Node.js ${node_major}.x from NodeSource apt repository."
    configure_nodesource_repo || return 1
    apt_get_retry update || return 1
    apt_get_retry install -y nodejs || return 1
    verify_nodejs || return 1
}

install_from_node_binary() {
    echo "Falling back to Node.js ${node_major}.x official binary distribution." >&2

    arch="$(node_binary_arch)" || return 1
    base_url="https://nodejs.org/dist/latest-v${node_major}.x"
    tmp_dir="$(mktemp -d)" || return 1
    sums_file="${tmp_dir}/SHASUMS256.txt"

    curl_fetch -o "$sums_file" "${base_url}/SHASUMS256.txt" || {
        rm -rf "$tmp_dir"
        return 1
    }
    node_file="$(
        awk -v arch="$arch" '
            $2 ~ "^node-v[0-9.]+-linux-" arch "\\.tar\\.gz$" {
                print $2
                exit
            }
        ' "$sums_file"
    )"

    if [ -z "$node_file" ]; then
        echo "No Node.js ${node_major}.x binary found for linux-${arch}." >&2
        rm -rf "$tmp_dir"
        return 1
    fi

    checksum="$(awk -v file="$node_file" '$2 == file { print $1; exit }' "$sums_file")"
    if [ -z "$checksum" ]; then
        echo "No checksum found for ${node_file}." >&2
        rm -rf "$tmp_dir"
        return 1
    fi

    curl_fetch -o "${tmp_dir}/${node_file}" "${base_url}/${node_file}" || {
        rm -rf "$tmp_dir"
        return 1
    }
    (
        cd "$tmp_dir" && \
            printf "%s  %s\n" "$checksum" "$node_file" | sha256sum -c -
    ) || {
        rm -rf "$tmp_dir"
        return 1
    }

    rm -rf /opt/nodejs
    mkdir -p /opt/nodejs || {
        rm -rf "$tmp_dir"
        return 1
    }
    tar -xzf "${tmp_dir}/${node_file}" -C /opt/nodejs --strip-components=1 || {
        rm -rf "$tmp_dir"
        return 1
    }
    ln -sf /opt/nodejs/bin/node /usr/local/bin/node || return 1
    ln -sf /opt/nodejs/bin/npm /usr/local/bin/npm || return 1
    ln -sf /opt/nodejs/bin/npx /usr/local/bin/npx || return 1

    rm -rf "$tmp_dir"
    verify_nodejs || return 1
}

if install_from_nodesource; then
    :
else
    status="$?"
    echo "NodeSource install failed with ${status}; trying fallback installer." >&2
    clear_nodesource_repo
    install_from_node_binary
fi

node --version
npm --version
rm -rf /var/lib/apt/lists/*
