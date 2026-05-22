#!/bin/sh
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -u

retry_attempts="${CURL_RETRY_ATTEMPTS:-5}"
retry_delay="${CURL_RETRY_DELAY:-10}"
curl_retry_args="${CURL_RETRY_ARGS:---connect-timeout 30 --max-time 600}"

# Ubuntu 20.04 ships curl 7.68, which does not support --retry-all-errors.
# The outer loop retries every curl failure, including TLS EOF failures.
curl_retry_args="$(printf "%s" "$curl_retry_args" | sed 's/--retry-all-errors//g')"

attempt=1
while [ "$attempt" -le "$retry_attempts" ]; do
    # Intentionally allow word splitting so Docker build args can pass curl flags.
    # shellcheck disable=SC2086
    curl -fsSL $curl_retry_args "$@"
    status="$?"
    if [ "$status" -eq 0 ]; then
        exit 0
    fi

    if [ "$attempt" -eq "$retry_attempts" ]; then
        exit "$status"
    fi

    echo "curl failed with ${status}; retrying in ${retry_delay}s (${attempt}/${retry_attempts})." >&2
    sleep "$retry_delay"
    attempt=$((attempt + 1))
done

exit 1
