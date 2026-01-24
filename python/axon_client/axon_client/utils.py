# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""
Utility functions for Axon recorder client.
"""


def build_base_url(host: str, port: int) -> str:
    """Build base URL from host and port"""
    # If host already includes scheme, use as-is
    if "://" in host:
        return host.rstrip("/")
    # Default to http://
    return f"http://{host}:{port}"
