# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

"""Tests for axon_client utility functions."""

import pytest

from axon_client.utils import build_base_url


class TestBuildBaseUrl:
    """Tests for build_base_url utility function."""

    def test_default_http(self):
        """Test default HTTP scheme."""
        url = build_base_url("localhost", 8080)
        assert url == "http://localhost:8080"

    def test_custom_port(self):
        """Test with custom port."""
        url = build_base_url("example.com", 9000)
        assert url == "http://example.com:9000"

    def test_https_scheme(self):
        """Test with HTTPS URL (scheme preserved)."""
        url = build_base_url("https://example.com", 8080)
        assert url == "https://example.com:8080"

    def test_full_url_with_scheme(self):
        """Test that full URL with scheme overrides everything."""
        url = build_base_url("https://example.com:8080", 9999)
        assert url == "https://example.com:8080"

    def test_host_without_scheme(self):
        """Test host without scheme uses default http://."""
        url = build_base_url("example.com", 8080)
        assert url == "http://example.com:8080"

    def test_trailing_slash_removed(self):
        """Test that trailing slash is removed."""
        url = build_base_url("example.com/", 8080)
        assert url == "http://example.com"  # No trailing slash

    def test_ipv4_localhost(self):
        """Test with IPv4 address."""
        url = build_base_url("127.0.0.1", 8080)
        assert url == "http://127.0.0.1:8080"

    def test_ipv6_localhost(self):
        """Test with IPv6 address."""
        url = build_base_url("::1", 8080)
        assert url == "http://::1:8080"
