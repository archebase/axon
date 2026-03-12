#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon Plugin Package Build Script
# =============================================================================
# Generic script for building standalone plugin packages.
# Usage: build-plugin.sh <package-name> <source-directory>
# =============================================================================

# Detect if running in Docker
if [ -d "/axon" ] && [ -f "/axon/CMakeLists.txt" ]; then
    PROJECT_ROOT="/axon"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PACKAGE_DIR="$(dirname "${SCRIPT_DIR}")"
    PROJECT_ROOT="$(cd "${PACKAGE_DIR}/../.." && pwd)"
fi

PACKAGE_DIR="${PROJECT_ROOT}/packaging/deb"
OUTPUT_DIR="${PACKAGE_DIR}/output"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

if [ $# -lt 2 ]; then
    log_error "Usage: $0 <package-name> <source-directory>"
    exit 1
fi

PKG_NAME="$1"
SOURCE_DIR="$2"
DEBIAN_DIR="${PACKAGE_DIR}/${PKG_NAME}/debian"

if [ ! -d "$DEBIAN_DIR" ]; then
    log_error "Debian directory not found: $DEBIAN_DIR"
    exit 1
fi

if [ ! -d "$SOURCE_DIR" ]; then
    log_error "Source directory not found: ${SOURCE_DIR}"
    exit 1
fi

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Get version
if [ -f "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" ]; then
    VERSION="$(grep 'project.*VERSION' "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" | sed 's/.*VERSION \([0-9.]*\).*/\1/')"
fi
if [ -z "$VERSION" ]; then
    VERSION="0.2.1"
fi
DEBIAN_VERSION="${VERSION}-1"

log_info "Building ${PKG_NAME}..."

# Save current directory
ORIGINAL_DIR="$(pwd)"

# Create temp directory with debian files
TEMP_DIR="${SOURCE_DIR}/.debian-build"
rm -rf "${TEMP_DIR}"
cp -r "${DEBIAN_DIR}" "${TEMP_DIR}"

# Move to actual debian directory
rm -rf "${SOURCE_DIR}/debian"
mv "${TEMP_DIR}" "${SOURCE_DIR}/debian"

# Build from source directory
cd "${SOURCE_DIR}"
if dpkg-buildpackage -b -uc -us -j"$(nproc)" 2>&1; then
    log_info "Successfully built ${PKG_NAME}"

    # Move built packages to output directory
    find .. -maxdepth 1 -name "${PKG_NAME}_*.deb" -exec mv {} "${OUTPUT_DIR}/" \; 2>/dev/null || true

    # Clean up
    rm -rf "${SOURCE_DIR}/debian"
    cd "${ORIGINAL_DIR}"
    exit 0
else
    log_error "Failed to build ${PKG_NAME}"
    rm -rf "${SOURCE_DIR}/debian"
    cd "${ORIGINAL_DIR}"
    exit 1
fi
