#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon Standalone Package Build Script
# =============================================================================
# Builds Debian packages that don't require ROS dependencies:
# - axon-recorder
# - axon-config
# - axon-panel
# - axon-transfer
# - axon-dispatcher
# - axon-agent
# - axon-all (meta-package)
#
# Environment:
#   DISTRO - Ubuntu codename (focal|jammy|noble). Default: jammy
#   OUTPUT_SUFFIX - Optional suffix for output directory
# =============================================================================

# Detect if running in Docker (check for /axon directory with CMakeLists.txt)
if [ -d "/axon" ] && [ -f "/axon/CMakeLists.txt" ]; then
    # Running in Docker container
    PROJECT_ROOT="/axon"
else
    # Running locally
    # Get script location first
    SCRIPT_SOURCE="${BASH_SOURCE[0]}"
    while [ -h "$SCRIPT_SOURCE" ]; do
        SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_SOURCE")" && pwd)"
        SCRIPT_SOURCE="$(readlink "$SCRIPT_SOURCE")"
        [[ $SCRIPT_SOURCE != /* ]] && SCRIPT_SOURCE="$SCRIPT_DIR/$SCRIPT_SOURCE"
    done
    SCRIPT_DIR="$(cd -P "$(dirname "$SCRIPT_SOURCE")" && pwd)"
    PACKAGE_DIR="$(dirname "${SCRIPT_DIR}")"
    PROJECT_ROOT="$(cd "${PACKAGE_DIR}/../.." && pwd)"
fi

# Always set PACKAGE_DIR relative to PROJECT_ROOT
PACKAGE_DIR="${PROJECT_ROOT}/packaging/deb"
DISTRO="${DISTRO:-jammy}"
OUTPUT_DIR="${PACKAGE_DIR}/output/${DISTRO}"
BUILD_DIR="${PACKAGE_DIR}/build/${DISTRO}"
SCRIPT_DIR="${PACKAGE_DIR}/scripts"

# Map distro names to display names
case "$DISTRO" in
    focal)  DISTRO_DISPLAY="20.04 (Focal)" ;;
    jammy)  DISTRO_DISPLAY="22.04 (Jammy)" ;;
    noble)  DISTRO_DISPLAY="24.04 (Noble)" ;;
    *)      DISTRO_DISPLAY="$DISTRO" ;;
esac

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

dump_cmake_logs_on_failure() {
    local pkg_name="$1"
    local build_area="$2"
    local cmake_base="${build_area}/obj-x86_64-linux-gnu/CMakeFiles"

    log_warn "CMake configure/build failed for ${pkg_name}, dumping diagnostics if available"

    if [ -f "${cmake_base}/CMakeError.log" ]; then
        echo ""
        echo "========== ${pkg_name}: CMakeError.log =========="
        cat "${cmake_base}/CMakeError.log" || true
        echo "========== end CMakeError.log =========="
        echo ""
    else
        log_warn "CMakeError.log not found: ${cmake_base}/CMakeError.log"
    fi

    if [ -f "${cmake_base}/CMakeOutput.log" ]; then
        echo ""
        echo "========== ${pkg_name}: CMakeOutput.log =========="
        cat "${cmake_base}/CMakeOutput.log" || true
        echo "========== end CMakeOutput.log =========="
        echo ""
    else
        log_warn "CMakeOutput.log not found: ${cmake_base}/CMakeOutput.log"
    fi
}

# Create output and build directories
mkdir -p "${OUTPUT_DIR}"
mkdir -p "${BUILD_DIR}"

# Clean output directory for this distro only
rm -f "${OUTPUT_DIR}"/*.deb 2>/dev/null || true

# Clean build directory (centralized location for all intermediate artifacts)
rm -rf "${BUILD_DIR:?}"/*
mkdir -p "${BUILD_DIR}"

# Clean any stray debian directories from app directories (legacy cleanup)
for app_dir in ${PROJECT_ROOT}/apps/*; do
    rm -rf "${app_dir}/debian" "${app_dir}/obj-"* "${app_dir}/.debian-build" 2>/dev/null || true
done 2>/dev/null || true

# Check for required tools
for tool in dpkg-buildpackage dpkg-deb fakeroot; do
    if ! command -v "$tool" &> /dev/null; then
        log_error "Required tool not found: $tool"
        log_error "Install with: sudo apt-get install debhelper dpkg-dev fakeroot"
        exit 1
    fi
done

# Get version from project
if [ -f "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" ]; then
    VERSION="$(grep 'project.*VERSION' "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" 2>/dev/null | sed 's/.*VERSION \([0-9.]*\).*/\1/' || true)"
fi
if [ -z "$VERSION" ]; then
    VERSION="0.3.0"
fi
DEBIAN_VERSION="${VERSION}-1"

log_info "Building Axon standalone packages version ${DEBIAN_VERSION} for Ubuntu ${DISTRO_DISPLAY}"
log_info "Project root: ${PROJECT_ROOT}"
log_info "Build directory: ${BUILD_DIR}"
log_info "Output directory: ${OUTPUT_DIR}"

# Build function for individual packages
# Arguments: pkg_name (debian package name), app_dir (source directory), pkg_dir (packaging/deb subdirectory)
build_app_package() {
    local pkg_name="$1"
    local app_dir="$2"
    local pkg_dir="$3"  # packaging/deb subdirectory name (e.g., "recorder" not "axon-recorder")
    local debian_dir="${PACKAGE_DIR}/${pkg_dir}/debian"
    # Add distro suffix to package name for multi-distro support
    local suffixed_pkg_name="${pkg_name}-${DISTRO}"
    local build_area="${BUILD_DIR}/${suffixed_pkg_name}"

    if [ ! -d "$debian_dir" ]; then
        log_error "Debian directory not found: $debian_dir"
        return 1
    fi

    if [ ! -d "$app_dir" ]; then
        log_error "App directory not found: ${app_dir}"
        return 1
    fi

    log_info "Building ${suffixed_pkg_name}..."

    # Save current directory
    local original_dir="$(pwd)"
    local build_success=0

    # Clean and create build area
    rm -rf "${build_area}"
    mkdir -p "${build_area}"

    # Copy source to build area (excluding build artifacts and .git)
    mkdir -p "${build_area}"
    # Use tar instead of rsync for better compatibility
    (cd "${app_dir}" && tar cf - \
        --exclude='.git*' \
        --exclude='build' \
        --exclude='install' \
        --exclude='log' \
        --exclude='*.pyc' \
        --exclude='__pycache__' \
        --exclude='debian' \
        --exclude='obj-*' \
        --exclude='.debian-build' \
        --exclude='CMakeCache.txt' \
        --exclude='CMakeFiles' \
        .) | (cd "${build_area}" && tar xf -)

    # Copy debian files to build area
    cp -r "${debian_dir}" "${build_area}/debian"

    # Modify control file to add distro suffix to package name
    if [ -f "${build_area}/debian/control" ]; then
        sed -i "s/^Package: ${pkg_name}$/Package: ${suffixed_pkg_name}/" "${build_area}/debian/control"
        sed -i "s/^Source: ${pkg_name}$/Source: ${suffixed_pkg_name}/" "${build_area}/debian/control"
        # Update dependencies that reference other axon packages (add distro suffix)
        # Use perl for better regex support (handles end-of-line and various delimiters)
        perl -i -pe 's/\b(axon-recorder|axon-config|axon-panel|axon-transfer|axon-dispatcher|axon-agent)\b(?!-)/${1}-'"${DISTRO}"'/g unless /^\s*Provides:/' "${build_area}/debian/control"
    fi

    # Modify changelog to add distro suffix and update version
    if [ -f "${build_area}/debian/changelog" ]; then
        sed -i "s/^${pkg_name} /${suffixed_pkg_name} /g" "${build_area}/debian/changelog"
        # Update version from CMakeLists.txt (format: package (version) ...)
        sed -i "s/([0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]\+)/(${DEBIAN_VERSION})/" "${build_area}/debian/changelog"
    fi

    # Build from build area
    cd "${build_area}"
    if AXON_REPO_ROOT="${PROJECT_ROOT}" dpkg-buildpackage -b -uc -us -j"$(nproc)" 2>&1; then
        log_info "Successfully built ${suffixed_pkg_name}"
        build_success=1
    else
        dump_cmake_logs_on_failure "${suffixed_pkg_name}" "${build_area}"
        # Check if package was created despite the error
        if find .. -maxdepth 1 -name "${suffixed_pkg_name}_*.deb" -type f -exec test -f {} \; -print | head -1 | grep -q .; then
            log_info "Package ${suffixed_pkg_name} was created (ignoring post-package error)"
            build_success=1
        else
            log_error "Failed to build ${suffixed_pkg_name}"
        fi
    fi

    # Move built packages to output directory (even if build had post-package errors)
    find "${BUILD_DIR}" -maxdepth 1 -name "${suffixed_pkg_name}_*.deb" -type f -exec mv {} "${OUTPUT_DIR}/" \; 2>/dev/null || true

    # Clean up build area (keep directory for inspection)
    # rm -rf "${build_area}"
    cd "${original_dir}"

    if [ "$build_success" -eq 1 ]; then
        return 0
    else
        return 1
    fi
}

# Build packages in order

# 1. axon-recorder (core recorder with HTTP RPC API)
build_app_package "axon-recorder" "${PROJECT_ROOT}/apps/axon_recorder" "recorder" || exit 1

# 2. axon-config (robot configuration CLI tool)
build_app_package "axon-config" "${PROJECT_ROOT}/apps/axon_config" "config" || exit 1

# 3. axon-transfer (S3 transfer daemon)
build_app_package "axon-transfer" "${PROJECT_ROOT}/apps/axon_transfer" "transfer" || exit 1

# 4. axon-panel (web control panel - requires Node.js)
build_app_package "axon-panel" "${PROJECT_ROOT}/apps/axon_panel" "panel" || exit 1

# 5. axon-dispatcher (unified CLI dispatcher)
build_app_package "axon-dispatcher" "${PROJECT_ROOT}/apps/axon_dispatcher" "dispatcher" || exit 1

# 6. axon-agent (orchestration and process management agent)
build_app_package "axon-agent" "${PROJECT_ROOT}/apps/axon_agent" "agent" || exit 1

# 7. axon-all (meta-package)
log_info "Building axon-all-${DISTRO} (meta-package)..."
meta_pkg_name="axon-all-${DISTRO}"
temp_meta_dir="${BUILD_DIR}/${meta_pkg_name}"
rm -rf "$temp_meta_dir"
mkdir -p "$temp_meta_dir"
cd "$temp_meta_dir"

mkdir debian
cat > debian/control <<EOF
Source: ${meta_pkg_name}
Section: utils
Priority: optional
Maintainer: ArcheBase <noreply@archebase.com>

Package: ${meta_pkg_name}
Architecture: any
Depends: axon-recorder-${DISTRO}, axon-config-${DISTRO}, axon-panel-${DISTRO},
         axon-transfer-${DISTRO}, axon-dispatcher-${DISTRO},
         axon-agent-${DISTRO},
         \${misc:Depends}
Description: Axon - Complete installation for Ubuntu ${DISTRO_DISPLAY}
 This meta-package installs all core Axon tools:
 - axon-recorder: Core recording engine
 - axon-config: Robot configuration tool
 - axon-panel: Web-based control interface
 - axon-transfer: S3 transfer daemon
 - axon-dispatcher: Unified CLI entry point
 - axon-agent: Orchestration and process management agent
 .
 Built for Ubuntu ${DISTRO_DISPLAY}.
 Plugin packages must be installed separately based on your needs.
EOF
echo "7" > debian/compat
cat > debian/rules <<'EOF'
#!/usr/bin/make -f
%:
	dh $@
EOF
chmod +x debian/rules
cat > debian/changelog <<EOF
${meta_pkg_name} (${DEBIAN_VERSION}) unstable; urgency=medium
  * Initial release for Ubuntu ${DISTRO_DISPLAY}
 -- ArcheBase <noreply@archebase.com>  $(date -R)
EOF

if dpkg-buildpackage -b -uc -us; then
    log_info "Successfully built ${meta_pkg_name}"
    mv ../${meta_pkg_name}_*.deb "${OUTPUT_DIR}/" 2>/dev/null || true
else
    log_warn "Failed to build ${meta_pkg_name} meta-package"
fi
cd "${PROJECT_ROOT}"
rm -rf "$temp_meta_dir"

# Summary
log_info "==================================================================="
log_info "Build complete for Ubuntu ${DISTRO_DISPLAY}! Packages created:"
log_info "==================================================================="
ls -lh "${OUTPUT_DIR}"/*.deb 2>/dev/null || log_warn "No packages found in ${OUTPUT_DIR}"
log_info "==================================================================="

# Show package info
for deb in "${OUTPUT_DIR}"/*.deb; do
    if [ -f "$deb" ]; then
        log_info "Package: $(basename "$deb")"
        dpkg-deb -I "$deb" | grep -E "Package:|Version:|Installed-Size:" || true
    fi
done
