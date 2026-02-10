#!/bin/bash
# @file test_full_workflow.sh
# @brief Full workflow test for mock middleware (without axon_recorder)
#
# This script demonstrates the complete mock middleware functionality:
# 1. Build mock middleware
# 2. Test plugin loading via PluginLoader
# 3. Test direct E2E functionality
# 4. Show message publishing and subscription

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

log_section() {
    echo -e "\n${YELLOW}================================${NC}"
    echo -e "${YELLOW}$1${NC}"
    echo -e "${YELLOW}================================${NC}"
}

# ==============================================================================
# Main
# ==============================================================================

main() {
    log_section "Mock Middleware Full Workflow Test"

    log_step "1. Building mock middleware..."
    cd "${PROJECT_ROOT}"
    make build-mock

    log_step "2. Running plugin load test (tests C ABI interface)..."
    make test-mock-load

    log_step "3. Running E2E test (tests direct plugin functionality)..."
    make test-mock-e2e

    log_section "All Tests Completed Successfully!"

    log_info "Summary:"
    log_info "  ✓ Mock middleware built"
    log_info "  ✓ Plugin C ABI interface verified"
    log_info "  ✓ E2E functionality verified"
    log_info ""
    log_info "The mock middleware is ready for:"
    log_info "  - Integration testing with axon_recorder"
    log_info "  - CI/CD pipelines (no ROS dependencies)"
    log_info "  - Plugin development reference"
}

main "$@"
