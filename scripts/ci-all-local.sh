#!/bin/bash
# =============================================================================
# Local CI Orchestrator Script
# =============================================================================
# Mirrors .github/workflows/ci.yml for local testing.
# Runs the full CI pipeline in stages.
#
# Usage:
#   ./scripts/ci-all-local.sh [options]
#
# Options:
#   --quick          Skip time-consuming tests (ROS, E2E)
#   --cpp-only       Only run C++ tests (unit + integration)
#   --coverage       Run with coverage instrumentation
#   --parallel       Run independent stages in parallel (experimental)
#   --help           Show this help message
#
# Pipeline Stages:
#   Stage 0: Format, Lint, REUSE Compliance
#   Stage 1: C++ Unit Tests + Zenoh Plugin Tests (parallel)
#   Stage 2: C++ Integration Tests + ROS Tests + E2E Tests (after Stage 1)
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default settings
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
QUICK_MODE=false
CPP_ONLY=false
RUN_COVERAGE=false
PARALLEL_MODE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --quick)
            QUICK_MODE=true
            shift
            ;;
        --cpp-only)
            CPP_ONLY=true
            shift
            ;;
        --coverage)
            RUN_COVERAGE=true
            shift
            ;;
        --parallel)
            PARALLEL_MODE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --quick          Skip time-consuming tests (ROS, E2E)"
            echo "  --cpp-only       Only run C++ tests (unit + integration)"
            echo "  --coverage       Run with coverage instrumentation"
            echo "  --parallel       Run independent stages in parallel (experimental)"
            echo "  --help           Show this help message"
            echo ""
            echo "Pipeline Stages:"
            echo "  Stage 0: Format, Lint, REUSE Compliance"
            echo "  Stage 1: C++ Unit Tests + Zenoh Plugin Tests"
            echo "  Stage 2: C++ Integration Tests + ROS Tests + E2E Tests"
            echo ""
            echo "Examples:"
            echo "  $0                    # Full CI pipeline"
            echo "  $0 --quick            # Quick check (skip ROS/E2E)"
            echo "  $0 --cpp-only         # Only C++ tests"
            echo "  $0 --coverage         # Full CI with coverage"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${CYAN}=============================================${NC}"
echo -e "${CYAN}    Axon Local CI Pipeline${NC}"
echo -e "${CYAN}=============================================${NC}"
echo ""
echo -e "Quick Mode:   ${YELLOW}${QUICK_MODE}${NC}"
echo -e "C++ Only:     ${YELLOW}${CPP_ONLY}${NC}"
echo -e "Coverage:     ${YELLOW}${RUN_COVERAGE}${NC}"
echo -e "Parallel:     ${YELLOW}${PARALLEL_MODE}${NC}"
echo ""

cd "${PROJECT_ROOT}"

# Track failed stages
FAILED_STAGES=()

# =============================================================================
# Stage 0: Format, Lint, REUSE Compliance
# =============================================================================
run_stage_0() {
    echo -e "${BLUE}=============================================${NC}"
    echo -e "${BLUE}Stage 0: Format, Lint & License Compliance${NC}"
    echo -e "${BLUE}=============================================${NC}"
    echo ""

    # Format check
    echo -e "${YELLOW}[0/3] Running format check...${NC}"
    if make format-ci; then
        echo -e "${GREEN}✓ Format check passed${NC}"
    else
        echo -e "${RED}✗ Format check failed${NC}"
        echo -e "${YELLOW}Run 'make format' to fix formatting issues${NC}"
        FAILED_STAGES+=("format")
        return 1
    fi

    # REUSE compliance
    echo ""
    echo -e "${YELLOW}[1/3] Running REUSE compliance check...${NC}"
    if "${SCRIPT_DIR}/ci-reuse-local.sh"; then
        echo -e "${GREEN}✓ REUSE compliance passed${NC}"
    else
        echo -e "${RED}✗ REUSE compliance failed${NC}"
        FAILED_STAGES+=("reuse")
        return 1
    fi

    echo ""
    echo -e "${GREEN}Stage 0 completed successfully!${NC}"
    return 0
}

# =============================================================================
# Stage 1: C++ Unit Tests + Zenoh Plugin Tests
# =============================================================================
run_stage_1() {
    echo ""
    echo -e "${BLUE}=============================================${NC}"
    echo -e "${BLUE}Stage 1: C++ Unit Tests & Zenoh Plugin${NC}"
    echo -e "${BLUE}=============================================${NC}"
    echo ""

    local COVERAGE_ARG=""
    if [ "$RUN_COVERAGE" = true ]; then
        COVERAGE_ARG="--coverage"
    fi

    # C++ Unit Tests
    echo -e "${YELLOW}[0/2] Running C++ unit tests...${NC}"
    if "${SCRIPT_DIR}/ci-cpp-unit-local.sh" ${COVERAGE_ARG}; then
        echo -e "${GREEN}✓ C++ unit tests passed${NC}"
    else
        echo -e "${RED}✗ C++ unit tests failed${NC}"
        FAILED_STAGES+=("cpp-unit")
        return 1
    fi

    # Zenoh Plugin Tests
    echo ""
    echo -e "${YELLOW}[1/2] Running Zenoh plugin tests...${NC}"
    if "${SCRIPT_DIR}/ci-zenoh-local.sh" ${COVERAGE_ARG}; then
        echo -e "${GREEN}✓ Zenoh plugin tests passed${NC}"
    else
        echo -e "${RED}✗ Zenoh plugin tests failed${NC}"
        FAILED_STAGES+=("zenoh")
        return 1
    fi

    echo ""
    echo -e "${GREEN}Stage 1 completed successfully!${NC}"
    return 0
}

# =============================================================================
# Stage 2: C++ Integration Tests + ROS Tests + E2E Tests
# =============================================================================
run_stage_2() {
    echo ""
    echo -e "${BLUE}=============================================${NC}"
    echo -e "${BLUE}Stage 2: Integration & ROS Tests${NC}"
    echo -e "${BLUE}=============================================${NC}"
    echo ""

    local COVERAGE_ARG=""
    if [ "$RUN_COVERAGE" = true ]; then
        COVERAGE_ARG="--coverage"
    fi

    local STAGE_FAILED=false

    # C++ Integration Tests
    echo -e "${YELLOW}[0/3] Running C++ integration tests...${NC}"
    if "${SCRIPT_DIR}/ci-cpp-integration-local.sh" ${COVERAGE_ARG}; then
        echo -e "${GREEN}✓ C++ integration tests passed${NC}"
    else
        echo -e "${RED}✗ C++ integration tests failed${NC}"
        FAILED_STAGES+=("cpp-integration")
        STAGE_FAILED=true
    fi

    # Skip ROS and E2E tests in quick mode or cpp-only mode
    if [ "$QUICK_MODE" = true ] || [ "$CPP_ONLY" = true ]; then
        echo ""
        echo -e "${YELLOW}Skipping ROS and E2E tests (--quick or --cpp-only mode)${NC}"
    else
        # ROS Tests
        echo ""
        echo -e "${YELLOW}[1/3] Running ROS tests...${NC}"
        if "${SCRIPT_DIR}/ci-ros-local.sh" ${COVERAGE_ARG}; then
            echo -e "${GREEN}✓ ROS tests passed${NC}"
        else
            echo -e "${RED}✗ ROS tests failed${NC}"
            FAILED_STAGES+=("ros")
            STAGE_FAILED=true
        fi

        # E2E Tests
        echo ""
        echo -e "${YELLOW}[2/3] Running E2E tests...${NC}"
        if "${SCRIPT_DIR}/ci-e2e-local.sh" ${COVERAGE_ARG}; then
            echo -e "${GREEN}✓ E2E tests passed${NC}"
        else
            echo -e "${RED}✗ E2E tests failed${NC}"
            FAILED_STAGES+=("e2e")
            STAGE_FAILED=true
        fi
    fi

    if [ "$STAGE_FAILED" = true ]; then
        echo ""
        echo -e "${RED}Stage 2 had failures${NC}"
        return 1
    fi

    echo ""
    echo -e "${GREEN}Stage 2 completed successfully!${NC}"
    return 0
}

# =============================================================================
# Run Pipeline
# =============================================================================
START_TIME=$(date +%s)

# Stage 0: Always run
if ! run_stage_0; then
    echo ""
    echo -e "${RED}Pipeline failed at Stage 0${NC}"
    exit 1
fi

# Stage 1: C++ Unit Tests + Zenoh
if ! run_stage_1; then
    echo ""
    echo -e "${RED}Pipeline failed at Stage 1${NC}"
    exit 1
fi

# Stage 2: Integration + ROS + E2E (skip in quick/cpp-only mode)
if [ "$QUICK_MODE" = false ] && [ "$CPP_ONLY" = false ]; then
    if ! run_stage_2; then
        echo ""
        echo -e "${RED}Pipeline failed at Stage 2${NC}"
        exit 1
    fi
fi

# =============================================================================
# Summary
# =============================================================================
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))
MINUTES=$((DURATION / 60))
SECONDS=$((DURATION % 60))

echo ""
echo -e "${CYAN}=============================================${NC}"
echo -e "${CYAN}         CI Pipeline Summary${NC}"
echo -e "${CYAN}=============================================${NC}"
echo ""

if [ ${#FAILED_STAGES[@]} -eq 0 ]; then
    echo -e "${GREEN}✅ All CI checks passed!${NC}"
    echo ""
    echo -e "Duration: ${YELLOW}${MINUTES}m ${SECONDS}s${NC}"
    echo ""
    echo -e "${GREEN}Pipeline completed successfully!${NC}"
    exit 0
else
    echo -e "${RED}❌ Some CI checks failed${NC}"
    echo ""
    echo -e "Failed stages: ${RED}${FAILED_STAGES[*]}${NC}"
    echo -e "Duration: ${YELLOW}${MINUTES}m ${SECONDS}s${NC}"
    exit 1
fi
