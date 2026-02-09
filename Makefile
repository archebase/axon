# Makefile for Axon by ArcheBase
# Supports both C++ core libraries and ROS (1/2) middlewares

.DEFAULT_GOAL := help

# Use bash as the shell for all recipes (required for ROS setup.bash scripts)
SHELL := /bin/bash

# Project root (current directory)
PROJECT_ROOT := $(shell pwd)

# Detect ROS version
ROS_VERSION ?= $(shell if [ -n "$$ROS_VERSION" ]; then echo $$ROS_VERSION; elif [ -n "$$ROS_DISTRO" ]; then echo 1; else echo 2; fi)
ROS_DISTRO ?= $(shell echo $$ROS_DISTRO)

# Build directories (relative to this Makefile location)
BUILD_DIR := build
COVERAGE_DIR := coverage

# =============================================================================
# clang-format setup - detect version 21 (clang-format-21 or clang-format)
# =============================================================================

CLANG_FORMAT_VERSION := 21

# Try to find clang-format version 21 (prefer clang-format-21, then clang-format)
CLANG_FORMAT := $(shell \
	for cmd in clang-format-21 clang-format; do \
		if command -v $$cmd >/dev/null 2>&1; then \
			VERSION=$$($$cmd --version 2>/dev/null | grep -oE 'version [0-9]+' | grep -oE '[0-9]+' | head -1); \
			if [ "$$VERSION" = "$(CLANG_FORMAT_VERSION)" ]; then \
				echo $$cmd; \
				break; \
			fi; \
		fi; \
	done \
)

# Check if we found a suitable clang-format
ifneq ($(CLANG_FORMAT),)
    $(info ✓ Using $(CLANG_FORMAT) (version $(CLANG_FORMAT_VERSION)))
else
    $(info ⚠ clang-format version $(CLANG_FORMAT_VERSION) not found)
    $(info   Linux: wget -qO- https://apt.llvm.org/llvm.sh | sudo bash -s $(CLANG_FORMAT_VERSION))
    $(info   macOS: brew install llvm@$(CLANG_FORMAT_VERSION))
endif

# Build type
BUILD_TYPE ?= Release

# Detect number of CPU cores
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Colors for output (only use if stdout is a TTY)
# Check if stdout is a terminal, otherwise use empty strings
ifeq ($(shell test -t 1 && echo yes),yes)
  RED := \033[0;31m
  GREEN := \033[0;32m
  YELLOW := \033[0;33m
  BLUE := \033[0;34m
  NC := \033[0m # No Color
else
  RED :=
  GREEN :=
  YELLOW :=
  BLUE :=
  NC :=
endif

# Helper to print colored output (printf interprets escape sequences)
PRINTF := printf "%s\n"

# =============================================================================
# Help Target
# =============================================================================

.PHONY: all help

all: build-core
	@printf "%s\n" "$(GREEN)✓ All components built$(NC)"

help:
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║     Axon by ArcheBase - Build System                     ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Core C++ Libraries:$(NC)"
	@printf "%s\n" "  $(BLUE)make build-core$(NC)       - Build all C++ libraries and apps"
	@printf "%s\n" "  $(BLUE)make test-mcap$(NC)       - Build and run axon_mcap tests"
	@printf "%s\n" "  $(BLUE)make test-uploader$(NC)   - Build and run axon_uploader tests"
	@printf "%s\n" "  $(BLUE)make test-logging$(NC)    - Build and run axon_logging tests"
	@printf "%s\n" "  $(BLUE)make test-core$(NC)       - Run all C++ library tests"
	@printf "%s\n" "  $(BLUE)make test-axon-config$(NC) - Build and run axon_config tests"
	@printf "%s\n" "  $(BLUE)make coverage-core$(NC)   - Run all C++ tests with coverage"
	@echo ""
	@printf "%s\n" "$(YELLOW)Applications:$(NC)"
	@printf "%s\n" "  $(BLUE)make app$(NC)              - Build all applications"
	@printf "%s\n" "  $(BLUE)make app-axon-recorder$(NC) - Build axon_recorder plugin loader"
	@printf "%s\n" "  $(BLUE)make app-axon-config$(NC)  - Build axon_config tool"
	@echo ""
	@printf "%s\n" "$(YELLOW)Mock Middleware (Testing):$(NC)"
	@printf "%s\n" "  $(BLUE)make build-mock$(NC)           - Build mock middleware plugin"
	@printf "%s\n" "  $(BLUE)make test-mock-e2e$(NC)        - Run mock plugin E2E test (standalone)"
	@printf "%s\n" "  $(BLUE)make test-mock-load$(NC)       - Run mock plugin load test"
	@printf "%s\n" "  $(BLUE)make test-mock-integration$(NC) - Run mock middleware integration E2E test"
	@printf "%s\n" "  $(BLUE)make test-mock-all$(NC)         - Run all mock middleware tests"
	@echo ""
	@printf "%s\n" "$(YELLOW)ROS Middlewares:$(NC)"
	@printf "%s\n" "  $(BLUE)make build$(NC)              - Build (auto-detects ROS1/ROS2)"
	@printf "%s\n" "  $(BLUE)make build-ros1$(NC)        - Build ROS1 (Noetic)"
	@printf "%s\n" "  $(BLUE)make build-ros2$(NC)        - Build ROS2 (Humble/Jazzy/Rolling)"
	@printf "%s\n" "  $(BLUE)make build-zenoh$(NC)       - Build Zenoh plugin"
	@echo ""
	@printf "%s\n" "$(YELLOW)Docker tests:$(NC)"
	@printf "%s\n" "  $(BLUE)make docker-test-ros1$(NC)         - ROS 1 (Noetic) tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-ros2-humble$(NC) - ROS 2 Humble tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-ros2-jazzy$(NC)  - ROS 2 Jazzy tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-ros2-rolling$(NC) - ROS 2 Rolling tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-all$(NC)          - Run all Docker tests"
	@echo ""
	@printf "%s\n" "$(YELLOW)General:$(NC)"
	@printf "%s\n" "  $(BLUE)make clean$(NC)             - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make install$(NC)           - Install package"
	@printf "%s\n" "  $(BLUE)make debug/release$(NC)      - Build with Debug/Release config"
	@echo ""
	@printf "%s\n" "$(YELLOW)Code quality:$(NC)"
	@printf "%s\n" "  $(BLUE)make format$(NC)            - Format with clang-format"
	@printf "%s\n" "  $(BLUE)make lint$(NC)              - Lint with cppcheck"
	@echo ""
	@printf "%s\n" "$(YELLOW)CI Testing (All Docker-based):$(NC)"
	@printf "%s\n" "  $(BLUE)make ci$(NC)                - Quick CI (format + lint + C++ tests)"
	@printf "%s\n" "  $(BLUE)make ci-quick$(NC)          - Fastest check (format + lint only)"
	@printf "%s\n" "  $(BLUE)make ci-all$(NC)            - Full CI validation (all tests)"
	@printf "%s\n" "  $(BLUE)make ci-cpp$(NC)            - C++ library tests (unit + integration)"
	@printf "%s\n" "  $(BLUE)make ci-cpp-unit$(NC)       - C++ unit tests only"
	@printf "%s\n" "  $(BLUE)make ci-cpp-integration$(NC) - C++ integration tests (with MinIO)"
	@printf "%s\n" "  $(BLUE)make ci-ros$(NC)            - ROS tests (all distros)"
	@printf "%s\n" "  $(BLUE)make ci-ros1$(NC)           - ROS1 Noetic tests"
	@printf "%s\n" "  $(BLUE)make ci-ros2$(NC)           - ROS2 (Humble + Jazzy + Rolling) tests"
	@printf "%s\n" "  $(BLUE)make ci-e2e$(NC)            - E2E tests (ROS1 + ROS2 Humble)"
	@printf "%s\n" "  $(BLUE)make ci-coverage$(NC)       - Coverage tests (C++ + ROS2 + Recorder)"
	@printf "%s\n" "  $(BLUE)make ci-coverage-cpp$(NC)   - C++ library coverage"
	@printf "%s\n" "  $(BLUE)make ci-coverage-ros$(NC)   - ROS2 coverage (Docker)"
	@printf "%s\n" "  $(BLUE)make ci-coverage-recorder$(NC) - Axon recorder coverage (local)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Docker Testing:$(NC)"
	@printf "%s\n" "  $(BLUE)make docker-test-cpp$(NC)   - C++ tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-zenoh$(NC) - Zenoh plugin tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-zenoh-integration$(NC) - Axon + Zenoh integration tests"
	@printf "%s\n" "  $(BLUE)make docker-test-ros$(NC)   - ROS tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-e2e$(NC)   - E2E tests in Docker (ROS1 + Humble)"
	@printf "%s\n" "  $(BLUE)make docker-test-e2e-all$(NC) - E2E tests (all ROS distros)"
	@printf "%s\n" "  $(BLUE)make docker-coverage$(NC)   - ROS2 coverage in Docker"
	@echo ""
	@printf "%s\n" "$(YELLOW)E2E Testing:$(NC)"
	@printf "%s\n" "  $(BLUE)make e2e$(NC)                - E2E tests (local, requires build)"
	@printf "%s\n" "  $(BLUE)make docker-test-e2e-ros1$(NC)       - ROS1 E2E tests"
	@printf "%s\n" "  $(BLUE)make docker-test-e2e-ros2-humble$(NC) - ROS2 Humble E2E tests"
	@printf "%s\n" "  $(BLUE)make docker-test-e2e-ros2-jazzy$(NC)    - ROS2 Jazzy E2E tests"
	@printf "%s\n" "  $(BLUE)make docker-test-e2e-ros2-rolling$(NC) - ROS2 Rolling E2E tests"
	@echo ""
	@printf "%s\n" "$(YELLOW)Utility:$(NC)"
	@printf "%s\n" "  $(BLUE)make coverage-merge$(NC)    - Merge all coverage reports"
	@printf "%s\n" "  $(BLUE)make coverage-html-cpp$(NC) - Generate C++ coverage HTML"
	@printf "%s\n" "  $(BLUE)make format-ci$(NC)         - Check code formatting (CI style)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Current:$(NC) ROS=$(ROS_VERSION) DISTRO=$(ROS_DISTRO) TYPE=$(BUILD_TYPE)"

# =============================================================================
# Core C++ Library Targets
# =============================================================================

.PHONY: build-core build-mcap build-logging build-uploader
.PHONY: test-mcap test-uploader test-logging test-core
.PHONY: coverage-mcap coverage-uploader coverage-logging coverage-core

# Build core libraries (axon_logging, axon_mcap, axon_uploader)
build-core:
	@printf "%s\n" "$(YELLOW)Building core libraries...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON \
			--log-level=ERROR >/dev/null 2>&1 || \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON
	@cmake --build $(BUILD_DIR) -j$(NPROC) --target axon_logging axon_mcap
	@printf "%s\n" "$(GREEN)✓ Core libraries built$(NC)"

# Build individual core libraries
build-mcap:
	@printf "%s\n" "$(YELLOW)Building axon_mcap (with tests)...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_mcap built$(NC)"

build-logging:
	@printf "%s\n" "$(YELLOW)Building axon_logging (with tests)...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_logging built$(NC)"

build-uploader:
	@printf "%s\n" "$(YELLOW)Building axon_uploader (with tests)...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_BUILD_UPLOADER=ON \
			$(CMAKE_EXTRA_ARGS) && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_uploader built$(NC)"

# Test axon_mcap
test-mcap: build-mcap
	@printf "%s\n" "$(YELLOW)Running axon_mcap tests...$(NC)"
	@cd $(BUILD_DIR)/axon_mcap && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_mcap tests passed$(NC)"

# Test axon_uploader
test-uploader: build-core
	@printf "%s\n" "$(YELLOW)Running axon_uploader tests...$(NC)"
	@if [ -d "$(BUILD_DIR)/axon_uploader" ]; then \
		cd $(BUILD_DIR)/axon_uploader && ctest --output-on-failure && \
		printf "%s\n" "$(GREEN)✓ axon_uploader tests passed$(NC)"; \
	else \
		printf "%s\n" "$(YELLOW)⚠ axon_uploader not built (requires AWS SDK, enable with -DAXON_BUILD_UPLOADER=ON)$(NC)"; \
	fi

# Test axon_logging
test-logging: build-core
	@printf "%s\n" "$(YELLOW)Running axon_logging tests...$(NC)"
	@cd $(BUILD_DIR)/axon_logging && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_logging tests passed$(NC)"

# Test all C++ libraries
test-core: test-mcap test-uploader test-logging
	@printf "%s\n" "$(GREEN)✓ All C++ library tests passed$(NC)"

# =============================================================================
# Python Tests
# =============================================================================

.PHONY: test-python test-python-models test-python-client test-python-retry test-python-exceptions test-python-zenoh test-python-coverage

# Python package directory
PYTHON_CLIENT_DIR := python/axon_client

# Check if pytest is available
HAS_PYTEST := $(shell command -v pytest >/dev/null 2>&1; echo $$?)

# Test Python models
test-python-models:
	@if [ $(HAS_PYTEST) -ne 0 ]; then \
		printf "%s\n" "$(RED)Error: pytest not found. Install with: pip install pytest$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Running Python model tests...$(NC)"
	@cd $(PYTHON_CLIENT_DIR) && pytest tests/test_models.py -v
	@printf "%s\n" "$(GREEN)✓ Python model tests passed$(NC)"

# Test Python HTTP client
test-python-client:
	@if [ $(HAS_PYTEST) -ne 0 ]; then \
		printf "%s\n" "$(RED)Error: pytest not found. Install with: pip install pytest$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Running Python client tests...$(NC)"
	@cd $(PYTHON_CLIENT_DIR) && pytest tests/test_client.py tests/test_async_client.py -v
	@printf "%s\n" "$(GREEN)✓ Python client tests passed$(NC)"

# Test Python retry logic
test-python-retry:
	@if [ $(HAS_PYTEST) -ne 0 ]; then \
		printf "%s\n" "$(RED)Error: pytest not found. Install with: pip install pytest$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Running Python retry tests...$(NC)"
	@cd $(PYTHON_CLIENT_DIR) && pytest tests/test_retry.py -v
	@printf "%s\n" "$(GREEN)✓ Python retry tests passed$(NC)"

# Test Python exceptions
test-python-exceptions:
	@if [ $(HAS_PYTEST) -ne 0 ]; then \
		printf "%s\n" "$(RED)Error: pytest not found. Install with: pip install pytest$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Running Python exception tests...$(NC)"
	@cd $(PYTHON_CLIENT_DIR) && pytest tests/test_exceptions.py -v
	@printf "%s\n" "$(GREEN)✓ Python exception tests passed$(NC)"

# Test Python Zenoh publisher
test-python-zenoh:
	@if [ $(HAS_PYTEST) -ne 0 ]; then \
		printf "%s\n" "$(RED)Error: pytest not found. Install with: pip install pytest$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Running Python Zenoh publisher tests...$(NC)"
	@cd $(PYTHON_CLIENT_DIR) && pytest tests/test_zenoh_publisher.py -v
	@printf "%s\n" "$(GREEN)✓ Python Zenoh publisher tests passed$(NC)"

# Test all Python components
test-python: test-python-models test-python-client test-python-retry test-python-exceptions test-python-zenoh
	@printf "%s\n" "$(GREEN)✓ All Python tests passed$(NC)"

# Test Python with coverage
test-python-coverage:
	@if [ $(HAS_PYTEST) -ne 0 ]; then \
		printf "%s\n" "$(RED)Error: pytest not found. Install with: pip install pytest pytest-cov$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Running Python tests with coverage...$(NC)"
	@cd $(PYTHON_CLIENT_DIR) && pytest tests/ --cov=axon_client --cov-report=term-missing --cov-report=html
	@printf "%s\n" "$(GREEN)✓ Python coverage report: $(PYTHON_CLIENT_DIR)/htmlcov/index.html$(NC)"

# Coverage for axon_mcap
coverage-mcap:
	@printf "%s\n" "$(YELLOW)Building axon_mcap with coverage...$(NC)"
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_ENABLE_COVERAGE=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(BUILD_DIR)/axon_mcap && ctest --output-on-failure
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		cd $(BUILD_DIR) && \
		lcov --capture --directory . --output-file coverage_raw.info && \
		lcov --remove coverage_raw.info \
			'/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/c++/*' \
			--output-file coverage.info && \
		lcov --list coverage.info; \
		printf "%s\n" "$(GREEN)✓ Coverage report: $(BUILD_DIR)/coverage.info$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: lcov not found$(NC)"; \
		exit 1; \
	fi

# Coverage for axon_uploader
coverage-uploader:
	@printf "%s\n" "$(YELLOW)Building axon_uploader with coverage...$(NC)"
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_ENABLE_COVERAGE=ON \
			-DAXON_BUILD_UPLOADER=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(BUILD_DIR)/axon_uploader && ctest --output-on-failure -j1
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		cd $(BUILD_DIR) && \
		lcov --capture --directory . --output-file coverage_raw.info && \
		lcov --remove coverage_raw.info \
			'/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/c++/*' \
			--output-file coverage.info && \
		lcov --list coverage.info; \
		printf "%s\n" "$(GREEN)✓ Coverage report: $(BUILD_DIR)/coverage.info$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: lcov not found$(NC)"; \
		exit 1; \
	fi

# Coverage for axon_logging
coverage-logging:
	@printf "%s\n" "$(YELLOW)Building axon_logging with coverage...$(NC)"
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_ENABLE_COVERAGE=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(BUILD_DIR)/axon_logging && ctest --output-on-failure
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		cd $(BUILD_DIR) && \
		lcov --capture --directory . --output-file coverage_raw.info && \
		lcov --remove coverage_raw.info \
			'/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/c++/*' \
			--output-file coverage.info && \
		lcov --list coverage.info; \
		printf "%s\n" "$(GREEN)✓ Coverage report: $(BUILD_DIR)/coverage.info$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: lcov not found$(NC)"; \
		exit 1; \
	fi

# Coverage for all C++ libraries
coverage-core: coverage-mcap coverage-uploader coverage-logging
	@printf "%s\n" "$(GREEN)✓ All C++ library coverage reports generated$(NC)"

# =============================================================================
# Application Targets
# =============================================================================

.PHONY: app app-axon-recorder app-axon-config
.PHONY: test-axon-config

# Build all applications
app: build-core
	@printf "%s\n" "$(YELLOW)Building applications...$(NC)"
	@cmake --build $(BUILD_DIR) -j$(NPROC) --target axon_recorder axon_config
	@printf "%s\n" "$(GREEN)✓ All applications built$(NC)"

# Build axon_recorder (plugin loader library)
app-axon-recorder:
	@printf "%s\n" "$(YELLOW)Building axon_recorder...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON >/dev/null 2>&1 || \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON
	@cmake --build $(BUILD_DIR)/axon_recorder -j$(NPROC) --target axon_recorder
	@printf "%s\n" "$(GREEN)✓ axon_recorder built$(NC)"

# Build axon_config (robot configuration tool)
app-axon-config:
	@printf "%s\n" "$(YELLOW)Building axon_config...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON >/dev/null 2>&1 || \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON
	@cmake --build $(BUILD_DIR)/axon_config -j$(NPROC) --target axon_config
	@printf "%s\n" "$(GREEN)✓ axon_config built$(NC)"

# Test axon_config
test-axon-config: build-core
	@printf "%s\n" "$(YELLOW)Running axon_config tests...$(NC)"
	@cd build/axon_config && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_config tests passed$(NC)"

# =============================================================================
# Mock Middleware Targets
# =============================================================================

# Build mock middleware
build-mock:
	@printf "%s\n" "$(YELLOW)Building mock middleware...$(NC)"
	@mkdir -p middlewares/mock/src/mock_plugin/build
	@cd middlewares/mock/src/mock_plugin/build && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DCMAKE_INSTALL_PREFIX=$(PROJECT_ROOT)/middlewares/mock/install && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ Mock middleware built$(NC)"

# Test mock plugin E2E
test-mock-e2e: build-mock
	@printf "%s\n" "$(YELLOW)Running mock plugin E2E test...$(NC)"
	@cd middlewares/mock/src/mock_plugin/build && ./test_mock_plugin_e2e
	@printf "%s\n" "$(GREEN)✓ Mock plugin E2E test passed$(NC)"

# Test mock plugin load
test-mock-load: build-mock
	@printf "%s\n" "$(YELLOW)Running mock plugin load test...$(NC)"
	@cd middlewares/mock/src/mock_plugin/build && \
		./test_mock_plugin_load ./libmock_plugin.so
	@printf "%s\n" "$(GREEN)✓ Mock plugin load test passed$(NC)"

# Test mock middleware integration with axon_recorder
test-mock-integration: build-mock build-core
	@printf "%s\n" "$(YELLOW)Running mock middleware integration E2E test...$(NC)"
	@cd middlewares/mock && ./test_e2e_with_mock.sh
	@printf "%s\n" "$(GREEN)✓ Mock middleware integration test passed$(NC)"

# Run all mock middleware tests
test-mock-all: build-mock
	@printf "%s\n" "$(YELLOW)Running all mock middleware tests...$(NC)"
	@cd middlewares/mock && ./test_full_workflow.sh
	@printf "%s\n" "$(GREEN)✓ All mock middleware tests passed$(NC)"

# =============================================================================
# ROS Middleware Targets
# =============================================================================

.PHONY: build build-ros1 build-ros2 build-zenoh

# Main build target (ROS)
build:
	@if [ "$(ROS_VERSION)" = "1" ]; then \
		$(MAKE) build-ros1; \
	else \
		$(MAKE) build-ros2; \
	fi
	@printf "%s\n" "$(GREEN)✓ Build complete!$(NC)"

# Build for ROS1
build-ros1:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_BUILD_ROS1_PLUGIN=ON \
			-DAXON_BUILD_ROS2_PLUGIN=OFF \
			-DAXON_BUILD_ZENOH_PLUGIN=OFF && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ ROS1 plugin built$(NC)"

# Build for ROS2
build-ros2:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_BUILD_ROS1_PLUGIN=OFF \
			-DAXON_BUILD_ROS2_PLUGIN=ON \
			-DAXON_BUILD_ZENOH_PLUGIN=OFF && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ ROS2 plugin built$(NC)"

# Build for Zenoh plugin
build-zenoh:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_BUILD_ROS1_PLUGIN=OFF \
			-DAXON_BUILD_ROS2_PLUGIN=OFF \
			-DAXON_BUILD_ZENOH_PLUGIN=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ Zenoh plugin built$(NC)"

# =============================================================================
# General Targets
# =============================================================================

.PHONY: test clean install debug release

# Library tests (legacy target - defaults to C++ tests)
test: test-core

# Clean all build artifacts
clean:
	@printf "%s\n" "$(YELLOW)Cleaning build artifacts...$(NC)"
	@rm -rf $(BUILD_DIR) $(COVERAGE_DIR)
	@cd middlewares/ros2 && rm -rf build install log 2>/dev/null || true
	@cd middlewares/ros1 && catkin clean --yes 2>/dev/null || true
	@rm -rf middlewares/mock/src/mock_plugin/build middlewares/mock/install 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ All build artifacts cleaned$(NC)"

# Install target
install: build
	@printf "%s\n" "$(YELLOW)Installing package...$(NC)"
	@cd $(BUILD_DIR) && cmake --install .
	@printf "%s\n" "$(GREEN)✓ Package installed$(NC)"

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=Debug build

# Release build (default)
release:
	@$(MAKE) BUILD_TYPE=Release build

# =============================================================================
# Docker Build & Test Targets
# =============================================================================

.PHONY: docker-build-ros1 docker-test-ros1
.PHONY: docker-build-ros2-humble docker-test-ros2-humble
.PHONY: docker-build-ros2-jazzy docker-test-ros2-jazzy
.PHONY: docker-build-ros2-rolling docker-test-ros2-rolling
.PHONY: docker-build docker-test-all docker-test docker-test-compose
.PHONY: docker-build-cpp docker-test-cpp
.PHONY: docker-build-zenoh docker-test-zenoh docker-test-zenoh-integration
.PHONY: docker-test-ros docker-test-e2e docker-test-e2e-ros1 docker-test-e2e-ros2-humble docker-test-e2e-ros2-jazzy docker-test-e2e-ros2-rolling docker-test-e2e-all
.PHONY: docker-coverage

docker-build-ros1:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS1...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros1 -t axon:ros1 . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros1: docker-build-ros1
	@printf "%s\n" "$(YELLOW)Running tests in ROS1 Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=noetic \
		-e ROS_VERSION=1 \
		axon:ros1 \
		/usr/local/bin/run_integration.sh --clean
	@printf "%s\n" "$(GREEN)✓ ROS1 tests passed$(NC)"

docker-build-ros2-humble:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS2 Humble...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros2.humble -t axon:ros2-humble . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros2-humble: docker-build-ros2-humble
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Humble Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=humble \
		-e ROS_VERSION=2 \
		axon:ros2-humble \
		/usr/local/bin/run_integration.sh --clean
	@printf "%s\n" "$(GREEN)✓ ROS2 Humble tests passed$(NC)"

docker-build-ros2-jazzy:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS2 Jazzy...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros2.jazzy -t axon:ros2-jazzy . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros2-jazzy: docker-build-ros2-jazzy
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Jazzy Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=jazzy \
		-e ROS_VERSION=2 \
		axon:ros2-jazzy \
		/usr/local/bin/run_integration.sh --clean
	@printf "%s\n" "$(GREEN)✓ ROS2 Jazzy tests passed$(NC)"

docker-build-ros2-rolling:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS2 Rolling...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@DOCKER_BUILDKIT=1 docker build -f docker/Dockerfile.ros2.rolling -t axon:ros2-rolling . || \
		(echo "$(RED)Build failed. See docker/TROUBLESHOOTING.md$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ Docker image built$(NC)"

docker-test-ros2-rolling: docker-build-ros2-rolling
	@printf "%s\n" "$(YELLOW)Running tests in ROS2 Rolling Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=rolling \
		-e ROS_VERSION=2 \
		axon:ros2-rolling \
		/usr/local/bin/run_integration.sh --clean
	@printf "%s\n" "$(GREEN)✓ ROS2 Rolling tests passed$(NC)"

docker-build: docker-build-ros1 docker-build-ros2-humble docker-build-ros2-jazzy docker-build-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All Docker images built$(NC)"

# Run tests in all Docker containers
docker-test-all: docker-test-ros1 docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All Docker tests passed!$(NC)"

# Quick Docker test (single version)
docker-test:
	@if [ "$(ROS_VERSION)" = "1" ]; then \
		$(MAKE) docker-test-ros1; \
	else \
		$(MAKE) docker-test-ros2-humble; \
	fi

# Run tests using docker compose (all versions in parallel)
docker-test-compose:
	@printf "%s\n" "$(YELLOW)Running tests in all Docker containers (docker compose)...$(NC)"
	@cd docker && docker compose -f docker-compose.test.yml up --build --abort-on-container-exit
	@printf "%s\n" "$(GREEN)✓ All Docker tests passed!$(NC)"

# =============================================================================
# Code Quality Targets
# =============================================================================

.PHONY: format lint

# Format code
format:
	@if ! command -v $(CLANG_FORMAT) >/dev/null 2>&1; then \
		printf "%s\n" "$(YELLOW)⚠ $(CLANG_FORMAT) not found, installing...$(NC)"; \
		printf "%s\n" "$(YELLOW)Installing dependencies...$(NC)"; \
		sudo apt install -y lsb-release wget software-properties-common gnupg >/dev/null 2>&1; \
		wget -qO- https://apt.llvm.org/llvm.sh | sudo bash -s $(CLANG_FORMAT_VERSION) >/dev/null 2>&1 && \
		sudo apt install -y $(CLANG_FORMAT) >/dev/null 2>&1; \
		printf "%s\n" "$(GREEN)✓ Installed $(CLANG_FORMAT)$(NC)"; \
	fi
	@printf "%s\n" "$(YELLOW)Formatting C/C++ code...$(NC)"
	@printf "%s\n" "$(YELLOW)  Using: $(CLANG_FORMAT)$(NC)"
	@printf "%s\n" "$(YELLOW)  Formatting core/ libraries...$(NC)"
	@find core \
		\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
		! -path "*/build/*" ! -path "*/build_*/*" -print0 2>/dev/null | \
		xargs -0 $(CLANG_FORMAT) -i
	@printf "%s\n" "$(YELLOW)  Formatting middlewares/ code...$(NC)"
	@find middlewares \
		\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
		! -path "*/build/*" ! -path "*/install/*" ! -path "*/devel/*" \
		! -path "*/depthlitez/*" -print0 2>/dev/null | \
		xargs -0 $(CLANG_FORMAT) -i
	@printf "%s\n" "$(YELLOW)  Formatting apps/ code...$(NC)"
	@find apps \
		\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
		! -path "*/build/*" -print0 2>/dev/null | \
		xargs -0 $(CLANG_FORMAT) -i
	@printf "%s\n" "$(GREEN)✓ C/C++ code formatted$(NC)"

# Lint code
lint:
	@printf "%s\n" "$(YELLOW)Linting C++ code...$(NC)"
	@if command -v cppcheck >/dev/null 2>&1; then \
		find core middlewares apps \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" \
			! -path "*/test/*" \
			! -path "*/install/*" \
			! -path "*/devel/*" \
			! -path "*/depthlitez/*" \
			-print0 2>/dev/null | \
		xargs -0 cppcheck --enable=all \
			--suppress=missingInclude \
			-Icore/axon_mcap/include \
			-Icore/axon_uploader/include \
			-Icore/axon_logging/include \
			-Iapps/axon_recorder; \
		printf "%s\n" "$(GREEN)✓ C++ linting passed$(NC)"; \
	else \
		printf "%s\n" "$(YELLOW)⚠ cppcheck not found, skipping$(NC)"; \
	fi


# =============================================================================
# Coverage Targets (ROS)
# =============================================================================

.PHONY: coverage-ros2 coverage-html coverage clean-coverage

# Build with coverage and run tests (ROS 2)
coverage-ros2:
	@printf "%s\n" "$(YELLOW)Building with coverage instrumentation (ROS 2)...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		exit 1; \
	fi
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR) $(COVERAGE_DIR)
	@source /opt/ros/$${ROS_DISTRO}/setup.bash && \
		cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_ENABLE_COVERAGE=ON \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_BUILD_ROS2_PLUGIN=ON \
			-DAXON_BUILD_ROS1_PLUGIN=OFF \
			-DAXON_BUILD_ZENOH_PLUGIN=OFF && \
		cmake --build . -j$(NPROC) && \
		ctest --output-on-failure || true
	@printf "%s\n" "$(GREEN)✓ Tests completed, generating coverage report...$(NC)"
	@LCOV_MAJOR=$$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9]+' || echo "1"); \
	IGNORE_FLAGS=""; \
	if [ "$$LCOV_MAJOR" -ge 2 ]; then IGNORE_FLAGS="--ignore-errors mismatch,unused"; fi; \
	lcov --capture \
		--directory $(BUILD_DIR) \
		--output-file $(COVERAGE_DIR)/coverage_raw.info \
		--rc lcov_branch_coverage=1 \
		$$IGNORE_FLAGS || true; \
	lcov --remove $(COVERAGE_DIR)/coverage_raw.info \
		'/usr/*' '/opt/*' '*/_deps/*' '*/generated/*' '*/googletest/*' \
		'*/gtest/*' '*/gmock/*' \
		'*/test/*' \
		'*/rosidl_typesupport_cpp/*' \
		'*/rosidl_typesupport_introspection_cpp/*' \
		'*/rosidl_generator_cpp/*' \
		'*/mcap-*/*' '*/mcap/include/*' \
		--output-file $(COVERAGE_DIR)/coverage.info \
		--rc lcov_branch_coverage=1 \
		$$IGNORE_FLAGS || cp $(COVERAGE_DIR)/coverage_raw.info $(COVERAGE_DIR)/coverage.info
	@lcov --list $(COVERAGE_DIR)/coverage.info || true
	@printf "%s\n" "$(GREEN)✓ Coverage report: $(COVERAGE_DIR)/coverage.info$(NC)"

# Generate HTML coverage report
coverage-html: coverage-ros2
	@printf "%s\n" "$(YELLOW)Generating HTML coverage report...$(NC)"
	@mkdir -p $(COVERAGE_DIR)/html
	@genhtml $(COVERAGE_DIR)/coverage.info \
		--output-directory $(COVERAGE_DIR)/html \
		--title "Axon Test Coverage" \
		--legend \
		--branch-coverage || true
	@printf "%s\n" "$(GREEN)✓ HTML report: $(COVERAGE_DIR)/html/index.html$(NC)"

# Run coverage in Docker (ROS 2 Humble)
docker-coverage: docker-build-ros2-humble
	@printf "%s\n" "$(YELLOW)Running coverage in ROS2 Humble Docker container...$(NC)"
	@mkdir -p $(COVERAGE_DIR)
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/coverage:/workspace/coverage \
		-e ROS_DISTRO=humble \
		-e ROS_VERSION=2 \
		axon:ros2-humble \
		/usr/local/bin/run_integration.sh --coverage --coverage-output /workspace/coverage
	@printf "%s\n" "$(GREEN)✓ Coverage report: $(COVERAGE_DIR)/coverage.info$(NC)"

# Clean coverage data
clean-coverage:
	@printf "%s\n" "$(YELLOW)Cleaning coverage data...$(NC)"
	@rm -rf $(COVERAGE_DIR)
	@find $(BUILD_DIR) -name "*.gcda" -delete 2>/dev/null || true
	@find $(BUILD_DIR) -name "*.gcno" -delete 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Coverage data cleaned$(NC)"

# Default coverage target
coverage: coverage-ros2
	@printf "%s\n" "$(GREEN)✓ Coverage complete$(NC)"

# =============================================================================
# Utility Targets
# =============================================================================

.PHONY: check-ros build-safe test-safe verify

# Check if ROS is sourced
check-ros:
	@if [ -z "$$ROS_DISTRO" ] && [ -z "$$ROS_VERSION" ]; then \
		echo "$(RED)Error: ROS environment not set.$(NC)"; \
		echo "For ROS1: source /opt/ros/noetic/setup.bash"; \
		echo "For ROS2: source /opt/ros/humble/setup.bash (or jazzy/rolling)"; \
		exit 1; \
	fi

# Build with ROS check
build-safe: check-ros build

# Test with ROS check
test-safe: check-ros test

# Verify build (build + test)
verify: build test
	@printf "%s\n" "$(GREEN)✓ Verification complete - build and tests passed!$(NC)"

# =============================================================================
# Convenient Aliases
# =============================================================================

.PHONY: cov-humble cov cov-html clean-cov

# Coverage shortcuts
cov-humble: docker-coverage
cov: docker-coverage
cov-html: docker-coverage
	@open $(COVERAGE_DIR)/html/index.html 2>/dev/null || true

clean-cov: clean-coverage

# =============================================================================
# CI Testing Targets (All Docker-based for Consistency)
# =============================================================================
# These targets mirror what CI runs, allowing developers to test locally
# before pushing. All tests run in Docker to match CI environment exactly.
# See .github/workflows/ for CI workflow definitions.

.PHONY: ci ci-quick ci-all ci-cpp ci-cpp-unit ci-cpp-integration ci-ros ci-ros1 ci-ros2 ci-e2e ci-docker
.PHONY: ci-coverage-cpp ci-coverage-ros ci-coverage-recorder ci-coverage

# ci: Quick CI check (format + lint + C++ tests in Docker)
ci: format-ci lint docker-test-cpp
	@printf "%s\n" "$(GREEN)✓ Quick CI checks passed!$(NC)"
	@printf "%s\n" "$(YELLOW)Run 'make ci-all' for full CI validation (including ROS and E2E)$(NC)"

# ci-quick: Fastest CI check (format + lint only, no tests)
ci-quick: format-ci lint
	@printf "%s\n" "$(GREEN)✓ Format and lint checks passed$(NC)"
	@printf "%s\n" "$(YELLOW)Run 'make ci' for quick tests or 'make ci-all' for full validation$(NC)"

# ci-all: Run complete CI test suite (all checks in Docker)
ci-all: format-ci lint docker-test-all docker-test-ros
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║     ✓ ALL CI CHECKS PASSED!                               ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"

# ci-cpp: C++ library tests in Docker (unit + integration)
ci-cpp: docker-test-cpp
	@printf "%s\n" "$(GREEN)✓ C++ library tests passed$(NC)"
	@printf "%s\n" "$(YELLOW)For coverage: make ci-coverage-cpp$(NC)"

# ci-cpp-unit: C++ unit tests only in Docker
ci-cpp-unit: docker-build-cpp
	@printf "%s\n" "$(YELLOW)Running C++ unit tests in Docker...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e AXON_ENABLE_COVERAGE=OFF \
		axon:cpp-test \
		/bin/bash -c ' \
			for lib in axon_mcap axon_logging; do \
				echo "Building $$lib..." && \
				cd /workspace/axon && \
				mkdir -p build/$$lib && \
				cd build/$$lib && \
				cmake ../../core/$$lib \
					-DCMAKE_BUILD_TYPE=Release \
					-DAXON_REPO_ROOT=/workspace/axon \
					-DAXON_BUILD_TESTS=ON && \
				make -j$$(nproc) && \
				ctest --output-on-failure -L unit || exit 1; \
			done \
		'
	@printf "%s\n" "$(GREEN)✓ C++ unit tests passed$(NC)"

# ci-cpp-integration: C++ integration tests in Docker (with MinIO)
ci-cpp-integration: docker-build-cpp
	@printf "%s\n" "$(YELLOW)Running C++ integration tests in Docker with MinIO...$(NC)"
	@docker run -d --name minio-ci \
		--network host \
		-e MINIO_ROOT_USER=minioadmin \
		-e MINIO_ROOT_PASSWORD=minioadmin \
		quay.io/minio/minio server /data --console-address ":9001" || true
	@sleep 3
	@docker run --rm --network host \
		-e MC_HOST_local=http://minioadmin:minioadmin@localhost:9000 \
		quay.io/minio/mc mb local/axon-raw-data --ignore-existing || true
	@DOCKER_BUILDKIT=1 docker run --rm \
		--network host \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e AWS_ACCESS_KEY_ID=minioadmin \
		-e AWS_SECRET_ACCESS_KEY=minioadmin \
		axon:cpp-test \
		/bin/bash -c '\
			cd /workspace/axon && \
			mkdir -p build/axon_uploader && \
			cd build/axon_uploader && \
			cmake ../../core/axon_uploader \
				-DCMAKE_BUILD_TYPE=Debug \
				-DAXON_REPO_ROOT=/workspace/axon \
				-DAXON_UPLOADER_BUILD_TESTS=ON && \
			make -j$$(nproc) && \
			./test_edge_uploader \
		' || \
		(docker stop minio-ci 2>/dev/null || true && \
		 docker rm minio-ci 2>/dev/null || true && \
		 exit 1)
	@docker stop minio-ci 2>/dev/null || true
	@docker rm minio-ci 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ C++ integration tests passed$(NC)"

# ci-ros: ROS tests in Docker (all distros)
ci-ros: docker-test-ros
	@printf "%s\n" "$(GREEN)✓ ROS tests passed$(NC)"

# ci-ros1: ROS1 tests in Docker
ci-ros1: docker-test-ros1
	@printf "%s\n" "$(GREEN)✓ ROS1 tests passed$(NC)"

# ci-ros2: ROS2 tests in Docker (all distros)
ci-ros2: docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ ROS2 tests passed$(NC)"

# ci-e2e: E2E tests in Docker (ROS1 + ROS2 Humble)
ci-e2e: docker-test-e2e-ros1 docker-test-e2e-ros2-humble
	@printf "%s\n" "$(GREEN)✓ E2E tests passed$(NC)"

# ci-docker: Alias for docker-test-all
ci-docker: docker-test-all
	@printf "%s\n" "$(GREEN)✓ All Docker tests passed$(NC)"

# ci-coverage-cpp: C++ coverage in Docker
ci-coverage-cpp: docker-build-cpp
	@printf "%s\n" "$(YELLOW)Running C++ coverage in Docker...$(NC)"
	@mkdir -p $(COVERAGE_DIR)
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/coverage:/workspace/coverage \
		-e AXON_ENABLE_COVERAGE=ON \
		axon:cpp-test \
		/bin/bash -c ' \
			for lib in axon_mcap axon_uploader axon_logging; do \
				echo "Building $$lib with coverage..." && \
				cd /workspace/axon && \
				mkdir -p build/$$lib && \
				cd build/$$lib && \
				cmake ../../core/$$lib \
					-DCMAKE_BUILD_TYPE=Debug \
					-DAXON_ENABLE_COVERAGE=ON \
					-DAXON_BUILD_TESTS=ON \
					-DAXON_REPO_ROOT=/workspace/axon && \
				make -j$$(nproc) && \
				ctest --output-on-failure && \
				lcov --capture --directory . \
					--output-file /workspace/coverage/$${lib}_coverage.info \
					--rc lcov_branch_coverage=1 || true; \
			done \
		' || \
		(printf "%s\n" "$(RED)✗ Coverage tests failed$(NC)" && exit 1)
	@printf "%s\n" "$(GREEN)✓ C++ coverage complete: $(COVERAGE_DIR)/*.info$(NC)"
	@printf "%s\n" "$(YELLOW)View HTML report: make coverage-html-cpp$(NC)"


# ci-coverage-ros: ROS2 coverage in Docker (Humble only for speed)
ci-coverage-ros: docker-coverage
	@printf "%s\n" "$(GREEN)✓ ROS2 coverage complete$(NC)"

# ci-coverage-recorder: Axon recorder coverage (local, no ROS required)
ci-coverage-recorder:
	@printf "%s\n" "$(YELLOW)Building axon_recorder with coverage...$(NC)"
	@rm -rf apps/axon_recorder/build
	@mkdir -p apps/axon_recorder/build $(COVERAGE_DIR)
	@cd apps/axon_recorder/build && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_BUILD_TESTS=ON \
			-DAXON_ENABLE_COVERAGE=ON \
			-DAXON_BUILD_WITH_CORE=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd apps/axon_recorder/build/test && for test in test_*; do ./$$test 2>&1 | grep -E "PASSED|FAILED" || true; done
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		LCOV_MAJOR=$$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9]+' || echo "1"); \
		IGNORE_FLAGS=""; \
		if [ "$$LCOV_MAJOR" -ge 2 ]; then IGNORE_FLAGS="--ignore-errors mismatch,unused"; fi; \
		lcov --capture \
			--directory apps/axon_recorder/build \
			--output-file $(COVERAGE_DIR)/axon_recorder_coverage.info \
			--rc lcov_branch_coverage=1 \
			$$IGNORE_FLAGS || true; \
		lcov --remove $(COVERAGE_DIR)/axon_recorder_coverage.info \
			'/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/generated/*' \
			'*/googletest/*' '*/gtest/*' '*/gmock/*' \
			'*/single_include/*' \
			'*/c++/*' \
			--output-file $(COVERAGE_DIR)/axon_recorder_coverage.info \
			--rc lcov_branch_coverage=1 \
			$$IGNORE_FLAGS || true; \
		lcov --list $(COVERAGE_DIR)/axon_recorder_coverage.info || true; \
		printf "%s\n" "$(GREEN)✓ Axon recorder coverage: $(COVERAGE_DIR)/axon_recorder_coverage.info$(NC)"; \
	else \
		printf "%s\n" "$(YELLOW)Warning: lcov not found - skipping coverage report generation$(NC)"; \
		printf "%s\n" "$(YELLOW)Tests were built and run successfully, but coverage report requires lcov$(NC)"; \
	fi

# ci-coverage: All coverage tests (C++ + ROS2 + Recorder)
ci-coverage: ci-coverage-cpp ci-coverage-ros ci-coverage-recorder
	@printf "%s\n" "$(YELLOW)Merging coverage reports...$(NC)"
	@lcov -o $(COVERAGE_DIR)/coverage_merged.info \
		-a $(COVERAGE_DIR)/axon_mcap_coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/axon_uploader_coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/axon_logging_coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/axon_recorder_coverage.info 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ All coverage complete: $(COVERAGE_DIR)/coverage_merged.info$(NC)"
	@printf "%s\n" "$(YELLOW)View HTML report: make coverage-html$(NC)"

# =============================================================================
# E2E Test Targets
# =============================================================================

.PHONY: e2e

e2e:
	@printf "%s\n" "$(YELLOW)Running E2E tests locally...$(NC)"
	@if [ ! -f "$(BUILD_DIR)/axon_recorder/axon_recorder" ] && \
	   [ ! -f "$(BUILD_DIR)/apps/axon_recorder/axon_recorder" ] && \
	   [ ! -f "apps/axon_recorder/build/axon_recorder" ]; then \
		printf "%s\n" "$(RED)Error: axon_recorder not built.$(NC)"; \
		printf "%s\n" "$(YELLOW)Build with: make build-core$(NC)"; \
		exit 1; \
	fi
	@cd apps/axon_recorder/test/e2e && ./run_e2e_tests.sh
	@printf "%s\n" "$(GREEN)✓ E2E tests passed$(NC)"

# =============================================================================
# Docker Test Targets (Mirroring CI)
# =============================================================================

# docker-test-cpp: C++ library tests in Docker
docker-test-cpp: docker-build-cpp
	@printf "%s\n" "$(YELLOW)Running C++ tests in Docker...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		axon:cpp-test \
		/bin/bash -c ' \
			for lib in axon_mcap axon_uploader axon_logging; do \
				echo "Building $$lib..." && \
				cd /workspace/axon && \
				mkdir -p build/$$lib && \
				cd build/$$lib && \
				cmake ../../core/$$lib \
					-DCMAKE_BUILD_TYPE=Release \
					-DAXON_REPO_ROOT=/workspace/axon \
					-DAXON_BUILD_TESTS=ON && \
				make -j$$(nproc) && \
				ctest --output-on-failure || exit 1; \
			done \
		'
	@printf "%s\n" "$(GREEN)✓ C++ tests passed$(NC)"

# docker-build-cpp: Build C++ test Docker image
docker-build-cpp:
	@printf "%s\n" "$(YELLOW)Building C++ test Docker image...$(NC)"
	@DOCKER_BUILDKIT=1 docker build \
		-f docker/Dockerfile.cpp-test \
		-t axon:cpp-test \
		--build-arg BUILDKIT_INLINE_CACHE=1 \
		.
	@printf "%s\n" "$(GREEN)✓ C++ test image built$(NC)"

# docker-build-zenoh: Build Zenoh test Docker image
docker-build-zenoh:
	@printf "%s\n" "$(YELLOW)Building Zenoh test Docker image...$(NC)"
	@DOCKER_BUILDKIT=1 docker build \
		-f docker/Dockerfile.zenoh \
		-t axon:zenoh \
		--build-arg BUILDKIT_INLINE_CACHE=1 \
		.
	@printf "%s\n" "$(GREEN)✓ Zenoh test image built$(NC)"

# docker-test-zenoh: Run Zenoh plugin tests in Docker
docker-test-zenoh: docker-build-zenoh
	@printf "%s\n" "$(YELLOW)Running Zenoh plugin tests in Docker...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-w /workspace/axon/middlewares/zenoh \
		axon:zenoh \
		bash -c "rm -rf build && mkdir build && cd build && cmake .. && make && make test"
	@printf "%s\n" "$(GREEN)✓ Zenoh plugin tests passed$(NC)"

# docker-test-zenoh-integration: Run Axon + Zenoh integration tests
docker-test-zenoh-integration: docker-build-zenoh
	@printf "%s\n" "$(YELLOW)Running Axon + Zenoh integration tests...$(NC)"
	@cd docker && docker-compose -f docker-compose.zenoh.yml build
	@cd docker && docker-compose -f docker-compose.zenoh.yml up --abort-on-container-exit --exit-code-from test-subscriber
	@cd docker && docker-compose -f docker-compose.zenoh.yml down -v
	@printf "%s\n" "$(GREEN)✓ Zenoh integration tests passed$(NC)"

# docker-test-ros: Run all ROS tests in Docker
docker-test-ros: docker-test-ros1 docker-test-ros2-humble
	@printf "%s\n" "$(GREEN)✓ All ROS tests passed$(NC)"

# docker-test-e2e: Run all E2E tests in Docker
docker-test-e2e: docker-test-e2e-ros1 docker-test-e2e-ros2-humble
	@printf "%s\n" "$(GREEN)✓ All E2E tests passed$(NC)"

# docker-test-e2e-ros1: Run E2E tests in ROS1 Docker container
docker-test-e2e-ros1: docker-build-ros1
	@printf "%s\n" "$(YELLOW)Running E2E tests in ROS1 Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=noetic \
		-e ROS_VERSION=1 \
		--network host \
		axon:ros1 \
		/usr/local/bin/run_e2e_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS1 E2E tests passed$(NC)"

# docker-test-e2e-ros2-humble: Run E2E tests in ROS2 Humble Docker container
docker-test-e2e-ros2-humble: docker-build-ros2-humble
	@printf "%s\n" "$(YELLOW)Running E2E tests in ROS2 Humble Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=humble \
		-e ROS_VERSION=2 \
		--network host \
		axon:ros2-humble \
		/usr/local/bin/run_e2e_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Humble E2E tests passed$(NC)"

# docker-test-e2e-ros2-jazzy: Run E2E tests in ROS2 Jazzy Docker container
docker-test-e2e-ros2-jazzy: docker-build-ros2-jazzy
	@printf "%s\n" "$(YELLOW)Running E2E tests in ROS2 Jazzy Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=jazzy \
		-e ROS_VERSION=2 \
		--network host \
		axon:ros2-jazzy \
		/usr/local/bin/run_e2e_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Jazzy E2E tests passed$(NC)"

# docker-test-e2e-ros2-rolling: Run E2E tests in ROS2 Rolling Docker container
docker-test-e2e-ros2-rolling: docker-build-ros2-rolling
	@printf "%s\n" "$(YELLOW)Running E2E tests in ROS2 Rolling Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=rolling \
		-e ROS_VERSION=2 \
		--network host \
		axon:ros2-rolling \
		/usr/local/bin/run_e2e_tests.sh
	@printf "%s\n" "$(GREEN)✓ ROS2 Rolling E2E tests passed$(NC)"

# docker-test-e2e-all: Run E2E tests for all ROS distributions
docker-test-e2e-all: docker-test-e2e-ros1 docker-test-e2e-ros2-humble docker-test-e2e-ros2-jazzy docker-test-e2e-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All E2E tests passed$(NC)"

# =============================================================================
# Utility Targets for CI Testing
# =============================================================================

.PHONY: coverage-html-cpp coverage-merge format-ci

# coverage-html-cpp: Generate HTML coverage report for C++
coverage-html-cpp:
	@printf "%s\n" "$(YELLOW)Generating C++ coverage HTML report...$(NC)"
	@genhtml $(COVERAGE_DIR)/axon_mcap_coverage.info \
		$(COVERAGE_DIR)/axon_uploader_coverage.info \
		$(COVERAGE_DIR)/axon_logging_coverage.info \
		-o $(COVERAGE_DIR)/html --rc lcov_branch_coverage=1 2>/dev/null || \
		genhtml $(COVERAGE_DIR)/coverage_merged.info \
		-o $(COVERAGE_DIR)/html --rc lcov_branch_coverage=1
	@printf "%s\n" "$(GREEN)✓ Coverage report: $(COVERAGE_DIR)/html/index.html$(NC)"

# format-ci: Check code formatting (CI style check, no modifications)
# coverage-merge: Merge coverage from all libraries into one report
coverage-merge:
	@printf "%s\n" "$(YELLOW)Merging coverage reports...$(NC)"
	@mkdir -p $(COVERAGE_DIR)
	@lcov -o $(COVERAGE_DIR)/coverage_merged.info \
		-a $(COVERAGE_DIR)/axon_mcap_coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/axon_uploader_coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/axon_logging_coverage.info 2>/dev/null || true \
		-a $(COVERAGE_DIR)/coverage.info 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Coverage merged: $(COVERAGE_DIR)/coverage_merged.info$(NC)"

# format-ci: Check code formatting (CI style check, no modifications)
format-ci:
	@if ! command -v $(CLANG_FORMAT) >/dev/null 2>&1; then \
		printf "%s\n" "$(RED)✗ $(CLANG_FORMAT) not found$(NC)"; \
		printf "%s\n" "$(YELLOW)Please install: wget -qO- https://apt.llvm.org/llvm.sh | sudo bash -s $(CLANG_FORMAT_VERSION)$(NC)"; \
		exit 1; \
	fi
	@printf "%s\n" "$(YELLOW)Checking code formatting...$(NC)"
	@printf "%s\n" "$(YELLOW)  Using: $(CLANG_FORMAT)$(NC)"
	@find core middlewares/ros1 middlewares/ros2 middlewares/filters apps \
		\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
		! -path "*/build/*" ! -path "*/build_*/*" ! -path "*/install/*" ! -path "*/devel/*" \
		! -path "*/depthlitez/*" \
		-print0 2>/dev/null | \
		xargs -0 $(CLANG_FORMAT) --dry-run --Werror > /dev/null 2>&1 || \
		(printf "%s\n" "$(RED)✗ Code formatting check failed$(NC)" && \
		 printf "%s\n" "$(YELLOW)Run 'make format' to fix formatting issues$(NC)" && \
			printf "%s\n" "$(YELLOW)Or format files individually: $(CLANG_FORMAT) -i <file>$(NC)" && \
			exit 1)
	@printf "%s\n" "$(GREEN)✓ Code formatting check passed$(NC)"
