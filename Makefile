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
	@printf "%s\n" "$(YELLOW)General:$(NC)"
	@printf "%s\n" "  $(BLUE)make clean$(NC)             - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make install$(NC)           - Install package"
	@printf "%s\n" "  $(BLUE)make debug/release$(NC)      - Build with Debug/Release config"
	@echo ""
	@printf "%s\n" "$(YELLOW)Code quality:$(NC)"
	@printf "%s\n" "  $(BLUE)make format$(NC)            - Format with clang-format"
	@printf "%s\n" "  $(BLUE)make lint$(NC)              - Lint with cppcheck"
	@echo ""
	@printf "%s\n" "$(YELLOW)CI Testing (local, no Docker):$(NC)"
	@printf "%s\n" "  $(BLUE)make ci-local$(NC)          - Quick CI (format + lint + C++ tests)"
	@printf "%s\n" "  $(BLUE)make ci-local-quick$(NC)    - Fastest check (format + lint only)"
	@printf "%s\n" "  $(BLUE)make ci-local-cpp-unit$(NC) - C++ unit tests"
	@printf "%s\n" "  $(BLUE)make ci-local-cpp-integration$(NC) - C++ integration (with MinIO)"
	@printf "%s\n" "  $(BLUE)make ci-local-reuse$(NC)    - REUSE license compliance"
	@echo ""
	@printf "%s\n" "$(YELLOW)CI Docker Testing:$(NC)"
	@printf "%s\n" "  $(BLUE)make ci-docker$(NC)         - Quick CI in Docker (format + lint + C++ tests)"
	@printf "%s\n" "  $(BLUE)make ci-docker-all$(NC)     - Full CI validation (all tests in Docker)"
	@printf "%s\n" "  $(BLUE)make ci-docker-cpp$(NC)     - C++ tests in Docker"
	@printf "%s\n" "  $(BLUE)make ci-docker-ros$(NC)     - ROS tests in Docker (all distros)"
	@printf "%s\n" "  $(BLUE)make e2e-docker$(NC)       - All E2E tests in Docker"
	@printf "%s\n" "  $(BLUE)make ci-docker-zenoh$(NC)   - Zenoh tests in Docker"
	@printf "%s\n" "  $(BLUE)make ci-docker-coverage$(NC) - Coverage tests in Docker"
	@echo ""
	@printf "%s\n" "$(YELLOW)E2E Testing:$(NC)"
	@printf "%s\n" "  $(BLUE)make e2e$(NC)                - E2E tests (local, requires build)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Utility:$(NC)"
	@printf "%s\n" "  $(BLUE)make coverage-merge$(NC)    - Merge all coverage reports"
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
.PHONY: docker-build-cpp docker-build-zenoh docker-coverage

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
	@./scripts/format-code.sh

# Lint code
lint:
	@./scripts/lint-code.sh


# =============================================================================
# Coverage Targets (ROS)
# =============================================================================

.PHONY: coverage-ros2 coverage-html coverage clean-coverage

# Build with coverage and run tests (ROS 2)
coverage-ros2:
	@./scripts/run-coverage.sh ros2

# Generate HTML coverage report
coverage-html:
	@./scripts/run-coverage.sh all --html

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
	@./scripts/run-coverage.sh --clean

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
# CI Testing Targets
# =============================================================================

.PHONY: ci-local ci-local-quick
.PHONY: ci-local-cpp-unit ci-local-cpp-integration ci-local-reuse
.PHONY: ci-docker ci-docker-all ci-docker-cpp ci-docker-ros ci-docker-zenoh ci-docker-coverage
.PHONY: ci-docker-ros1 ci-docker-ros2
.PHONY: ci-docker-coverage-cpp ci-docker-coverage-ros
.PHONY: e2e-docker e2e-docker-ros1 e2e-docker-ros2-humble e2e-docker-ros2-jazzy e2e-docker-ros2-rolling

# =============================================================================
# Local CI (no Docker)
# =============================================================================

# ci-local: Quick CI check (format + lint + C++ tests, no Docker)
ci-local: format-ci lint ci-local-cpp-unit
	@printf "%s\n" "$(GREEN)✓ Quick CI checks passed!$(NC)"
	@printf "%s\n" "$(YELLOW)Run 'make ci-docker' for Docker-based tests$(NC)"

# ci-local-quick: Fastest CI check (format + lint only)
ci-local-quick: format-ci lint
	@printf "%s\n" "$(GREEN)✓ Format and lint checks passed$(NC)"

# ci-local-cpp-unit: C++ unit tests (no Docker)
ci-local-cpp-unit:
	@./scripts/ci-cpp-unit-local.sh

# ci-local-cpp-integration: C++ integration tests with MinIO (no Docker)
ci-local-cpp-integration:
	@./scripts/ci-cpp-integration-local.sh

# ci-local-reuse: REUSE license compliance (no Docker)
ci-local-reuse:
	@./scripts/ci-reuse-local.sh

# =============================================================================
# Docker CI (Docker-based)
# =============================================================================

# ci-docker: Quick CI in Docker (format + lint + C++ tests)
ci-docker: format-ci lint ci-docker-cpp
	@printf "%s\n" "$(GREEN)✓ Quick Docker CI checks passed!$(NC)"

# ci-docker-all: Full CI validation (all tests in Docker)
ci-docker-all: format-ci lint docker-test-ros1 docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║     ✓ ALL DOCKER CI CHECKS PASSED!                       ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"

# ci-docker-cpp: C++ library tests in Docker
ci-docker-cpp:
	@./scripts/docker-test-cpp.sh
	@printf "%s\n" "$(GREEN)✓ C++ tests passed$(NC)"

# ci-docker-ros: ROS tests in Docker (all distros)
ci-docker-ros:
	@./scripts/docker-test-ros.sh noetic && ./scripts/docker-test-ros.sh humble
	@printf "%s\n" "$(GREEN)✓ ROS tests passed$(NC)"

# ci-docker-ros1: ROS1 tests in Docker
ci-docker-ros1: docker-test-ros1
	@printf "%s\n" "$(GREEN)✓ ROS1 tests passed$(NC)"

# ci-docker-ros2: ROS2 tests in Docker (all distros)
ci-docker-ros2: docker-test-ros2-humble docker-test-ros2-jazzy docker-test-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ ROS2 tests passed$(NC)"

# ci-docker-zenoh: Zenoh tests in Docker
ci-docker-zenoh:
	@./scripts/ci-docker-zenoh.sh

# ci-docker-coverage: All coverage tests in Docker
ci-docker-coverage: ci-docker-coverage-cpp ci-docker-coverage-ros
	@./scripts/run-coverage.sh all --html

# ci-docker-coverage-cpp: C++ coverage in Docker
ci-docker-coverage-cpp:
	@./scripts/docker-test-cpp.sh --coverage

# ci-docker-coverage-ros: ROS2 coverage in Docker
ci-docker-coverage-ros: docker-coverage
	@printf "%s\n" "$(GREEN)✓ ROS2 coverage complete$(NC)"

# =============================================================================
# E2E Test Targets
# =============================================================================

.PHONY: e2e e2e-docker e2e-docker-ros1 e2e-docker-ros2-humble e2e-docker-ros2-jazzy e2e-docker-ros2-rolling

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

# e2e-docker: All E2E tests in Docker
e2e-docker: e2e-docker-ros1 e2e-docker-ros2-humble e2e-docker-ros2-jazzy e2e-docker-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All E2E tests passed$(NC)"

# e2e-docker-ros1: ROS1 E2E tests in Docker
e2e-docker-ros1:
	@./scripts/docker-test-e2e.sh noetic

# e2e-docker-ros2-humble: ROS2 Humble E2E tests in Docker
e2e-docker-ros2-humble:
	@./scripts/docker-test-e2e.sh humble

# e2e-docker-ros2-jazzy: ROS2 Jazzy E2E tests in Docker
e2e-docker-ros2-jazzy:
	@./scripts/docker-test-e2e.sh jazzy

# e2e-docker-ros2-rolling: ROS2 Rolling E2E tests in Docker
e2e-docker-ros2-rolling:
	@./scripts/docker-test-e2e.sh rolling

# =============================================================================
# Docker Build Targets
# =============================================================================

.PHONY: docker-build-cpp docker-build-zenoh

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

# =============================================================================
# Utility Targets
# =============================================================================

.PHONY: coverage-merge format-ci

# coverage-merge: Merge coverage from all libraries into one report
coverage-merge:
	@./scripts/run-coverage.sh all

# format-ci: Check code formatting (CI style check, no modifications)
format-ci:
	@./scripts/format-code.sh --check

# =============================================================================
# Additional CI Local Targets (with coverage variants)
# =============================================================================

.PHONY: ci-local-cpp-unit-coverage ci-local-cpp-integration-coverage ci-local-reuse-sbom

# ci-local-cpp-unit-coverage: C++ unit tests with coverage
ci-local-cpp-unit-coverage:
	@./scripts/ci-cpp-unit-local.sh --coverage

# ci-local-cpp-integration-coverage: C++ integration tests with coverage
ci-local-cpp-integration-coverage:
	@./scripts/ci-cpp-integration-local.sh --coverage

# ci-local-reuse-sbom: REUSE compliance with SPDX SBOM generation
ci-local-reuse-sbom:
	@./scripts/ci-reuse-local.sh --sbom
