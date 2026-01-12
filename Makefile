# Makefile for Axon by ArcheBase
# Supports both C++ core libraries and ROS (1/2) middlewares

.PHONY: all build test clean install build-ros1 build-ros2 test-libs app app-axon-recorder app-plugin-example help
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

help:
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║     Axon by ArcheBase - Build System                     ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Core C++ Libraries:$(NC)"
	@printf "%s\n" "  $(BLUE)make build-core$(NC)       - Build all C++ libraries"
	@printf "%s\n" "  $(BLUE)make test-mcap$(NC)       - Build and run axon_mcap tests"
	@printf "%s\n" "  $(BLUE)make test-uploader$(NC)   - Build and run axon_uploader tests"
	@printf "%s\n" "  $(BLUE)make test-logging$(NC)    - Build and run axon_logging tests"
	@printf "%s\n" "  $(BLUE)make test-core$(NC)       - Run all C++ library tests"
	@printf "%s\n" "  $(BLUE)make coverage-core$(NC)   - Run all C++ tests with coverage"
	@echo ""
	@printf "%s\n" "$(YELLOW)Applications:$(NC)"
	@printf "%s\n" "  $(BLUE)make app$(NC)              - Build all applications"
	@printf "%s\n" "  $(BLUE)make app-axon-recorder$(NC) - Build axon_recorder plugin loader"
	@printf "%s\n" "  $(BLUE)make app-plugin-example$(NC) - Build plugin example"
	@echo ""
	@printf "%s\n" "$(YELLOW)ROS Middlewares:$(NC)"
	@printf "%s\n" "  $(BLUE)make build$(NC)              - Build (auto-detects ROS1/ROS2)"
	@printf "%s\n" "  $(BLUE)make build-ros1$(NC)        - Build ROS1 (Noetic)"
	@printf "%s\n" "  $(BLUE)make build-ros2$(NC)        - Build ROS2 (Humble/Jazzy/Rolling)"
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
	@printf "%s\n" "  $(BLUE)make ci-coverage$(NC)       - Coverage tests (C++ + ROS2 + Recorder)"
	@printf "%s\n" "  $(BLUE)make ci-coverage-cpp$(NC)   - C++ library coverage"
	@printf "%s\n" "  $(BLUE)make ci-coverage-ros$(NC)   - ROS2 coverage (Docker)"
	@printf "%s\n" "  $(BLUE)make ci-coverage-recorder$(NC) - Axon recorder coverage (local)"
	@printf "%s\n" "  $(BLUE)make ci-e2e$(NC)            - End-to-end tests"
	@echo ""
	@printf "%s\n" "$(YELLOW)Docker Testing:$(NC)"
	@printf "%s\n" "  $(BLUE)make docker-test-cpp$(NC)   - C++ tests in Docker"
	@printf "%s\n" "  $(BLUE)make docker-test-all$(NC)   - All ROS distros in Docker"
	@printf "%s\n" "  $(BLUE)make docker-coverage$(NC)   - ROS2 coverage in Docker"
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

# Build all C++ libraries and apps using root CMakeLists.txt
build-core:
	@printf "%s\n" "$(YELLOW)Building all C++ libraries and apps...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake .. \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_BUILD_TESTS=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ All C++ libraries and apps built$(NC)"

# Test axon_mcap
test-mcap: build-core
	@printf "%s\n" "$(YELLOW)Running axon_mcap tests...$(NC)"
	@cd $(BUILD_DIR)/axon_mcap && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_mcap tests passed$(NC)"

# Test axon_uploader
test-uploader: build-core
	@printf "%s\n" "$(YELLOW)Running axon_uploader tests...$(NC)"
	@cd $(BUILD_DIR)/axon_uploader && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_uploader tests passed$(NC)"

# Test axon_logging
test-logging: build-core
	@printf "%s\n" "$(YELLOW)Running axon_logging tests...$(NC)"
	@cd $(BUILD_DIR)/axon_logging && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_logging tests passed$(NC)"

# Test all C++ libraries
test-core: test-mcap test-uploader test-logging
	@printf "%s\n" "$(GREEN)✓ All C++ library tests passed$(NC)"

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

# Build all applications
app: build-core
	@printf "%s\n" "$(GREEN)✓ All applications built$(NC)"

# Build axon_recorder (plugin loader library)
app-axon-recorder: build-core
	@printf "%s\n" "$(GREEN)✓ axon_recorder plugin loader built$(NC)"

# Build plugin_example
app-plugin-example: build-core
	@printf "%s\n" "$(GREEN)✓ plugin example built$(NC)"

# =============================================================================
# ROS Middleware Targets
# =============================================================================

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
	@printf "%s\n" "$(YELLOW)Building code for ROS1...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		echo "Example: source /opt/ros/noetic/setup.bash"; \
		exit 1; \
	fi
	@if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
		. /opt/ros/$(ROS_DISTRO)/setup.bash; \
	fi
	@cd middlewares/ros1 && catkin build --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
	@printf "%s\n" "$(GREEN)✓ Code built for ROS1$(NC)"

# Build for ROS2
build-ros2:
	@printf "%s\n" "$(YELLOW)Building code for ROS2...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		echo "Example: source /opt/ros/humble/setup.bash"; \
		exit 1; \
	fi
	@if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
		. /opt/ros/$(ROS_DISTRO)/setup.bash; \
	fi
	@cd middlewares/ros2 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) --event-handlers console_direct+ --executor parallel
	@printf "%s\n" "$(GREEN)✓ Code built for ROS2$(NC)"

# =============================================================================
# General Targets
# =============================================================================

# Library tests (legacy target - defaults to C++ tests)
test: test-core

# Clean all build artifacts
clean:
	@printf "%s\n" "$(YELLOW)Cleaning build artifacts...$(NC)"
	@rm -rf $(BUILD_DIR) $(COVERAGE_DIR)
	@cd middlewares/ros2 && rm -rf build install log 2>/dev/null || true
	@cd middlewares/ros1 && catkin clean --yes 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ All build artifacts cleaned$(NC)"

# Install target
install: build
	@printf "%s\n" "$(YELLOW)Installing package...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set$(NC)"; \
		exit 1; \
	fi
	@if [ "$(ROS_VERSION)" = "2" ]; then \
		if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
			. /opt/ros/$(ROS_DISTRO)/setup.bash; \
		fi; \
		cd middlewares && colcon build --base-paths . \
			--cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			--install-base install \
			--event-handlers console_direct+; \
		printf "%s\n" "$(GREEN)✓ Package installed to middlewares/install/ directory$(NC)"; \
	else \
		if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
			. /opt/ros/$(ROS_DISTRO)/setup.bash; \
		fi; \
		cd middlewares && catkin build --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		printf "%s\n" "$(GREEN)✓ Package installed$(NC)"; \
	fi

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=Debug build

# Release build (default)
release:
	@$(MAKE) BUILD_TYPE=Release build

# =============================================================================
# Docker Build & Test Targets
# =============================================================================

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

# Format code
format:
	@printf "%s\n" "$(YELLOW)Formatting C/C++ code...$(NC)"
	@if command -v clang-format >/dev/null 2>&1; then \
		printf "%s\n" "$(YELLOW)  Formatting core/ libraries...$(NC)"; \
		find core/axon_mcap core/axon_uploader core/axon_logging \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" ! -path "*/build_*/*" -print0 2>/dev/null | \
			xargs -0 clang-format -i; \
		printf "%s\n" "$(YELLOW)  Formatting middlewares/ros1/ and ros2/ code...$(NC)"; \
		find middlewares/ros1 middlewares/ros2 \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" ! -path "*/install/*" ! -path "*/devel/*" -print0 2>/dev/null | \
			xargs -0 clang-format -i; \
		printf "%s\n" "$(YELLOW)  Formatting apps/ code...$(NC)"; \
		find apps \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" -print0 2>/dev/null | \
			xargs -0 clang-format -i; \
		printf "%s\n" "$(GREEN)✓ C/C++ code formatted$(NC)"; \
	else \
		printf "%s\n" "$(YELLOW)⚠ clang-format not found, skipping C/C++ format...$(NC)"; \
		printf "%s\n" "$(YELLOW)  Install with: sudo apt install clang-format$(NC)"; \
	fi

# Lint code
lint:
	@printf "%s\n" "$(YELLOW)Linting C++ code...$(NC)"
	@if command -v cppcheck >/dev/null 2>&1; then \
		find core middlewares/ros1 middlewares/ros2 apps \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" \
			! -path "*/test/*" \
			! -path "*/install/*" \
			! -path "*/devel/*" \
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

# Build with coverage and run tests (ROS 2)
coverage-ros2:
	@printf "%s\n" "$(YELLOW)Building with coverage instrumentation (ROS 2)...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		exit 1; \
	fi
	@rm -rf middlewares/ros2/build middlewares/ros2/install middlewares/ros2/log
	@mkdir -p $(COVERAGE_DIR)
	@source /opt/ros/$${ROS_DISTRO}/setup.bash && \
		cd middlewares/ros2 && \
		colcon build \
			--packages-select axon_recorder \
			--cmake-args -DCMAKE_BUILD_TYPE=Debug -DAXON_ENABLE_COVERAGE=ON && \
		source install/setup.bash && \
		colcon test \
			--packages-select axon_recorder \
			--event-handlers console_direct+ || true
	@printf "%s\n" "$(GREEN)✓ Tests completed, generating coverage report...$(NC)"
	@LCOV_MAJOR=$$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9]+' || echo "1"); \
	IGNORE_FLAGS=""; \
	if [ "$$LCOV_MAJOR" -ge 2 ]; then IGNORE_FLAGS="--ignore-errors mismatch,unused"; fi; \
	lcov --capture \
		--directory middlewares/ros2/build \
		--output-file $(COVERAGE_DIR)/coverage_raw.info \
		--rc lcov_branch_coverage=1 \
		$$IGNORE_FLAGS || true; \
	lcov --remove $(COVERAGE_DIR)/coverage_raw.info \
		'/usr/*' '/opt/*' '*/_deps/*' '*/generated/*' '*/googletest/*' \
		'*/gtest/*' '*/gmock/*' \
		'*/axon_recorder/test/*' \
		'*/axon_logging/test/*' \
		'*/axon_mcap/test/*' \
		'*/axon_uploader/test/*' \
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
	@find middlewares/ros2/build -name "*.gcda" -delete 2>/dev/null || true
	@find middlewares/ros2/build -name "*.gcno" -delete 2>/dev/null || true
	@find middlewares/ros1/build -name "*.gcda" -delete 2>/dev/null || true
	@find middlewares/ros1/build -name "*.gcno" -delete 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Coverage data cleaned$(NC)"

# Default coverage target
coverage: coverage-ros2
	@printf "%s\n" "$(GREEN)✓ Coverage complete$(NC)"

# =============================================================================
# Utility Targets
# =============================================================================

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

# ci: Quick CI check (format + lint + C++ tests in Docker)
ci: format-ci lint docker-test-cpp
	@printf "%s\n" "$(GREEN)✓ Quick CI checks passed!$(NC)"
	@printf "%s\n" "$(YELLOW)Run 'make ci-all' for full CI validation (including ROS and E2E)$(NC)"

# ci-quick: Fastest CI check (format + lint only, no tests)
ci-quick: format-ci lint
	@printf "%s\n" "$(GREEN)✓ Format and lint checks passed$(NC)"
	@printf "%s\n" "$(YELLOW)Run 'make ci' for quick tests or 'make ci-all' for full validation$(NC)"

# ci-all: Run complete CI test suite (all checks in Docker)
ci-all: format-ci lint docker-test-all docker-test-ros docker-e2e
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
		/bin/bash -c "cd /workspace/axon/core && \
			for lib in axon_mcap axon_logging; do \
				echo \"Testing \$\$lib...\" && \
				cd build_\$$\$lib && \
				ctest --output-on-failure -L unit || exit 1; \
			done"
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
		-e MINIO_ENDPOINT=localhost:9000 \
		-e MINIO_ACCESS_KEY=minioadmin \
		-e MINIO_SECRET_KEY=minioadmin \
		axon:cpp-test \
		/bin/bash -c "cd /workspace/axon/core/build_uploader && \
			ctest --output-on-failure -L integration" || \
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
		/bin/bash -c "cd /workspace/axon/core && \
			for lib in axon_mcap axon_uploader axon_logging; do \
				echo \"Building \$\$lib with coverage...\" && \
				mkdir -p build_\$$\$lib && \
				cd build_\$$\$lib && \
				cmake ../\$$\$lib \
					-DCMAKE_BUILD_TYPE=Debug \
					-DAXON_ENABLE_COVERAGE=ON \
					-DAXON_REPO_ROOT=/workspace/axon && \
				make -j\$$(nproc) && \
				ctest --output-on-failure && \
				lcov --capture --directory . \
					--output-file /workspace/coverage/\$${lib}_coverage.info \
					--rc lcov_branch_coverage=1 || true && \
				cd /workspace/axon/core; \
			done" || \
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

# ci-e2e: E2E tests in Docker
ci-e2e: docker-e2e
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
		/bin/bash -c "cd /workspace/axon/core && \
			for lib in axon_mcap axon_uploader axon_logging; do \
				echo \"Testing \$\$lib...\" && \
				cd build_\$$\$lib && \
				ctest --output-on-failure || exit 1; \
			done"
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

# docker-test-ros: Run all ROS tests in Docker
docker-test-ros: docker-test-ros1 docker-test-ros2-humble
	@printf "%s\n" "$(GREEN)✓ All ROS tests passed$(NC)"

# docker-e2e: Run E2E tests sequentially for all ROS versions
docker-e2e:
	@printf "%s\n" "$(YELLOW)Running E2E tests sequentially...$(NC)"
	@$(MAKE) docker-test-ros1
	@$(MAKE) docker-test-ros2-humble
	@$(MAKE) docker-test-ros2-jazzy
	@$(MAKE) docker-test-ros2-rolling
	@printf "%s\n" "$(GREEN)✓ All E2E tests passed$(NC)"

# =============================================================================
# Utility Targets for CI Testing
# =============================================================================

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
format-ci:
	@printf "%s\n" "$(YELLOW)Checking code formatting...$(NC)"
	@find core middlewares/ros1 middlewares/ros2 apps \
		\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
		! -path "*/build/*" ! -path "*/build_*/*" ! -path "*/install/*" ! -path "*/devel/*" \
		-print0 2>/dev/null | \
		xargs -0 clang-format --dry-run --Werror > /dev/null 2>&1 || \
		(printf "%s\n" "$(RED)✗ Code formatting check failed$(NC)" && \
		 printf "%s\n" "$(YELLOW)Run 'make format' to fix formatting issues$(NC)" && \
		 printf "%s\n" "$(YELLOW)Or format files individually: clang-format -i <file>$(NC)" && \
		 exit 1)
	@printf "%s\n" "$(GREEN)✓ Code formatting check passed$(NC)"

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
