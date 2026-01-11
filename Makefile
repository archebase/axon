# Makefile for Axon by ArcheBase
# Supports both C++ core libraries and ROS (1/2) middlewares

.PHONY: all build test clean install build-ros1 build-ros2 test-libs help
.DEFAULT_GOAL := help

# Use bash as the shell for all recipes (required for ROS setup.bash scripts)
SHELL := /bin/bash

# Project root (current directory)
PROJECT_ROOT := $(shell pwd)

# Detect ROS version
ROS_VERSION ?= $(shell if [ -n "$$ROS_VERSION" ]; then echo $$ROS_VERSION; elif [ -n "$$ROS_DISTRO" ]; then echo 1; else echo 2; fi)
ROS_DISTRO ?= $(shell echo $$ROS_DISTRO)

# Build directories (relative to this Makefile location)
BUILD_DIR := core/build
UPLOADER_BUILD_DIR := core/build_uploader
LOGGING_BUILD_DIR := core/build_logging
TEST_BUILD_DIR := core/build
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
	@printf "%s\n" "$(YELLOW)Current:$(NC) ROS=$(ROS_VERSION) DISTRO=$(ROS_DISTRO) TYPE=$(BUILD_TYPE)"

# =============================================================================
# Core C++ Library Targets
# =============================================================================

# Build all C++ libraries
build-core: build-mcap build-uploader build-logging
	@printf "%s\n" "$(GREEN)✓ All C++ libraries built$(NC)"

# Main axon_mcap library
build-mcap:
	@printf "%s\n" "$(YELLOW)Building axon_mcap library...$(NC)"
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
		cmake ../axon_mcap \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_MCAP_BUILD_TESTS=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_mcap build complete$(NC)"

# axon_uploader library
build-uploader:
	@printf "%s\n" "$(YELLOW)Building axon_uploader library...$(NC)"
	@mkdir -p $(UPLOADER_BUILD_DIR)
	@cd $(UPLOADER_BUILD_DIR) && \
		cmake ../axon_uploader \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_UPLOADER_BUILD_TESTS=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_uploader build complete$(NC)"

# axon_logging library
build-logging:
	@printf "%s\n" "$(YELLOW)Building axon_logging library...$(NC)"
	@mkdir -p $(LOGGING_BUILD_DIR)
	@cd $(LOGGING_BUILD_DIR) && \
		cmake ../axon_logging \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DAXON_LOGGING_BUILD_TESTS=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(GREEN)✓ axon_logging build complete$(NC)"

# Test axon_mcap
test-mcap: build-mcap
	@printf "%s\n" "$(YELLOW)Running axon_mcap tests...$(NC)"
	@cd $(BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_mcap tests passed$(NC)"

# Test axon_uploader
test-uploader: build-uploader
	@printf "%s\n" "$(YELLOW)Running axon_uploader tests...$(NC)"
	@cd $(UPLOADER_BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(GREEN)✓ axon_uploader tests passed$(NC)"

# Test axon_logging
test-logging: build-logging
	@printf "%s\n" "$(YELLOW)Running axon_logging tests...$(NC)"
	@cd $(LOGGING_BUILD_DIR) && ctest --output-on-failure
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
		cmake ../axon_mcap \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_MCAP_BUILD_TESTS=ON \
			-DAXON_MCAP_ENABLE_COVERAGE=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(BUILD_DIR) && ctest --output-on-failure
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
	@rm -rf $(UPLOADER_BUILD_DIR)
	@mkdir -p $(UPLOADER_BUILD_DIR)
	@cd $(UPLOADER_BUILD_DIR) && \
		cmake ../axon_uploader \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_UPLOADER_BUILD_TESTS=ON \
			-DAXON_UPLOADER_ENABLE_COVERAGE=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(UPLOADER_BUILD_DIR) && ctest --output-on-failure -j1
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		cd $(UPLOADER_BUILD_DIR) && \
		lcov --capture --directory . --output-file coverage_raw.info && \
		lcov --remove coverage_raw.info \
			'/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/c++/*' \
			--output-file coverage.info && \
		lcov --list coverage.info; \
		printf "%s\n" "$(GREEN)✓ Coverage report: $(UPLOADER_BUILD_DIR)/coverage.info$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: lcov not found$(NC)"; \
		exit 1; \
	fi

# Coverage for axon_logging
coverage-logging:
	@printf "%s\n" "$(YELLOW)Building axon_logging with coverage...$(NC)"
	@rm -rf $(LOGGING_BUILD_DIR)
	@mkdir -p $(LOGGING_BUILD_DIR)
	@cd $(LOGGING_BUILD_DIR) && \
		cmake ../axon_logging \
			-DCMAKE_BUILD_TYPE=Debug \
			-DAXON_LOGGING_BUILD_TESTS=ON \
			-DAXON_LOGGING_ENABLE_COVERAGE=ON && \
		cmake --build . -j$(NPROC)
	@printf "%s\n" "$(YELLOW)Running tests...$(NC)"
	@cd $(LOGGING_BUILD_DIR) && ctest --output-on-failure
	@printf "%s\n" "$(YELLOW)Generating coverage report...$(NC)"
	@if command -v lcov >/dev/null 2>&1; then \
		cd $(LOGGING_BUILD_DIR) && \
		lcov --capture --directory . --output-file coverage_raw.info && \
		lcov --remove coverage_raw.info \
			'/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/c++/*' \
			--output-file coverage.info && \
		lcov --list coverage.info; \
		printf "%s\n" "$(GREEN)✓ Coverage report: $(LOGGING_BUILD_DIR)/coverage.info$(NC)"; \
	else \
		printf "%s\n" "$(RED)Error: lcov not found$(NC)"; \
		exit 1; \
	fi

# Coverage for all C++ libraries
coverage-core: coverage-mcap coverage-uploader coverage-logging
	@printf "%s\n" "$(GREEN)✓ All C++ library coverage reports generated$(NC)"

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
	@rm -rf $(BUILD_DIR) $(UPLOADER_BUILD_DIR) $(LOGGING_BUILD_DIR) $(COVERAGE_DIR)
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
		/usr/local/bin/run_integration.sh
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
		/usr/local/bin/run_integration.sh
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
		/usr/local/bin/run_integration.sh
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
		/usr/local/bin/run_integration.sh
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

# Run tests using docker-compose (all versions in parallel)
docker-test-compose:
	@printf "%s\n" "$(YELLOW)Running tests in all Docker containers (docker-compose)...$(NC)"
	@cd docker && docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit
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
		printf "%s\n" "$(GREEN)✓ C/C++ code formatted$(NC)"; \
	else \
		printf "%s\n" "$(YELLOW)⚠ clang-format not found, skipping C/C++ format...$(NC)"; \
		printf "%s\n" "$(YELLOW)  Install with: sudo apt install clang-format$(NC)"; \
	fi

# Lint code
lint:
	@printf "%s\n" "$(YELLOW)Linting C++ code...$(NC)"
	@if command -v cppcheck >/dev/null 2>&1; then \
		find core middlewares/ros1 middlewares/ros2 \
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
			--cmake-args -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON && \
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
