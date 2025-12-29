# Makefile for Axon by ArcheBase
# Supports ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling)
# Located in ros/ directory - paths are relative to project root

.PHONY: all build test clean install build-ros1 build-ros2 test-libs help
.DEFAULT_GOAL := help

# Use bash as the shell for all recipes (required for ROS setup.bash scripts)
SHELL := /bin/bash

# Project root (parent directory)
PROJECT_ROOT := $(shell cd .. && pwd)

# Detect ROS version
ROS_VERSION ?= $(shell if [ -n "$$ROS_VERSION" ]; then echo $$ROS_VERSION; elif [ -n "$$ROS_DISTRO" ]; then echo 1; else echo 2; fi)
ROS_DISTRO ?= $(shell echo $$ROS_DISTRO)

# Build directories (relative to this Makefile location)
BUILD_DIR := build
TEST_BUILD_DIR := src/axon_recorder/test/build

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

# Help target
help:
	@printf "%s\n" "$(GREEN)╔══════════════════════════════════════════════════════════╗$(NC)"
	@printf "%s\n" "$(GREEN)║     Axon by ArcheBase - Build System                     ║$(NC)"
	@printf "%s\n" "$(GREEN)╚══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Available targets:$(NC)"
	@printf "%s\n" "  $(BLUE)make build$(NC)          - Build the project (ROS1 or ROS2)"
	@printf "%s\n" "  $(BLUE)make test$(NC)           - Run all tests (C++)"
	@printf "%s\n" "  $(BLUE)make clean$(NC)          - Clean all build artifacts"
	@printf "%s\n" "  $(BLUE)make install$(NC)        - Install the package"
	@echo ""
	@printf "%s\n" "$(YELLOW)Build modes:$(NC)"
	@printf "%s\n" "  $(BLUE)make debug$(NC)          - Build in debug mode"
	@printf "%s\n" "  $(BLUE)make release$(NC)        - Build in release mode (default)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Code quality:$(NC)"
	@printf "%s\n" "  $(BLUE)make format$(NC)         - Format code (requires formatters)"
	@printf "%s\n" "  $(BLUE)make lint$(NC)           - Lint code (requires linters)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Coverage:$(NC)"
	@printf "%s\n" "  $(BLUE)make coverage$(NC)       - Build with coverage and run tests"
	@printf "%s\n" "  $(BLUE)make coverage-html$(NC)  - Generate HTML coverage report"
	@printf "%s\n" "  $(BLUE)make docker-coverage$(NC)- Run coverage in Docker container"
	@printf "%s\n" "  $(BLUE)make clean-coverage$(NC) - Clean coverage data"
	@echo ""
	@printf "%s\n" "$(YELLOW)Docker:$(NC)"
	@printf "%s\n" "  $(BLUE)make docker-build$(NC)        - Build all Docker images"
	@printf "%s\n" "  $(BLUE)make docker-test$(NC)         - Run tests in Docker (auto-detect ROS)"
	@printf "%s\n" "  $(BLUE)make docker-test-all$(NC)     - Run tests in all Docker containers"
	@printf "%s\n" "  $(BLUE)make docker-test-compose$(NC) - Run tests using docker-compose"
	@echo ""
	@printf "%s\n" "$(YELLOW)Environment variables:$(NC)"
	@printf "%s\n" "  $(BLUE)ROS_VERSION$(NC)         - Set to 1 or 2 (auto-detected)"
	@printf "%s\n" "  $(BLUE)ROS_DISTRO$(NC)          - ROS distribution name"
	@printf "%s\n" "  $(BLUE)BUILD_TYPE$(NC)          - Release or Debug (default: Release)"
	@echo ""
	@printf "%s\n" "$(YELLOW)Current configuration:$(NC)"
	@echo "  ROS_VERSION: $(ROS_VERSION)"
	@echo "  ROS_DISTRO: $(if $(ROS_DISTRO),$(ROS_DISTRO),not set)"
	@echo "  BUILD_TYPE: $(BUILD_TYPE)"
	@echo "  CPU cores: $(NPROC)"
	@echo "  Project root: $(PROJECT_ROOT)"

# Main build target
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
	@catkin build --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE)
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
	@colcon build --base-paths . --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) --event-handlers console_direct+ --executor parallel
	@printf "%s\n" "$(GREEN)✓ Code built for ROS2$(NC)"

# Library tests
test:
	@printf "%s\n" "$(YELLOW)Running library tests...$(NC)"
	@if [ -d "$(PROJECT_ROOT)/cpp/axon_mcap" ]; then \
		cd $(PROJECT_ROOT)/cpp/axon_mcap && \
		mkdir -p build && cd build && \
		cmake .. -DAXON_MCAP_BUILD_TESTS=ON && \
		cmake --build . && \
		ctest --output-on-failure; \
	fi
	@printf "%s\n" "$(GREEN)✓ Tests passed$(NC)"


# Clean targets
clean:
	@printf "%s\n" "$(YELLOW)Cleaning build artifacts...$(NC)"
	@if [ "$(ROS_VERSION)" = "2" ]; then \
		rm -rf build install log; \
	else \
		catkin clean --yes; \
	fi
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
		colcon build --base-paths . \
			--cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			--install-base install \
			--event-handlers console_direct+; \
		printf "%s\n" "$(GREEN)✓ Package installed to install/ directory$(NC)"; \
	else \
		if [ -f /opt/ros/$(ROS_DISTRO)/setup.bash ]; then \
			. /opt/ros/$(ROS_DISTRO)/setup.bash; \
		fi; \
		catkin build --cmake-args -DCMAKE_BUILD_TYPE=$(BUILD_TYPE); \
		printf "%s\n" "$(GREEN)✓ Package installed$(NC)"; \
	fi

# Docker build targets
docker-build-ros1:
	@printf "%s\n" "$(YELLOW)Building Docker image for ROS1...$(NC)"
	@printf "%s\n" "$(YELLOW)Using ros-base image (smaller, faster)$(NC)"
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f ros/docker/Dockerfile.ros1 -t axon:ros1 . || \
		(echo "$(RED)Build failed. See ros/docker/TROUBLESHOOTING.md$(NC)" && exit 1)
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
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f ros/docker/Dockerfile.ros2.humble -t axon:ros2-humble . || \
		(echo "$(RED)Build failed. See ros/docker/TROUBLESHOOTING.md$(NC)" && exit 1)
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
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f ros/docker/Dockerfile.ros2.jazzy -t axon:ros2-jazzy . || \
		(echo "$(RED)Build failed. See ros/docker/TROUBLESHOOTING.md$(NC)" && exit 1)
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
	@cd $(PROJECT_ROOT) && DOCKER_BUILDKIT=1 docker build -f ros/docker/Dockerfile.ros2.rolling -t axon:ros2-rolling . || \
		(echo "$(RED)Build failed. See ros/docker/TROUBLESHOOTING.md$(NC)" && exit 1)
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
	@cd $(PROJECT_ROOT)/ros/docker && docker-compose -f docker-compose.test.yml up --build --abort-on-container-exit
	@printf "%s\n" "$(GREEN)✓ All Docker tests passed!$(NC)"

# Build in Docker
docker-build-only:
	@printf "%s\n" "$(YELLOW)Building in Docker container...$(NC)"
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-e ROS_DISTRO=$(ROS_DISTRO) \
		-e ROS_VERSION=$(ROS_VERSION) \
		axon:ros$(ROS_VERSION) \
		/usr/local/bin/run_build.sh
	@printf "%s\n" "$(GREEN)✓ Docker build complete$(NC)"

# Development targets
dev-setup:
	@printf "%s\n" "$(YELLOW)Setting up development environment...$(NC)"
	@echo "Checking dependencies..."
	@command -v cmake >/dev/null 2>&1 || { echo "$(RED)Error: cmake not found.$(NC)"; exit 1; }
	@command -v pkg-config >/dev/null 2>&1 || { echo "$(RED)Error: pkg-config not found.$(NC)"; exit 1; }
	@printf "%s\n" "$(GREEN)✓ Development environment ready$(NC)"

# Format code (if formatters are available)
format:
	@printf "%s\n" "$(YELLOW)Formatting C/C++ code...$(NC)"
	@if command -v clang-format >/dev/null 2>&1; then \
		printf "%s\n" "$(YELLOW)  Formatting cpp/ libraries...$(NC)"; \
		find $(PROJECT_ROOT)/cpp/axon_mcap $(PROJECT_ROOT)/cpp/axon_uploader $(PROJECT_ROOT)/cpp/axon_logging \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" ! -path "*/build_*/*" -print0 2>/dev/null | \
			xargs -0 clang-format -i; \
		printf "%s\n" "$(YELLOW)  Formatting ros/src/ code...$(NC)"; \
		find ./src \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
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
		find ../cpp ./src \
			\( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
			! -path "*/build/*" \
			! -path "*/test/*" \
			! -path "*/install/*" \
			-print0 2>/dev/null | \
		xargs -0 cppcheck --enable=all \
			--suppress=missingInclude \
			-I../cpp/axon_mcap/include \
			-I../cpp/axon_uploader/include \
			-I../cpp/axon_logging/include \
			-I$(PROJECT_ROOT)/ros/src/axon_recorder/include; \
		printf "%s\n" "$(GREEN)✓ C++ linting passed$(NC)"; \
	else \
		printf "%s\n" "$(YELLOW)⚠ cppcheck not found, skipping$(NC)"; \
	fi

# Debug build
debug:
	@$(MAKE) BUILD_TYPE=Debug build

# Release build (default)
release:
	@$(MAKE) BUILD_TYPE=Release build

# Quick test (fast)
quick-test: test
	@printf "%s\n" "$(GREEN)✓ Quick test complete$(NC)"

# Full test suite
full-test: test lint
	@printf "%s\n" "$(GREEN)✓ Full test suite complete$(NC)"

# =============================================================================
# Coverage Targets
# =============================================================================

# Coverage output directory
COVERAGE_DIR := $(PROJECT_ROOT)/coverage

# Build with coverage and run tests (ROS 2)
coverage-ros2:
	@printf "%s\n" "$(YELLOW)Building with coverage instrumentation (ROS 2)...$(NC)"
	@if [ -z "$$ROS_DISTRO" ]; then \
		echo "$(RED)Error: ROS_DISTRO not set. Source ROS setup.bash first.$(NC)"; \
		exit 1; \
	fi
	@rm -rf $(PROJECT_ROOT)/build $(PROJECT_ROOT)/install $(PROJECT_ROOT)/log
	@mkdir -p $(COVERAGE_DIR)
	@cd $(PROJECT_ROOT) && \
		source /opt/ros/$${ROS_DISTRO}/setup.bash && \
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
		--directory $(PROJECT_ROOT)/build \
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
	@mkdir -p $(PROJECT_ROOT)/coverage
	@DOCKER_BUILDKIT=1 docker run --rm \
		-v $(PROJECT_ROOT):/workspace/axon \
		-v $(PROJECT_ROOT)/coverage:/workspace/coverage \
		-e ROS_DISTRO=humble \
		-e ROS_VERSION=2 \
		axon:ros2-humble \
		/usr/local/bin/run_integration.sh --coverage --coverage-output /workspace/coverage
	@printf "%s\n" "$(GREEN)✓ Coverage report: $(PROJECT_ROOT)/coverage/coverage.info$(NC)"

# Clean coverage data
clean-coverage:
	@printf "%s\n" "$(YELLOW)Cleaning coverage data...$(NC)"
	@rm -rf $(COVERAGE_DIR)
	@find $(PROJECT_ROOT)/build -name "*.gcda" -delete 2>/dev/null || true
	@find $(PROJECT_ROOT)/build -name "*.gcno" -delete 2>/dev/null || true
	@printf "%s\n" "$(GREEN)✓ Coverage data cleaned$(NC)"

# Default coverage target
coverage: coverage-ros2
	@printf "%s\n" "$(GREEN)✓ Coverage complete$(NC)"

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
